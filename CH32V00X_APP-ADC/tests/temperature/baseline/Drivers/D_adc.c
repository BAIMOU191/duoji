/* D_adc.c ADC1驱动：电流+电位器规则组同步触发DMA，电压注入组软件采样 */

#include "D_adc.h"
#include "D_tim.h"
#include "debug.h"   /* Delay_Us / Delay_Ms */

#define ADC_CURRENT_CHANNEL       ADC_Channel_OPA
#define ADC_ENCODER_CHANNEL       ADC_Channel_2   /* PC4：电位器抽头 */
#define ADC_VOLTAGE_CHANNEL       ADC_Channel_6
#define ADC_ENCODER_SAMPLE_TIME   ADC_SampleTime_CyclesMode7
#define ADC_VOLTAGE_SAMPLE_TIME   ADC_SampleTime_CyclesMode7

/* 分频与采样档由 D_tim.h 的时序配置反查，两处不可能再各改各的：
 * 配了没有映射的值会直接编译失败，而不是静默按错误的采样点触发。 */
#if   ADC_CLK_DIV == 2U
  #define ADC_RCC_CLK_DIV         RCC_PCLK2_Div2   /* 24MHz */
#elif ADC_CLK_DIV == 4U
  #define ADC_RCC_CLK_DIV         RCC_PCLK2_Div4   /* 12MHz */
#elif ADC_CLK_DIV == 6U
  #define ADC_RCC_CLK_DIV         RCC_PCLK2_Div6   /*  8MHz */
#else
  #error "ADC_CLK_DIV 没有对应的 RCC 分频宏，见 D_tim.h 的时序配置说明"
#endif

/* 只登记已在参考手册里核对过周期数的档位。要换更短的采样档，先查清该
 * 档的实际ADCCLK周期数，再同时补上这里的映射和 D_tim.h 的半周期数。 */
#if   ADC_SAMPLE_HALFCYCLES == 57U
  #define ADC_CURRENT_SAMPLE_TIME ADC_SampleTime_CyclesMode3  /* 28.5周期 */
#else
  #error "ADC_SAMPLE_HALFCYCLES 未登记对应的采样档，见 D_tim.h 的时序配置说明"
#endif

/* 规则组是2个转换的扫描序列：每次触发先转电流(OPA)，紧接着转电位器。
 * 电流排第1个，孔径落点与单通道时完全一致，D_tim.h 那套驱动段时序不受
 * 影响；电位器的转换排在其后，会越过驱动段末尾进入下一段续流区，对这个
 * 近似直流的分压信号没有影响。 */
#define ADC_DMA_RANKS             2U   /* 规则组序列长度 */
#define ADC_DMA_RANK_CURRENT      0U   /* 电流在每组转换里的下标 */
#define ADC_DMA_RANK_ENCODER      1U   /* 电位器在每组转换里的下标 */
#define ADC_DMA_FILTER_SAMPLES    20U  /* 每块的触发次数 = 1个控制周期 */
#define ADC_DMA_HALF_SAMPLES      (ADC_DMA_FILTER_SAMPLES / 2U)
#define ADC_DMA_BUFFER_SIZE       (ADC_DMA_FILTER_SAMPLES * ADC_DMA_RANKS)
#define ADC_CONVERSION_TIMEOUT    100000UL
#define ADC_ZERO_SETTLE_MS        4U

/* 20kHz PWM每周期在驱动段中点触发一次规则组，扫描出1个电流样本和1个
 * 电位器样本，20组汇总一次，即每1ms发布一个控制周期的平均绕组电流和
 * 平均电位器读数，DMA完成中断保持1kHz。
 *
 * 缓冲区按转换顺序交错存放：[电流0, 电位器0, 电流1, 电位器1, ...]。
 *
 * 缓冲区用半传输+完成两个中断做乒乓求和：半传输时前10组已经写完、
 * DMA正在写后10组，此时只累加前半；完成时只累加后半再发布。这样
 * 中断永远不会读到DMA正在写的那一半，也把单次中断的时长减半。 */
static volatile uint16_t s_adc_dma_buffer[ADC_DMA_BUFFER_SIZE];
static volatile uint32_t s_current_half_sum;  /* 半传输时锁存的前半和 */
static volatile uint16_t s_current_filtered;  /* 一阶滤波输出(未扣零点) */
static volatile uint16_t s_current_offset;    /* 零电流时的ADC读数 */
static volatile uint8_t  s_current_valid;     /* 电流结果有效标志 */
static volatile uint8_t  s_discard_dma_block; /* 换向/窗口变化后丢弃一块 */
static volatile uint8_t  s_current_window_off; /* 1=触发点不在驱动段，电流无效 */
static volatile uint32_t s_encoder_half_sum;  /* 电位器前半块的和 */
static volatile uint16_t s_encoder_q4;        /* 电位器整块均值，Q4(计数×16) */
static volatile uint8_t  s_encoder_valid;     /* 电位器结果有效标志 */

static void ADC_Current_Zero_Calibrate(void);

/*
 * @fn      ADC_Block_Sum
 * @brief   在[from, to)这些转换组里累加指定通道的样本
 * @param   from 起始组下标
 * @param   to   结束组下标(不含)
 * @param   rank 通道在每组转换里的下标(ADC_DMA_RANK_*)
 * @return  区间和
 */
static uint32_t ADC_Block_Sum(uint8_t from, uint8_t to, uint8_t rank)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = from; i < to; i++)
        sum += s_adc_dma_buffer[i * ADC_DMA_RANKS + rank];
    return sum;
}

/*
 * @fn      ADC_Current_Filter_Publish
 * @brief   由整块样本和计算均值并执行alpha为1/2的一阶滤波
 * @param   sum 本块20个样本的总和
 * @return  无
 */
static void ADC_Current_Filter_Publish(uint32_t sum)
{
    uint16_t average;

    average = (uint16_t)((sum + ADC_DMA_FILTER_SAMPLES / 2U)
                       / ADC_DMA_FILTER_SAMPLES);

    if (s_current_valid)
        s_current_filtered = (uint16_t)(((uint32_t)s_current_filtered
                                        + average + 1U) / 2U);
    else
        s_current_filtered = average;

    s_current_valid = 1;
}

/*
 * @fn      ADC_Encoder_Publish
 * @brief   由整块样本和计算电位器均值，保留Q4小数位
 * @param   sum 本块20个电位器样本的总和
 * @return  无
 *
 * 电位器是机械分压，输出本来就近似直流，20点均值已经压掉采样噪声，
 * 这里不再叠一阶滤波：角度环需要的相位裕度比那点噪声更值钱。
 *
 * 但**均值必须保留小数位**。抽头上的噪声大于1个LSB(实测静止时读数在相邻
 * 3个LSB之间跳)，这正是随机抖动能被平均恢复成亚LSB分辨率的前提；除完再
 * 四舍五入回整数，等于把20点平均辛苦挣来的精度当场扔掉。位置环的量化噪声
 * 会被观测器放大成速度噪声，最后变成静止时的PWM抖动，代价一路传到底。
 *
 * 上限：20*4095*16/20 = 65520，uint16 正好装得下。
 */
static void ADC_Encoder_Publish(uint32_t sum)
{
    s_encoder_q4 = (uint16_t)((sum * D_ADC_ENCODER_SCALE
                             + ADC_DMA_FILTER_SAMPLES / 2U)
                            / ADC_DMA_FILTER_SAMPLES);
    s_encoder_valid = 1;
}

/*
 * @fn      D_ADC_DMA_Init
 * @brief   初始化ADC1：OPA电流+IN2电位器规则组PWM同步扫描DMA，
 *          IN6电压注入组软件触发
 * @param   无
 * @return  无
 */
void D_ADC_DMA_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    DMA_InitTypeDef  dma  = {0};
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD
                        | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(ADC_RCC_CLK_DIV);

    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin = GPIO_Pin_6;   /* PD6 = IN6：电池分压 */
    GPIO_Init(GPIOD, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_4;   /* PC4 = IN2：电位器抽头 */
    GPIO_Init(GPIOC, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = ENABLE;   /* 一次触发扫完电流+电位器 */
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = ADC_DMA_RANKS;
    ADC_Init(ADC1, &adc);

    /* 采样档与 D_tim.h 的 ADC_SAMPLE_HALFCYCLES 由上面的 #if 绑定。
     * 电流必须排第1：它的孔径落点由TIM2的CCR4精确定位，任何前置转换
     * 都会把它推离驱动段中点。电位器排第2，用最长采样档匹配分压器的
     * 高源阻抗。 */
    ADC_RegularChannelConfig(ADC1, ADC_CURRENT_CHANNEL, 1,
                             ADC_CURRENT_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC1, ADC_ENCODER_CHANNEL, 2,
                             ADC_ENCODER_SAMPLE_TIME);

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_VOLTAGE_CHANNEL, 1,
                              ADC_VOLTAGE_SAMPLE_TIME);
    ADC_ExternalTrigInjectedConvConfig(ADC1,
                                       ADC_ExternalTrigInjecConv_None);
    ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);
    ADC_AutoInjectedConvCmd(ADC1, DISABLE);

    ADC_Sample_ModeConfig(ADC1, ADC_Sample_NoOver_1M_Mode);
    ADC_BufferCmd(ADC1, DISABLE);

    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)s_adc_dma_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = ADC_DMA_BUFFER_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);
    DMA_ClearFlag(DMA1_FLAG_GL1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);

    nvic.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    s_current_filtered = 0;
    s_current_offset = 0;
    s_current_half_sum = 0;
    s_current_valid = 0;
    s_discard_dma_block = 0;
    s_current_window_off = 0;
    s_encoder_half_sum = 0;
    s_encoder_q4 = 0;
    s_encoder_valid = 0;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC | ADC_FLAG_JEOC);
    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    Delay_Us(2);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    ADC_Current_Zero_Calibrate();
}

/*
 * @fn      ADC_Current_Zero_Calibrate
 * @brief   在零电流窗口测量OPA失调与ADC零点的合成偏置
 * @param   无
 * @return  无
 *
 * **必须在第一次 D_Motor_Set 之前调用。** 上电后TIM1_CH1与TIM2_CH2的
 * 比较值都是0，两路输出恒低，对应AT8236真值表 IN1=IN2=0 -> OUT双高阻，
 * 绕组无电流，检流电阻把ISEN节点拉到地。这是全程唯一能保证真零电流的
 * 窗口：此后 D_Motor_Set(0) 是两路全高的刹车态，虽然两个下管的电流在
 * ISEN节点上进出相消、净值也接近零，但不作保证。
 * (此时AT8236可能已因IN1=IN2=0超过tSLEEP进入睡眠，不影响本测量：
 *  检流电阻仍把节点钳在地，读到的就是放大链自身的偏置。)
 *
 * 60mΩ配8倍增益下1个LSB约1.68mA，OPA几毫伏的失调放大8倍就是几十个LSB，
 * 不扣掉会在小电流段产生固定的正偏。
 */
static void ADC_Current_Zero_Calibrate(void)
{
    uint32_t timeout = ADC_CONVERSION_TIMEOUT;

    /* 借满幅值把触发点摆到周期中部，此时驱动段占满整周期，必定有效 */
    D_ADC_Current_Window_Set(D_TIM2_ADC_Trigger_Set(TIM_PWM_RESOLUTION));
    D_ADC_Current_Filter_Reset();

    while (s_current_valid == 0U && --timeout != 0U) {}
    if (timeout != 0U)
    {
        Delay_Ms(ADC_ZERO_SETTLE_MS);   /* 再叠几块，让一阶滤波收敛 */
        s_current_offset = s_current_filtered;
    }

    /* 恢复无窗口态，等待 D_Motor_Set 按实际幅值接管。触发本身继续跑，
     * 电位器那一路的角度采样不能跟着停。 */
    D_ADC_Current_Window_Set(D_TIM2_ADC_Trigger_Set(0));
    D_ADC_Current_Filter_Reset();
}

/*
 * @fn      D_ADC_DMA_ISR
 * @brief   半传输累加前半块，传输完成累加后半块并发布两个通道
 * @param   无
 * @return  无
 *
 * 电位器不参与丢块：丢块是为电流服务的(换向或采样窗口移动后，一块里会
 * 混进属于旧工况的绕组电流)，而分压器的读数与PWM工况无关，混块里的20点
 * 仍然都是当前角度的有效样本，丢掉只会让角度反馈平白缺一拍。
 */
void D_ADC_DMA_ISR(void)
{
    if (DMA_GetITStatus(DMA1_IT_TE1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_GL1);
        s_current_valid = 0;
        s_current_filtered = 0;
        s_encoder_valid = 0;
        return;
    }

    if (DMA_GetITStatus(DMA1_IT_HT1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        s_current_half_sum = ADC_Block_Sum(0U, ADC_DMA_HALF_SAMPLES,
                                           ADC_DMA_RANK_CURRENT);
        s_encoder_half_sum = ADC_Block_Sum(0U, ADC_DMA_HALF_SAMPLES,
                                           ADC_DMA_RANK_ENCODER);
    }

    if (DMA_GetITStatus(DMA1_IT_TC1) == RESET) return;
    DMA_ClearITPendingBit(DMA1_IT_TC1);

    ADC_Encoder_Publish(s_encoder_half_sum
        + ADC_Block_Sum(ADC_DMA_HALF_SAMPLES, ADC_DMA_FILTER_SAMPLES,
                        ADC_DMA_RANK_ENCODER));

    if (s_current_window_off)
    {
        /* 触发点停在续流段：这一块电流样本采的不是绕组电流。窗口回来时
         * DMA指针停在缓冲区中间，那一块还会混进这些样本，所以一并要丢。 */
        s_current_valid = 0;
        s_discard_dma_block = 1;
        return;
    }

    if (s_discard_dma_block)
    {
        s_discard_dma_block = 0;
        return;
    }

    ADC_Current_Filter_Publish(s_current_half_sum
        + ADC_Block_Sum(ADC_DMA_HALF_SAMPLES, ADC_DMA_FILTER_SAMPLES,
                        ADC_DMA_RANK_CURRENT));
}

/*
 * @fn      D_ADC_Current_Filter_Reset
 * @brief   采样窗口变化或换向时使旧样本失效并丢弃下一整块
 * @param   无
 * @return  无
 *
 * 触发点是常开的(见 D_tim.c 的 ADC_TRIGGER_PARK_CCR)，所以"低占空比无窗口"
 * 不再表现为收不到TC，而是由 D_ADC_Current_Window_Set 显式标无效。这里只
 * 负责"旧样本作废+丢下一块"：换向前后两块来自不同电流方向，窗口移动
 * 前后两块来自不同的采样点，混在一个均值里都是错的。
 */
void D_ADC_Current_Filter_Reset(void)
{
    uint32_t irq_state = __get_MSTATUS();

    __disable_irq();
    s_current_filtered = 0;
    s_current_half_sum = 0;
    s_current_valid = 0;
    s_discard_dma_block = 1;
    __set_MSTATUS(irq_state);
}

/*
 * @fn      D_ADC_Current_Window_Set
 * @brief   声明当前是否存在合法的电流采样窗口
 * @param   valid D_TIM2_ADC_Trigger_Set 的返回值
 * @return  无
 *
 * 触发点现在是常开的(见 D_tim.c 的 ADC_TRIGGER_PARK_CCR)，"占空比太低所以
 * 没有电流样本"不再能靠"收不到TC中断"来表达，必须由设置触发点的人显式说。
 * 电位器角度不受影响：它跟工况无关，触发在哪儿都是有效读数。
 */
void D_ADC_Current_Window_Set(uint8_t valid)
{
    uint32_t irq_state = __get_MSTATUS();

    __disable_irq();
    s_current_window_off = (uint8_t)(valid ? 0U : 1U);
    if (s_current_window_off)
    {
        s_current_filtered = 0;
        s_current_half_sum = 0;
        s_current_valid = 0;
    }
    __set_MSTATUS(irq_state);
}

/*
 * @fn      D_ADC_Current_Read
 * @brief   原子读取扣除零点后的滤波ADC值
 * @param   value ADC结果输出，结果无效时保持调用方原值
 * @return  1=存在有效结果，0=无采样窗口或滤波块未就绪
 */
uint8_t D_ADC_Current_Read(uint16_t *value)
{
    uint32_t irq_state = __get_MSTATUS();
    uint8_t valid;
    int32_t corrected;

    if (value == 0) return 0;
    __disable_irq();
    valid = s_current_valid;
    corrected = (int32_t)s_current_filtered - (int32_t)s_current_offset;
    __set_MSTATUS(irq_state);

    if (valid)
        *value = (corrected > 0) ? (uint16_t)corrected : 0U;
    return valid;
}

/*
 * @fn      D_ADC_Current_Offset_Get
 * @brief   读取上电标定得到的零电流ADC偏置，供诊断与标定报告使用
 * @param   无
 * @return  零点ADC计数
 */
uint16_t D_ADC_Current_Offset_Get(void)
{
    return s_current_offset;
}

/*
 * @fn      D_ADC_Encoder_Read
 * @brief   原子读取电位器通道的整块均值，Q4定点(ADC计数×16)
 * @param   value_q4 结果输出，单位见 D_ADC_ENCODER_SCALE；无效时保持原值
 * @return  1=存在有效结果，0=参数无效或开机后还没攒满一块
 *
 * 触发是常开的：幅值低到放不下电流采样窗口时，D_TIM2_ADC_Trigger_Set 只把
 * 触发点挪到续流段而不关掉它，所以电机停转、保持位置、卸力这些时候角度
 * 照样按1kHz刷新——位置环最需要反馈的恰恰是这些时候。
 *
 * 返回0只会出现在开机后还没攒满第一块的那1ms。抽头转出碳膜(电位器死区)
 * 在这一层看不出来：那要靠下拉电阻把读数钳到地，由 A_Sensor 按门限判定。
 */
uint8_t D_ADC_Encoder_Read(uint16_t *value_q4)
{
    uint32_t irq_state = __get_MSTATUS();
    uint8_t valid;
    uint16_t q4;

    if (value_q4 == 0) return 0;
    __disable_irq();
    valid = s_encoder_valid;
    q4 = s_encoder_q4;
    __set_MSTATUS(irq_state);

    if (valid) *value_q4 = q4;
    return valid;
}

/*
 * @fn      D_ADC_Voltage_Read
 * @brief   软件触发一次注入组电压转换
 * @param   无
 * @return  单次电压通道ADC值，超时返回0
 *
 * 期间规则组外部触发被关闭，电流采样停摆十几微秒，返回前主动丢弃
 * 一整块，避免把跨越停摆点的混合样本当成一个控制周期的平均值。
 */
uint16_t D_ADC_Voltage_Read(void)
{
    uint32_t timeout = ADC_CONVERSION_TIMEOUT;
    uint16_t value = 0;

    ADC_ExternalTrigConvCmd(ADC1, DISABLE);
    Delay_Us(4);

    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET
        && --timeout != 0U) {}

    if (timeout != 0U)
        value = ADC_GetInjectedConversionValue(ADC1,
                                               ADC_InjectedChannel_1);

    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
    D_ADC_Current_Filter_Reset();
    return value;
}
