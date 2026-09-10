/* D_adc.c ADC1驱动：电流PWM驱动段同步触发DMA，电压注入组软件采样 */

#include "D_adc.h"
#include "D_tim.h"
#include "debug.h"   /* Delay_Us / Delay_Ms */

#define ADC_CURRENT_CHANNEL       ADC_Channel_OPA
#define ADC_VOLTAGE_CHANNEL       ADC_Channel_6
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
#define ADC_DMA_FILTER_SAMPLES    20U
#define ADC_DMA_HALF_SAMPLES      (ADC_DMA_FILTER_SAMPLES / 2U)
#define ADC_CONVERSION_TIMEOUT    100000UL
#define ADC_ZERO_SETTLE_MS        4U

/* 20kHz PWM每周期在驱动段中点产生1个电流样本，20点汇总一次，
 * 即每1ms发布一个控制周期的平均绕组电流，DMA完成中断保持1kHz。
 *
 * 缓冲区用半传输+完成两个中断做乒乓求和：半传输时前10点已经写完、
 * DMA正在写后10点，此时只累加前半；完成时只累加后半再发布。这样
 * 中断永远不会读到DMA正在写的那一半，也把单次中断的时长减半。 */
static volatile uint16_t s_current_dma_buffer[ADC_DMA_FILTER_SAMPLES];
static volatile uint32_t s_current_half_sum;  /* 半传输时锁存的前半和 */
static volatile uint16_t s_current_filtered;  /* 一阶滤波输出(未扣零点) */
static volatile uint16_t s_current_offset;    /* 零电流时的ADC读数 */
static volatile uint8_t  s_current_valid;     /* 电流结果有效标志 */
static volatile uint8_t  s_discard_dma_block; /* 换向/窗口变化后丢弃一块 */

static void ADC_Current_Zero_Calibrate(void);

/*
 * @fn      ADC_Current_Sum
 * @brief   累加缓冲区[from, to)区间的样本
 * @param   from 起始下标
 * @param   to   结束下标(不含)
 * @return  区间和
 */
static uint32_t ADC_Current_Sum(uint8_t from, uint8_t to)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = from; i < to; i++)
        sum += s_current_dma_buffer[i];
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
 * @fn      D_ADC_DMA_Init
 * @brief   初始化ADC1：OPA电流规则组PWM同步DMA，IN6电压注入组软件触发
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
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(ADC_RCC_CLK_DIV);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOD, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    /* 采样档与 D_tim.h 的 ADC_SAMPLE_HALFCYCLES 由上面的 #if 绑定 */
    ADC_RegularChannelConfig(ADC1, ADC_CURRENT_CHANNEL, 1,
                             ADC_CURRENT_SAMPLE_TIME);

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
    dma.DMA_MemoryBaseAddr = (uint32_t)s_current_dma_buffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = ADC_DMA_FILTER_SAMPLES;
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
    (void)D_TIM2_ADC_Trigger_Set(TIM_PWM_RESOLUTION);
    D_ADC_Current_Filter_Reset();

    while (s_current_valid == 0U && --timeout != 0U) {}
    if (timeout != 0U)
    {
        Delay_Ms(ADC_ZERO_SETTLE_MS);   /* 再叠几块，让一阶滤波收敛 */
        s_current_offset = s_current_filtered;
    }

    /* 恢复关闭态，等待 D_Motor_Set 按实际幅值接管 */
    (void)D_TIM2_ADC_Trigger_Set(0);
    D_ADC_Current_Filter_Reset();
}

/*
 * @fn      D_ADC_DMA_ISR
 * @brief   半传输累加前半块，传输完成累加后半块并发布
 * @param   无
 * @return  无
 */
void D_ADC_DMA_ISR(void)
{
    if (DMA_GetITStatus(DMA1_IT_TE1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_GL1);
        s_current_valid = 0;
        s_current_filtered = 0;
        return;
    }

    if (DMA_GetITStatus(DMA1_IT_HT1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        s_current_half_sum = ADC_Current_Sum(0U, ADC_DMA_HALF_SAMPLES);
    }

    if (DMA_GetITStatus(DMA1_IT_TC1) == RESET) return;
    DMA_ClearITPendingBit(DMA1_IT_TC1);

    if (s_discard_dma_block)
    {
        s_discard_dma_block = 0;
        return;
    }

    ADC_Current_Filter_Publish(s_current_half_sum
        + ADC_Current_Sum(ADC_DMA_HALF_SAMPLES, ADC_DMA_FILTER_SAMPLES));
}

/*
 * @fn      D_ADC_Current_Filter_Reset
 * @brief   采样窗口变化或换向时使旧样本失效并丢弃下一整块
 * @param   无
 * @return  无
 *
 * 采样窗口被关闭后DMA不再收到请求、TC也不会再来，valid将一直保持0，
 * 于是"低占空比无窗口"天然表现为无效而不是陈旧值。窗口重新打开时
 * DMA指针停在缓冲区中间，下一块会混入停摆前的旧样本，所以同样要丢。
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
