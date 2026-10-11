/* D_adc.c ADC1驱动：规则组PWM同步触发DMA(电流)，电压注入组在规则组空档里采样 */

#include "D_adc.h"
#include "D_tim.h"
#include "debug.h"   /* Delay_Us */

#define ADC_CURRENT_CHANNEL       ADC_Channel_OPA
#define ADC_VOLTAGE_CHANNEL       ADC_Channel_6
#define ADC_VOLTAGE_SAMPLE_TIME   ADC_SampleTime_CyclesMode7

/* 分频与采样档由 D_tim.h 的时序配置反查，未映射的值直接编译失败 */
#if   ADC_CLK_DIV == 2U
  #define ADC_RCC_CLK_DIV         RCC_PCLK2_Div2   /* 24MHz */
#elif ADC_CLK_DIV == 4U
  #define ADC_RCC_CLK_DIV         RCC_PCLK2_Div4   /* 12MHz */
#elif ADC_CLK_DIV == 6U
  #define ADC_RCC_CLK_DIV         RCC_PCLK2_Div6   /*  8MHz */
#else
  #error "ADC_CLK_DIV 没有对应的 RCC 分频宏，见 D_tim.h 的时序配置说明"
#endif

/* 只登记已在参考手册核对过周期数的档位 */
#if   ADC_SAMPLE_HALFCYCLES == 57U
  #define ADC_CURRENT_SAMPLE_TIME ADC_SampleTime_CyclesMode3  /* 28.5周期 */
#else
  #error "ADC_SAMPLE_HALFCYCLES 未登记对应的采样档，见 D_tim.h 的时序配置说明"
#endif

/* 有电位器时每次触发先转电流(孔径落点由CCR4定位，必须排第1)，紧接着转电位器 */
#define ADC_DMA_RANK_CURRENT      0U   /* 电流在每组转换里的下标 */
#define ADC_DMA_RANKS             1U   /* 规则组序列长度 */
#define ADC_DMA_FILTER_SAMPLES    20U  /* 每块的触发次数 = 1个控制周期 */
#define ADC_DMA_HALF_SAMPLES      (ADC_DMA_FILTER_SAMPLES / 2U)
#define ADC_DMA_BUFFER_SIZE       (ADC_DMA_FILTER_SAMPLES * ADC_DMA_RANKS)
#define ADC_CONVERSION_TIMEOUT    100000UL
#define ADC_ZERO_BLOCKS           16U  /* 零点标定平均的块数，见 ADC_Current_Zero_Calibrate */

/* 电池电压(注入组)的采样时机。注入组设为"规则组转完自动接着转"(JAUTO)：每个PWM周期都在电流、电位器之后
 * 紧跟一次电压转换，由硬件排队，不碰规则组触发、不插进正在进行的扫描。DMA 半传输/传输完成中断里读最新结果，
 * 电机持续出力时也每0.5ms刷新——原来只在触发点停进续流段(几乎不出力)时才采，带载保持或连续运动期间
 * 欠压保护和电压补偿都看不到电压变化。采样落在PWM周期的哪个位置随占空比变，读数带纹波：
 * 电压补偿取8样本均值，保护要越界持续50ms才动作，都不受影响。
 *
 * 时序约束：规则组扫描 + 一次注入转换必须在下一次规则组触发之前结束，否则会丢一次触发、DMA 交错错位。
 * 触发点 CCR4 随占空比在 [TRIG_MIN, TRIG_MAX] 内移动，最坏是本周期最晚触发、下周期最早触发：
 *     TRIG_MAX + 扫描 + 注入 < 一个周期 + TRIG_MIN
 * Mode7=239.5周期，与Mode3=28.5周期同一张表，按TIM 48MHz计数。 */
#define ADC_SLOW_CONV_TICKS  ((479U + ADC_CONV_HALFCYCLES) * ADC_CLK_DIV / 2U)
#define ADC_FAST_CONV_TICKS  ((ADC_SAMPLE_HALFCYCLES + ADC_CONV_HALFCYCLES) * ADC_CLK_DIV / 2U)
#define ADC_SCAN_TICKS       (ADC_FAST_CONV_TICKS + (ADC_DMA_RANKS - 1U) * ADC_SLOW_CONV_TICKS + ADC_LATENCY_TICKS)
/* 驱动段内最晚的触发点出现在刚够放下窗口的最小幅值，取 D_TIM2_ADC_Trigger_Set 两个候选里大的；最早出现在满幅 */
#define ADC_TRIG_BLANK_AT_MIN (TIM_PWM_RESOLUTION - ADC_TRIGGER_MIN_PWM_TICKS + ADC_BLANK_TICKS - ADC_LATENCY_TICKS)
#define ADC_TRIG_CENTER_AT_MIN (TIM_PWM_RESOLUTION - ADC_TRIGGER_MIN_PWM_TICKS / 2U - ADC_LATENCY_TICKS - ADC_SAMPLE_TICKS / 2U)
#define ADC_TRIG_CENTER_AT_FULL (TIM_PWM_RESOLUTION - TIM_PWM_RESOLUTION / 2U - ADC_LATENCY_TICKS - ADC_SAMPLE_TICKS / 2U)
#define ADC_TRIG_MAX_TICKS     ((ADC_TRIG_BLANK_AT_MIN > ADC_TRIG_CENTER_AT_MIN) ? ADC_TRIG_BLANK_AT_MIN : ADC_TRIG_CENTER_AT_MIN)
#define ADC_TRIG_LATEST  ((ADC_TRIG_MAX_TICKS > ADC_TRIGGER_PARK_CCR) ? ADC_TRIG_MAX_TICKS : ADC_TRIGGER_PARK_CCR)
#define ADC_TRIG_EARLIEST ((ADC_TRIG_CENTER_AT_FULL < ADC_TRIGGER_PARK_CCR) ? ADC_TRIG_CENTER_AT_FULL : ADC_TRIGGER_PARK_CCR)
_Static_assert (ADC_TRIG_LATEST + ADC_SCAN_TICKS + ADC_SLOW_CONV_TICKS
                < TIM_PWM_RESOLUTION + ADC_TRIG_EARLIEST,
                "规则组扫描加一次注入转换放不进最短的触发间隔，会丢触发");

/* 块均值的"除以20"换成乘倒数(本芯片没有除法指令，这个中断每0.5ms一次)。
 * M=ceil(2^24/20)，被除数 < 2^24/(M*20-2^24) 时与除法逐位相同；电位器块最大 20*4095*16+10 也在范围内 */
#define ADC_AVG_DIV_S 24U
#define ADC_AVG_DIV_M ((uint32_t)(((1UL << ADC_AVG_DIV_S) + ADC_DMA_FILTER_SAMPLES - 1U) / ADC_DMA_FILTER_SAMPLES))
#define ADC_AVG_DIV(x) ((uint32_t)(((uint64_t)(x) * ADC_AVG_DIV_M) >> ADC_AVG_DIV_S))
_Static_assert ((uint64_t)(ADC_AVG_DIV_M * ADC_DMA_FILTER_SAMPLES - (1UL << ADC_AVG_DIV_S))
                * (ADC_DMA_FILTER_SAMPLES * 4095UL * 16UL + ADC_DMA_FILTER_SAMPLES) < (1UL << ADC_AVG_DIV_S),
                "块均值的倒数乘法不再与除法逐位相同");

/* 20kHz 每周期触发一次，20组汇总即每1ms发布一块；缓冲区按 [电流, 电位器] 交错存放。
 * 半传输中断累加前半、完成中断累加后半再发布，永远不读DMA正在写的那一半。 */
static volatile uint16_t s_adc_dma_buffer[ADC_DMA_BUFFER_SIZE];
static volatile uint32_t s_current_half_sum;  /* 半传输时锁存的前半和 */
static volatile uint16_t s_current_avg;       /* 一阶滤波后的电流读数(未扣零点) */
static volatile uint16_t s_current_offset;    /* 零电流时的ADC读数 */
static volatile uint8_t  s_current_valid;     /* 1=至少攒满过一块，读数可用 */
static volatile uint8_t  s_current_in_window; /* 1=本块采在驱动段内，读的是真绕组电流 */
static volatile uint8_t  s_current_window_off; /* 1=触发点已停到续流段 */
static volatile uint16_t s_current_seq;       /* 每发布一块加一，供零点标定攒块 */
static volatile uint16_t s_voltage_raw;       /* 最近一次电池电压注入转换结果(ADC计数) */
static volatile uint8_t  s_voltage_valid;     /* 1=至少转完过一次 */

static void ADC_Current_Zero_Calibrate(void);

/* 在[from, to)转换组里累加指定通道的样本 */
static uint32_t ADC_Block_Sum(uint8_t from, uint8_t to, uint8_t rank)
{
    uint32_t sum = 0;
    uint8_t i;

    for (i = from; i < to; i++)
        sum += s_adc_dma_buffer[i * ADC_DMA_RANKS + rank];
    return sum;
}

/* 发布本块均值并叠 alpha=1/2 一阶滤波。第一块直接置数：从0起步会让随后的零点标定偏低。
 * 换向和窗口开合不清滤波器状态。 */
static void ADC_Current_Publish(uint32_t sum)
{
    uint16_t average = (uint16_t)ADC_AVG_DIV(sum + ADC_DMA_FILTER_SAMPLES / 2U);

    s_current_avg = s_current_valid
                  ? (uint16_t)(((uint32_t)s_current_avg + average + 1U) / 2U)
                  : average;
    s_current_in_window = (uint8_t)(!s_current_window_off);
    s_current_valid = 1;
    s_current_seq++;
}


/* 初始化ADC1：OPA电流(+IN2电位器)规则组PWM同步DMA，IN6电压注入组跟在每次规则组之后自动转换 */
void D_ADC_DMA_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    DMA_InitTypeDef  dma  = {0};
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(ADC_RCC_CLK_DIV);

    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin = GPIO_Pin_6;   /* PD6 = IN6：电池分压 */
    GPIO_Init(GPIOD, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = (ADC_DMA_RANKS > 1U) ? ENABLE : DISABLE; /* 一次触发扫完规则组 */
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = ADC_DMA_RANKS;
    ADC_Init(ADC1, &adc);

    /* 电流必须排第1(孔径落点由CCR4定位)；电位器排第2，用长采样档匹配分压器高源阻抗 */
    ADC_RegularChannelConfig(ADC1, ADC_CURRENT_CHANNEL, 1,
                             ADC_CURRENT_SAMPLE_TIME);

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_VOLTAGE_CHANNEL, 1,
                              ADC_VOLTAGE_SAMPLE_TIME);
    ADC_ExternalTrigInjectedConvConfig(ADC1,
                                       ADC_ExternalTrigInjecConv_None);
    ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);
    ADC_AutoInjectedConvCmd(ADC1, ENABLE);   /* 每次规则组转完自动接一次电压，时序见文件开头的断言 */

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

    s_current_avg = 0;
    s_current_offset = 0;
    s_current_half_sum = 0;
    s_current_valid = 0;
    s_current_in_window = 0;
    s_current_seq = 0;
    s_current_window_off = 0;
    s_voltage_raw = 0;
    s_voltage_valid = 0;

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC | ADC_FLAG_JEOC);
    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    Delay_Us(2);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    ADC_Current_Zero_Calibrate();
}

/* 标定零电流偏置(OPA失调+ADC零点)，1LSB≈1.68mA，不扣会在小电流段固定正偏。
 * 必须在第一次 D_Motor_Set 之前调用：此时两路输出恒低、绕组高阻，是唯一保证真零电流的窗口。 */
static void ADC_Current_Zero_Calibrate(void)
{
    uint32_t sum = 0;
    uint16_t last = s_current_seq;
    uint8_t  blocks;

    /* 平均16块再定零点；任一块超时就放弃、零点保持0(宁可整体偏高，不扣没测准的值) */
    for (blocks = 0U; blocks < ADC_ZERO_BLOCKS; blocks++)
    {
        uint32_t timeout = ADC_CONVERSION_TIMEOUT;

        while (s_current_seq == last && --timeout != 0U) {}
        if (timeout == 0U) return;   /* 采样没跑起来，不写零点 */
        last = s_current_seq;
        sum += s_current_avg;
    }

    s_current_offset = (uint16_t)((sum + ADC_ZERO_BLOCKS / 2U) / ADC_ZERO_BLOCKS);
}

/* 电池电压，每个半块中断调用一次：取硬件自动转换的最新注入结果 */
static void ADC_Voltage_Service(void)
{
    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET) return;
    s_voltage_raw = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    s_voltage_valid = 1;
}

/* 半传输累加前半块，传输完成累加后半块并发布；电流只标记是否采在驱动段，不丢块 */
void D_ADC_DMA_ISR(void)
{
    if (DMA_GetITStatus(DMA1_IT_TE1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_GL1);
        s_current_valid = 0;
        s_current_avg = 0;
        return;
    }

    if (DMA_GetITStatus(DMA1_IT_HT1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        s_current_half_sum = ADC_Block_Sum(0U, ADC_DMA_HALF_SAMPLES,
                                           ADC_DMA_RANK_CURRENT);
        ADC_Voltage_Service();
    }

    if (DMA_GetITStatus(DMA1_IT_TC1) == RESET) return;
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    ADC_Voltage_Service();


    /* 每块照常发布，只如实标记是否采在驱动段：过流/限流看标志，RIV 查询只要最新读数 */
    ADC_Current_Publish(s_current_half_sum
        + ADC_Block_Sum(ADC_DMA_HALF_SAMPLES, ADC_DMA_FILTER_SAMPLES,
                        ADC_DMA_RANK_CURRENT));
}

/* 声明采样点是否落在驱动段；只记标志不清读数，电位器读数不受影响 */
void D_ADC_Current_Window_Set(uint8_t valid)
{
    s_current_window_off = (uint8_t)(valid ? 0U : 1U);
}

/* 读最近一块扣零点后的均值；返回1=采在驱动段(真绕组电流)，0=续流段或尚无数据 */
uint8_t D_ADC_Current_Read(uint16_t *value)
{
    uint32_t irq_state = __get_MSTATUS();
    uint8_t valid;
    uint8_t in_window;
    int32_t corrected;

    if (value == 0) return 0;
    __disable_irq();
    valid     = s_current_valid;
    in_window = s_current_in_window;
    corrected = (int32_t)s_current_avg - (int32_t)s_current_offset;
    __set_MSTATUS(irq_state);

    *value = (valid && corrected > 0) ? (uint16_t)corrected : 0U;
    return (uint8_t)(valid && in_window);
}

/* 上电标定的零电流ADC偏置，供诊断 */



/* 读最近一次电池电压(ADC计数)：只读缓存，不启动转换、不动触发；开机后尚无结果时返回0 */
uint16_t D_ADC_Voltage_Read(void)
{
    uint32_t irq_state = __get_MSTATUS();
    uint16_t value;

    __disable_irq();
    value = s_voltage_valid ? s_voltage_raw : 0U;
    __set_MSTATUS(irq_state);
    return value;
}
