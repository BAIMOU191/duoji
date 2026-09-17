/* D_adc.c ADC1驱动：规则组PWM同步触发DMA(电流，可选电位器)，电压注入组软件采样 */

#include "D_adc.h"
#include "D_tim.h"
#include "debug.h"   /* Delay_Us */

#define ADC_CURRENT_CHANNEL       ADC_Channel_OPA
#define ADC_VOLTAGE_CHANNEL       ADC_Channel_6
#define ADC_VOLTAGE_SAMPLE_TIME   ADC_SampleTime_CyclesMode7
#if D_ADC_ENCODER_ENABLE
#define ADC_ENCODER_CHANNEL       ADC_Channel_2   /* PC4：电位器抽头 */
#define ADC_ENCODER_SAMPLE_TIME   ADC_SampleTime_CyclesMode7
#endif

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
#if D_ADC_ENCODER_ENABLE
#define ADC_DMA_RANKS             2U   /* 规则组序列长度 */
#define ADC_DMA_RANK_ENCODER      1U   /* 电位器在每组转换里的下标 */
#else
#define ADC_DMA_RANKS             1U   /* 规则组序列长度 */
#endif
#define ADC_DMA_FILTER_SAMPLES    20U  /* 每块的触发次数 = 1个控制周期 */
#define ADC_DMA_HALF_SAMPLES      (ADC_DMA_FILTER_SAMPLES / 2U)
#define ADC_DMA_BUFFER_SIZE       (ADC_DMA_FILTER_SAMPLES * ADC_DMA_RANKS)
#define ADC_CONVERSION_TIMEOUT    100000UL
#define ADC_ZERO_BLOCKS           16U  /* 零点标定平均的块数，见 ADC_Current_Zero_Calibrate */

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
#if D_ADC_ENCODER_ENABLE
static volatile uint32_t s_encoder_half_sum;  /* 电位器前半块的和 */
static volatile uint16_t s_encoder_q4;        /* 电位器整块均值，Q4(计数×16) */
static volatile uint8_t  s_encoder_valid;     /* 电位器结果有效标志 */
#endif

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
    uint16_t average = (uint16_t)((sum + ADC_DMA_FILTER_SAMPLES / 2U)
                                / ADC_DMA_FILTER_SAMPLES);

    s_current_avg = s_current_valid
                  ? (uint16_t)(((uint32_t)s_current_avg + average + 1U) / 2U)
                  : average;
    s_current_in_window = (uint8_t)(!s_current_window_off);
    s_current_valid = 1;
    s_current_seq++;
}

#if D_ADC_ENCODER_ENABLE
/* 电位器整块均值保留Q4小数(抽头噪声大于1LSB，平均可恢复亚LSB分辨率)；不叠滤波以保相位裕度。上限65520 */
static void ADC_Encoder_Publish(uint32_t sum)
{
    s_encoder_q4 = (uint16_t)((sum * D_ADC_ENCODER_SCALE
                             + ADC_DMA_FILTER_SAMPLES / 2U)
                            / ADC_DMA_FILTER_SAMPLES);
    s_encoder_valid = 1;
}
#endif

/* 初始化ADC1：OPA电流(+IN2电位器)规则组PWM同步DMA，IN6电压注入组软件触发 */
void D_ADC_DMA_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    DMA_InitTypeDef  dma  = {0};
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
#if D_ADC_ENCODER_ENABLE
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD
                        | RCC_PB2Periph_ADC1, ENABLE);
#else
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1, ENABLE);
#endif
    RCC_ADCCLKConfig(ADC_RCC_CLK_DIV);

    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin = GPIO_Pin_6;   /* PD6 = IN6：电池分压 */
    GPIO_Init(GPIOD, &gpio);
#if D_ADC_ENCODER_ENABLE
    gpio.GPIO_Pin = GPIO_Pin_4;   /* PC4 = IN2：电位器抽头 */
    GPIO_Init(GPIOC, &gpio);
#endif

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
#if D_ADC_ENCODER_ENABLE
    ADC_RegularChannelConfig(ADC1, ADC_ENCODER_CHANNEL, 2,
                             ADC_ENCODER_SAMPLE_TIME);
#endif

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

    s_current_avg = 0;
    s_current_offset = 0;
    s_current_half_sum = 0;
    s_current_valid = 0;
    s_current_in_window = 0;
    s_current_seq = 0;
    s_current_window_off = 0;
#if D_ADC_ENCODER_ENABLE
    s_encoder_half_sum = 0;
    s_encoder_q4 = 0;
    s_encoder_valid = 0;
#endif

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

/* 半传输累加前半块，传输完成累加后半块并发布；电流只标记是否采在驱动段，不丢块 */
void D_ADC_DMA_ISR(void)
{
    if (DMA_GetITStatus(DMA1_IT_TE1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_GL1);
        s_current_valid = 0;
        s_current_avg = 0;
#if D_ADC_ENCODER_ENABLE
        s_encoder_valid = 0;
#endif
        return;
    }

    if (DMA_GetITStatus(DMA1_IT_HT1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        s_current_half_sum = ADC_Block_Sum(0U, ADC_DMA_HALF_SAMPLES,
                                           ADC_DMA_RANK_CURRENT);
#if D_ADC_ENCODER_ENABLE
        s_encoder_half_sum = ADC_Block_Sum(0U, ADC_DMA_HALF_SAMPLES,
                                           ADC_DMA_RANK_ENCODER);
#endif
    }

    if (DMA_GetITStatus(DMA1_IT_TC1) == RESET) return;
    DMA_ClearITPendingBit(DMA1_IT_TC1);

#if D_ADC_ENCODER_ENABLE
    ADC_Encoder_Publish(s_encoder_half_sum
        + ADC_Block_Sum(ADC_DMA_HALF_SAMPLES, ADC_DMA_FILTER_SAMPLES,
                        ADC_DMA_RANK_ENCODER));
#endif

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
uint16_t D_ADC_Current_Offset_Get(void)
{
    return s_current_offset;
}

#if D_ADC_ENCODER_ENABLE
/* 读电位器整块均值(Q4)，返回0仅在开机后未攒满第一块；触发常开，停转/保持/卸力时照常1kHz刷新 */
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
#endif

/* 软件触发一次注入组电压转换，超时返回0 */
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
    return value;
}
