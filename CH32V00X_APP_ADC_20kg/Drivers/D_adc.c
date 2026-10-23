/* D_adc.c ADC1：TIM2_CH4 触发的规则组DMA(电流 + 电位器) + 自动注入组(电池电压)，每1ms发布一块均值 */

#include "D_adc.h"
#include "debug.h"   /* Delay_Us */

#define BLOCK         20U  /* 每块的触发次数 = 20个PWM周期 = 1ms */
#define RANKS         2U   /* 规则组 [电流, 电位器]，DMA 缓冲区按此交错 */
#define ZERO_BLOCKS   16U  /* 上电零点标定的块数 */
#define ZERO_SHIFT    8U   /* 续流段零点跟踪：每块走差值的1/256，时间常数约0.26s，只跟温漂不跟噪声 */
/* 采样段变了，作废的块数。块与1ms控制拍对齐(块一完成就是控制拍)，控制在拍头改占空比，下个周期生效，
 * 变化最多带进下一块开头一两个采样：作废下一块即可。其它时刻(卸力、保护)的变化也落在被作废的这一块里 */
#define SETTLE_BLOCKS 1U

/* "除以n"换成乘倒数(芯片无除法指令，中断里每0.5ms一次)；在被除数范围内与除法逐位相同 */
#define DIV_M(n)    ((uint32_t)(((1UL << 24) + (n) - 1U) / (n)))
#define DIVN(x, n)  ((uint32_t)(((uint64_t)(x) * DIV_M(n)) >> 24))
#define DIV_MAX_X   (BLOCK * 4095UL * 16UL + BLOCK)
_Static_assert((uint64_t)(DIV_M(BLOCK - 2U) * (BLOCK - 2U) - (1UL << 24)) * DIV_MAX_X < (1UL << 24),
               "DIVN no longer matches division");

static volatile uint16_t s_buf[BLOCK * RANKS];   /* DMA循环缓冲：[电流, 电位器] × 20 */
static volatile struct {
    uint32_t cur_half, pot_half;  /* 半传输时锁存的前半块和 */
    uint16_t cur_min, cur_max;    /* 前半块电流的最小、最大采样 */
    uint16_t pot_min, pot_max;    /* 前半块电位器的最小、最大采样 */
    int32_t  cur_q4;              /* 最新块电流：扣零点、按方向带符号，Q4 */
    int32_t  zero_q12;            /* 电流零点，Q12：上电标定，续流段持续跟踪 */
    uint16_t raw_q4;              /* 最新块电流原始均值(未扣零点)，Q4 */
    uint16_t pot_q4;              /* 最新块电位器均值，Q4 */
    uint16_t vbat;                /* 最近一次电池电压，ADC计数 */
    uint16_t seq;                 /* 每发布一块加一，零点标定用 */
    uint8_t  phase, settle;       /* 采样点所在段、还要作废的块数 */
    uint8_t  drive;               /* 1=最新块整块采在同一方向的驱动段 */
    uint8_t  ready;               /* 1=至少发布过一块 */
} s;

/* 半块某一路的和，同时更新最小、最大采样 */
static uint32_t Block_Sum(uint8_t from, uint8_t to, uint8_t rank, uint16_t *min, uint16_t *max)
{
    uint32_t sum = 0U;

    while (from < to)
    {
        uint16_t v = s_buf[from++ * RANKS + rank];

        sum += v;
        if (v < *min) *min = v;
        if (v > *max) *max = v;
    }
    return sum;
}

/* 发布一块电流：20个采样去掉最大、最小各一个再平均(挡掉单个开关/换向尖峰，以及段变化时混进来的一个旧采样)，
 * 扣零点、反向驱动段取负；整块停在续流段时跟踪零点。不在这里再滤波，使用者需要再自己滤 */
static void Current_Publish(uint32_t sum, uint16_t min, uint16_t max)
{
    uint16_t raw   = (uint16_t)DIVN((sum - min - max) * D_ADC_CURRENT_SCALE + (BLOCK - 2U) / 2U, BLOCK - 2U);
    uint8_t  clean = (uint8_t)(s.settle == 0U);
    int32_t  zero;

    if (!clean) s.settle--;
    if (clean && s.phase == D_CUR_ZERO) s.zero_q12 += (((int32_t)raw << 8) - s.zero_q12) >> ZERO_SHIFT;
    zero     = (s.zero_q12 + 128) >> 8;
    s.raw_q4 = raw;
    s.cur_q4 = (s.phase == D_CUR_REV) ? zero - (int32_t)raw : (int32_t)raw - zero;
    s.drive  = (uint8_t)(clean && (s.phase == D_CUR_FWD || s.phase == D_CUR_REV));
}

/* 注入组结果：每次规则组之后硬件自动转换一次，半块中断里取最新的 */
static void Voltage_Take(void)
{
    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET) return;
    s.vbat = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
}

/* 上电零点：此时两路输出恒低、绕组高阻，保证真零电流；平均16块。采样没跑起来就不写，留给续流段跟踪 */
static void Current_Zero_Calibrate(void)
{
    uint32_t sum = 0U;
    uint16_t last = s.seq;
    uint8_t  n;

    for (n = 0U; n < ZERO_BLOCKS; n++)
    {
        uint32_t timeout = 100000UL;

        while (s.seq == last && --timeout != 0U) {}
        if (timeout == 0U) return;
        last = s.seq;
        sum += s.raw_q4;
    }
    s.zero_q12 = (int32_t)(((sum + ZERO_BLOCKS / 2U) / ZERO_BLOCKS) << 8);
}

void D_ADC_DMA_Init(void)
{
    ADC_InitTypeDef  adc  = {0};
    DMA_InitTypeDef  dma  = {0};
    GPIO_InitTypeDef gpio = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_GPIOD | RCC_PB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div2);             /* 24MHz，与 D_ADC_CUR_xxx 时序一致 */

    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin  = GPIO_Pin_6;                  /* PD6 = IN6：电池分压 */
    GPIO_Init(GPIOD, &gpio);
    gpio.GPIO_Pin  = GPIO_Pin_4;                  /* PC4 = IN2：电位器抽头 */
    GPIO_Init(GPIOC, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T2_TRGO;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = RANKS;
    ADC_Init(ADC1, &adc);
    /* 电流排第1，孔径时刻由触发点决定，运放输出阻抗低用短采样档；电位器源阻抗高用长采样档 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_OPA, 1, ADC_SampleTime_CyclesMode3);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,   2, ADC_SampleTime_CyclesMode7);
    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_CyclesMode7);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
    ADC_ExternalTrigInjectedConvCmd(ADC1, DISABLE);
    ADC_AutoInjectedConvCmd(ADC1, ENABLE);        /* 每次规则组转完自动接一次电压 */
    ADC_Sample_ModeConfig(ADC1, ADC_Sample_NoOver_1M_Mode);
    ADC_BufferCmd(ADC1, DISABLE);

    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->RDATAR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = BLOCK * RANKS;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);
    DMA_ClearFlag(DMA1_FLAG_GL1);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);

    nvic.NVIC_IRQChannel                   = DMA1_Channel1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    ADC_ClearFlag(ADC1, ADC_FLAG_EOC | ADC_FLAG_JEOC);
    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    Delay_Us(2);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    Current_Zero_Calibrate();
}

/* 半传输累加前半块，传输完成累加后半块并发布；永远不读DMA正在写的那一半。返回1=刚发布一块(作1ms控制拍) */
uint8_t D_ADC_DMA_ISR(void)
{
    uint16_t min, max;
    uint32_t sum;

    if (DMA_GetITStatus(DMA1_IT_TE1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_GL1);
        s.ready = 0U;
        s.drive = 0U;
        return 0U;
    }
    if (DMA_GetITStatus(DMA1_IT_HT1) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        min = 0xFFFFU; max = 0U;
        s.cur_half = Block_Sum(0U, BLOCK / 2U, 0U, &min, &max);
        s.cur_min  = min;
        s.cur_max  = max;
        min = 0xFFFFU; max = 0U;
        s.pot_half = Block_Sum(0U, BLOCK / 2U, 1U, &min, &max);
        s.pot_min  = min;
        s.pot_max  = max;
        Voltage_Take();
    }
    if (DMA_GetITStatus(DMA1_IT_TC1) == RESET) return 0U;
    DMA_ClearITPendingBit(DMA1_IT_TC1);
    Voltage_Take();

    /* 电位器同样去掉最大、最小各一个采样再平均：挡掉抽头跳动的单个尖峰。块内匀速转动时两端的采样对称去掉，均值不偏 */
    min = s.pot_min; max = s.pot_max;
    sum = s.pot_half + Block_Sum(BLOCK / 2U, BLOCK, 1U, &min, &max);
    s.pot_q4 = (uint16_t)DIVN((sum - min - max) * D_ADC_ENCODER_SCALE + (BLOCK - 2U) / 2U, BLOCK - 2U);
    min = s.cur_min; max = s.cur_max;
    sum = s.cur_half + Block_Sum(BLOCK / 2U, BLOCK, 0U, &min, &max);
    Current_Publish(sum, min, max);
    s.ready = 1U;
    s.seq++;
    return 1U;
}

/* 段变了先置作废计数再换段：DMA中断夹在两句之间也不会把混合块当成干净的 */
void D_ADC_Current_Phase(uint8_t phase)
{
    if (phase == s.phase) return;
    s.settle = SETTLE_BLOCKS;
    s.phase  = phase;
}

uint8_t D_ADC_Current_Read(int32_t *value_q4)
{
    uint32_t irq = __get_MSTATUS();
    uint8_t  ok;

    __disable_irq();
    *value_q4 = s.ready ? s.cur_q4 : 0;
    ok = (uint8_t)(s.ready && s.drive);
    __set_MSTATUS(irq);
    return ok;
}

uint8_t D_ADC_Encoder_Read(uint16_t *value_q4)
{
    uint32_t irq = __get_MSTATUS();
    uint8_t  ok;

    __disable_irq();
    ok = s.ready;
    if (ok) *value_q4 = s.pot_q4;
    __set_MSTATUS(irq);
    return ok;
}

uint16_t D_ADC_Voltage_Read(void)
{
    return s.vbat;   /* 16位对齐读取本身是原子的 */
}
