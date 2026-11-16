/* D_tim.c 定时器：TIM1/TIM2 同相20kHz时基与PWM、TIM2_CH4 ADC触发、TIM1_CH2 舵机PWM输入捕获 */

#include "D_tim.h"

/* 输入捕获(48MHz)：脉宽500~2500us两端各放宽50us，带内夹回量程、带外整帧丢弃；周期20ms±2ms才认 */
#define TICKS_US        48U
#define PULSE_MIN_US    500U
#define PULSE_MAX_US    2500U
#define PULSE_TOL_US    50U
#define PERIOD_MIN_US   18000U
#define PERIOD_MAX_US   22000U
/* 中断里不调软件除法：ticks/48 改乘倒数取高32位，ticks < 2^32/48 时与整除相同(脉宽带内 ticks ≤ 12.5万) */
#define TICKS_US_RECIP  ((uint32_t)((0x100000000ULL + TICKS_US - 1U) / TICKS_US))
_Static_assert((PULSE_MAX_US + PULSE_TOL_US + 1U) * TICKS_US < 0xFFFFFFFFUL / TICKS_US, "捕获换算超出倒数乘法的精确范围");

static volatile uint32_t s_ovf;       /* TIM1溢出计数，与捕获值组成32位时间戳 */
static volatile uint16_t s_pulse_us;  /* 最新有效脉宽，读后清零 */
static struct {
    uint32_t rise;        /* 上升沿时间戳 */
    uint8_t  falling;     /* 1=等下降沿 */
    uint8_t  have_rise;   /* 已有一个上升沿，可以算周期 */
    uint8_t  period_ok;   /* 最近两个上升沿的间隔在周期容差内 */
} s_ic;

/* 捕获状态机回到"等上升沿" */
static void IC_Reset(void)
{
    s_ic.rise = 0U;
    s_ic.falling = s_ic.have_rise = s_ic.period_ok = 0U;
    TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);
}

/* 20kHz 向上计数时基，ARR 预装载 */
static void Tim_Base(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef base = {0};

    base.TIM_Period      = TIM_PWM_RESOLUTION - 1;
    base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &base);
    TIM_ARRPreloadConfig(tim, ENABLE);
}

/* 比较通道配置：mode=PWM1/PWM2，out=是否出引脚，pulse=初值 */
static TIM_OCInitTypeDef Tim_OC(uint16_t mode, uint16_t out, uint16_t pulse)
{
    TIM_OCInitTypeDef oc;

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = mode;
    oc.TIM_OutputState = out;
    oc.TIM_Pulse       = pulse;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    return oc;
}

void D_TIM2_PWM_Init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    TIM_OCInitTypeDef oc;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_3;                  /* PD3：TIM2_CH2 */
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    Tim_Base(TIM2);
    oc = Tim_OC(TIM_OCMode_PWM1, TIM_OutputState_Enable, 0U);   /* 初值0：常低，电机高阻 */
    TIM_OC2Init(TIM2, &oc);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    /* CH4 不出引脚：CNT 走到 CCR4 时 OC4REF 上升沿经 TRGO 触发一次ADC，每周期一次 */
    oc = Tim_OC(TIM_OCMode_PWM2, TIM_OutputState_Disable, TIM_TRIG_PARK);
    TIM_OC4Init(TIM2, &oc);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC4Ref);
    /* 由 TIM1 更新(ITR0)复位：两个定时器计数同相，两路PWM与采样点在同一个周期边界生效 */
    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_Cmd(TIM2, ENABLE);
}

void D_TIM1_PWM_IC_Init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    NVIC_InitTypeDef  nvic = {0};
    TIM_OCInitTypeDef oc;
    TIM_ICInitTypeDef ic;

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_GPIOA | RCC_PB2Periph_TIM1, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_2;                  /* PD2：TIM1_CH1 */
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_1);               /* PA1：选源前开漏置高释放共线信号，PWM模式时再切捕获 */
    gpio.GPIO_Pin  = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &gpio);

    Tim_Base(TIM1);
    oc = Tim_OC(TIM_OCMode_PWM1, TIM_OutputState_Enable, 0U);   /* 初值0：常低，电机高阻 */
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);      /* 复位TIM2 */

    TIM_ICStructInit(&ic);                         /* CH2 输入捕获：上升沿起，不滤波 */
    ic.TIM_Channel     = TIM_Channel_2;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x00;
    TIM_ICInit(TIM1, &ic);

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2 | TIM_IT_Update);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM1_CC_IRQn;   /* 捕获：由 D_PWM_Input_Enable 打开 */
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = DISABLE;
    NVIC_Init(&nvic);
    nvic.NVIC_IRQChannel                   = TIM1_UP_IRQn;   /* 更新：溢出计数 + 1ms时基 */
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

uint16_t D_PWM_Read(void)
{
    uint32_t irq = __get_MSTATUS();
    uint16_t pulse;

    __disable_irq();
    pulse = s_pulse_us;
    s_pulse_us = 0U;
    __set_MSTATUS(irq);
    return pulse;
}

void D_PWM_Input_Enable(uint8_t en)
{
    GPIO_InitTypeDef gpio = {0};

    TIM_ITConfig(TIM1, TIM_IT_CC2, DISABLE);
    NVIC_DisableIRQ(TIM1_CC_IRQn);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
    s_ovf = 0U;
    IC_Reset();
    s_pulse_us = 0U;

    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    if (en)
    {
        gpio.GPIO_Mode = GPIO_Mode_IPU;            /* 上拉输入，捕获外部信号 */
        GPIO_Init(GPIOA, &gpio);
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);   /* 切引脚期间可能已捕到沿，开中断前再清一次 */
        TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
        TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
        NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
    else
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        gpio.GPIO_Mode = GPIO_Mode_Out_OD;         /* 开漏且锁存为1：只释放信号线，不主动驱高 */
        GPIO_Init(GPIOA, &gpio);
    }
}

/* 捕获中断：上升沿量周期、下降沿量脉宽，周期和脉宽都在容差内才发布 */
void D_TIM1_CC_ISR(void)
{
    uint16_t flags = TIM1->INTFR;
    uint16_t cap;
    uint32_t ts, ticks, us;

    if ((flags & (TIM_IT_CC2 | TIM_FLAG_CC2OF)) == 0U) return;
    if (flags & TIM_FLAG_CC2OF)                    /* 漏了沿：整帧重来 */
    {
        IC_Reset();
        TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
        return;
    }

    cap = TIM_GetCapture2(TIM1);
    ts  = s_ovf * TIM_PWM_RESOLUTION + cap;
    if ((flags & TIM_IT_Update) && cap < TIM_PWM_RESOLUTION / 2)   /* 溢出尚未计数，补一个周期 */
        ts += TIM_PWM_RESOLUTION;
    ticks = ts - s_ic.rise;

    if (!s_ic.falling)
    {
        s_ic.period_ok = (uint8_t)(s_ic.have_rise && ticks >= PERIOD_MIN_US * TICKS_US && ticks <= PERIOD_MAX_US * TICKS_US);
        s_ic.have_rise = 1U;
        s_ic.rise      = ts;
        s_ic.falling   = 1U;
        TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_Low);    /* 改捕下降沿 */
    }
    else
    {
        if (s_ic.period_ok && ticks >= (PULSE_MIN_US - PULSE_TOL_US) * TICKS_US
                           && ticks <= (PULSE_MAX_US + PULSE_TOL_US) * TICKS_US)
        {
            us = (uint32_t)(((uint64_t)(ticks + TICKS_US / 2U) * TICKS_US_RECIP) >> 32);   /* 四舍五入到 us */
            s_pulse_us = (uint16_t)((us < PULSE_MIN_US) ? PULSE_MIN_US : ((us > PULSE_MAX_US) ? PULSE_MAX_US : us));
        }
        s_ic.falling = 0U;
        TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);   /* 改回捕上升沿 */
    }
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
}

/* 更新中断：溢出计数；计数和清标志之间不许被抢占，保证时间戳同一纪元 */
void D_TIM1_UP_ISR(void)
{
    uint32_t irq;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET) return;
    irq = __get_MSTATUS();
    __disable_irq();
    s_ovf++;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    __set_MSTATUS(irq);
}
