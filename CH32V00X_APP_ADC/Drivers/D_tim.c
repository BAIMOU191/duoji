/* D_tim.c 定时器驱动：TIM1为主时基，TIM2同步输出PWM并产生ADC触发 */

#include "D_tim.h"

#define PULSE_MIN_US       500    /* 协议脉宽下限(us)，上报值不会低于它 */
#define PULSE_MAX_US       2500   /* 协议脉宽上限(us)，上报值不会高于它 */
/* 捕获窗口两端各放宽容差：容差带内的脉宽夹回协议量程上报(信号源满杆常略超端点)，窗口外整帧丢弃 */
#define PULSE_TOLERANCE_US 50
#define PERIOD_NOMINAL_US  20000  /* 输入信号标称周期(us) */
#define PERIOD_TOLERANCE_US 2000  /* 周期容差(us) */
#define PERIOD_MIN_US      (PERIOD_NOMINAL_US - PERIOD_TOLERANCE_US)
#define PERIOD_MAX_US      (PERIOD_NOMINAL_US + PERIOD_TOLERANCE_US)
#define TIMER_TICKS_US     48     /* 每微秒定时器计数 */
#define PULSE_MIN_TICKS    ((uint32_t)(PULSE_MIN_US - PULSE_TOLERANCE_US) * TIMER_TICKS_US)
#define PULSE_MAX_TICKS    ((uint32_t)(PULSE_MAX_US + PULSE_TOLERANCE_US) * TIMER_TICKS_US)
#define PERIOD_MIN_TICKS   ((uint32_t)PERIOD_MIN_US * TIMER_TICKS_US)
#define PERIOD_MAX_TICKS   ((uint32_t)PERIOD_MAX_US * TIMER_TICKS_US)

/* 电流采样触发点。AT8236慢衰减下一个PWM周期分两段：
 *   [0, ARR-mag)   续流段，两下管导通，检流电阻净电流为零；
 *   [ARR-mag, ARR) 驱动段，全部绕组电流流过检流电阻。
 * 采样窗口必须落在周期尾部的驱动段，时序常数见 D_tim.h。 */
/* 窗口放不下时触发点停到续流段正中而不关闭：电位器角度仍需1kHz刷新，电流由上层标为无效 */
#define ADC_TRIGGER_PARK_CCR          (TIM_PWM_RESOLUTION / 2U)

static uint8_t  s_ic_wait_falling; /* 0=等待上升沿，1=等待下降沿 */
static uint8_t  s_ic_have_rise;    /* 已捕获上升沿标志 */
static uint32_t s_ic_rise_time;    /* 上升沿时间戳 */
static uint8_t  s_ic_period_valid; /* 输入周期有效标志 */

static volatile uint32_t s_ic_overflow; /* TIM1溢出计数，用于32位时间戳 */
static volatile uint16_t s_pulse_us;    /* 最新捕获脉宽，0=无有效帧 */

/* 复位捕获状态机，恢复上升沿捕获 */
static void PWM_ResetCaptureState(void)
{
    s_ic_wait_falling = 0;
    s_ic_have_rise = 0;
    s_ic_rise_time = 0;
    s_ic_period_valid = 0;
    TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);
}

/* 清除当前捕获结果 */
static void PWM_InvalidateFrames(void)
{
    s_pulse_us = 0;
}

/* 初始化TIM2：PD3输出20kHz PWM，CH4产生ADC采样TRGO */
void D_TIM2_PWM_Init(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure        = {0};
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef        TIM_OCInitStructure       = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_TIM2, ENABLE);

    /* PD3：TIM2_CH2 PWM输出，复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 时基：48MHz / 2400 = 20kHz */
    TIM_TimeBaseInitStructure.TIM_Prescaler   = 0;
    TIM_TimeBaseInitStructure.TIM_Period      = TIM_PWM_RESOLUTION - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* CH2 PWM1模式，初始占空比0 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* CH4仅作内部采样标记：CNT==CCR4 时OC4REF上升沿经TRGO触发ADC规则组 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse       = ADC_TRIGGER_PARK_CCR;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC4Ref);

    /* TIM2以TIM1_TRGO(ITR0)为复位触发，保证正反转PWM与CH4同相。 */
    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

    TIM_Cmd(TIM2, ENABLE);
}

/* 按PWM幅值把电流采样点定位到驱动段内；返回0=占空比过低，触发点已停到续流段。
 * 占空比够宽时孔径居中于驱动段；偏窄时后移到刚好避开 tBLANK 的最早位置。
 * CCR4预装载，与PWM比较值同一周期边界生效，不错拍。 */
uint8_t D_TIM2_ADC_Trigger_Set(uint16_t pwm_ticks)
{
    uint32_t centered;   /* 孔径居中于驱动段所需的CCR4 */
    uint32_t blank_min;  /* 满足tBLANK前沿余量的最小CCR4 */
    uint32_t compare;

    if (pwm_ticks > TIM_PWM_RESOLUTION)
        pwm_ticks = TIM_PWM_RESOLUTION;

    if (pwm_ticks < ADC_TRIGGER_MIN_PWM_TICKS)
    {
        TIM_SetCompare4(TIM2, ADC_TRIGGER_PARK_CCR);
        return 0;   /* 转换继续，但电流那一路的读数此刻没有意义 */
    }

    /* 驱动段 = [RES-mag, RES)，中点 = RES - mag/2 */
    centered  = (uint32_t)TIM_PWM_RESOLUTION - pwm_ticks / 2U
              - ADC_LATENCY_TICKS - ADC_SAMPLE_TICKS / 2U;
    blank_min = (uint32_t)TIM_PWM_RESOLUTION - pwm_ticks
              + ADC_BLANK_TICKS - ADC_LATENCY_TICKS;

    compare = (centered > blank_min) ? centered : blank_min;
    TIM_SetCompare4(TIM2, (uint16_t)compare);
    return 1;
}

/* 初始化TIM1：PD2输出20kHz PWM，PA1输入捕获，更新中断提供调度时基 */
void D_TIM1_PWM_IC_Init(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure        = {0};
    NVIC_InitTypeDef         NVIC_InitStructure        = {0};
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef        TIM_OCInitStructure       = {0};
    TIM_ICInitTypeDef        TIM_ICInitStructure       = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD | RCC_PB2Periph_GPIOA | RCC_PB2Periph_TIM1, ENABLE);

    /* PD2：TIM1_CH1 PWM输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* PA1：选源前先以开漏高电平释放，PWM模式启用时再切输入捕获 */
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 时基：48MHz / 2400 = 20kHz，与TIM2一致 */
    TIM_TimeBaseInitStructure.TIM_Prescaler      = 0;
    TIM_TimeBaseInitStructure.TIM_Period         = TIM_PWM_RESOLUTION - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision  = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode    = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    /* CH1 PWM1模式，初始占空比0 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update); /* TRGO同步TIM2 */
    /* 两个方向PWM和ADC采样点都在周期边界同步生效。 */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* CH2 输入捕获，上升沿起始，不启用数字滤波 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel    = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x00;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2 | TIM_IT_Update);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    /* CC中断：捕获处理 */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* UP中断：捕获溢出计数，并在中断入口中分频生成1ms调度时基。 */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/* 读最新捕获脉宽(us)，读后清除，无新帧返回0 */
uint16_t D_PWM_Read(void)
{
    uint32_t irq_state = __get_MSTATUS();
    uint16_t pulse;

    __disable_irq();
    pulse = s_pulse_us;
    s_pulse_us = 0;
    __set_MSTATUS(irq_state);
    return pulse;
}

/* 使能/失能输入捕获：使能时PA1上拉输入，失能时开漏置高释放 */
void D_PWM_Input_Enable(uint8_t en)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;

    TIM_ITConfig(TIM1, TIM_IT_CC2, DISABLE);
    NVIC_DisableIRQ(TIM1_CC_IRQn);
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
    s_ic_overflow = 0;
    PWM_ResetCaptureState();
    PWM_InvalidateFrames();

    if (en)
    {
        gpio.GPIO_Mode = GPIO_Mode_IPU; /* 上拉输入，捕获外部信号 */
        GPIO_Init(GPIOA, &gpio);
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
        TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
        TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
        NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
    else
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        gpio.GPIO_Mode = GPIO_Mode_Out_OD; /* 开漏置1，释放共线信号 */
        GPIO_Init(GPIOA, &gpio);
    }
}

/* 捕获中断：双沿测量高电平脉宽并校验输入周期 */
void D_TIM1_CC_ISR(void)
{
    uint16_t flags = TIM1->INTFR;
    uint16_t cap;
    uint32_t ts;
    uint32_t ticks;
    uint32_t pulse_us;

    if ((flags & (TIM_IT_CC2 | TIM_FLAG_CC2OF)) == 0) return;

    if (flags & TIM_FLAG_CC2OF)
    {
        PWM_ResetCaptureState();
        TIM_ClearFlag(TIM1, TIM_FLAG_CC2OF);
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
        return;
    }

    cap = TIM_GetCapture2(TIM1);
    ts = s_ic_overflow * TIM_PWM_RESOLUTION + cap;
    if ((flags & TIM_IT_Update) && cap < (TIM_PWM_RESOLUTION / 2))
        ts += TIM_PWM_RESOLUTION;

    if (!s_ic_wait_falling)
    {
        ticks = ts - s_ic_rise_time;
        s_ic_period_valid = s_ic_have_rise
                       && ticks >= PERIOD_MIN_TICKS
                       && ticks <= PERIOD_MAX_TICKS;
        s_ic_have_rise = 1;
        s_ic_rise_time = ts;
        s_ic_wait_falling = 1;
        TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_Low); /* 切换为下降沿捕获 */
    }
    else
    {
        ticks = ts - s_ic_rise_time;
        if (s_ic_period_valid && ticks >= PULSE_MIN_TICKS
                            && ticks <= PULSE_MAX_TICKS)
        {
            /* 窗口内才认，认了夹进协议量程 */
            pulse_us = (ticks + TIMER_TICKS_US / 2) / TIMER_TICKS_US;
            if (pulse_us < PULSE_MIN_US) pulse_us = PULSE_MIN_US;
            if (pulse_us > PULSE_MAX_US) pulse_us = PULSE_MAX_US;
            s_pulse_us = (uint16_t)pulse_us;
        }
        s_ic_wait_falling = 0;
        TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High); /* 切换回上升沿捕获 */
    }

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
}

/* 溢出中断：累加溢出计数，供32位时间戳 */
void D_TIM1_UP_ISR(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        uint32_t irq_state = __get_MSTATUS();

        /* 禁止在计数和清标志之间抢占，保证时间戳同一纪元 */
        __disable_irq();
        s_ic_overflow++;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        __set_MSTATUS(irq_state);
    }
}
