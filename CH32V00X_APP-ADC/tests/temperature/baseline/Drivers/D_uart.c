/* D_uart.c USART1单线半双工驱动：收发各一条环形队列，中断只搬字节。 */

#include "D_uart.h"

/* 半双工方向切换状态。三个量都被中断和任务共同访问，必须volatile。 */
static RingBuf_t        s_rx_rb;        /* 接收队列：ISR写，任务读             */
static RingBuf_t        s_tx_rb;        /* 发送队列：任务写，ISR读             */
static volatile uint8_t s_tx_busy;      /* 1=正在发送，末字节尚未离开引脚      */
static volatile uint8_t s_enabled;      /* 1=本驱动占用PC0；0=引脚已释放       */
static uint32_t         s_pending_baud; /* !=0时表示有一次待生效的波特率切换   */

/*
 * @fn      Uart_ApplyParams
 * @brief   按指定波特率重配USART1，供初始化与波特率切换共用
 * @param   baudrate 目标波特率
 * @return  无
 *
 * USART_Init只清CTLR1的M/PCE/PS/TE/RE和CTLR3的CTSE/RTSE，
 * 中断使能位与半双工位(HDSEL)都会被保留，所以切换波特率不会掉中断。
 */
static void Uart_ApplyParams(uint32_t baudrate)
{
    USART_InitTypeDef usart = {0};

    usart.USART_BaudRate            = baudrate;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Cmd(USART1, DISABLE);
    USART_Init(USART1, &usart);
    USART_HalfDuplexCmd(USART1, ENABLE); /* 单线：TX/RX内部相连，空闲时释放引脚 */
    USART_Cmd(USART1, ENABLE);
}

/*
 * @fn      Uart_TxStart
 * @brief   把队列首字节压进数据寄存器并打开TXE中断
 * @param   无
 * @return  无
 *
 * **只允许在关中断且 s_tx_busy==0 时调用。** 此时ISR的发送分支是关着的
 * (TXE/TC中断都已失能)，任务在这里取一次tail不会和ISR抢同一个读指针。
 *
 * 顺序不能反：先装首字节再开TXE中断。TXE在空闲时本来就是1，只开中断
 * 而不写数据寄存器不会产生新的触发沿，整帧会永远停在起跑线上。
 */
static void Uart_TxStart(void)
{
    uint8_t byte; /* 待装载的首字节 */

    if (!C_Ring_Buf_Get(&s_tx_rb, &byte)) return;

    s_tx_busy = 1U;
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); /* 半双工：不接收自己发的 */
    USART_ITConfig(USART1, USART_IT_TC,   DISABLE);
    USART_SendData(USART1, byte);
    USART_ITConfig(USART1, USART_IT_TXE,  ENABLE);
}

/*
 * @fn      Uart_TxDone
 * @brief   末字节已完全移出引脚，丢掉自回环并恢复接收
 * @param   无
 * @return  无
 */
static void Uart_TxDone(void)
{
    /* 半双工期间自己发出的字节仍会进接收移位寄存器，开中断前先倒掉，
     * 顺带清掉这段时间累积的ORE，否则第一个真正的回复字节会被吃掉。 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        (void)USART_ReceiveData(USART1);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    s_tx_busy = 0U;
}

/*
 * @fn      D_USART1_Cfg
 * @brief   初始化USART1：PC0重映射、半双工、指定波特率、接收中断
 * @param   baudrate 初始波特率(由配置区的baud_code换算得到)
 * @return  无
 */
void D_USART1_Cfg(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO
                        | RCC_PB2Periph_USART1, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1, ENABLE); /* USART1_TX -> PC0 */

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP; /* 半双工下由外设控制方向 */
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    C_Ring_Buf_Init(&s_rx_rb);
    C_Ring_Buf_Init(&s_tx_rb);
    s_tx_busy      = 0U;
    s_pending_baud = 0U;

    Uart_ApplyParams(baudrate);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    s_enabled = 1U;
}

/*
 * @fn      D_UART_Enable
 * @brief   接管或释放PC0，用于PWM输入与串口总线共线时的独占仲裁
 * @param   en 1=接管，0=释放
 * @return  无
 *
 * PA1(PWM捕获)与PC0(串口)接的是同一根信号线，任何时刻只能有一个占用它。
 * 释放时必须把引脚切成浮空输入并丢掉未发完的数据，否则会把总线钉死。
 */
void D_UART_Enable(uint8_t en)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;

    if (en)
    {
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &gpio);
        USART_Cmd(USART1, ENABLE);
        s_enabled = 1U;
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        NVIC_EnableIRQ(USART1_IRQn);
    }
    else
    {
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
        USART_ITConfig(USART1, USART_IT_TXE,  DISABLE);
        USART_ITConfig(USART1, USART_IT_TC,   DISABLE);
        NVIC_DisableIRQ(USART1_IRQn);
        USART_Cmd(USART1, DISABLE);

        s_enabled = 0U;
        s_tx_busy = 0U;
        C_Ring_Buf_Init(&s_tx_rb); /* 丢弃未发完的回复，避免下次接管时喷旧数据 */

        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* 完全松手，交还共线信号 */
        GPIO_Init(GPIOC, &gpio);
    }
}

uint8_t D_UART_Is_Enabled(void)
{
    return s_enabled;
}

/*
 * @fn      D_UART1_Rx_Get
 * @brief   从接收队列取1个字节
 * @param   data 输出字节
 * @return  1=取到，0=队列空
 */
uint8_t D_UART1_Rx_Get(uint8_t *data)
{
    return (uint8_t)(C_Ring_Buf_Get(&s_rx_rb, data) ? 1U : 0U);
}

/*
 * @fn      D_UART1_Tx_Write
 * @brief   整帧入队；若发送机空闲则立即启动
 * @param   data 待发送数据
 * @param   len  字节数
 * @return  1=已全部入队，0=串口未使能/参数无效/队列剩余空间不足
 *
 * 整帧要么全进要么全不进——半条回复发出去比不发更难排查。
 * 帧已在发送途中时追加的字节会接在后面连续发出，不需要等上一帧结束。
 */
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len)
{
    uint32_t irq_state; /* 进入临界区前的mstatus */
    uint8_t  i;         /* 入队游标               */

    if (!s_enabled || data == 0 || len == 0U) return 0U;
    /* 读count时ISR可能正在推进tail，只会让读数偏大(偏保守)，不会误判为够用 */
    if ((uint16_t)C_Ring_Buf_Get_Count(&s_tx_rb) + len >= RING_BUF_SIZE) return 0U;

    for (i = 0U; i < len; i++) (void)C_Ring_Buf_Put(&s_tx_rb, data[i]);

    irq_state = __get_MSTATUS();
    __disable_irq();
    if (!s_tx_busy) Uart_TxStart();
    __set_MSTATUS(irq_state);
    return 1U;
}

/*
 * @fn      D_UART_SetBaud_Deferred
 * @brief   登记一次波特率切换，等发送队列排空后由D_UART_Service执行
 * @param   baudrate 目标波特率
 * @return  无
 *
 * 上位机改波特率时，"OK"必须用旧波特率发完才能切，否则回复是乱码。
 */
void D_UART_SetBaud_Deferred(uint32_t baudrate)
{
    s_pending_baud = baudrate;
}

/*
 * @fn      D_UART_Service
 * @brief   周期收尾：发送队列排空后执行登记的波特率切换
 * @param   无
 * @return  无
 */
void D_UART_Service(void)
{
    if (s_pending_baud == 0U) return;
    if (s_tx_busy || C_Ring_Buf_Get_Count(&s_tx_rb) != 0U) return;

    Uart_ApplyParams(s_pending_baud);
    s_pending_baud = 0U;
}

/*
 * @fn      D_UART1_ISR
 * @brief   USART1收发中断处理，由USART1_IRQHandler直接调用
 * @param   无
 * @return  无
 *
 * 三个分支互斥：发送期间RXNE中断是关的，TXE与TC也不会同时使能。
 */
void D_UART1_ISR(void)
{
    uint8_t byte; /* 本次搬运的字节 */

    if (!s_enabled) return;

    /* ---- 接收：入队即可，解析交给任务 ---- */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        byte = (uint8_t)USART_ReceiveData(USART1);
        (void)C_Ring_Buf_Put(&s_rx_rb, byte); /* 队列满时丢弃，帧同步靠'#'恢复 */
        return;
    }

    /* ---- 发送：数据寄存器空，续下一个字节 ---- */
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (C_Ring_Buf_Get(&s_tx_rb, &byte))
        {
            USART_SendData(USART1, byte);
            return;
        }

        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        /* 中断若被Flash写入等长临界区推迟，TC可能已经置位；此时再去等
         * TC中断就永远等不到了，直接收尾。 */
        if (USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
            Uart_TxDone();
        else
            USART_ITConfig(USART1, USART_IT_TC, ENABLE);
        return;
    }

    /* ---- 发送：末字节已完全移出引脚，把线还给接收 ---- */
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        Uart_TxDone();
    }
}
