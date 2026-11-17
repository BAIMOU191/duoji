/* D_uart.c USART1单线半双工驱动：收发都走DMA，中断只做收尾。
 *   接收：DMA1通道5循环搬进 s_rx_dma，不再逐字节进中断——高优先级中断、写Flash停顿期间照样收得住；
 *         总线空闲(IDLE)中断通知串口任务立即解析，不等下一个1ms拍。
 *   发送：DMA1通道4整帧发出；DMA搬完后开USART TC中断，末字节真正移出引脚再把线还给接收。
 *         发送期间关RE(接收器)，半双工自回环不会进接收缓冲。
 *   发送用两块缓冲：一块正在发，另一块攒后续整帧，发完自动接着发。 */

#include "D_uart.h"
#include <string.h>

#define RX_DMA        DMA1_Channel5   /* USART1_RX 固定映射 */
#define TX_DMA        DMA1_Channel4   /* USART1_TX 固定映射 */
#define RX_BUF_SIZE   256U            /* 1M 波特率下约 2.5ms 的数据量，够撑过写Flash停顿 */
#define TX_BUF_SIZE   D_UART_TX_FRAME_MAX

#define UART_TX_STALL_MS 20U   /* 发送忙却这么久没进展：判收尾丢失，强制回接收(最低波特率一字节约1ms) */

static uint8_t           s_rx_dma[RX_BUF_SIZE]; /* DMA循环写入 */
static uint16_t          s_rx_tail;             /* 任务读到的位置 */
static volatile uint8_t *s_rx_notify;           /* IDLE时置1，通常是串口任务的就绪标志 */

static uint8_t           s_tx_buf[2][TX_BUF_SIZE];
static volatile uint8_t  s_tx_cur;       /* 正在发送(或刚发完)的缓冲下标 */
static volatile uint8_t  s_tx_pend_len;  /* 另一块缓冲里攒着、尚未开始发的字节数 */
static volatile uint8_t  s_tx_busy;      /* 1=正在发送，末字节尚未离开引脚 */
static volatile uint8_t  s_enabled;      /* 1=本驱动占用PC0；0=引脚已释放 */
static uint32_t          s_pending_baud; /* !=0时表示有一次待生效的波特率切换 */
static volatile uint8_t  s_tx_progress;  /* 每启动一帧/每次收尾加1：服务里据此判断发送是否停滞 */
static uint8_t           s_tx_seen;
static uint16_t          s_tx_seen_cnt;
static uint8_t           s_tx_stall_ms;

/* 按指定波特率重配USART1；USART_Init 不碰中断使能位，HDSEL/DMA请求位切换后重新置上 */
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
    USART_DMACmd(USART1, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

/* 接收DMA当前写到的位置 */
static uint16_t Uart_RxHead(void)
{
    uint16_t head = (uint16_t)(RX_BUF_SIZE - (uint16_t)RX_DMA->CNTR);

    return (head >= RX_BUF_SIZE) ? 0U : head;   /* 循环重装瞬间可能读到 0 */
}

/* 把待发缓冲换成当前缓冲并启动DMA。只在关中断、s_tx_pend_len!=0、DMA空闲时调用 */
static void Uart_TxKick(void)
{
    uint8_t len = s_tx_pend_len;

    s_tx_cur ^= 1U;
    s_tx_pend_len = 0U;
    s_tx_busy = 1U;
    s_tx_progress++;

    USART1->CTLR1 &= (uint16_t)~USART_CTLR1_RE;   /* 半双工：不接收自己发的 */
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    TX_DMA->CFGR &= (uint16_t)~DMA_CFGR1_EN;
    TX_DMA->MADDR = (uint32_t)s_tx_buf[s_tx_cur];
    TX_DMA->CNTR  = len;
    DMA1->INTFCR  = DMA1_IT_GL4;
    TX_DMA->CFGR |= DMA_CFGR1_EN;
}

/* 末字节已移出引脚：恢复接收 */
static void Uart_TxDone(void)
{
    TX_DMA->CFGR &= (uint16_t)~DMA_CFGR1_EN;
    s_tx_busy = 0U;
    s_tx_progress++;
    USART1->CTLR1 |= USART_CTLR1_RE;
}

/* 初始化USART1：PC0重映射、半双工、指定波特率、收发DMA、IDLE/TC中断 */
void D_USART1_Cfg(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    DMA_InitTypeDef  dma = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO
                        | RCC_PB2Periph_USART1, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    GPIO_PinRemapConfig(GPIO_PartialRemap3_USART1, ENABLE); /* USART1_TX -> PC0 */

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP; /* 半双工下由外设控制方向 */
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 接收：外设->内存，循环 */
    DMA_DeInit(RX_DMA);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)s_rx_dma;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = RX_BUF_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_VeryHigh;   /* 来一个字节只给一个字节时间，比ADC急 */
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(RX_DMA, &dma);

    /* 发送：内存->外设，单次；地址和长度每帧现填 */
    DMA_DeInit(TX_DMA);
    dma.DMA_MemoryBaseAddr = (uint32_t)s_tx_buf[0];
    dma.DMA_DIR            = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize     = 1U;
    dma.DMA_Mode           = DMA_Mode_Normal;
    dma.DMA_Priority       = DMA_Priority_Medium;
    DMA_Init(TX_DMA, &dma);
    DMA_ITConfig(TX_DMA, DMA_IT_TC, ENABLE);

    s_rx_tail      = 0U;
    s_tx_cur       = 0U;
    s_tx_pend_len  = 0U;
    s_tx_busy      = 0U;
    s_pending_baud = 0U;

    Uart_ApplyParams(baudrate);
    (void)USART1->STATR;                 /* 清上电残留的 IDLE/错误位 */
    (void)USART1->DATAR;
    DMA_Cmd(RX_DMA, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    s_enabled = 1U;
}

/* 登记IDLE通知标志(串口任务的就绪标志)：收完一段数据立即调度解析 */
void D_UART_SetRxNotify(volatile uint8_t *flag)
{
    s_rx_notify = flag;
}

/* 接管或释放PC0(与PA1的PWM捕获共线，只能独占)；释放时切浮空输入并丢弃未发数据 */
void D_UART_Enable(uint8_t en)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t irq_state;

    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;

    if (en)
    {
        gpio.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &gpio);
        s_rx_tail = Uart_RxHead();        /* 释放期间收到的(PWM共线噪声)全部作废 */
        USART1->CTLR1 |= USART_CTLR1_RE;
        USART_Cmd(USART1, ENABLE);
        s_enabled = 1U;
        USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
        NVIC_EnableIRQ(USART1_IRQn);
        NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    }
    else
    {
        irq_state = __get_MSTATUS();
        __disable_irq();
        USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);
        USART_ITConfig(USART1, USART_IT_TC,   DISABLE);
        TX_DMA->CFGR &= (uint16_t)~DMA_CFGR1_EN;
        NVIC_DisableIRQ(USART1_IRQn);
        NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        USART_Cmd(USART1, DISABLE);
        s_enabled = 0U;
        s_tx_busy = 0U;
        s_tx_pend_len = 0U;               /* 丢弃未发完的回复，避免下次接管时喷旧数据 */
        __set_MSTATUS(irq_state);

        gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* 完全松手，交还共线信号 */
        GPIO_Init(GPIOC, &gpio);
    }
}

/* 从接收缓冲取1字节，0=没有新数据 */
uint8_t D_UART1_Rx_Get(uint8_t *data)
{
    if (s_rx_tail == Uart_RxHead()) return 0U;
    *data = s_rx_dma[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % RX_BUF_SIZE);
    return 1U;
}

/* 整帧入队(全进或全不进)，发送空闲则立即启动；0=未使能/参数无效/空间不足 */
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len)
{
    uint32_t irq_state;
    uint8_t  ok = 0U;

    if (!s_enabled || data == 0 || len == 0U) return 0U;

    irq_state = __get_MSTATUS();
    __disable_irq();
    if ((uint16_t)s_tx_pend_len + len < TX_BUF_SIZE)
    {
        memcpy(&s_tx_buf[s_tx_cur ^ 1U][s_tx_pend_len], data, len);
        s_tx_pend_len = (uint8_t)(s_tx_pend_len + len);
        if (!s_tx_busy) Uart_TxKick();
        ok = 1U;
    }
    __set_MSTATUS(irq_state);
    return ok;
}

/* 登记波特率切换，等发送排空后由 D_UART_Service 执行(回复须用旧波特率发完) */
void D_UART_SetBaud_Deferred(uint32_t baudrate)
{
    s_pending_baud = baudrate;
}

/* 发送停滞兜底(每1ms由串口任务调用)：发送忙而 UART_TX_STALL_MS 内DMA计数与收尾都没动，说明收尾丢了。
 * 丢失则接收器永远关着(主循环与喂狗照常、不会复位)，这里中止DMA、丢弃残帧，按收尾流程把线还给接收 */
static void Uart_TxWatchdog(void)
{
    uint32_t irq_state;
    uint16_t cnt = (uint16_t)TX_DMA->CNTR;

    if (!s_tx_busy || s_tx_progress != s_tx_seen || cnt != s_tx_seen_cnt)
    {
        s_tx_seen = s_tx_progress; s_tx_seen_cnt = cnt; s_tx_stall_ms = 0U;
        return;
    }
    if (++s_tx_stall_ms < UART_TX_STALL_MS) return;
    irq_state = __get_MSTATUS();
    __disable_irq();
    if (s_tx_busy && s_tx_progress == s_tx_seen)
    {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        s_tx_pend_len = 0U;
        Uart_TxDone();
    }
    __set_MSTATUS(irq_state);
    s_tx_stall_ms = 0U;
}

/* 周期收尾：发送停滞兜底；发送排空后执行登记的波特率切换 */
void D_UART_Service(void)
{
    if (s_enabled) Uart_TxWatchdog();
    if (s_pending_baud == 0U || !D_UART_Tx_Idle()) return;

    Uart_ApplyParams(s_pending_baud);
    s_pending_baud = 0U;
}

/* 发送已排空且末字节已移出引脚：此后切波特率或复位都不会截断回复 */
uint8_t D_UART_Tx_Idle(void)
{
    return (uint8_t)(!s_tx_busy && s_tx_pend_len == 0U);
}

/* DMA1通道4：整帧已搬进USART，等TC确认末字节移出引脚 */
void D_UART1_TxDma_ISR(void)
{
    if (!(DMA1->INTFR & DMA1_IT_TC4)) return;
    DMA1->INTFCR = DMA1_IT_GL4;
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
}

/* USART1：IDLE=一段数据收完，通知任务；TC=一帧发完，接着发下一块或还线给接收 */
void D_UART1_ISR(void)
{
    uint16_t sr = (uint16_t)USART1->STATR;

    if (!s_enabled) return;

    /* IDLE 要"读STATR再读DATAR"才清。RXNE=1 时DATAR里是DMA还没取走的新字节，不能抢读，等DMA取走后再清 */
    if (sr & USART_STATR_IDLE)
    {
        if (!(sr & USART_STATR_RXNE)) (void)USART1->DATAR;
        if (s_rx_notify) *s_rx_notify = 1U;
    }
    else if ((sr & (USART_STATR_ORE | USART_STATR_NE | USART_STATR_FE)) && !(sr & USART_STATR_RXNE))
        (void)USART1->DATAR;   /* 残留错误位：同法清掉 */

    if ((USART1->CTLR1 & USART_CTLR1_TCIE) && (sr & USART_STATR_TC))
    {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        if (s_tx_pend_len) Uart_TxKick();   /* 发送期间又攒了帧：接着发，接收器继续关着 */
        else Uart_TxDone();
    }
}
