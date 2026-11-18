/* servo_port_example.c —— 移植示例(STM32 HAL，USART1 单线半双工)，按自己的芯片改这几个函数即可
 *
 * 硬件：舵机信号线接 USART TX(开漏/半双工模式)，或经 74HC126 等方向芯片接 TX/RX，方向脚用 set_dir 控制。
 * 接收：中断里把字节塞进环形缓冲，read_byte 从缓冲取；不要在 read_byte 里阻塞等待。
 * 本文件只是示例，不参与编译时请从工程里排除。
 */
#include "servo_drv.h"
#include "stm32f1xx_hal.h"          /* 换成自己的芯片头文件 */

extern UART_HandleTypeDef huart1;

/* ---------------- 接收环形缓冲(中断写，主循环读) ---------------- */
#define RX_SIZE 128U
static volatile uint8_t  s_rx[RX_SIZE];
static volatile uint16_t s_head, s_tail;
static uint8_t s_rx_byte;

void Servo_UartRxStart(void)                         /* 初始化后调用一次，开始单字节中断接收 */
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)  /* HAL 接收完成回调 */
{
    if (h == &huart1)
    {
        uint16_t next = (uint16_t)((s_head + 1U) % RX_SIZE);
        if (next != s_tail) { s_rx[s_head] = s_rx_byte; s_head = next; }   /* 满了丢新字节 */
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }
}

/* ---------------- 回调实现 ---------------- */
static void port_write(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);   /* 阻塞发完；一帧内字节须连续 */
}

static int port_read_byte(uint8_t *b)
{
    if (s_tail == s_head) return 0;
    *b = s_rx[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) % RX_SIZE);
    return 1;
}

static uint32_t port_millis(void)
{
    return HAL_GetTick();
}

static void port_set_dir(uint8_t tx)                 /* 用 HAL 半双工模式时切收发方向；外接方向芯片则改成控制方向脚 */
{
    if (tx) HAL_HalfDuplex_EnableTransmitter(&huart1);
    else  { HAL_HalfDuplex_EnableReceiver(&huart1); Servo_UartRxStart(); }
}

static void port_set_baud(uint32_t bps)              /* 舵机改波特率/恢复默认后，主控同步切换 */
{
    huart1.Init.BaudRate = bps;
    HAL_HalfDuplex_Init(&huart1);
    Servo_UartRxStart();
}

/* ---------------- 使用示例 ---------------- */
static Servo_Bus_t g_bus;

void Servo_Demo(void)
{
    static const Servo_Port_t port = { port_write, port_read_byte, port_millis, port_set_dir, port_set_baud };
    Servo_Info_t info;
    Servo_Telemetry_t t;
    int r;

    Servo_Init(&g_bus, &port);
    g_bus.bcast_max_id = 10U;                         /* 总线上最大ID是10：广播只需等约 120+50 ms */
    Servo_UartRxStart();

    r = Servo_ReadInfo(&g_bus, 0, &info);             /* 先读能力：速度/加减速设置不能超过 info.vmax 等 */
    if (r != SERVO_OK) return;                        /* r<0 驱动错误(超时等)，1~7 舵机状态码 */

    Servo_SetMaxSpeed(&g_bus, 0, info.vmax / 2U, SERVO_STORE_RAM);   /* 临时限速一半，掉电恢复 */
    Servo_SetTorqueLimit(&g_bus, 0, 80U, SERVO_STORE_FLASH);         /* 扭矩上限 80%，掉电保存 */

    Servo_MoveUs(&g_bus, 0, 1500U, 1000U);            /* 1000ms 转到中位 */
    HAL_Delay(1100);
    Servo_ReadTelemetry(&g_bus, 0, &t);               /* t.position、t.current、t.voltage... */

    if (info.type == 2U || info.type == 4U)           /* 磁编码版才有多圈 */
        Servo_MoveTurns(&g_bus, 0, 2048U, 2U, 1000U); /* 到中位并额外转 2 圈，每圈 1000ms */

    Servo_Move(&g_bus, SERVO_BROADCAST_ID, 1024U, 500U);   /* 广播：所有舵机一起动 */
}
