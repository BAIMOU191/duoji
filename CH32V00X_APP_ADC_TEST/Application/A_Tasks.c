/* A_Tasks.c —— 应用任务注册表，只定"谁/多久/多优先"，业务在各模块。
 *   优先级 周期   任务         模块
 *     0    1ms  ServoControl  A_Servo
 *     1    1ms  Uart          A_UartCmd + iap
 *     2   10ms  Protect       A_Protect
 *     3   20ms  State         A_StatusLed + 喂狗
 *     4   20ms  ConfigSave    A_Config
 *     5   20ms  Print         A_Debug(默认关)
 */
#include "A_Tasks.h"
#include "A_Servo.h"
#include "A_UartCmd.h"
#include "A_Protect.h"
#include "A_StatusLed.h"
#include "A_Debug.h"
#include "A_Config.h"
#include "C_Task_Scheduler.h"
#include "D_iwdg.h"
#include "D_adc.h"
#include "D_uart.h"
#include "iap.h"

#if 0   /* TEST 固件：不跑舵机控制和保护 */
static void Task_ServoControl(void)
{
    A_Servo_Control(); /* 1ms控制拍 */
}
#endif

static void Task_Uart(void)
{
    A_Uart_Process();
    IAP_Service();
}

#if 0   /* TEST 固件：不跑舵机控制和保护 */
static void Task_Protect(void)
{
    A_Protect_Task(); /* 10ms基拍 */
}
#endif

/* TEST 固件：每 50ms 打印一次电位器 ADC 原始值(0~4095，1ms 块均值)，格式 "pot:1234\r\n"。
 * 总线空闲(无半帧且 10ms 无接收)才发，不影响查询和升级 */
static void Task_PotPrint(void)
{
    uint8_t  s[12], n = 0U, i;
    uint8_t  d[5];
    uint8_t  k = 0U;
    uint16_t q4, v;

    if (!D_ADC_Encoder_Read(&q4) || !D_UART_Tx_Idle() || !A_Uart_DebugReady()) return;
    v = (uint16_t)((q4 + 8U) >> 4);
    do { d[k++] = (uint8_t)('0' + v % 10U); v /= 10U; } while (v != 0U);
    s[n++] = 'p'; s[n++] = 'o'; s[n++] = 't'; s[n++] = ':';
    for (i = k; i > 0U; i--) s[n++] = d[i - 1U];
    s[n++] = '\r'; s[n++] = '\n';
    (void)D_UART1_Tx_Write(s, n);
}

static void Task_State(void)
{
    A_StatusLed_Task();
    D_IWDG_Feed(); /* 卡住则看门狗复位 */
}

#define CONFIG_SAVE_PERIOD_MS     20U  /* 顺延后的重试周期 */
#define CONFIG_SAVE_HOLD_MAX_MS  500U  /* 运动中最多顺延，防止流指令饿死落盘 */

static uint16_t s_save_hold_ms;

static void Task_ConfigSave(void)
{
    /* 写Flash会冻结CPU数ms：运动中顺延(参数仍即时生效)，到上限强写 */
    if (!A_Servo_CanSave() && (s_save_hold_ms += CONFIG_SAVE_PERIOD_MS) < CONFIG_SAVE_HOLD_MAX_MS) return;
    s_save_hold_ms = 0U;
    (void)A_Config_SaveTask(); /* 参数变脏时异步写Flash */
}

#if TASK_FIREWATER_ENABLE
static void Task_Print(void)
{
    A_Debug_Task();
}
#endif

/*                           入口函数        周期ms                  优先级(0最高) */
static Task_t s_tasks[] = {
    // TASK_DEF(Task_ServoControl, 1,                        0),   /* TEST 固件：关闭 */
    TASK_DEF(Task_Uart,         1,                        1),
    // TASK_DEF(Task_Protect,      10,                       2),   /* TEST 固件：关闭 */
    TASK_DEF(Task_PotPrint,     50,                       2),      /* TEST 固件：电位器原始值 */
    TASK_DEF(Task_State,        STATUS_LED_PERIOD_MS,     3),
    TASK_DEF(Task_ConfigSave,   CONFIG_SAVE_PERIOD_MS,    4), /* 事件触发，运动中顺延 */
#if TASK_FIREWATER_ENABLE
    TASK_DEF(Task_Print,        TASK_FIREWATER_PERIOD_MS, 5),
#endif
};

void A_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
    A_Config_BindSaveEvent(C_Task_EventFlag(Task_ConfigSave));
}
