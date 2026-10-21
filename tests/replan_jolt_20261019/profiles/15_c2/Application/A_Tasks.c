/* A_Tasks.c —— 应用任务入口与注册表。
 *   优先级  周期   任务          内容
 *     0     1ms   ServoControl  实测位置 -> 观测器/轨迹 -> 控制器 -> H桥
 *     1     1ms   Uart          收帧、执行、回复、升级服务
 *     2     10ms  Protect       电流/电压/温度/堵转保护与功率限制
 *     3     20ms  State         LED状态/故障指示与喂狗
 *     4     事件  ConfigSave    参数变脏时写Flash
 *     5     20ms  Print         FireWater运行诊断(非阻塞串口入队)
 * 每个任务在本文件保留一个明确入口，业务逻辑放在对应模块。
 */
#include "A_Tasks.h"
#include "A_Servo.h"
#include "A_UartCmd.h"
#include "A_Protect.h"
#include "A_Config.h"
#include "C_Task_Scheduler.h"
#include "D_iwdg.h"
#include "D_ws2812.h"
#include "D_uart.h"
#include "iap.h"
#include <stdio.h>

/* 50Hz刷新，每帧色阶前进5：红->黄->绿->青->蓝->紫->红，约6.12秒一轮。
 * 单帧沿用驱动约32us发送+60us锁存，不增加额外延时。 */
#define TASK_STATE_PERIOD_MS  20U
#define STATE_LED_STEP        5U
#define STATE_LED_CYCLE       (6U * 255U)

typedef enum {
    STATE_LED_RAINBOW = 0,
    STATE_LED_OVERTEMP,
    STATE_LED_OVERCURRENT,
    STATE_LED_POWER_FAULT,
    STATE_LED_DEVICE_FAULT,
    STATE_LED_STANDBY
} StateLedMode_t;

/* RGB + 半个闪烁周期的20ms拍数：0=常亮，25=慢闪(500ms亮/灭)。
 * 表项顺序对应 StateLedMode_t；正常运行的RGB由渐变算法生成。 */
static const uint8_t s_state_led_patterns[][4] = {
    {  0U,   0U,   0U,  0U}, /* 正常：彩虹渐变 */
    {255U,  80U,   0U,  0U}, /* 过温：橙常亮 */
    {255U,   0U, 255U,  0U}, /* 过流：紫常亮 */
    {255U,   0U,   0U, 25U}, /* 电源异常(过压/欠压/电压无效)：红慢闪 */
    {  0U, 255U,   0U, 25U}, /* 设备异常(位置反馈/未分类故障)：绿慢闪 */
    {  0U,   0U, 255U,  0U}  /* 暂停或卸力：蓝常亮 */
};

/* 调试开关：设0恢复无诊断输出；周期单位ms，默认50Hz。可用编译宏覆盖。 */
#ifndef TASK_FIREWATER_ENABLE
#define TASK_FIREWATER_ENABLE    0
#endif
#ifndef TASK_FIREWATER_PERIOD_MS
#define TASK_FIREWATER_PERIOD_MS  20U
#endif
#if TASK_FIREWATER_ENABLE && (TASK_FIREWATER_PERIOD_MS < 1 || TASK_FIREWATER_PERIOD_MS > 65535)
#error "TASK_FIREWATER_PERIOD_MS must be in [1, 65535]"
#endif

/* ---- 任务入口 ---- */

static void Task_ServoControl(void)
{
    A_Servo_Control(); /* 1ms控制拍 */
}

static void Task_Uart(void)
{
    A_Uart_Process();
    IAP_Service();
}

static void Task_Protect(void)
{
    A_Protect_Task(); /* 10ms基拍，内部再分频 */
}

/* 仅读现有状态，不重读传感器、不清故障、不干预运动。
 * 面向用户保留六种灯效，故障优先：过温 > 过流 > 电源异常 > 设备异常。
 * 堵转后保持和自动脱困不单独设灯色，具体诊断仍由现有查询接口提供。 */
static StateLedMode_t StateLed_Select(void)
{
    uint16_t fault = A_Protect_Fault();
    uint8_t state = A_Servo_GetWorkState();

    if (fault & PROT_FAULT_OVERTEMP)     return STATE_LED_OVERTEMP;
    if (fault & PROT_FAULT_OVERCUR)      return STATE_LED_OVERCURRENT;
    if (fault & (PROT_FAULT_OVERVOLT | PROT_FAULT_UNDERVOLT | PROT_FAULT_VOLT_INVALID))
        return STATE_LED_POWER_FAULT;

    switch (state)
    {
    case SERVO_WORK_IDLE:
    case SERVO_WORK_MOVING:
    case SERVO_WORK_RECOVERING:   return STATE_LED_RAINBOW;
    case SERVO_WORK_PAUSED:
    case SERVO_WORK_RELEASE_LOW:
    case SERVO_WORK_RELEASE_HIGH: return STATE_LED_STANDBY;
    default:                     return STATE_LED_DEVICE_FAULT;
    }
}

static void StateLed_Rainbow(uint16_t phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t level = (uint8_t)(phase % 255U);

    /* 六段线性插值，全整数运算；首尾衔接，不插入熄灭帧。 */
    switch (phase / 255U)
    {
    case 0U: *r = 255U; *g = level; break;
    case 1U: *r = (uint8_t)(255U - level); *g = 255U; break;
    case 2U: *g = 255U; *b = level; break;
    case 3U: *g = (uint8_t)(255U - level); *b = 255U; break;
    case 4U: *r = level; *b = 255U; break;
    default: *r = 255U; *b = (uint8_t)(255U - level); break;
    }
}

static void Task_State(void)
{
    static uint16_t phase;
    static uint8_t previous_mode = 0xFFU;
    static uint8_t blink_tick;
    StateLedMode_t mode = StateLed_Select();
    uint8_t r = s_state_led_patterns[mode][0];
    uint8_t g = s_state_led_patterns[mode][1];
    uint8_t b = s_state_led_patterns[mode][2];
    uint8_t blink_half = s_state_led_patterns[mode][3];

    if ((uint8_t)mode != previous_mode)
    {
        previous_mode = (uint8_t)mode;
        blink_tick = 0U; /* 切换后从亮相开始，不等待旧模式的闪烁周期结束。 */
    }
    if (mode == STATE_LED_RAINBOW)
    {
        StateLed_Rainbow(phase, &r, &g, &b);
        phase = (uint16_t)(phase + STATE_LED_STEP);
        if (phase >= STATE_LED_CYCLE) phase = (uint16_t)(phase - STATE_LED_CYCLE);
    }
    else if (blink_half != 0U)
    {
        if (blink_tick >= blink_half) r = g = b = 0U;
        if (++blink_tick >= 2U * blink_half) blink_tick = 0U;
    }
    D_WS2812_Send(r, g, b);
    D_IWDG_Feed(); /* 主循环卡住时让看门狗复位。 */
}

static void Task_ConfigSave(void)
{
    A_Config_SaveTask(); /* 参数变脏时异步写Flash */
}

#if TASK_FIREWATER_ENABLE
/* VOFA+ FireWater: servo:ch0,ch1,...,ch17\r\n
 * 通道顺序(整数，避免浮点printf)：
 *  0 目标位置(cdeg)       1 轨迹参考位置(cdeg)  2 实测位置(cdeg)
 *  3 参考速度(cdeg/s)     4 观测器速度(cdeg/s)  5 实际PWM(物理方向，±2400)
 *  6 前馈(mV)            7 反馈(mV)            8 积分(mV)
 *  9 起转补偿(mV)       10 控制输出(mV)       11 扰动估计(mV)
 * 12 输出饱和(0/1)      13 控制器状态(0运动 1收尾 2保持 3没闭环)  14 有效标志(flags)
 * 15 保护故障位掩码     16 绕组电流(mA，与PWM同号=在出力；flags bit3=0 时测不到)
 * 17 电位器ADC原始均值(Q4计数，未换算)；卸力时 2/17 照常刷新，可手转扫描
 * 位置/速度以角度增大为正；PWM、电流以硬件方向为正，比较符号时注意MOTOR_SIGN。
 * bit0=闭环本拍更新，bit1=最近位置采样有效，bit2=观测器粘住预测，bit3=电流本块测得到。
 * USART1/PC0使用配置波特率(默认115200，8N1)，仅串口输入模式可输出；
 * PWM输入时驱动已释放共线PC0，本任务不会重新接管。半双工发送期间收不到命令，
 * 高频命令调试可增大周期或关闭打印。禁止直接printf，以免阻塞1ms控制任务。
 */
static void Task_Print(void)
{
    ServoDebugTelemetry_t data;
    char frame[RING_BUF_SIZE];
    int len;

    if (IAP_UpgradePending() || !D_UART_Tx_Idle() || !A_Uart_DebugReady()) return;
    A_Servo_GetDebugTelemetry(&data);
    if (data.input_source != SERVO_INPUT_TX) return; /* PWM输入共线，不能接管PC0 */
    len = snprintf(frame, sizeof(frame),
        "servo:%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%ld,%u,%u,%u,%u,%d,%u\r\n",
        (long)data.target_cdeg, (long)data.ref_cdeg, (long)data.actual_cdeg,
        (long)data.ref_cdps, (long)data.actual_cdps, (int)data.pwm,
        (int)data.ff_mv, (int)data.fb_mv, (int)data.i_mv, (int)data.stick_mv,
        (int)data.out_mv, (long)data.dist_mv, (unsigned)data.saturated,
        (unsigned)data.state, (unsigned)data.flags, (unsigned)A_Protect_Fault(), (int)data.cur_ma,
        (unsigned)data.pot_q4);
    /* 环形队列最多容纳127字节；超长/队列不足丢整帧，不能发截断的CSV。 */
    if (len > 0 && (unsigned)len < sizeof(frame))
        (void)D_UART1_Tx_Write((const uint8_t *)frame, (uint8_t)len);
}
#endif

/*                           入口函数        周期ms 优先级(0最高) */
static Task_t s_tasks[] = {
    TASK_DEF(Task_ServoControl, 1,    0),
    TASK_DEF(Task_Uart,         1,    1),
    TASK_DEF(Task_Protect,      10,   2),
    TASK_DEF(Task_State, TASK_STATE_PERIOD_MS, 3),
    TASK_DEF(Task_ConfigSave,   0,    4), /* 参数变脏时触发。 */
#if TASK_FIREWATER_ENABLE
    TASK_DEF(Task_Print, TASK_FIREWATER_PERIOD_MS, 5),
#endif
};

void A_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
    A_Config_BindSaveEvent(C_Task_EventFlag(Task_ConfigSave));
}
