/* A_StatusLed.c —— 状态指示灯。
 * 仅读现有状态，不重读传感器、不清故障、不干预运动。
 * 面向用户保留六种灯效，故障优先：过温 > 过流 > 电源异常 > 设备异常。
 * 堵转后保持和自动脱困不单独设灯色，具体诊断仍由现有查询接口提供。
 */
#include "A_StatusLed.h"
#include "A_Servo.h"
#include "A_Protect.h"
#include "D_ws2812.h"
#include <stdint.h>

/* 50Hz刷新，每帧色阶前进5：红->黄->绿->青->蓝->紫->红，约6.12秒一轮。
 * 单帧沿用驱动约32us发送+60us锁存，不增加额外延时。 */
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

void A_StatusLed_Task(void)
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
}
