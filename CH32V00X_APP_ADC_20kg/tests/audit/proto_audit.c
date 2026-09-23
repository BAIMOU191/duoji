/*
 * proto_audit.c —— 协议层审计：组帧、解码、分发、回复组帧
 *
 * 这里把 A_UartCmd.c 单独拎出来测，下面的舵机/保护/传感器全换成"只记录被调了
 * 什么"的探针。测的是协议契约本身：
 *     哪一串字节解成哪条指令、带什么参数
 *     哪条指令该回复、回复长什么样
 *     改名之后 SMI/SMX 与 AMI/AMX 有没有接反
 * 用真舵机跑这些反而看不清——目标位置对不对和"指令有没有被正确分发"是两回事。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "A_Config.h"
#include "A_Servo.h"
#include "A_Protect.h"
#include "A_Sensor.h"

Config_t g_config;

static int failures;
#define CHECK(test) do { if (!(test)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); failures++; } } while (0)

/* ---- 探针：记录协议层到底下发了什么 ---- */
static struct {
    int      submit, submit_turns, pause, resume, stop, release, restore;
    int      cal_mid, cal_zero, travel_min, travel_max, pulse_min, pulse_max;
    int      save_start, apply_cfg, set_mode, set_torque, dirty;
    uint16_t pwm, value, mode, torque;
    uint8_t  turns, release_high;
} spy;

/* 这些探针要能模拟"操作失败"，因为失败路径是静默的——回复有没有被抑制也要测 */
static uint8_t fail_cal, fail_travel, fail_pulse, fail_save, fail_mode, fail_torque;

void     A_Config_MarkDirty(void) { spy.dirty++; }
void     A_Config_Default(uint8_t keep_id) { (void)keep_id; }
uint32_t A_Config_Baudrate(void) { return 115200U; }

void A_Servo_Submit(uint16_t pwm, uint16_t value) {
    spy.submit++; spy.pwm = pwm; spy.value = value;
}
void A_Servo_SubmitTurns(uint16_t pwm, uint8_t turns, uint16_t turn_ms) {
    spy.submit_turns++; spy.pwm = pwm; spy.turns = turns; spy.value = turn_ms;
}
void A_Servo_Pause(void)   { spy.pause++; }
void A_Servo_Resume(void)  { spy.resume++; }
void A_Servo_Stop(void)    { spy.stop++; }
void A_Servo_Release(uint8_t high) { spy.release++; spy.release_high = high; }
void A_Servo_RestoreTorque(void)   { spy.restore++; }
void A_Servo_ApplyConfig(void)     { spy.apply_cfg++; }
void A_Servo_ApplyTrajConfig(void) { spy.apply_cfg++; }
uint8_t  A_Servo_SetMode(uint8_t m) { spy.set_mode++; spy.mode = m; return !fail_mode; }
uint16_t A_Servo_GetPositionPwm(void) { return 1234U; }
uint8_t  A_Servo_CalibrateMid(void)  { spy.cal_mid++;  return !fail_cal; }
uint8_t  A_Servo_CalibrateZero(void) { spy.cal_zero++; return !fail_cal; }
uint8_t  A_Servo_SetTravelEnd(uint8_t is_min) {
    if (is_min) spy.travel_min++; else spy.travel_max++; return !fail_travel;
}
uint8_t  A_Servo_SetPulseLimit(uint8_t is_min) {
    if (is_min) spy.pulse_min++; else spy.pulse_max++; return !fail_pulse;
}
uint8_t  A_Servo_SaveStartup(void) { spy.save_start++; return !fail_save; }

uint8_t  A_Protect_SetTorque(uint8_t p) { spy.set_torque++; spy.torque = p; return !fail_torque; }
uint8_t  A_Protect_GetTorque(void) { return 50U; }
static uint16_t stub_current_ma = 103U;
uint16_t A_Protect_GetCurrent(void) { return stub_current_ma; }

static uint16_t stub_voltage_cv = 700U; /* 厘伏，7.00V */
int32_t  A_Encoder_Read(void) { return 0; }
uint16_t A_Voltage_Read(void) { return stub_voltage_cv; }
int16_t  A_Temperature_Read(void) { return 253; }
uint8_t  A_Current_Read(uint16_t *ma) { (void)ma; return 0; }

void D_UART_SetBaud_Deferred(uint32_t b) { (void)b; }
void D_UART_Service(void) {}
void IAP_Rx_Deal(uint8_t b) { (void)b; }

/* 回复捕获：整帧攒在这里，用例直接跟字符串比 */
static char reply[128];
static int  reply_len;
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len) {
    if (reply_len + len < (int)sizeof(reply) - 1) {
        memcpy(reply + reply_len, data, len);
        reply_len += len;
        reply[reply_len] = '\0';
    }
    return 1U;
}
/* 接收队列：用例把一帧灌进来，A_Uart_Process 自己排空 */
static const char *rx_cursor;
static const uint8_t *rx_bytes;
static uint8_t rx_left;
uint8_t D_UART1_Rx_Get(uint8_t *data) {
    if (rx_left) { *data = *rx_bytes++; rx_left--; return 1U; }
    if (rx_cursor == 0 || *rx_cursor == '\0') return 0U;
    *data = (uint8_t)*rx_cursor++;
    return 1U;
}

#include "A_UartCmd.c"

/* 送一帧进协议层，返回回复(没有回复则是空串) */
static const char *send(const char *frame) {
    memset(&spy, 0, sizeof(spy));
    reply_len = 0; reply[0] = '\0';
    rx_cursor = frame;
    A_Uart_Process();
    return reply;
}

static void send_bin(uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t len, uint8_t bad_crc) {
    static uint8_t frame[BIN_FRAME_MAX];
    uint8_t i;

    memset(&spy, 0, sizeof(spy));
    reply_len = 0; reply[0] = '\0'; rx_cursor = 0;
    frame[0] = BIN_HEAD1; frame[1] = BIN_HEAD2; frame[2] = id;
    frame[3] = cmd; frame[4] = len;
    for (i = 0U; i < len; i++) frame[5U + i] = data[i];
    frame[5U + len] = (uint8_t)(Bin_Crc8(frame + 2U, (uint8_t)(3U + len)) ^ bad_crc);
    frame[6U + len] = BIN_TAIL1; frame[7U + len] = BIN_TAIL2;
    rx_bytes = frame; rx_left = (uint8_t)(8U + len);
    A_Uart_Process();
}

/* ==================== 运动指令 ==================== */

static void check_move(void) {
    CHECK(!strcmp(send("#000P1500T1000!"), ""));      /* 运动指令不回复 */
    CHECK(spy.submit == 1 && spy.pwm == 1500 && spy.value == 1000);
    CHECK(spy.submit_turns == 0);

    /* 越界脉宽在解码层就该被拒，连 A_Servo_Submit 都不该调到 */
    send("#000P2600T1000!"); CHECK(spy.submit == 0);
    send("#000P0400T1000!"); CHECK(spy.submit == 0);
    send("#000P9999T1000!"); CHECK(spy.submit == 0);
    /* 边界值本身合法 */
    send("#000P0500T0000!"); CHECK(spy.submit == 1 && spy.pwm == 500);
    send("#000P2500T0000!"); CHECK(spy.submit == 1 && spy.pwm == 2500);
}

static void check_move_turns(void) {
    CHECK(!strcmp(send("#000P1500N3T1000!"), ""));
    CHECK(spy.submit_turns == 1);
    CHECK(spy.pwm == 1500 && spy.turns == 3 && spy.value == 1000);
    CHECK(spy.submit == 0);                           /* 不能同时走普通路径 */

    send("#000P0500N0T0001!");
    CHECK(spy.submit_turns == 1 && spy.pwm == 500 && spy.turns == 0 && spy.value == 1);
    send("#000P2500N9T9999!");
    CHECK(spy.submit_turns == 1 && spy.turns == 9 && spy.value == 9999);

    /* 格式不对的一律不认，绝不能退化成普通运动指令 */
    send("#000P1500N3T100!");   CHECK(spy.submit_turns == 0 && spy.submit == 0);
    send("#000P1500NXT1000!");  CHECK(spy.submit_turns == 0 && spy.submit == 0);
    send("#000P1500M3T1000!");  CHECK(spy.submit_turns == 0 && spy.submit == 0);
    send("#000P2600N3T1000!");  CHECK(spy.submit_turns == 0 && spy.submit == 0);
    send("#000P1500N3X1000!");  CHECK(spy.submit_turns == 0 && spy.submit == 0);
}

/* ==================== 改名之后的四条标定指令 ==================== */

static void check_renamed_commands(void) {
    /* AMI/AMX = 原来的 SMI/SMX：收行程 */
    CHECK(!strcmp(send("#000PAMI!"), "#000POK!")); CHECK(spy.travel_min == 1);
    CHECK(!strcmp(send("#000PAMX!"), "#000POK!")); CHECK(spy.travel_max == 1);
    CHECK(spy.pulse_min == 0 && spy.pulse_max == 0);

    /* SMI/SMX = 新功能：设脉冲边界 */
    CHECK(!strcmp(send("#000PSMI!"), "#000POK!")); CHECK(spy.pulse_min == 1);
    CHECK(!strcmp(send("#000PSMX!"), "#000POK!")); CHECK(spy.pulse_max == 1);
    CHECK(spy.travel_min == 0 && spy.travel_max == 0);

    /* 两族指令绝不能互相串台 */
    send("#000PAMI!"); CHECK(spy.pulse_min == 0);
    send("#000PSMX!"); CHECK(spy.travel_max == 0);

    /* 失败路径静默：不回 OK，也不回别的 */
    fail_travel = 1; CHECK(!strcmp(send("#000PAMI!"), "")); CHECK(spy.travel_min == 1);
    fail_travel = 0;
    fail_pulse = 1;  CHECK(!strcmp(send("#000PSMI!"), "")); CHECK(spy.pulse_min == 1);
    fail_pulse = 0;
}

static void check_calibration_commands(void) {
    CHECK(!strcmp(send("#000PSCK!"), "#000POK!"));
    CHECK(spy.cal_mid == 1 && spy.cal_zero == 0);
    CHECK(!strcmp(send("#000PSCZ!"), "#000POK!"));
    CHECK(spy.cal_zero == 1 && spy.cal_mid == 0);

    fail_cal = 1;
    CHECK(!strcmp(send("#000PSCK!"), "")); /* 失败静默，与原有 SCK 一致 */
    CHECK(!strcmp(send("#000PSCZ!"), ""));
    fail_cal = 0;

    /* SCZ 不能被别的 SC 前缀指令吃掉，也不能反过来吃掉别人 */
    send("#000PSCD!"); CHECK(spy.cal_zero == 0 && spy.cal_mid == 0);
    CHECK(!strcmp(send("#000PCSD!"), "#000POK!")); CHECK(spy.save_start == 1);
}

/* ==================== 新查询指令 ==================== */

static void check_power_query(void) {
    stub_current_ma = 103U; stub_voltage_cv = 700U;
    CHECK(!strcmp(send("#000PRIV!"), "#000PI0103V7.0!"));

    stub_current_ma = 0U;    stub_voltage_cv = 0U;
    CHECK(!strcmp(send("#000PRIV!"), "#000PI0000V0.0!"));

    stub_current_ma = 9999U; stub_voltage_cv = 1265U;   /* 12.65V -> 12.7V */
    CHECK(!strcmp(send("#000PRIV!"), "#000PI9999V12.7!"));

    /* 超过4位的电流不截断位数，宁可帧变长也不要报一个错的数 */
    stub_current_ma = 12345U; stub_voltage_cv = 740U;
    CHECK(!strcmp(send("#000PRIV!"), "#000PI12345V7.4!"));

    /* RIV 不能和 RTV/RAD 串台 */
    stub_current_ma = 103U; stub_voltage_cv = 700U;
    CHECK(!strcmp(send("#000PRAD!"), "#000P1234!"));
    CHECK(strncmp(send("#000PRTV!"), "#000PDP", 7) == 0);
}

/* VER 回复 Servo-V主.次.修订；期望值由版本宏现拼，发版只改版本号不用改这里 */
static void check_version_query(void) {
    char expect[32];

    snprintf(expect, sizeof(expect), "#000PServo-V%u.%u.%u!",
             SERVO_VERSION_MAJOR, SERVO_VERSION_MINOR, SERVO_VERSION_PATCH);
    CHECK(!strcmp(send("#000PVER!"), expect));
}

/* ==================== 组帧与寻址 ==================== */

static void check_framing(void) {
    /* 广播地址照常执行 */
    send("#255P1500T1000!"); CHECK(spy.submit == 1);
    /* 别人的ID一概不理 */
    send("#001P1500T1000!"); CHECK(spy.submit == 0);
    /* 帧头重同步：前一帧被截断，下一帧照样能收 */
    send("#000P15#000P1500T1000!"); CHECK(spy.submit == 1 && spy.pwm == 1500);
    /* 缺帧尾、缺帧头、超长一律丢弃 */
    send("#000P1500T1000");  CHECK(spy.submit == 0);
    send("000P1500T1000!");  CHECK(spy.submit == 0);
    send("#000X1500T1000!"); CHECK(spy.submit == 0);
    send("#000P1500T1000000000000000000000000000000!"); CHECK(spy.submit == 0);
    /* 连着两帧都要执行 */
    memset(&spy, 0, sizeof(spy)); reply_len = 0; reply[0] = '\0';
    rx_cursor = "#000P1500T1000!#000P2000T2000!"; A_Uart_Process();
    CHECK(spy.submit == 2 && spy.pwm == 2000);
}

static void check_binary_protocol(void) {
    uint8_t set_vmax[5] = { 1U, 200U, 0U, 0U, 0U };
    uint8_t bad_vmax[5] = { 1U, 0U, 0U, 0U, 0U };
    uint8_t move[4] = { 0xDCU, 0x05U, 0xE8U, 0x03U }; /* 1500us, 1000ms */

    g_config.traj_vmax_dps = TUNE_VMAX_DPS;
    g_config.traj_amax_dps2 = TUNE_AMAX_DPS2;
    g_config.traj_dmax_dps2 = TUNE_DMAX_DPS2;
    g_config.traj_jmax_dps3 = TUNE_JMAX_DPS3;
    g_config.traj_move_min_ms = TUNE_MOVE_MIN_MS;

    send_bin(0U, 0x01U, 0, 0U, 0U);
    CHECK(reply_len == 12 && (uint8_t)reply[0] == 0xAA && (uint8_t)reply[1] == 0x66);
    CHECK((uint8_t)reply[3] == 0x81 && (uint8_t)reply[4] == 4U && (uint8_t)reply[5] == 0U);
    CHECK((uint8_t)reply[6] == SERVO_VERSION_MAJOR);
    CHECK((uint8_t)reply[9] == Bin_Crc8((const uint8_t *)reply + 2U, 7U));

    send_bin(0U, 0x10U, move, sizeof(move), 0U);
    CHECK(spy.submit == 1 && spy.pwm == 1500U && spy.value == 1000U);
    CHECK(reply_len == 9 && (uint8_t)reply[3] == 0x90 && (uint8_t)reply[5] == 0U);

    send_bin(0U, 0x51U, set_vmax, sizeof(set_vmax), 0U);
    CHECK(g_config.traj_vmax_dps == 200U && spy.apply_cfg == 1 && spy.dirty == 1);
    CHECK(reply_len == 9 && (uint8_t)reply[3] == 0xD1 && (uint8_t)reply[5] == 0U);

    send_bin(0U, 0x51U, bad_vmax, sizeof(bad_vmax), 0U);
    CHECK(g_config.traj_vmax_dps == 200U && spy.apply_cfg == 0 && spy.dirty == 0);
    CHECK(reply_len == 9 && (uint8_t)reply[5] == BIN_ST_VALUE);

    send_bin(0U, 0x52U, 0, 0U, 0U);
    CHECK(reply_len == 29 && (uint8_t)reply[4] == 21U && (uint8_t)reply[5] == 0U);
    CHECK(Bin_Get32((const uint8_t *)reply + 6U) == 200U);

    send_bin(255U, 0x10U, move, sizeof(move), 0U);
    CHECK(spy.submit == 1 && reply_len == 0); /* 广播执行但不应答 */

    send_bin(0U, 0x10U, move, sizeof(move), 1U);
    CHECK(spy.submit == 0 && reply_len == 0); /* CRC错误静默丢弃 */

    send_bin(1U, 0x10U, move, sizeof(move), 0U);
    CHECK(spy.submit == 0 && reply_len == 0); /* 非本机地址 */
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    g_config.servo_id = 0U;
    g_config.servo_mode = SERVO_MODE_270_CW;
    g_config.baud_code = 5U;

    check_move();
    check_move_turns();
    check_renamed_commands();
    check_calibration_commands();
    check_power_query();
    check_version_query();
    check_framing();
    check_binary_protocol();

    printf("proto failures=%d\n", failures);
    return failures ? 1 : 0;
}
