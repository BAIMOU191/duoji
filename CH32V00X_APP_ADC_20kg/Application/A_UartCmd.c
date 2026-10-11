/* A_UartCmd.c 串口协议层：一根线上三种帧按帧头独占分流——
 *   明文   #IIIP<指令>!            见 Text_*
 *   二进制 AA 66 ID CMD LEN DATA CRC8 66 AA   见 Bin_*
 *   升级   AA 55 ...               交给 iap.c */

#include "A_UartCmd.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include "A_Protect.h"
#include "D_uart.h"
#include "iap.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define TEXT_FRAME_MAX 32U
#define TEXT_REPLY_MAX 48U

#define BIN_HEAD1     0xAAU
#define BIN_HEAD2     0x66U
#define BIN_TAIL1     0x66U
#define BIN_TAIL2     0xAAU
#define BIN_DATA_MAX  32U
#define BIN_FRAME_MAX (8U + BIN_DATA_MAX)
#define BIN_BROADCAST 255U
#define BIN_TELEMETRY_BYTES 16U

/* 应答状态码 */
#define BIN_ST_OK 0U
#define BIN_ST_CMD 1U
#define BIN_ST_LENGTH 2U
#define BIN_ST_VALUE 3U
#define BIN_ST_STATE 4U
#define BIN_ST_FAILED 5U
#define BIN_ST_READONLY 6U
#define BIN_ST_UNSUP 7U

/* 二进制命令 */
#define BIN_CMD_STATE 0x01U
#define BIN_CMD_READ 0x02U
#define BIN_CMD_WRITE 0x03U
#define BIN_CMD_RESERVED_04 0x04U
#define BIN_CMD_RESERVED_05 0x05U
#define BIN_CMD_RESERVED_06 0x06U
#define BIN_CMD_RESET_KEEP 0x07U
#define BIN_CMD_RESET_ALL 0x08U
#define BIN_CMD_REBOOT 0x09U

/* 参数ID：00~0F硬件，10~1F固件，20~7F配置(落盘)，80~BF运行态(SRAM)，C0以上只读状态 */
#define PARAM_HW_TYPE 0x00U
#define PARAM_HW_VERSION 0x01U
#define PARAM_HW_VOLTAGE 0x02U
#define PARAM_HW_TORQUE 0x03U
#define PARAM_HW_VMAX 0x04U
#define PARAM_HW_AMAX 0x05U
#define PARAM_HW_DMAX 0x06U
#define PARAM_HW_JMAX 0x07U
#define PARAM_HW_MOVE_MIN 0x08U
#define PARAM_FW_VERSION 0x10U
#define PARAM_SERVO_ID 0x20U
#define PARAM_BAUD_CODE 0x21U
#define PARAM_SERVO_MODE 0x22U
#define PARAM_BOOT_MODE 0x23U
#define PARAM_TORQUE_LIMIT 0x24U
#define PARAM_STARTUP_PWM 0x25U
#define PARAM_TRAJ_VMAX 0x30U
#define PARAM_TRAJ_AMAX 0x31U
#define PARAM_TRAJ_DMAX 0x32U
#define PARAM_TRAJ_JMAX 0x33U
#define PARAM_MOVE_MIN 0x34U
#define PARAM_HOLD_STIFF 0x35U
#define PARAM_PROT_CURRENT_LIMIT 0x40U
#define PARAM_PROT_CURRENT_TIME 0x41U
#define PARAM_PROT_TEMP_LIMIT 0x42U
#define PARAM_PROT_TEMP_TIME 0x43U
#define PARAM_PROT_STALL_THRESHOLD 0x44U
#define PARAM_PROT_STALL_TRIP 0x45U
#define PARAM_PROT_ENABLE 0x46U
#define PARAM_VOLTAGE_MIN 0x47U
#define PARAM_VOLTAGE_MAX 0x48U
#define PARAM_MIN_START_TORQUE 0x49U
#define PARAM_MOTION_REQUEST 0x80U
#define PARAM_MOTION_ACTION 0x81U
#define PARAM_TORQUE_MODE 0x82U
#define PARAM_DAMPING 0x83U
#define PARAM_RT_TORQUE_LIMIT 0x84U
#define PARAM_RT_TRAJ_VMAX 0x85U
#define PARAM_RT_TRAJ_AMAX 0x86U
#define PARAM_RT_TRAJ_DMAX 0x87U
#define PARAM_RT_TRAJ_JMAX 0x88U
#define PARAM_RT_MOVE_MIN 0x89U
#define PARAM_RT_HOLD_STIFF 0x8AU
#define PARAM_CAL_ACTION 0x8BU
#define PARAM_SYSTEM_ACTION 0x8CU
#define PARAM_WORK_STATE 0xC0U
#define PARAM_FAULT 0xC1U
#define PARAM_POSITION_PWM 0xC2U
#define PARAM_TEMPERATURE 0xC3U
#define PARAM_VOLTAGE 0xC4U
#define PARAM_CURRENT 0xC5U
#define PARAM_STALLED 0xC6U
#define PARAM_TELEMETRY 0xC7U   /* 七项实时状态集合 */

#define Bin_Get16(p) ((uint16_t)((p)[0] | (uint16_t)(p)[1] << 8))
#define Bin_Get32(p) (((uint32_t)(p)[0] | (uint32_t)(p)[1] << 8 | (uint32_t)(p)[2] << 16 | (uint32_t)(p)[3] << 24))
#define Bin_Put16(p, v) ((void)Bin_PutN((p), (v), 2U))
#define Bin_Put32(p, v) ((void)Bin_PutN((p), (v), 4U))

enum { WIRE_IDLE = 0, WIRE_HEAD_AA, WIRE_TEXT, WIRE_BINARY, WIRE_IAP };

static uint8_t s_text[TEXT_FRAME_MAX], s_text_len;
static uint8_t s_reply[TEXT_REPLY_MAX], s_reply_len;
static uint8_t s_bin[BIN_FRAME_MAX], s_bin_len;
static uint8_t s_wire, s_wire_idle_ms;

/* ============================== 明文协议 ============================== */

/* 定长十进制数字串，非数字或超16位判非法 */
static bool Text_Digits(const uint8_t *p, uint8_t n, uint16_t *out)
{
    uint32_t v = 0U;

    if (n == 0U) return false;
    while (n--)
    {
        if (*p < '0' || *p > '9' || (v = v * 10U + (uint32_t)(*p++ - '0')) > 0xFFFFU) return false;
    }
    *out = (uint16_t)v;
    return true;
}

enum { T_VER, T_ID, T_ID_SET, T_ULK, T_ULM, T_ULR, T_MOD, T_MOD_SET, T_RAD, T_DPT, T_DCT, T_DST, T_BD, T_BD_SET,
       T_SP, T_SP_SET, T_AMI, T_AMX, T_SMI, T_SMX, T_SCK, T_SCZ, T_RIV, T_CSD, T_CSM, T_CSM_SET, T_CLE0, T_CLE, T_RTV };

/* 关键字 + 紧跟的参数位数；同一关键字靠总长度区分 */
static const struct { char key[5]; uint8_t digits; uint8_t code; } k_text[] = {
    {"VER", 0, T_VER}, {"ID", 0, T_ID}, {"ID", 3, T_ID_SET}, {"ULK", 0, T_ULK}, {"ULM", 0, T_ULM}, {"ULR", 0, T_ULR},
    {"MOD", 0, T_MOD}, {"MOD", 1, T_MOD_SET}, {"MOD", 2, T_MOD_SET}, {"RAD", 0, T_RAD}, {"DPT", 0, T_DPT},
    {"DCT", 0, T_DCT}, {"DST", 0, T_DST}, {"BD", 0, T_BD}, {"BD", 1, T_BD_SET}, {"SP", 0, T_SP}, {"SP", 5, T_SP_SET},
    {"AMI", 0, T_AMI}, {"AMX", 0, T_AMX}, {"SMI", 0, T_SMI}, {"SMX", 0, T_SMX}, {"SCK", 0, T_SCK}, {"SCZ", 0, T_SCZ},
    {"RIV", 0, T_RIV}, {"CSD", 0, T_CSD}, {"CSM", 0, T_CSM}, {"CSM", 1, T_CSM_SET}, {"CLE0", 0, T_CLE0},
    {"CLE", 0, T_CLE}, {"RTV", 0, T_RTV},
};

static void Reply_Char(uint8_t c) { if (s_reply_len < TEXT_REPLY_MAX) s_reply[s_reply_len++] = c; }
static void Reply_Text(const char *t) { while (*t) Reply_Char((uint8_t)*t++); }

/* 无符号十进制，不足 width 位左侧补零 */
static void Reply_UInt(uint32_t v, uint8_t width)
{
    uint8_t d[10], n = 0U;

    do { d[n++] = (uint8_t)('0' + v % 10U); v /= 10U; } while (v && n < sizeof(d));
    while (n < width) d[n++] = '0';
    while (n) Reply_Char(d[--n]);
}

/* 回复帧头：# + 本机ID三位 + P */
static void Reply_Begin(void)
{
    s_reply_len = 0U;
    Reply_Char('#');
    Reply_UInt(g_config.servo_id, 3U);
    Reply_Char('P');
}

static void Reply_Send(void)
{
    Reply_Char('!');
    (void)D_UART1_Tx_Write(s_reply, s_reply_len);   /* 队列放不下整帧丢弃 */
}

static void Reply_Value(const char *tag, uint32_t v, uint8_t width)
{
    Reply_Begin(); Reply_Text(tag); Reply_UInt(v, width); Reply_Send();
}

/* 电压回复 V+一位小数，厘伏四舍五入到0.1V */
static void Reply_Volt(void)
{
    uint16_t dv = (uint16_t)((A_Voltage_Read() + 5U) / 10U);
    Reply_Char('V'); Reply_UInt(dv / 10U, 0U); Reply_Char('.'); Reply_UInt(dv % 10U, 1U);
}

/* 执行一条明文指令。运动/停止指令和失败的设置指令按协议不回复 */
static void Text_Execute(uint8_t code, uint16_t arg)
{
    uint8_t ok = 1U;
    int16_t temp;

    switch (code)
    {
    case T_VER:
        Reply_Begin(); Reply_Text("Servo-V");
        Reply_UInt(SERVO_VERSION_MAJOR, 0U); Reply_Char('.');
        Reply_UInt(SERVO_VERSION_MINOR, 0U); Reply_Char('.');
        Reply_UInt(SERVO_VERSION_PATCH, 0U); Reply_Send();
        return;
    case T_ID:  Reply_Begin(); Reply_Send(); return;   /* 帧头本身就带ID */
    case T_MOD: Reply_Value("MOD", g_config.servo_mode, 0U); return;
    case T_CSM: Reply_Value("CSM", g_config.boot_mode, 0U); return;
    case T_BD:  Reply_Value("BAUD", A_Config_Baudrate(), 0U); return;
    case T_RAD: Reply_Value("", A_Servo_GetPositionPwm(), 4U); return;
    case T_SP:  Reply_Value("SP", A_Protect_GetTorque(), 5U); return;
    case T_DST: A_Servo_Stop(); return;
    case T_RIV:   /* I+4位毫安 + V+一位小数，例 #000PI0103V7.0! */
        Reply_Begin(); Reply_Char('I'); Reply_UInt(A_Protect_GetCurrent(), 4U); Reply_Volt(); Reply_Send();
        return;
    case T_RTV:
        temp = A_Temperature_Read();
        temp = (temp == (int16_t)-32768) ? 0 : temp / 10;
        Reply_Begin(); Reply_Text("DP"); Reply_UInt(A_Servo_GetPositionPwm(), 4U); Reply_Char('T');
        if (temp < 0) Reply_Char('-');
        Reply_UInt((uint32_t)((temp < 0) ? -temp : temp), 0U); Reply_Volt(); Reply_Send();
        return;
    case T_ID_SET:  g_config.servo_id = (uint8_t)arg; A_Config_MarkDirty(); break;   /* 用新ID回复 */
    case T_ULK: A_Servo_Release(0U); break;
    case T_ULM: (void)A_Servo_SetDamping(100U); A_Servo_Release(1U); break;
    case T_ULR: A_Servo_RestoreTorque(); break;
    case T_DPT: A_Servo_Pause(); break;
    case T_DCT: A_Servo_Resume(); break;
    case T_MOD_SET: ok = A_Servo_SetMode((uint8_t)arg); break;
    case T_SP_SET:  ok = A_Protect_SetTorque(arg); break;
    case T_SCK: ok = A_Servo_CalibrateMid(); break;
    case T_SCZ: ok = A_Servo_CalibrateZero(); break;
    case T_AMI: ok = A_Servo_SetTravelEnd(1U); break;
    case T_AMX: ok = A_Servo_SetTravelEnd(0U); break;
    case T_SMI: ok = A_Servo_SetPulseLimit(1U); break;
    case T_SMX: ok = A_Servo_SetPulseLimit(0U); break;
    case T_CSD: ok = A_Servo_SaveStartup(); break;
    case T_CSM_SET: g_config.boot_mode = (uint8_t)arg; A_Config_MarkDirty(); break;
    case T_BD_SET:  g_config.baud_code = (uint8_t)arg; A_Config_MarkDirty(); break;
    default:   /* T_CLE0 / T_CLE：恢复默认，只差保不保留ID */
        A_Config_Default((uint8_t)(code == T_CLE0));
        A_Servo_ApplyConfig();
        break;
    }
    if (!ok) return;
    Reply_Begin(); Reply_Text("OK"); Reply_Send();
    if (code == T_BD_SET || code == T_CLE0 || code == T_CLE)
        D_UART_SetBaud_Deferred(A_Config_Baudrate());   /* 先用旧波特率把OK发完再切 */
}

/* 校验帧结构与目标ID，解码执行。参数越界的设置指令整条不认 */
static void Text_Frame(const uint8_t *f, uint8_t len)
{
    const uint8_t *body = f + 5;
    uint8_t  n = (uint8_t)(len - 6U), i, k;
    uint16_t id, pwm, arg = 0U;

    if (len < 7U || f[0] != '#' || f[4] != 'P' || f[len - 1U] != '!' || !Text_Digits(f + 1, 3U, &id)
        || (id != g_config.servo_id && id != 255U)) return;   /* 255=广播 */
    if (n == 9U && body[4] == 'T' && Text_Digits(body, 4U, &pwm) && Text_Digits(body + 5, 4U, &arg))
    {   /* nnnnTnnnn：目标脉宽 + 时间(ms)。电位器不支持整圈的多圈指令 nnnnNnTnnnn */
        if (pwm >= SERVO_PWM_MIN && pwm <= SERVO_PWM_MAX) A_Servo_Submit(pwm, arg);
        return;
    }
    for (i = 0U; i < sizeof(k_text) / sizeof(k_text[0]); i++)
    {
        k = (uint8_t)strlen(k_text[i].key);
        if (n != k + k_text[i].digits || memcmp(body, k_text[i].key, k) != 0
            || (k_text[i].digits && !Text_Digits(body + k, k_text[i].digits, &arg))) continue;
        switch (k_text[i].code)
        {
        case T_ID_SET:  if (arg > 254U) return; break;
        case T_MOD_SET: if (!SERVO_MODE_IS_SUPPORTED(arg) || arg > SERVO_MODE_TIMED_CCW) return; break;
        case T_BD_SET:  if (arg < 1U || arg > 8U) return; break;
        case T_SP_SET:  if (arg > A_Config_TorqueMax()) return; break;   /* 5位数，0.01 kgf·cm */
        case T_CSM_SET: if (arg < SERVO_BOOT_GOTO_START || arg > SERVO_BOOT_RELEASE) return; break;
        default: break;
        }
        Text_Execute(k_text[i].code, arg);
        return;
    }
}

/* ============================== 二进制协议 ============================== */

/* 小端写入，返回字节数 */
static uint8_t Bin_PutN(uint8_t *out, uint32_t v, uint8_t size)
{
    uint8_t i;

    for (i = 0U; i < size; i++) out[i] = (uint8_t)(v >> (8U * i));
    return size;
}

/* CRC-8/ATM：多项式0x07，初值0 */
static uint8_t Bin_Crc8(const uint8_t *p, uint8_t len)
{
    uint8_t crc = 0U, b;

    while (len--)
        for (crc ^= *p++, b = 0U; b < 8U; b++) crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x07U : 0U));
    return crc;
}

/* 应答：DATA = 状态码 + data */
static void Bin_Reply(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len)
{
    uint8_t f[BIN_FRAME_MAX];

    f[0] = BIN_HEAD1; f[1] = BIN_HEAD2; f[2] = g_config.servo_id; f[3] = (uint8_t)(cmd | 0x80U);
    f[4] = (uint8_t)(len + 1U); f[5] = status;
    memcpy(f + 6, data, len);
    f[6U + len] = Bin_Crc8(f + 2, (uint8_t)(4U + len));
    f[7U + len] = BIN_TAIL1; f[8U + len] = BIN_TAIL2;
    (void)D_UART1_Tx_Write(f, (uint8_t)(9U + len));
}

_Static_assert(BIN_TELEMETRY_BYTES + 2U <= BIN_DATA_MAX, "集合状态超出应答数据容量");

/* 七项实时状态，顺序与宽度是协议 */
static void Bin_Telemetry(uint8_t *out)
{
    ServoTelemetry_t s;
    ProtectTelemetry_t p;

    A_Servo_GetTelemetry(&s);
    A_Protect_GetTelemetry(&p);
    Bin_Put16(out, s.target_pwm);
    Bin_Put16(out + 2, s.position_pwm);
    Bin_Put16(out + 4, A_Torque_Estimate(p.current_ma, p.current_valid, A_Config_TorqueMax()));
    Bin_Put32(out + 6, (uint32_t)s.speed_cdps);
    Bin_Put16(out + 10, p.current_ma);
    Bin_Put16(out + 12, (uint16_t)p.temperature_dc);
    Bin_Put16(out + 14, p.voltage_cv);
}

/* 普通参数表：取值范围、Flash/SRAM 两份存放、写后是否让轨迹参数生效，读写共用。
 * 有联动或要调用其它模块的参数在 Bin_ParamWrite 的 switch 里写，表里只用它们的读取 */
#define PF_NONE 0xFFFFU   /* 没有这一份存放 */
#define PF_TRAJ 0x01U     /* 写后让轨迹参数生效 */
#define PF_BAUD 0x02U     /* 写后切波特率 */
#define PF_TMAX 0x04U     /* 上限取本机硬件扭矩 */
#define PCFG(f) ((uint16_t)offsetof(Config_t, f))
#define PRT(f)  ((uint16_t)offsetof(RuntimeConfig_t, f))

static const struct {
    uint8_t  id, size, flags;
    uint16_t cfg, rt;     /* 在 g_config / g_runtime_config 里的偏移 */
    uint32_t min, max;
} k_params[] = {
    { PARAM_SERVO_ID,     1, 0,       PCFG(servo_id),           PF_NONE, 0U, 254U },
    { PARAM_BAUD_CODE,    1, PF_BAUD, PCFG(baud_code),          PF_NONE, 1U, 8U },
    { PARAM_SERVO_MODE,   1, 0,       PCFG(servo_mode),         PF_NONE, 0U, 0U },
    { PARAM_BOOT_MODE,    1, 0,       PCFG(boot_mode),          PF_NONE, SERVO_BOOT_GOTO_START, SERVO_BOOT_RELEASE },
    { PARAM_TORQUE_LIMIT, 2, 0,       PCFG(torque_limit_ckgcm), PF_NONE, 0U, 0U },
    { PARAM_STARTUP_PWM,  2, 0,       PCFG(startup_pwm),        PF_NONE, SERVO_PWM_MIN, SERVO_PWM_MAX },
    { PARAM_TRAJ_VMAX,    2, PF_TRAJ, PCFG(traj_vmax_dps),      PRT(traj_vmax_dps),    0U, PLANT_VMAX_HW_CDPS / 100 },
    { PARAM_TRAJ_AMAX,    4, PF_TRAJ, PCFG(traj_amax_dps2),     PRT(traj_amax_dps2),   0U, PLANT_AMAX_HW_DPS2 },
    { PARAM_TRAJ_DMAX,    4, PF_TRAJ, PCFG(traj_dmax_dps2),     PRT(traj_dmax_dps2),   0U, PLANT_DMAX_HW_DPS2 },
    { PARAM_TRAJ_JMAX,    4, PF_TRAJ, PCFG(traj_jmax_dps3),     PRT(traj_jmax_dps3),   0U, PLANT_JMAX_HW_DPS3 },
    { PARAM_MOVE_MIN,     2, PF_TRAJ, PCFG(traj_move_min_ms),   PRT(traj_move_min_ms), 0U, 200U },
    { PARAM_HOLD_STIFF,   1, 0,       PCFG(hold_stiff),         PRT(hold_stiff),       0U, CFG_HOLD_STIFF_MAX },
    { PARAM_PROT_CURRENT_LIMIT, 2, 0, PCFG(prot_current_limit_ma), PRT(prot_current_limit_ma),
      PROT_CURRENT_LIMIT_MIN_MA, PROT_CURRENT_LIMIT_MAX_MA },
    { PARAM_PROT_CURRENT_TIME,  2, 0, PCFG(prot_current_time_ms),  PRT(prot_current_time_ms),
      PROT_CURRENT_TIME_MIN_MS, PROT_CURRENT_TIME_MAX_MS },
    { PARAM_PROT_TEMP_LIMIT,    2, 0, PCFG(prot_temp_limit_dC),    PRT(prot_temp_limit_dC),
      PROT_TEMP_LIMIT_MIN_DC, PROT_TEMP_LIMIT_MAX_DC },
    { PARAM_PROT_TEMP_TIME,     2, 0, PCFG(prot_temp_time_ms),     PRT(prot_temp_time_ms),
      PROT_TEMP_TIME_MIN_MS, PROT_TEMP_TIME_MAX_MS },
    { PARAM_PROT_STALL_THRESHOLD, 2, 0, PCFG(prot_stall_act_cdeg), PF_NONE, 0U, 0U },
    { PARAM_PROT_STALL_TRIP,    2, 0, PCFG(prot_stall_trip),       PRT(prot_stall_trip),
      PROT_STALL_TRIP_MIN, PROT_STALL_TRIP_MAX },
    { PARAM_PROT_ENABLE,        1, 0, PCFG(protect_enable),        PRT(protect_enable), 0U, 255U },
    { PARAM_VOLTAGE_MIN,        1, 0, PCFG(voltage_min_dv),        PF_NONE, 0U, 0U },
    { PARAM_VOLTAGE_MAX,        1, 0, PCFG(voltage_max_dv),        PF_NONE, 0U, 0U },
    { PARAM_MIN_START_TORQUE,   2, PF_TMAX, PCFG(min_start_torque_ckgcm), PRT(min_start_torque_ckgcm), 0U, 0U },
    { PARAM_TORQUE_MODE,    1, 0,       PF_NONE, PRT(torque_mode),        0U, 0U },
    { PARAM_DAMPING,        1, 0,       PF_NONE, PRT(damping_percent),    0U, 0U },
    { PARAM_RT_TORQUE_LIMIT, 2, PF_TMAX, PF_NONE, PRT(torque_limit_ckgcm), 0U, 0U },
    { PARAM_RT_TRAJ_VMAX,   2, PF_TRAJ, PF_NONE, PRT(traj_vmax_dps),      0U, PLANT_VMAX_HW_CDPS / 100 },
    { PARAM_RT_TRAJ_AMAX,   4, PF_TRAJ, PF_NONE, PRT(traj_amax_dps2),     0U, PLANT_AMAX_HW_DPS2 },
    { PARAM_RT_TRAJ_DMAX,   4, PF_TRAJ, PF_NONE, PRT(traj_dmax_dps2),     0U, PLANT_DMAX_HW_DPS2 },
    { PARAM_RT_TRAJ_JMAX,   4, PF_TRAJ, PF_NONE, PRT(traj_jmax_dps3),     0U, PLANT_JMAX_HW_DPS3 },
    { PARAM_RT_MOVE_MIN,    2, PF_TRAJ, PF_NONE, PRT(traj_move_min_ms),   0U, 200U },
    { PARAM_RT_HOLD_STIFF,  1, 0,       PF_NONE, PRT(hold_stiff),         0U, CFG_HOLD_STIFF_MAX },
};

/* 在表里找参数，返回下标，-1=不在表里 */
static int8_t Param_Find(uint8_t id)
{
    uint8_t i;

    for (i = 0U; i < sizeof(k_params) / sizeof(k_params[0]); i++)
        if (k_params[i].id == id) return (int8_t)i;
    return -1;
}

/* 字段地址：优先 Flash 份；偏移都来自 offsetof，天然对齐 */
static uint8_t *Param_Addr(uint16_t cfg, uint16_t rt)
{
    return (cfg != PF_NONE) ? (uint8_t *)&g_config + cfg : (uint8_t *)&g_runtime_config + rt;
}

static uint32_t Param_Load(const uint8_t *p, uint8_t size)
{
    return (size == 1U) ? *p : (size == 2U) ? *(const uint16_t *)p : *(const uint32_t *)p;
}

static void Param_Store(uint8_t *p, uint8_t size, uint32_t v)
{
    if (size == 1U)      *p = (uint8_t)v;
    else if (size == 2U) *(uint16_t *)p = (uint16_t)v;
    else                 *(uint32_t *)p = v;
}

static uint8_t Bin_ParamRead(uint8_t id, uint8_t *out, uint8_t *out_len)
{
    IAP_HwInfo_t hw;
    int8_t  t = Param_Find(id);
    uint8_t n;

    out[0] = id;
    if (t >= 0)
        n = Bin_PutN(out + 1, Param_Load(Param_Addr(k_params[t].cfg, k_params[t].rt), k_params[t].size), k_params[t].size);
    else if (id <= PARAM_HW_MOVE_MIN)
    {
        if (!IAP_HwInfoRead(&hw)) return BIN_ST_FAILED;
        switch (id)
        {
        case PARAM_HW_TYPE:    n = Bin_PutN(out + 1, hw.servo_type, 1U);     break;
        case PARAM_HW_VERSION: n = Bin_PutN(out + 1, hw.hw_version, 1U);     break;
        case PARAM_HW_VOLTAGE: n = Bin_PutN(out + 1, hw.voltage, 1U);        break;
        case PARAM_HW_TORQUE:  n = Bin_PutN(out + 1, hw.torque, 1U);         break;
        case PARAM_HW_VMAX:    n = Bin_PutN(out + 1, hw.max_speed_dps, 2U);  break;
        case PARAM_HW_AMAX:    n = Bin_PutN(out + 1, hw.max_accel_dps2, 4U); break;
        case PARAM_HW_DMAX:    n = Bin_PutN(out + 1, hw.max_decel_dps2, 4U); break;
        case PARAM_HW_JMAX:    n = Bin_PutN(out + 1, hw.max_jerk_dps3, 4U);  break;
        default:               n = Bin_PutN(out + 1, hw.min_move_cdeg, 2U);  break;
        }
    }
    else switch (id)
    {
    case PARAM_FW_VERSION:
        out[1] = SERVO_VERSION_MAJOR; out[2] = SERVO_VERSION_MINOR; out[3] = SERVO_VERSION_PATCH; n = 3U; break;
    case PARAM_WORK_STATE:   n = Bin_PutN(out + 1, A_Servo_GetWorkState(), 1U);         break;
    case PARAM_FAULT:        n = Bin_PutN(out + 1, A_Protect_Fault(), 2U);              break;
    case PARAM_POSITION_PWM: n = Bin_PutN(out + 1, A_Servo_GetPositionPwm(), 2U);       break;
    case PARAM_TEMPERATURE:  n = Bin_PutN(out + 1, (uint16_t)A_Temperature_Read(), 2U); break;
    case PARAM_VOLTAGE:      n = Bin_PutN(out + 1, A_Voltage_Read(), 2U);               break;
    case PARAM_CURRENT:      n = Bin_PutN(out + 1, A_Protect_GetCurrent(), 2U);         break;
    case PARAM_STALLED:      n = Bin_PutN(out + 1, A_Protect_Stalled(), 1U);            break;
    case PARAM_TELEMETRY:    Bin_Telemetry(out + 1); n = BIN_TELEMETRY_BYTES;           break;
    default: return BIN_ST_VALUE;
    }
    *out_len = (uint8_t)(1U + n);
    return BIN_ST_OK;
}

/* 动作类参数：data[0] 选第几个动作(1起)，失败返回 BIN_ST_FAILED */
static uint8_t Bin_Action(uint8_t (*const *acts)(void), uint8_t n, uint8_t pick)
{
    if (pick < 1U || pick > n) return BIN_ST_VALUE;
    return acts[pick - 1U]() ? BIN_ST_OK : BIN_ST_FAILED;
}

static uint8_t Cal_TravelMin(void) { return A_Servo_SetTravelEnd(1U); }
static uint8_t Cal_TravelMax(void) { return A_Servo_SetTravelEnd(0U); }
static uint8_t Cal_PulseMin(void)  { return A_Servo_SetPulseLimit(1U); }
static uint8_t Cal_PulseMax(void)  { return A_Servo_SetPulseLimit(0U); }

static uint8_t Bin_ParamWrite(uint8_t id, const uint8_t *data, uint8_t len, uint8_t *baud_changed)
{
    static uint8_t (*const k_cal[])(void) = {
        A_Servo_CalibrateMid, A_Servo_CalibrateZero, Cal_TravelMin, Cal_TravelMax, Cal_PulseMin, Cal_PulseMax, A_Servo_SaveStartup
    };
    uint32_t value = (len == 1U) ? data[0] : (len == 2U) ? Bin_Get16(data) : (len == 4U) ? Bin_Get32(data) : 0U, max;
    int8_t   t;

    switch (id)   /* 有联动或要调用其它模块的参数 */
    {
    case PARAM_SERVO_MODE:   if (len != 1U) return BIN_ST_LENGTH; return A_Servo_SetMode(data[0]) ? BIN_ST_OK : BIN_ST_VALUE;
    case PARAM_TORQUE_LIMIT: if (len != 2U) return BIN_ST_LENGTH; return A_Protect_SetTorque((uint16_t)value) ? BIN_ST_OK : BIN_ST_VALUE;
    case PARAM_TORQUE_MODE:  if (len != 1U) return BIN_ST_LENGTH; return A_Servo_SetTorqueMode(data[0]) ? BIN_ST_OK : BIN_ST_STATE;
    case PARAM_DAMPING:      if (len != 1U) return BIN_ST_LENGTH; return A_Servo_SetDamping(data[0]) ? BIN_ST_OK : BIN_ST_VALUE;
    case PARAM_PROT_STALL_THRESHOLD:   /* 对外单一阈值，参考位移阈值固定为它的4倍 */
        if (len != 2U) return BIN_ST_LENGTH;
        if (value < PROT_STALL_THRESHOLD_MIN_CDEG || value > PROT_STALL_THRESHOLD_MAX_CDEG) return BIN_ST_VALUE;
        g_config.prot_stall_act_cdeg = g_runtime_config.prot_stall_act_cdeg = (uint16_t)value;
        g_config.prot_stall_ref_cdeg = g_runtime_config.prot_stall_ref_cdeg = (uint16_t)(value * 4U);
        A_Config_MarkDirty(); return BIN_ST_OK;
    case PARAM_VOLTAGE_MIN:
    case PARAM_VOLTAGE_MAX:   /* 上下限之间至少留回差 */
        if (len != 1U) return BIN_ST_LENGTH;
        if ((id == PARAM_VOLTAGE_MIN) ? (value < PROT_VOLTAGE_CFG_MIN_DV || value + PROT_VOLTAGE_HYST_DV > g_config.voltage_max_dv)
                                      : (value > PROT_VOLTAGE_CFG_MAX_DV || g_config.voltage_min_dv + PROT_VOLTAGE_HYST_DV > value))
            return BIN_ST_VALUE;
        if (id == PARAM_VOLTAGE_MIN) g_config.voltage_min_dv = g_runtime_config.voltage_min_dv = (uint8_t)value;
        else                         g_config.voltage_max_dv = g_runtime_config.voltage_max_dv = (uint8_t)value;
        A_Config_MarkDirty(); return BIN_ST_OK;
    case PARAM_MOTION_ACTION:  /* 1=继续 2=暂停 3=停止 */
        if (len != 1U) return BIN_ST_LENGTH;
        if (data[0] == 1U) A_Servo_Resume();
        else if (data[0] == 2U) A_Servo_Pause();
        else if (data[0] == 3U) A_Servo_Stop();
        else return BIN_ST_VALUE;
        return BIN_ST_OK;
    case PARAM_MOTION_REQUEST: /* 脉宽2字节 + 时间2字节 */
        if (len != 4U) return BIN_ST_LENGTH;
        value = Bin_Get16(data);
        if (value < SERVO_PWM_MIN || value > SERVO_PWM_MAX) return BIN_ST_VALUE;
        if (!A_Protect_CanDrive()) return BIN_ST_STATE;
        A_Servo_Submit((uint16_t)value, Bin_Get16(data + 2)); return BIN_ST_OK;
    case PARAM_CAL_ACTION:     /* 1=SCK 2=SCZ 3=AMI 4=AMX 5=SMI 6=SMX 7=CSD */
        if (len != 1U) return BIN_ST_LENGTH;
        return Bin_Action(k_cal, (uint8_t)(sizeof(k_cal) / sizeof(k_cal[0])), data[0]);
    case PARAM_SYSTEM_ACTION:  /* 1=SRAM参数重载 2=恢复默认保留ID 3=恢复出厂 */
        if (len != 1U) return BIN_ST_LENGTH;
        if (data[0] == 1U) { A_Config_ReloadRuntime(0U); A_Servo_ApplyTrajConfig(); return BIN_ST_OK; }
        if (data[0] != 2U && data[0] != 3U) return BIN_ST_VALUE;
        A_Config_Default((uint8_t)(data[0] == 2U)); A_Servo_ApplyConfig(); *baud_changed = 1U;
        return BIN_ST_OK;
    default: break;
    }
    if ((t = Param_Find(id)) < 0) return (id < 0x20U || id >= 0x80U) ? BIN_ST_READONLY : BIN_ST_VALUE;
    if (len != k_params[t].size) return BIN_ST_LENGTH;
    max = (k_params[t].flags & PF_TMAX) ? A_Config_TorqueMax() : k_params[t].max;
    if (value < k_params[t].min || value > max) return BIN_ST_VALUE;
    if (k_params[t].cfg != PF_NONE) Param_Store((uint8_t *)&g_config + k_params[t].cfg, k_params[t].size, value);
    if (k_params[t].rt != PF_NONE)  Param_Store((uint8_t *)&g_runtime_config + k_params[t].rt, k_params[t].size, value);
    if (k_params[t].flags & PF_TRAJ) A_Servo_ApplyTrajConfig();
    if (k_params[t].flags & PF_BAUD) *baud_changed = 1U;
    if (k_params[t].cfg != PF_NONE) A_Config_MarkDirty();
    return BIN_ST_OK;
}

/* 一帧二进制：校验、过滤ID，执行并应答(广播不应答，且只开放实时运动控制的写入) */
static void Bin_Frame(const uint8_t *f, uint8_t len)
{
    uint8_t out[BIN_TELEMETRY_BYTES + 1U] = {0U}, out_len = 0U, st = BIN_ST_OK, baud = 0U;
    uint8_t id = f[2], cmd = f[3], n = f[4];
    const uint8_t *data = f + 5;

    if (len != 8U + n || f[6U + n] != BIN_TAIL1 || f[7U + n] != BIN_TAIL2 || f[5U + n] != Bin_Crc8(f + 2, (uint8_t)(3U + n))
        || (id != g_config.servo_id && id != BIN_BROADCAST) || (cmd & 0x80U)   /* 别的设备的应答帧 */
        || (id == BIN_BROADCAST && cmd != BIN_CMD_WRITE)) return;
    switch (cmd)
    {
    case BIN_CMD_STATE:
        if (n != 0U) { st = BIN_ST_LENGTH; break; }
        out[0] = A_Servo_GetWorkState(); out[1] = (uint8_t)A_Protect_Fault(); out_len = 2U;
        break;
    case BIN_CMD_READ:
        st = (n != 1U) ? BIN_ST_LENGTH : Bin_ParamRead(data[0], out, &out_len);
        break;
    case BIN_CMD_WRITE:
        if (n < 2U) st = BIN_ST_LENGTH;
        else if (id == BIN_BROADCAST && data[0] != PARAM_MOTION_REQUEST && data[0] != PARAM_MOTION_ACTION
                 && data[0] != PARAM_TORQUE_MODE && data[0] != PARAM_DAMPING) st = BIN_ST_STATE;
        else st = Bin_ParamWrite(data[0], data + 1, (uint8_t)(n - 1U), &baud);
        break;
    case BIN_CMD_RESET_KEEP:
    case BIN_CMD_RESET_ALL:
        if (n != 0U) { st = BIN_ST_LENGTH; break; }
        /* 恢复出厂会改ID：先用旧ID把应答放进发送队列 */
        if (cmd == BIN_CMD_RESET_ALL && id != BIN_BROADCAST) { Bin_Reply(cmd, BIN_ST_OK, out, 0U); id = BIN_BROADCAST; }
        A_Config_Default((uint8_t)(cmd == BIN_CMD_RESET_KEEP));
        A_Servo_ApplyConfig();
        baud = 1U;
        break;
    case BIN_CMD_RESERVED_04: case BIN_CMD_RESERVED_05: case BIN_CMD_RESERVED_06: case BIN_CMD_REBOOT:
        st = BIN_ST_UNSUP;
        break;
    default:
        st = BIN_ST_CMD;
        break;
    }
    if (id != BIN_BROADCAST) Bin_Reply(cmd, st, out, (st == BIN_ST_OK) ? out_len : 0U);
    if (st == BIN_ST_OK && baud) D_UART_SetBaud_Deferred(A_Config_Baudrate());
}

/* ============================== 收帧分流 ============================== */

/* 一根线三种帧：认出帧头后该协议独占整帧，数据区里的字节不会触发别的解析器 */
static void Wire_Byte(uint8_t b)
{
    switch (s_wire)
    {
    case WIRE_TEXT:   /* '#' 重新开始一帧，'!' 结束，超长丢弃 */
        if (b == '#') s_text_len = 0U;
        else if (s_text_len >= TEXT_FRAME_MAX) { s_wire = WIRE_IDLE; return; }
        s_text[s_text_len++] = b;
        if (b == '!') { Text_Frame(s_text, s_text_len); s_wire = WIRE_IDLE; }
        return;
    case WIRE_BINARY:
        s_bin[s_bin_len++] = b;
        if (s_bin_len == 5U && s_bin[4] > BIN_DATA_MAX) s_wire = WIRE_IDLE;            /* 长度非法 */
        else if (s_bin_len >= 5U && s_bin_len == 8U + s_bin[4]) { Bin_Frame(s_bin, s_bin_len); s_wire = WIRE_IDLE; }
        return;
    case WIRE_IAP:
        if (!IAP_Rx_Deal(b)) s_wire = WIRE_IDLE;
        return;
    case WIRE_HEAD_AA:
        if (b == BIN_HEAD2) { s_bin[0] = BIN_HEAD1; s_bin[1] = BIN_HEAD2; s_bin_len = 2U; s_wire = WIRE_BINARY; return; }
        if (b == IAP_FRAME_HEAD2) { (void)IAP_Rx_Deal(IAP_FRAME_HEAD1); (void)IAP_Rx_Deal(IAP_FRAME_HEAD2); s_wire = WIRE_IAP; return; }
        s_wire = WIRE_IDLE;   /* AA AA ...：最后一个AA继续当候选帧头 */
        /* fall through */
    default:
        if (b == '#') { s_text[0] = b; s_text_len = 1U; s_wire = WIRE_TEXT; }
        else if (b == BIN_HEAD1) s_wire = WIRE_HEAD_AA;
        return;
    }
}

/* 串口任务：排空接收队列 -> 分流执行 -> 半帧超时放弃 -> 驱动收尾(切波特率等) */
void A_Uart_Process(void)
{
    uint8_t b, received = 0U;

    while (D_UART1_Rx_Get(&b))
    {
        received = 1U;
        if (!IAP_UpgradePending()) Wire_Byte(b);   /* 已应答进入升级：只清空队列，不再执行 */
    }
    if (received) s_wire_idle_ms = 0U;
    else if (s_wire != WIRE_IDLE && ++s_wire_idle_ms >= 10U)
    {
        IAP_Rx_Reset();
        s_wire = WIRE_IDLE;
        s_wire_idle_ms = 0U;
    }
    D_UART_Service();
}
