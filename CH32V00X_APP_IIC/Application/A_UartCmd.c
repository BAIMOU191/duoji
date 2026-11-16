/* A_UartCmd.c 串口协议层：一根线上三种帧按帧头独占分流——
 *   明文   #IIIP<指令>!            见 Text_*
 *   二进制 AA 66 ID CMD LEN DATA CRC8 66 AA   见 Bin_*；参数地址、宽度、范围与读写规则在 A_Param.c
 *   升级   AA 55 ...               交给 iap.c
 * 本层只做成帧、校验、ID过滤与应答排队，不直接碰参数字段。
 * 广播(ID=255/FF)的应答延时 100ms+本机ID×2ms 发出，总线上有别的舵机在应答就排队顺延；发给本机的新帧作废未发应答。
 * 广播不能改ID(含会把ID清零的全部恢复出厂)，其余指令明文、二进制都允许。 */

#include "A_UartCmd.h"
#include "A_Config.h"
#include "A_Param.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include "A_Protect.h"
#include "D_uart.h"
#include "iap.h"
#include <stdbool.h>
#include <string.h>

#define TEXT_FRAME_MAX 32U
#define TEXT_REPLY_MAX 48U

/* ---- 二进制帧：AA 66 ID CMD LEN DATA CRC8 66 AA ---- */
#define BIN_HEAD1     0xAAU
#define BIN_HEAD2     0x66U
#define BIN_TAIL1     0x66U
#define BIN_TAIL2     0xAAU
#define BIN_DATA_MAX  32U
#define BIN_FRAME_MAX (8U + BIN_DATA_MAX)
#define BIN_BROADCAST 255U

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

_Static_assert(1U + PARAM_VALUE_MAX + 1U <= BIN_DATA_MAX, "最长参数值超出应答数据容量");

#define Q3_TO_US(q)       ((uint16_t)(((uint32_t)(q) + 4U) >> 3))   /* 文本协议：脉宽 Q3 四舍五入回 us */

/* 广播延时应答：100ms + 本机ID×2ms，多机错开避免半双工撞车；新帧到达则作废未发出的应答 */
#define BCAST_REPLY_BASE_MS  100U
#define BCAST_REPLY_ID_MS      2U
#define BCAST_QUIET_MS         2U   /* 总线静下来这么久(再加 ID×2ms)才轮到本机发 */

enum { WIRE_IDLE = 0, WIRE_HEAD_AA, WIRE_TEXT, WIRE_BINARY, WIRE_IAP };

static uint8_t s_text[TEXT_FRAME_MAX], s_text_len;
static uint8_t s_reply[TEXT_REPLY_MAX], s_reply_len;
static uint8_t s_bin[BIN_FRAME_MAX], s_bin_len;
static uint8_t s_wire, s_wire_idle_ms;
static uint8_t s_debug_rx_quiet_ms;   /* 距上次收到字节的ms数，饱和在10：诊断打印只在总线静默时插队 */
static uint8_t s_reply_bcast;   /* 1=当前帧是广播，应答改为延时发送 */
static uint8_t s_bin_bcast;
static uint8_t s_bcast_baud;    /* 广播引起的波特率切换，等延时应答发完再生效 */

static struct {
    uint8_t  pending;
    uint16_t wait_ms;
    uint8_t  len;
    uint8_t  frame[TEXT_REPLY_MAX];
} s_bcast;

static void Bcast_Cancel(void)
{
    s_bcast.pending = 0U;
    if (s_bcast_baud) { s_bcast_baud = 0U; D_UART_SetBaud_Deferred(A_Config_Baudrate()); }   /* 应答作废，欠着的波特率切换照常生效 */
}

/* 总线上有别人的数据(其它舵机的应答等)：等线静下来再发；静默时间按ID递增，多机依次排队不撞车 */
static void Bcast_Yield(void)
{
    uint16_t quiet = (uint16_t)(BCAST_QUIET_MS + (uint16_t)g_config.servo_id * BCAST_REPLY_ID_MS);

    if (s_bcast.pending && s_bcast.wait_ms < quiet) s_bcast.wait_ms = quiet;
}

/* 广播应答错开 100+id*2 ms 再发；帧内容当场拼好 */
static void Bcast_Defer(const uint8_t *f, uint8_t len)
{
    if (len == 0U || len > sizeof(s_bcast.frame)) return;
    memcpy(s_bcast.frame, f, len);
    s_bcast.len     = len;
    s_bcast.wait_ms = (uint16_t)(BCAST_REPLY_BASE_MS
                               + (uint16_t)g_config.servo_id * BCAST_REPLY_ID_MS);
    s_bcast.pending = 1U;
}

/* 1ms一拍：到期发出；发完再处理延迟波特率 */
static void Bcast_Tick(void)
{
    if (!s_bcast.pending) return;
    if (s_bcast.wait_ms > 1U) { s_bcast.wait_ms--; return; }
    s_bcast.pending = 0U;
    (void)D_UART1_Tx_Write(s_bcast.frame, s_bcast.len);
    if (s_bcast_baud)
    {
        s_bcast_baud = 0U;
        D_UART_SetBaud_Deferred(A_Config_Baudrate());
    }
}

/* 应答发出后切到当前配置的波特率；广播的等延时应答发完 */
static void Baud_AfterReply(uint8_t bcast)
{
    if (bcast) s_bcast_baud = 1U;
    else D_UART_SetBaud_Deferred(A_Config_Baudrate());   /* 先用旧波特率把应答发完再切 */
}

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
    {"DCT", 0, T_DCT}, {"DST", 0, T_DST}, {"BD", 0, T_BD}, {"BD", 1, T_BD_SET}, {"SP", 0, T_SP}, {"SP", 3, T_SP_SET},
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
    if (s_reply_bcast) { Bcast_Defer(s_reply, s_reply_len); return; }
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

/* 设置类指令走参数表：范围校验、落盘与联动与二进制写同一套 */
static uint8_t Text_Set(uint8_t addr, uint16_t value)
{
    uint8_t fx = 0U;

    return (uint8_t)(A_Param_WriteValue(addr, value, &fx) == PARAM_ST_OK);
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
    case T_RAD: Reply_Value("", Q3_TO_US(A_Servo_GetPositionQ3()), 4U); return;
    case T_SP:  Reply_Value("SP", A_Protect_GetTorque(), 3U); return;
    case T_DST: A_Servo_Stop(); return;
    case T_RIV:   /* I+[-]4位毫安(位置值增大方向为正) + V+一位小数，例 #000PI0103V7.0! / #000PI-0103V7.0! */
    {
        int16_t ma = A_Protect_GetCurrent();

        Reply_Begin(); Reply_Char('I');
        if (ma < 0) Reply_Char('-');
        Reply_UInt((uint32_t)((ma < 0) ? -(int32_t)ma : ma), 4U); Reply_Volt(); Reply_Send();
        return;
    }
    case T_RTV:
        temp = A_Protect_GetTemperature();
        temp = (temp == (int16_t)-32768) ? 0 : temp / 10;
        Reply_Begin(); Reply_Text("DP"); Reply_UInt(Q3_TO_US(A_Servo_GetPositionQ3()), 4U); Reply_Char('T');
        if (temp < 0) Reply_Char('-');
        Reply_UInt((uint32_t)((temp < 0) ? -temp : temp), 0U); Reply_Volt(); Reply_Send();
        return;
    case T_ID_SET:  ok = Text_Set(PARAM_SERVO_ID, arg); break;   /* 用新ID回复 */
    case T_MOD_SET: ok = Text_Set(PARAM_SERVO_MODE, arg); break;
    case T_SP_SET:  ok = Text_Set(PARAM_TORQUE_LIMIT, arg); break;   /* 3位数，百分比0-100 */
    case T_CSM_SET: ok = Text_Set(PARAM_BOOT_MODE, arg); break;
    case T_BD_SET:  ok = Text_Set(PARAM_BAUD_CODE, arg); break;
    case T_ULK: A_Servo_Release(0U); break;
    case T_ULM: (void)A_Servo_SetDamping(100U); A_Servo_Release(1U); break;
    case T_ULR: A_Servo_RestoreTorque(); break;
    case T_DPT: A_Servo_Pause(); break;
    case T_DCT: A_Servo_Resume(); break;
    case T_SCK: ok = A_Servo_CalibrateMid(); break;
    case T_SCZ: ok = A_Servo_CalibrateZero(); break;
    case T_AMI: ok = A_Servo_SetTravelEnd(1U); break;
    case T_AMX: ok = A_Servo_SetTravelEnd(0U); break;
    case T_SMI: ok = A_Servo_SetPulseLimit(1U); break;
    case T_SMX: ok = A_Servo_SetPulseLimit(0U); break;
    case T_CSD: ok = A_Servo_SaveStartup(); break;
    case T_CLE0: case T_CLE: /* CLE0仅保留ID；CLE全部恢复；二者波特率均恢复默认，并按上电动作走一次 */
        A_Servo_FactoryReset((uint8_t)(code == T_CLE0));
        break;
    default: return;
    }
    if (!ok) return;
    Reply_Begin(); Reply_Text("OK"); Reply_Send();
    if (code == T_BD_SET || code == T_CLE0 || code == T_CLE) Baud_AfterReply(s_reply_bcast);
}

/* 校验帧结构与目标ID，解码执行。参数越界的设置指令整条不认(参数表拒绝，不回复) */
static void Text_Frame(const uint8_t *f, uint8_t len)
{
    const uint8_t *body = f + 5;
    uint8_t  n = (uint8_t)(len - 6U), i, k;
    uint16_t id, pwm, arg = 0U;
#if ENCODER_IS_CIRCULAR
    uint16_t turns;
#endif

    if (len < 7U || f[0] != '#' || f[4] != 'P' || f[len - 1U] != '!' || !Text_Digits(f + 1, 3U, &id)
        || (id != g_config.servo_id && id != 255U)) return;   /* 255=广播 */
    Bcast_Cancel();   /* 发给本机的新帧：作废尚未发出的广播应答 */
#if ENCODER_IS_CIRCULAR
    if (n == 11U && body[4] == 'N' && body[6] == 'T'
        && Text_Digits(body, 4U, &pwm) && Text_Digits(body + 5, 1U, &turns)
        && Text_Digits(body + 7, 4U, &arg))
    {   /* nnnnNnTnnnn：目标脉宽 + 整圈数 + 每圈时间(ms)，仅整圈编码器 */
        (void)A_Servo_SubmitTurns(pwm, (uint8_t)turns, arg);
        return;
    }
#endif
    if (n == 9U && body[4] == 'T' && Text_Digits(body, 4U, &pwm) && Text_Digits(body + 5, 4U, &arg))
    {   /* nnnnTnnnn：目标脉宽 + 时间(ms) */
        (void)A_Servo_Submit(pwm, arg);   /* 明文运动指令按协议不回复 */
        return;
    }
    for (i = 0U; i < sizeof(k_text) / sizeof(k_text[0]); i++)
    {
        k = (uint8_t)strlen(k_text[i].key);
        if (n != k + k_text[i].digits || memcmp(body, k_text[i].key, k) != 0
            || (k_text[i].digits && !Text_Digits(body + k, k_text[i].digits, &arg))) continue;
        if (id == 255U && (k_text[i].code == T_ID_SET || k_text[i].code == T_CLE)) return;   /* 广播不能改ID(全部恢复会把ID清零) */
        s_reply_bcast = (uint8_t)(id == 255U);
        Text_Execute(k_text[i].code, arg);
        s_reply_bcast = 0U;
        return;
    }
}

/* ============================== 二进制协议 ============================== */

/* CRC-8/ATM：多项式0x07，初值0 */
static uint8_t Bin_Crc8(const uint8_t *p, uint8_t len)
{
    uint8_t crc = 0U, b;

    while (len--)
        for (crc ^= *p++, b = 0U; b < 8U; b++) crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x07U : 0U));
    return crc;
}

/* 应答：DATA = 状态码 + data。广播帧延时发送(100+id*2 ms) */
static void Bin_Reply(uint8_t cmd, uint8_t status, const uint8_t *data, uint8_t len)
{
    uint8_t f[BIN_FRAME_MAX];

    f[0] = BIN_HEAD1; f[1] = BIN_HEAD2; f[2] = g_config.servo_id; f[3] = (uint8_t)(cmd | 0x80U);
    f[4] = (uint8_t)(len + 1U); f[5] = status;
    memcpy(f + 6, data, len);
    f[6U + len] = Bin_Crc8(f + 2, (uint8_t)(4U + len));
    f[7U + len] = BIN_TAIL1; f[8U + len] = BIN_TAIL2;
    if (s_bin_bcast) { Bcast_Defer(f, (uint8_t)(9U + len)); return; }
    (void)D_UART1_Tx_Write(f, (uint8_t)(9U + len));
}

/* 广播写入里会改ID的：写ID参数、系统动作3(全部恢复出厂) */
#define BCAST_SETS_ID(d, n) ((d)[0] == PARAM_SERVO_ID || ((d)[0] == PARAM_SYSTEM_ACTION && (n) >= 2U && (d)[1] == 3U))

/* 一帧二进制：校验、滤ID、执行应答；广播不能改ID(含全部恢复出厂)，应答延时排队发出 */
static void Bin_Frame(const uint8_t *f, uint8_t len)
{
    uint8_t out[1U + PARAM_VALUE_MAX] = {0U}, out_len = 0U, st = PARAM_ST_OK, fx = 0U;
    uint8_t id = f[2], cmd = f[3], n = f[4];
    const uint8_t *data = f + 5;

    if (len != 8U + n || f[6U + n] != BIN_TAIL1 || f[7U + n] != BIN_TAIL2 || f[5U + n] != Bin_Crc8(f + 2, (uint8_t)(3U + n))
        || (id != g_config.servo_id && id != BIN_BROADCAST) || (cmd & 0x80U)) return;   /* 别的设备的应答帧 */
    Bcast_Cancel();
    s_bin_bcast = (uint8_t)(id == BIN_BROADCAST);
    switch (cmd)
    {
    case BIN_CMD_STATE:
        if (n != 0U) { st = PARAM_ST_LENGTH; break; }
        out[0] = A_Servo_GetWorkState(); out[1] = (uint8_t)A_Protect_Fault(); out_len = 2U;
        break;
    case BIN_CMD_READ:   /* 应答数据 = 地址 + 参数值 */
        if (n != 1U) { st = PARAM_ST_LENGTH; break; }
        out[0] = data[0];
        st = A_Param_Read(data[0], out + 1, &out_len);
        out_len++;
        break;
    case BIN_CMD_WRITE:
        if (n < 2U) st = PARAM_ST_LENGTH;
        else if (s_bin_bcast && BCAST_SETS_ID(data, n)) st = PARAM_ST_STATE;
        else st = A_Param_Write(data[0], data + 1, (uint8_t)(n - 1U), &fx);
        break;
    case BIN_CMD_RESET_KEEP:   /* 07仅保留ID；08全部恢复；二者均恢复默认波特率，并按上电动作走一次 */
    case BIN_CMD_RESET_ALL:
        if (n != 0U) { st = PARAM_ST_LENGTH; break; }
        if (s_bin_bcast && cmd == BIN_CMD_RESET_ALL) { st = PARAM_ST_STATE; break; }   /* 全部恢复会把ID清零 */
        A_Servo_FactoryReset((uint8_t)(cmd == BIN_CMD_RESET_KEEP));
        fx = PARAM_FX_BAUD;
        break;
    case BIN_CMD_RESERVED_04: case BIN_CMD_RESERVED_05: case BIN_CMD_RESERVED_06: case BIN_CMD_REBOOT:
        st = PARAM_ST_UNSUP;
        break;
    default:
        st = PARAM_ST_CMD;
        break;
    }
    Bin_Reply(cmd, st, out, (st == PARAM_ST_OK) ? out_len : 0U);
    if (st == PARAM_ST_OK && (fx & PARAM_FX_BAUD)) Baud_AfterReply(s_bin_bcast);
    s_bin_bcast = 0U;
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

/* 串口任务：排空接收队列 -> 分流执行 -> 半帧超时放弃 -> 广播延时应答 -> 驱动收尾(切波特率等) */
void A_Uart_Process(void)
{
    uint8_t b, received = 0U;

    while (D_UART1_Rx_Get(&b))
    {
        received = 1U;
        Bcast_Yield();    /* 总线有人在发：广播应答等线空了再排队发出 */
        if (!IAP_UpgradePending()) Wire_Byte(b);   /* 已应答进入升级：只清空队列，不再执行 */
    }
    if (received) s_wire_idle_ms = 0U;
    else if (s_wire != WIRE_IDLE && ++s_wire_idle_ms >= 10U)
    {
        IAP_Rx_Reset();
        s_wire = WIRE_IDLE;
        s_wire_idle_ms = 0U;
    }
    if (received) s_debug_rx_quiet_ms = 0U;
    else if (s_debug_rx_quiet_ms < 10U) s_debug_rx_quiet_ms++;
    Bcast_Tick();
    D_UART_Service();
}

uint8_t A_Uart_DebugReady(void)
{
    return (uint8_t)(s_wire == WIRE_IDLE && s_debug_rx_quiet_ms >= 10U);
}
