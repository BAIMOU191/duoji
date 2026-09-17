/* A_UartCmd.c 单文件协议层：组帧、解码、分发与应答组帧集中在此。 */

#include "A_UartCmd.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include "A_Protect.h"
#include "D_uart.h"
#include "iap.h"
#include <stdbool.h>
#include <string.h>

#define UART_FRAME_MAX 32U /** 单帧最大字节数，超长直接丢弃 */
#define UART_REPLY_MAX 48U /** 回复缓冲上限，最长的是版本号 */

/* 指令集：新增指令在此加一项、Uart_Decode 加匹配、Uart_Execute 加 case */
typedef enum {
    CMD_NONE = 0,      /* 未识别，静默丢弃            */
    CMD_MOVE,          /* nnnnTnnnn 位置/时间指令     */
    CMD_MOVE_TURNS,    /* nnnnNnTnnnn 多圈指令        */
    CMD_VER,           /* VER   读固件版本            */
    CMD_ID_GET,        /* ID    读总线ID              */
    CMD_ID_SET,        /* IDnnn 写总线ID              */
    CMD_RELEASE_LOW,   /* ULK   低阻力卸力            */
    CMD_RELEASE_HIGH,  /* ULM   高阻力卸力            */
    CMD_TORQUE_ON,     /* ULR   恢复扭矩              */
    CMD_MODE_GET,      /* MOD   读工作模式            */
    CMD_MODE_SET,      /* MODn  写工作模式            */
    CMD_POSITION_GET,  /* RAD   读当前位置(脉宽)      */
    CMD_PAUSE,         /* DPT   暂停                  */
    CMD_CONTINUE,      /* DCT   继续                  */
    CMD_STOP,          /* DST   停止                  */
    CMD_BAUD_GET,      /* BD    读波特率              */
    CMD_BAUD_SET,      /* BDn   写波特率              */
    CMD_MID_CAL,       /* SCK   中位校正(1500us)      */
    CMD_ZERO_CAL,      /* SCZ   零点校正(500us)       */
    CMD_START_SAVE,    /* CSD   保存当前位置为上电位  */
    CMD_BOOT_GET,      /* CSM   读上电动作            */
    CMD_BOOT_SET,      /* CSMn  写上电动作            */
    CMD_RESET_KEEP_ID, /* CLE0  恢复默认(保留ID)      */
    CMD_RESET_ALL,     /* CLE   恢复出厂              */
    CMD_RUNTIME_GET,   /* RTV   读位置/温度/电压      */
    CMD_TORQUE_GET,    /* SP    读扭矩上限            */
    CMD_TORQUE_SET,    /* SPnnn 写扭矩上限            */
    CMD_TRAVEL_MIN,    /* AMI   当前位置设为500us端   */
    CMD_TRAVEL_MAX,    /* AMX   当前位置设为2500us端  */
    CMD_PULSE_MIN,     /* SMI   当前位置的脉宽设为下界 */
    CMD_PULSE_MAX,     /* SMX   当前位置的脉宽设为上界 */
    CMD_POWER_GET      /* RIV   读电流与电压          */
} CommandType_t;

typedef struct {
    CommandType_t type;  /* 指令种类                                */
    uint16_t      pwm;   /* CMD_MOVE的目标脉宽(us)                  */
    uint16_t      value; /* CMD_MOVE的时间参数；其余指令的整数参数  */
    uint8_t       turns; /* CMD_MOVE_TURNS的圈数，0~9               */
} Command_t;

typedef struct {
    uint8_t data[UART_REPLY_MAX]; /* 待发字节   */
    uint8_t len;                  /* 已填充长度 */
} Reply_t;

static uint8_t s_frame[UART_FRAME_MAX]; /** 正在组装的接收帧 */
static uint8_t s_frame_idx;             /** 已收字节数，同时是写入下标 */
static uint8_t s_framing;               /** 1=已见到帧头，正在收帧中 */
static Reply_t s_reply;                 /** 唯一回复缓冲，只在任务上下文用 */

/* ============================== 解码 ============================== */

/* 定长十进制数字串转整数，出现非数字一律判非法，返回0=失败 */
static bool Uart_ParseDigits(const uint8_t *data, uint8_t len, uint16_t *out)
{
    uint16_t value = 0U; /* 累加结果 */
    uint8_t  i;          /* 位游标   */

    if (len == 0U || out == 0) return false;
    for (i = 0U; i < len; i++)
    {
        if (data[i] < '0' || data[i] > '9') return false;
        value = (uint16_t)(value * 10U + data[i] - '0');
    }
    *out = value;
    return true;
}

/* 指令体是否与关键字完全相等(长度也必须一致) */
static bool Uart_BodyIs(const uint8_t *body, uint8_t len, const char *text)
{
    uint8_t n = (uint8_t)strlen(text); /* 关键字长度 */

    return len == n && memcmp(body, text, n) == 0;
}

/* 指令体翻译成 Command_t，返回false=无法识别。同前缀的指令靠长度区分，长的先判(如 CLE0 在 CLE 之前) */
static bool Uart_Decode(const uint8_t *body, uint8_t len, Command_t *cmd)
{
    uint16_t arg; /* 指令携带的整数参数 */

    memset(cmd, 0, sizeof(*cmd));

    /* 两条定长运动指令，长的先判：
     *   nnnnTnnnn    T = 走到目标的时间(ms)
     *   nnnnNnTnnnn  N = 额外整圈数(0~9)，T = 每转一圈的时间(ms) */
    if (len == 11U && body[4] == 'N' && body[6] == 'T'
        && Uart_ParseDigits(body, 4U, &cmd->pwm)
        && Uart_ParseDigits(body + 5U, 1U, &arg)
        && Uart_ParseDigits(body + 7U, 4U, &cmd->value))
    {
        if (cmd->pwm < SERVO_PWM_MIN || cmd->pwm > SERVO_PWM_MAX) return false;
        cmd->turns = (uint8_t)arg;
        cmd->type  = CMD_MOVE_TURNS;
        return true;
    }

    if (len == 9U && body[4] == 'T'
        && Uart_ParseDigits(body, 4U, &cmd->pwm)
        && Uart_ParseDigits(body + 5U, 4U, &cmd->value))
    {
        if (cmd->pwm < SERVO_PWM_MIN || cmd->pwm > SERVO_PWM_MAX) return false;
        cmd->type = CMD_MOVE;
        return true;
    }

    if (Uart_BodyIs(body, len, "VER"))       cmd->type = CMD_VER;
    else if (Uart_BodyIs(body, len, "ID"))   cmd->type = CMD_ID_GET;
    else if (len == 5U && memcmp(body, "ID", 2U) == 0
             && Uart_ParseDigits(body + 2U, 3U, &arg) && arg <= 254U)
        cmd->type = CMD_ID_SET, cmd->value = arg;
    else if (Uart_BodyIs(body, len, "ULK"))  cmd->type = CMD_RELEASE_LOW;
    else if (Uart_BodyIs(body, len, "ULM"))  cmd->type = CMD_RELEASE_HIGH;
    else if (Uart_BodyIs(body, len, "ULR"))  cmd->type = CMD_TORQUE_ON;
    else if (Uart_BodyIs(body, len, "MOD"))  cmd->type = CMD_MODE_GET;
    else if (len >= 4U && len <= 5U && memcmp(body, "MOD", 3U) == 0
             && Uart_ParseDigits(body + 3U, (uint8_t)(len - 3U), &arg)
             && SERVO_MODE_IS_SUPPORTED(arg) && arg <= SERVO_MODE_TIMED_CCW)
        cmd->type = CMD_MODE_SET, cmd->value = arg;
    else if (Uart_BodyIs(body, len, "RAD"))  cmd->type = CMD_POSITION_GET;
    else if (Uart_BodyIs(body, len, "DPT"))  cmd->type = CMD_PAUSE;
    else if (Uart_BodyIs(body, len, "DCT"))  cmd->type = CMD_CONTINUE;
    else if (Uart_BodyIs(body, len, "DST"))  cmd->type = CMD_STOP;
    else if (Uart_BodyIs(body, len, "BD"))   cmd->type = CMD_BAUD_GET;
    else if (len == 3U && memcmp(body, "BD", 2U) == 0
             && Uart_ParseDigits(body + 2U, 1U, &arg) && arg >= 1U && arg <= 8U)
        cmd->type = CMD_BAUD_SET, cmd->value = arg;
    else if (Uart_BodyIs(body, len, "SP"))   cmd->type = CMD_TORQUE_GET;
    /* 帧头的P是格式固定的，所以 #000PSP050! 的指令体是 SP050，关键字是SP */
    else if (len == 5U && memcmp(body, "SP", 2U) == 0
             && Uart_ParseDigits(body + 2U, 3U, &arg) && arg <= PROT_TORQUE_MAX)
        cmd->type = CMD_TORQUE_SET, cmd->value = arg;
    else if (Uart_BodyIs(body, len, "AMI"))  cmd->type = CMD_TRAVEL_MIN;
    else if (Uart_BodyIs(body, len, "AMX"))  cmd->type = CMD_TRAVEL_MAX;
    else if (Uart_BodyIs(body, len, "SMI"))  cmd->type = CMD_PULSE_MIN;
    else if (Uart_BodyIs(body, len, "SMX"))  cmd->type = CMD_PULSE_MAX;
    else if (Uart_BodyIs(body, len, "SCK"))  cmd->type = CMD_MID_CAL;
    else if (Uart_BodyIs(body, len, "SCZ"))  cmd->type = CMD_ZERO_CAL;
    else if (Uart_BodyIs(body, len, "RIV"))  cmd->type = CMD_POWER_GET;
    else if (Uart_BodyIs(body, len, "CSD"))  cmd->type = CMD_START_SAVE;
    else if (Uart_BodyIs(body, len, "CSM"))  cmd->type = CMD_BOOT_GET;
    else if (len == 4U && memcmp(body, "CSM", 3U) == 0
             && Uart_ParseDigits(body + 3U, 1U, &arg)
             && arg >= SERVO_BOOT_GOTO_START && arg <= SERVO_BOOT_RELEASE)
        cmd->type = CMD_BOOT_SET, cmd->value = arg;
    else if (Uart_BodyIs(body, len, "CLE0")) cmd->type = CMD_RESET_KEEP_ID;
    else if (Uart_BodyIs(body, len, "CLE"))  cmd->type = CMD_RESET_ALL;
    else if (Uart_BodyIs(body, len, "RTV"))  cmd->type = CMD_RUNTIME_GET;

    return cmd->type != CMD_NONE;
}

/* ============================ 回复组装 ============================ */

/* 写入回复帧头：井号 + 3位本机ID + P */
static void Reply_Begin(void)
{
    uint8_t id = g_config.servo_id; /* 回复一律用本机当前ID，便于总线上区分 */

    s_reply.len = 0U;
    s_reply.data[s_reply.len++] = '#';
    s_reply.data[s_reply.len++] = (uint8_t)('0' + id / 100U);
    s_reply.data[s_reply.len++] = (uint8_t)('0' + id / 10U % 10U);
    s_reply.data[s_reply.len++] = (uint8_t)('0' + id % 10U);
    s_reply.data[s_reply.len++] = 'P';
}

/* 追加1个字符；缓冲满则静默丢弃(容量已按最长回复留足余量) */
static void Reply_Char(uint8_t ch)
{
    if (s_reply.len < UART_REPLY_MAX) s_reply.data[s_reply.len++] = ch;
}

/* 追加一个C字符串 */
static void Reply_Text(const char *text)
{
    while (*text && s_reply.len < UART_REPLY_MAX)
        s_reply.data[s_reply.len++] = (uint8_t)*text++;
}

/* 向回复缓冲追加无符号十进制数，不足width位在左侧补零 */
static void Reply_UInt(uint32_t value, uint8_t width)
{
    uint8_t digit[10]; /* 倒序暂存的数字字符 */
    uint8_t n = 0U;    /* 已生成位数         */

    do {
        digit[n++] = (uint8_t)('0' + value % 10U);
        value /= 10U;
    } while (value && n < sizeof(digit));

    while (n < width && n < sizeof(digit)) digit[n++] = '0';
    while (n) Reply_Char(digit[--n]);
}

/* 追加有符号十进制数，负数前置减号 */
static void Reply_SInt(int32_t value)
{
    if (value < 0)
    {
        Reply_Char('-');
        value = -value;
    }
    Reply_UInt((uint32_t)value, 0U);
}

/* 补帧尾'!'整帧交给串口驱动，队列放不下整帧丢弃 */
static void Reply_Send(void)
{
    Reply_Char('!');
    (void)D_UART1_Tx_Write(s_reply.data, s_reply.len);
}

/* 最常见的回复 */
static void Reply_OK(void)
{
    Reply_Begin(); Reply_Text("OK"); Reply_Send();
}

/* 组一条"标签+定长数值"的回复并发出 */
static void Reply_Value(const char *tag, uint32_t value, uint8_t width)
{
    Reply_Begin(); Reply_Text(tag); Reply_UInt(value, width); Reply_Send();
}

/* ============================== 执行 ============================== */

/* 执行已解码指令并组织回复；运动/停止指令及失败的设置类指令按协议不回复 */
static void Uart_Execute(const Command_t *cmd)
{
    int16_t  temp;     /* 温度，0.1摄氏度，-32768=传感器读取失败 */
    uint16_t voltage;  /* 电池电压，厘伏                         */
    uint16_t decivolt; /* 电池电压，0.1V，供RTV显示              */

    switch (cmd->type)
    {
    case CMD_MOVE: A_Servo_Submit(cmd->pwm, cmd->value); break;
    case CMD_MOVE_TURNS:
        A_Servo_SubmitTurns(cmd->pwm, cmd->turns, cmd->value);
        break;

    case CMD_VER:
        Reply_Begin(); Reply_Text(SERVO_VERSION); Reply_Send(); break;

    case CMD_ID_GET: /* 帧头本身就带ID，body留空即可 */
        Reply_Begin(); Reply_Send(); break;

    case CMD_ID_SET:
        g_config.servo_id = (uint8_t)cmd->value;
        A_Config_MarkDirty();
        Reply_OK(); /* 用新ID回复，上位机据此确认改名成功 */
        break;

    case CMD_RELEASE_LOW:  A_Servo_Release(0U);     Reply_OK(); break;
    case CMD_RELEASE_HIGH: A_Servo_Release(1U);     Reply_OK(); break;
    case CMD_TORQUE_ON:    A_Servo_RestoreTorque(); Reply_OK(); break;
    case CMD_PAUSE:        A_Servo_Pause();         Reply_OK(); break;
    case CMD_CONTINUE:     A_Servo_Resume();        Reply_OK(); break;
    case CMD_STOP:         A_Servo_Stop();                      break;

    case CMD_MODE_GET:     Reply_Value("MOD",  g_config.servo_mode, 0U); break;
    case CMD_BOOT_GET:     Reply_Value("CSM",  g_config.boot_mode,  0U); break;
    case CMD_BAUD_GET:     Reply_Value("BAUD", A_Config_Baudrate(), 0U); break;
    case CMD_POSITION_GET: Reply_Value("",     A_Servo_GetPositionPwm(), 4U); break;
    /* 回复格式与设置指令一致 */
    case CMD_TORQUE_GET:   Reply_Value("SP",   A_Protect_GetTorque(), 3U); break;

    case CMD_MODE_SET:     if (A_Servo_SetMode((uint8_t)cmd->value)) Reply_OK(); break;
    case CMD_TORQUE_SET:   if (A_Protect_SetTorque((uint8_t)cmd->value)) Reply_OK(); break;
    case CMD_MID_CAL:      if (A_Servo_CalibrateMid())               Reply_OK(); break;
    case CMD_ZERO_CAL:     if (A_Servo_CalibrateZero())              Reply_OK(); break;
    /* 失败(电机模式/读失败/剩余行程不足1度)一律静默 */
    case CMD_TRAVEL_MIN:   if (A_Servo_SetTravelEnd(1U))              Reply_OK(); break;
    case CMD_TRAVEL_MAX:   if (A_Servo_SetTravelEnd(0U))              Reply_OK(); break;
    /* 失败(电机模式/读失败/两端交叉)同样静默 */
    case CMD_PULSE_MIN:    if (A_Servo_SetPulseLimit(1U))             Reply_OK(); break;
    case CMD_PULSE_MAX:    if (A_Servo_SetPulseLimit(0U))             Reply_OK(); break;
    case CMD_START_SAVE:   if (A_Servo_SaveStartup())                Reply_OK(); break;

    case CMD_BOOT_SET:
        g_config.boot_mode = (uint8_t)cmd->value;
        A_Config_MarkDirty();
        Reply_OK();
        break;

    case CMD_BAUD_SET:
        g_config.baud_code = (uint8_t)cmd->value;
        A_Config_MarkDirty();
        Reply_OK();
        /* 先用旧波特率把OK发完，驱动排空后自己切过去 */
        D_UART_SetBaud_Deferred(A_Config_Baudrate());
        break;

    /* 两条恢复默认值指令只差"保不保留ID"一个布尔量，合并成一个分支 */
    case CMD_RESET_KEEP_ID:
    case CMD_RESET_ALL:
        A_Config_Default((uint8_t)(cmd->type == CMD_RESET_KEEP_ID));
        A_Servo_ApplyConfig();
        Reply_OK();
        D_UART_SetBaud_Deferred(A_Config_Baudrate()); /* 默认值可能改了波特率 */
        break;

    /* RIV：I+4位毫安 + V+1位小数电压，例 #000PI0103V7.0!；电流取保护模块最近一块读数 */
    case CMD_POWER_GET:
        voltage  = A_Voltage_Read();
        decivolt = (uint16_t)((voltage + 5U) / 10U); /* 厘伏四舍五入到0.1V */

        Reply_Begin();
        Reply_Char('I'); Reply_UInt(A_Protect_GetCurrent(), 4U);
        Reply_Char('V'); Reply_UInt(decivolt / 10U, 0U);
        Reply_Char('.'); Reply_UInt(decivolt % 10U, 1U);
        Reply_Send();
        break;

    case CMD_RUNTIME_GET:
        temp     = A_Temperature_Read();
        voltage  = A_Voltage_Read();
        decivolt = (uint16_t)((voltage + 5U) / 10U); /* 厘伏四舍五入到0.1V */

        Reply_Begin();
        Reply_Text("DP"); Reply_UInt(A_Servo_GetPositionPwm(), 4U);
        Reply_Char('T');  Reply_SInt(temp == (int16_t)-32768 ? 0 : temp / 10);
        Reply_Char('V');  Reply_UInt(decivolt / 10U, 0U);
        Reply_Char('.');  Reply_UInt(decivolt % 10U, 1U);
        Reply_Send();
        break;

    default: break;
    }
}

/* 校验帧结构、过滤目标ID，然后解码执行 */
static void Uart_ParseFrame(const uint8_t *frame, uint8_t len)
{
    uint16_t  id;  /* 帧里携带的目标ID */
    Command_t cmd; /* 解码结果         */

    /* 最短合法帧 = 帧头5字节 + 至少1字节指令体 + 帧尾1字节 */
    if (len < 7U || frame[0] != '#' || frame[4] != 'P'
                 || frame[len - 1U] != '!') return;
    if (!Uart_ParseDigits(frame + 1U, 3U, &id)) return;
    if (id != g_config.servo_id && id != 255U) return; /* 255=广播 */

    if (Uart_Decode(frame + 5U, (uint8_t)(len - 6U), &cmd)) Uart_Execute(&cmd);
}

/* 接收字节喂进组帧状态机，收满一帧就解析；'#' 无条件重启组帧 */
static void Uart_Frame_Byte(uint8_t byte)
{
    if (byte == '#')
    {
        s_framing   = 1U;
        s_frame_idx = 0U;
        s_frame[s_frame_idx++] = byte;
        return;
    }

    if (!s_framing) return;
    if (s_frame_idx >= UART_FRAME_MAX) { s_framing = 0U; return; } /* 超长丢弃 */

    s_frame[s_frame_idx++] = byte;
    if (byte == '!')
    {
        Uart_ParseFrame(s_frame, s_frame_idx);
        s_framing = 0U;
    }
}

/* 串口任务：排空接收队列 -> 组帧执行 -> 回复入队 -> 驱动收尾(PWM输入模式下队列自然为空) */
void A_Uart_Process(void)
{
    uint8_t byte; /* 本轮取出的接收字节 */

    while (D_UART1_Rx_Get(&byte))
    {
        Uart_Frame_Byte(byte); /* 舵机ASCII协议                       */
        IAP_Rx_Deal(byte);     /* 固件升级引导帧，与上面并行嗅探同一串 */
    }

    D_UART_Service(); /* 回复发完后执行延迟的波特率切换 */
}
