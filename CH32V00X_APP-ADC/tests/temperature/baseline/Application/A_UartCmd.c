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

#define UART_FRAME_MAX  32U                   /* 单帧最大字节数，超长直接丢弃 */
#define UART_REPLY_MAX  48U                   /* 回复缓冲上限，最长的是版本号 */

/* 指令集。新增指令只需在这里加一项、在Uart_Decode加一条匹配、
 * 在Uart_Execute加一个case，三处一一对应，不会漏。 */
typedef enum {
    CMD_NONE = 0,      /* 未识别，静默丢弃            */
    CMD_MOVE,          /* nnnnTnnnn 位置/时间指令     */
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
    CMD_MID_CAL,       /* SCK   中位校正              */
    CMD_START_SAVE,    /* CSD   保存当前位置为上电位  */
    CMD_BOOT_GET,      /* CSM   读上电动作            */
    CMD_BOOT_SET,      /* CSMn  写上电动作            */
    CMD_RESET_KEEP_ID, /* CLE0  恢复默认(保留ID)      */
    CMD_RESET_ALL,     /* CLE   恢复出厂              */
    CMD_RUNTIME_GET,   /* RTV   读位置/温度/电压      */
    CMD_TORQUE_GET,    /* SP    读扭矩上限            */
    CMD_TORQUE_SET,    /* SPnnn 写扭矩上限            */
    CMD_TRAVEL_MIN,    /* SMI   当前位置设为500us端   */
    CMD_TRAVEL_MAX     /* SMX   当前位置设为2500us端  */
} CommandType_t;

typedef struct {
    CommandType_t type;  /* 指令种类                                */
    uint16_t      pwm;   /* CMD_MOVE的目标脉宽(us)                  */
    uint16_t      value; /* CMD_MOVE的时间参数；其余指令的整数参数  */
} Command_t;

typedef struct {
    uint8_t data[UART_REPLY_MAX]; /* 待发字节   */
    uint8_t len;                  /* 已填充长度 */
} Reply_t;

static uint8_t s_frame[UART_FRAME_MAX]; /* 正在组装的接收帧              */
static uint8_t s_frame_idx;             /* 已收字节数，同时是写入下标    */
static uint8_t s_framing;               /* 1=已见到帧头，正在收帧中      */
static Reply_t s_reply;                 /* 唯一回复缓冲，只在任务上下文用 */

/* ============================== 解码 ============================== */

/*
 * @fn      Uart_ParseDigits
 * @brief   定长十进制数字串转整数，出现非数字一律判非法
 * @param   data 数字串起点
 * @param   len  位数
 * @param   out  结果输出
 * @return  true=全是数字且已写出结果
 */
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

/*
 * @fn      Uart_BodyIs
 * @brief   指令体是否与关键字完全相等(长度也必须一致)
 * @param   body 指令体
 * @param   len  指令体长度
 * @param   text 关键字
 * @return  true=完全匹配
 */
static bool Uart_BodyIs(const uint8_t *body, uint8_t len, const char *text)
{
    uint8_t n = (uint8_t)strlen(text); /* 关键字长度 */

    return len == n && memcmp(body, text, n) == 0;
}

/*
 * @fn      Uart_Decode
 * @brief   把指令体翻译成Command_t
 * @param   body 指令体(已去掉帧头ID和结尾感叹号)
 * @param   len  指令体长度
 * @param   cmd  输出
 * @return  true=识别成功
 *
 * 顺序有讲究：同前缀的指令必须靠长度区分，且长的先判。
 * 例如 CLE0 若排在 CLE 之后就会被当成 CLE 执行，把ID一起清掉。
 */
static bool Uart_Decode(const uint8_t *body, uint8_t len, Command_t *cmd)
{
    uint16_t arg; /* 指令携带的整数参数 */

    memset(cmd, 0, sizeof(*cmd));

    /* 运动指令 nnnnTnnnn 是唯一的定长格式，先单独匹配 */
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
    else if (Uart_BodyIs(body, len, "SMI"))  cmd->type = CMD_TRAVEL_MIN;
    else if (Uart_BodyIs(body, len, "SMX"))  cmd->type = CMD_TRAVEL_MAX;
    else if (Uart_BodyIs(body, len, "SCK"))  cmd->type = CMD_MID_CAL;
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

/* ============================ 回复组装 ============================
 * 统一构造器：新增回读指令只需组合 Text/UInt/SInt，不必再拼一遍帧头帧尾。
 * 全部写入唯一的 s_reply，只在任务上下文调用，不需要互斥。 */

/*
 * @fn      Reply_Begin
 * @brief   写入帧头：井号 + 3位本机ID + P
 * @param   无
 * @return  无
 */
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

/*
 * @fn      Reply_UInt
 * @brief   追加无符号十进制数，不足width位在左侧补零
 * @param   value 数值
 * @param   width 最小位数，0=不补零
 * @return  无
 */
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

/*
 * @fn      Reply_Send
 * @brief   补上帧尾感叹号并整帧交给串口驱动
 * @param   无
 * @return  无
 *
 * 队列放不下时整帧丢弃：宁可不回复，也不能回半条让上位机解析出错帧。
 */
static void Reply_Send(void)
{
    Reply_Char('!');
    (void)D_UART1_Tx_Write(s_reply.data, s_reply.len);
}

/* 最常见的两种回复，避免每个case重复三行 */
static void Reply_OK(void)
{
    Reply_Begin(); Reply_Text("OK"); Reply_Send();
}

static void Reply_Value(const char *tag, uint32_t value, uint8_t width)
{
    Reply_Begin(); Reply_Text(tag); Reply_UInt(value, width); Reply_Send();
}

/* ============================== 执行 ============================== */

/*
 * @fn      Uart_Execute
 * @brief   执行一条已解码指令并组织回复
 * @param   cmd 已解码指令
 * @return  无
 *
 * 不回复的三种情况都是协议规定而非遗漏：
 *   CMD_MOVE / CMD_STOP        —— 连续下发的运动指令，回复会挤占总线；
 *   MODE_SET/MID_CAL/START_SAVE 失败 —— 当前模式不支持该操作，静默忽略。
 */
static void Uart_Execute(const Command_t *cmd)
{
    int16_t  temp;     /* 温度，0.1摄氏度，-32768=传感器读取失败 */
    uint16_t voltage;  /* 电池电压，厘伏                         */
    uint16_t decivolt; /* 电池电压，0.1V，供RTV显示              */

    switch (cmd->type)
    {
    case CMD_MOVE: A_Servo_Submit(cmd->pwm, cmd->value); break;

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
    /* 回复格式与设置指令完全一致：收到 #000PSP050! 就照原样答一条回去 */
    case CMD_TORQUE_GET:   Reply_Value("SP",   A_Protect_GetTorque(), 3U); break;

    case CMD_MODE_SET:     if (A_Servo_SetMode((uint8_t)cmd->value)) Reply_OK(); break;
    case CMD_TORQUE_SET:   if (A_Protect_SetTorque((uint8_t)cmd->value)) Reply_OK(); break;
    case CMD_MID_CAL:      if (A_Servo_CalibrateMid())               Reply_OK(); break;
    /* 失败(电机模式/编码器读失败/剩余行程不足1度)一律静默，与SCK一致 */
    case CMD_TRAVEL_MIN:   if (A_Servo_SetTravelEnd(1U))              Reply_OK(); break;
    case CMD_TRAVEL_MAX:   if (A_Servo_SetTravelEnd(0U))              Reply_OK(); break;
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

/*
 * @fn      Uart_ParseFrame
 * @brief   校验帧结构、过滤目标ID，然后解码执行
 * @param   frame 完整帧
 * @param   len   帧长
 * @return  无
 */
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

/*
 * @fn      Uart_Frame_Byte
 * @brief   把1个接收字节喂进组帧状态机，收满一帧就解析
 * @param   byte 接收字节
 * @return  无
 *
 * 帧头无条件重启组帧：总线上丢字节或串进别的从机回复时，下一帧照样能同步。
 */
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

/*
 * @fn      A_Uart_Process
 * @brief   串口任务入口：排空接收队列 -> 组帧执行 -> 回复入队 -> 驱动收尾
 * @param   无
 * @return  无
 *
 * 不再判断输入源：PWM模式下 D_UART_Enable(0) 已关掉接收中断，队列自然是空的，
 * D_UART1_Tx_Write 也会直接拒绝——硬件状态就是唯一真相，不需要第二处开关。
 */
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
