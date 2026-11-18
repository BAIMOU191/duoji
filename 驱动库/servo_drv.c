/* servo_drv.c —— 总线舵机主控端驱动库（二进制协议），说明见 servo_drv.h */

#include "servo_drv.h"
#include <string.h>

#define HEAD1 0xAAU
#define HEAD2 0x66U
#define TAIL1 0x66U
#define TAIL2 0xAAU

#define BCAST_BASE_MS   100U   /* 广播应答基础延时 */
#define BCAST_ID_MS     2U     /* 每个ID错开 */

#define ADDR_BAUD       0x21U  /* 波特率档位(写成功后主控跟着切) */
#define ADDR_ID         0x20U  /* 舵机ID(禁止广播写) */
#define SYS_RESET_KEEP  2U     /* 8C=2 恢复默认保留ID */
#define SYS_RESET_ALL   3U     /* 8C=3 全部恢复默认(禁止广播) */
#define BAUD_DEFAULT    115200UL

/* ============================== 1. 底层 ============================== */

uint8_t Servo_Crc8(const uint8_t *p, uint16_t n)
{
    uint8_t crc = 0U, b;

    while (n--)
        for (crc ^= *p++, b = 0U; b < 8U; b++)
            crc = (uint8_t)((crc << 1) ^ ((crc & 0x80U) ? 0x07U : 0U));
    return crc;
}

uint16_t Servo_BuildFrame(uint8_t *out, uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    if (out == 0 || len > SERVO_DATA_MAX || (len && data == 0)) return 0U;
    out[0] = HEAD1; out[1] = HEAD2;
    out[2] = id; out[3] = cmd; out[4] = len;
    if (len) memcpy(out + 5, data, len);
    out[5U + len] = Servo_Crc8(out + 2, (uint16_t)(3U + len));
    out[6U + len] = TAIL1; out[7U + len] = TAIL2;
    return (uint16_t)(8U + len);
}

void Servo_ParserReset(Servo_Parser_t *ps) { ps->len = 0U; }

/* 收满一帧返回1；帧头错、LEN超限、帧尾或CRC错都丢弃重新找帧头 */
int Servo_ParseByte(Servo_Parser_t *ps, uint8_t b)
{
    uint8_t n;

    if (ps->len >= 5U && ps->len == 8U + ps->buf[4]) ps->len = 0U;   /* 上一帧已交付 */
    switch (ps->len)
    {
    case 0U:
        if (b == HEAD1) ps->buf[ps->len++] = b;
        return 0;
    case 1U:
        if (b == HEAD2)      ps->buf[ps->len++] = b;
        else if (b != HEAD1) ps->len = 0U;          /* AA AA 66：第二个 AA 仍当帧头 */
        return 0;
    default:
        break;
    }
    ps->buf[ps->len++] = b;
    if (ps->len < 5U) return 0;
    n = ps->buf[4];
    if (n > SERVO_DATA_MAX) { ps->len = 0U; return 0; }
    if (ps->len < 8U + n) return 0;
    if (ps->buf[6U + n] != TAIL1 || ps->buf[7U + n] != TAIL2
        || ps->buf[5U + n] != Servo_Crc8(ps->buf + 2, (uint16_t)(3U + n)))
    { ps->len = 0U; return 0; }
    return 1;
}

/* ============================== 2. 通用读写 ============================== */

void Servo_Init(Servo_Bus_t *bus, const Servo_Port_t *port)
{
    memset(bus, 0, sizeof(*bus));
    bus->port = *port;
    bus->timeout_ms = SERVO_DEFAULT_TIMEOUT;
    bus->bcast_max_id = SERVO_ID_MAX;
}

uint32_t Servo_BaudOf(uint8_t code)
{
    static const uint32_t k_baud[8] = { 9600UL, 19200UL, 38400UL, 57600UL, 115200UL, 128000UL, 256000UL, 1000000UL };
    return (code >= 1U && code <= 8U) ? k_baud[code - 1U] : 0UL;
}

static void Bus_Drain(Servo_Bus_t *bus)
{
    uint8_t b;
    while (bus->port.read_byte(&b)) { }
}

static void Bus_Send(Servo_Bus_t *bus, const uint8_t *f, uint16_t n)
{
    if (bus->port.set_dir) bus->port.set_dir(1U);
    bus->port.write(f, n);
    if (bus->port.set_dir) bus->port.set_dir(0U);
}

/* 广播：等所有舵机的延时应答过去，期间收到的字节全部丢弃 */
static void Bus_WaitBroadcast(Servo_Bus_t *bus)
{
    uint32_t t0 = bus->port.millis();
    uint32_t wait = BCAST_BASE_MS + (uint32_t)bus->bcast_max_id * BCAST_ID_MS + bus->timeout_ms;
    uint8_t b;

    while ((uint32_t)(bus->port.millis() - t0) < wait) (void)bus->port.read_byte(&b);
}

/* 成功后若舵机波特率变了，主控跟着切 */
static void Bus_FollowBaud(Servo_Bus_t *bus, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint32_t bps = 0UL;

    if (bus->port.set_baud == 0) return;
    if (cmd == SERVO_CMD_RESET_KEEP || cmd == SERVO_CMD_RESET_ALL) bps = BAUD_DEFAULT;
    else if (cmd == SERVO_CMD_WRITE && len == 2U && data[0] == ADDR_BAUD) bps = Servo_BaudOf(data[1]);
    else if (cmd == SERVO_CMD_WRITE && len == 2U && data[0] == SERVO_ADDR_SYSTEM
             && (data[1] == SYS_RESET_KEEP || data[1] == SYS_RESET_ALL)) bps = BAUD_DEFAULT;
    if (bps) bus->port.set_baud(bps);
}

int Servo_Transfer(Servo_Bus_t *bus, uint8_t id, uint8_t cmd, const uint8_t *data, uint8_t len,
                   uint8_t *rx, uint8_t rx_cap, uint8_t *rx_len)
{
    uint8_t  f[SERVO_FRAME_MAX], b, n;
    uint16_t flen;
    uint32_t t0;
    const uint8_t bcast = (uint8_t)(id == SERVO_BROADCAST_ID);

    if (rx_len) *rx_len = 0U;
    if (bus == 0 || bus->port.write == 0 || bus->port.read_byte == 0 || bus->port.millis == 0) return SERVO_ERR_ARG;
    if ((cmd == SERVO_CMD_READ || cmd == SERVO_CMD_WRITE) && len == 0U) return SERVO_ERR_ARG;
    if (bcast && (cmd == SERVO_CMD_STATE || cmd == SERVO_CMD_READ || cmd == SERVO_CMD_RESET_ALL
                  || (cmd == SERVO_CMD_WRITE && (data[0] == ADDR_ID
                      || (data[0] == SERVO_ADDR_SYSTEM && len == 2U && data[1] == SYS_RESET_ALL)))))
        return SERVO_ERR_BCAST;
    flen = Servo_BuildFrame(f, id, cmd, data, len);
    if (flen == 0U) return SERVO_ERR_ARG;

    Bus_Drain(bus);
    Servo_ParserReset(&bus->parser);
    Bus_Send(bus, f, flen);
    if (bcast)
    {
        Bus_WaitBroadcast(bus);
        Bus_FollowBaud(bus, cmd, data, len);
        return SERVO_OK;
    }

    t0 = bus->port.millis();
    while ((uint32_t)(bus->port.millis() - t0) < bus->timeout_ms)
    {
        if (!bus->port.read_byte(&b)) continue;
        if (!Servo_ParseByte(&bus->parser, b)) continue;
        /* 单线总线会回读自己发的请求帧(CMD 无 0x80)，跳过；只认目标ID的应答 */
        if (bus->parser.buf[2] != id || bus->parser.buf[3] != (uint8_t)(cmd | 0x80U)) continue;
        n = bus->parser.buf[4];
        if (n == 0U) return SERVO_ERR_REPLY;
        if (bus->parser.buf[5] != SERVO_OK) return bus->parser.buf[5];
        n = (uint8_t)(n - 1U);
        if (rx && n)
        {
            if (n > rx_cap) return SERVO_ERR_REPLY;
            memcpy(rx, bus->parser.buf + 6, n);
        }
        if (rx_len) *rx_len = n;
        Bus_FollowBaud(bus, cmd, data, len);
        return SERVO_OK;
    }
    return SERVO_ERR_TIMEOUT;
}

int Servo_ReadParam(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint8_t *val, uint8_t cap, uint8_t *len)
{
    uint8_t buf[SERVO_DATA_MAX], n;
    int r;

    if (len) *len = 0U;
    r = Servo_Transfer(bus, id, SERVO_CMD_READ, &addr, 1U, buf, sizeof(buf), &n);
    if (r != SERVO_OK) return r;
    if (n < 2U || buf[0] != addr || (uint8_t)(n - 1U) > cap) return SERVO_ERR_REPLY;   /* 应答 = 地址 + 值 */
    if (val) memcpy(val, buf + 1, (uint8_t)(n - 1U));
    if (len) *len = (uint8_t)(n - 1U);
    return SERVO_OK;
}

int Servo_WriteParam(Servo_Bus_t *bus, uint8_t id, uint8_t addr, const uint8_t *val, uint8_t len)
{
    uint8_t buf[SERVO_DATA_MAX];

    if (len == 0U || len >= SERVO_DATA_MAX || val == 0) return SERVO_ERR_ARG;
    buf[0] = addr;
    memcpy(buf + 1, val, len);
    return Servo_Transfer(bus, id, SERVO_CMD_WRITE, buf, (uint8_t)(len + 1U), 0, 0U, 0);
}

int Servo_ReadU8(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint8_t *v)
{
    uint8_t n;
    int r = Servo_ReadParam(bus, id, addr, v, 1U, &n);
    return (r == SERVO_OK && n != 1U) ? SERVO_ERR_REPLY : r;
}

int Servo_ReadU16(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint16_t *v)
{
    uint8_t b[2], n;
    int r = Servo_ReadParam(bus, id, addr, b, 2U, &n);

    if (r != SERVO_OK) return r;
    if (n != 2U) return SERVO_ERR_REPLY;
    if (v) *v = (uint16_t)(b[0] | (uint16_t)b[1] << 8);
    return SERVO_OK;
}

int Servo_WriteU8(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint8_t v)
{
    return Servo_WriteParam(bus, id, addr, &v, 1U);
}

int Servo_WriteU16(Servo_Bus_t *bus, uint8_t id, uint8_t addr, uint16_t v)
{
    uint8_t b[2];
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8);
    return Servo_WriteParam(bus, id, addr, b, 2U);
}

/* ============================== 3. 运动控制 ============================== */

int Servo_Move(Servo_Bus_t *bus, uint8_t id, uint16_t pos, uint16_t time_ms)
{
    uint8_t b[4];

    if (pos > SERVO_POS_MAX) return SERVO_ERR_ARG;
    b[0] = (uint8_t)pos; b[1] = (uint8_t)(pos >> 8);
    b[2] = (uint8_t)time_ms; b[3] = (uint8_t)(time_ms >> 8);
    return Servo_WriteParam(bus, id, SERVO_ADDR_MOVE, b, 4U);
}

int Servo_MoveUs(Servo_Bus_t *bus, uint8_t id, uint16_t us, uint16_t time_ms)
{
    if (us < 500U || us > 2500U) return SERVO_ERR_ARG;
    return Servo_Move(bus, id, SERVO_US_TO_POS(us), time_ms);
}

int Servo_MoveTurns(Servo_Bus_t *bus, uint8_t id, uint16_t pos, uint8_t turns, uint16_t turn_ms)
{
    uint8_t b[5];

    if (pos > SERVO_POS_MAX || turns > 9U) return SERVO_ERR_ARG;
    b[0] = (uint8_t)pos; b[1] = (uint8_t)(pos >> 8);
    b[2] = turns;
    b[3] = (uint8_t)turn_ms; b[4] = (uint8_t)(turn_ms >> 8);
    return Servo_WriteParam(bus, id, SERVO_ADDR_MOVE_TURNS, b, 5U);
}

int Servo_Resume(Servo_Bus_t *bus, uint8_t id) { return Servo_WriteU8(bus, id, SERVO_ADDR_MOTION, 1U); }
int Servo_Pause(Servo_Bus_t *bus, uint8_t id)  { return Servo_WriteU8(bus, id, SERVO_ADDR_MOTION, 2U); }
int Servo_Stop(Servo_Bus_t *bus, uint8_t id)   { return Servo_WriteU8(bus, id, SERVO_ADDR_MOTION, 3U); }

int Servo_SetTorqueMode(Servo_Bus_t *bus, uint8_t id, Servo_TorqueMode_t mode)
{
    if ((unsigned)mode > SERVO_TORQUE_DAMPING) return SERVO_ERR_ARG;
    return Servo_WriteU8(bus, id, SERVO_ADDR_TORQUE_MODE, (uint8_t)mode);
}

int Servo_SetDamping(Servo_Bus_t *bus, uint8_t id, uint8_t percent)
{
    if (percent > 100U) return SERVO_ERR_ARG;
    return Servo_WriteU8(bus, id, SERVO_ADDR_DAMPING, percent);
}

/* ============================== 4. 状态读取 ============================== */

int Servo_GetState(Servo_Bus_t *bus, uint8_t id, uint8_t *work_state, uint8_t *fault_lo)
{
    uint8_t b[2], n;
    int r = Servo_Transfer(bus, id, SERVO_CMD_STATE, 0, 0U, b, sizeof(b), &n);

    if (r != SERVO_OK) return r;
    if (n != 2U) return SERVO_ERR_REPLY;
    if (work_state) *work_state = b[0];
    if (fault_lo) *fault_lo = b[1];
    return SERVO_OK;
}

static uint16_t Le16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }

int Servo_ReadTelemetry(Servo_Bus_t *bus, uint8_t id, Servo_Telemetry_t *t)
{
    uint8_t b[14], n;
    int r = Servo_ReadParam(bus, id, SERVO_ADDR_TELEMETRY, b, sizeof(b), &n);

    if (r != SERVO_OK) return r;
    if (n != 14U || t == 0) return SERVO_ERR_REPLY;
    t->target      = Le16(b);
    t->position    = Le16(b + 2);
    t->torque      = (int16_t)Le16(b + 4);
    t->speed       = (int16_t)Le16(b + 6);
    t->current     = (int16_t)Le16(b + 8);
    t->temperature = (int16_t)Le16(b + 10);
    t->voltage     = Le16(b + 12);
    return SERVO_OK;
}

int Servo_ReadPosition(Servo_Bus_t *bus, uint8_t id, uint16_t *pos)     { return Servo_ReadU16(bus, id, SERVO_ADDR_POSITION, pos); }
int Servo_ReadSpeed(Servo_Bus_t *bus, uint8_t id, int16_t *speed)       { return Servo_ReadU16(bus, id, SERVO_ADDR_SPEED, (uint16_t *)speed); }
int Servo_ReadTemperature(Servo_Bus_t *bus, uint8_t id, int16_t *temp)  { return Servo_ReadU16(bus, id, SERVO_ADDR_TEMPERATURE, (uint16_t *)temp); }
int Servo_ReadVoltage(Servo_Bus_t *bus, uint8_t id, uint16_t *cv)       { return Servo_ReadU16(bus, id, SERVO_ADDR_VOLTAGE, cv); }
int Servo_ReadCurrent(Servo_Bus_t *bus, uint8_t id, int16_t *ma)        { return Servo_ReadU16(bus, id, SERVO_ADDR_CURRENT, (uint16_t *)ma); }
int Servo_ReadTorque(Servo_Bus_t *bus, uint8_t id, int16_t *pct)        { return Servo_ReadU16(bus, id, SERVO_ADDR_TORQUE, (uint16_t *)pct); }
int Servo_ReadFault(Servo_Bus_t *bus, uint8_t id, uint16_t *fault)      { return Servo_ReadU16(bus, id, SERVO_ADDR_FAULT, fault); }
int Servo_ReadStalled(Servo_Bus_t *bus, uint8_t id, uint8_t *stalled)   { return Servo_ReadU8(bus, id, SERVO_ADDR_STALLED, stalled); }

/* ============================== 5. 设备信息 ============================== */

int Servo_ReadInfo(Servo_Bus_t *bus, uint8_t id, Servo_Info_t *info)
{
    uint8_t n;
    int r;

    if (info == 0) return SERVO_ERR_ARG;
    if ((r = Servo_ReadU8(bus, id, SERVO_ADDR_HW_TYPE, &info->type)) != SERVO_OK) return r;
    if ((r = Servo_ReadU8(bus, id, SERVO_ADDR_HW_VERSION, &info->hw_version)) != SERVO_OK) return r;
    if ((r = Servo_ReadU8(bus, id, SERVO_ADDR_HW_VOLTAGE, &info->voltage)) != SERVO_OK) return r;
    if ((r = Servo_ReadU8(bus, id, SERVO_ADDR_HW_TORQUE, &info->torque_kg)) != SERVO_OK) return r;
    if ((r = Servo_ReadU16(bus, id, SERVO_ADDR_HW_VMAX, &info->vmax)) != SERVO_OK) return r;
    if ((r = Servo_ReadU16(bus, id, SERVO_ADDR_HW_AMAX, &info->amax)) != SERVO_OK) return r;
    if ((r = Servo_ReadU16(bus, id, SERVO_ADDR_HW_DMAX, &info->dmax)) != SERVO_OK) return r;
    if ((r = Servo_ReadU8(bus, id, SERVO_ADDR_HW_SMOOTH, &info->smooth_ms)) != SERVO_OK) return r;
    if ((r = Servo_ReadU8(bus, id, SERVO_ADDR_HW_MIN_STEP, &info->min_step)) != SERVO_OK) return r;
    if ((r = Servo_ReadParam(bus, id, SERVO_ADDR_FW_VERSION, info->fw, 3U, &n)) != SERVO_OK) return r;
    return (n == 3U) ? SERVO_OK : SERVO_ERR_REPLY;
}

/* ============================== 6. 参数设置 ============================== */

typedef struct {
    uint8_t  flash;    /* Flash 地址 */
    uint8_t  ram;      /* 对应 RAM 地址，0 = 没有 */
    uint8_t  size;     /* 字节数 */
    uint16_t min, max; /* 固定范围预检(随型号的上限交给舵机) */
} ParamDef_t;

static const ParamDef_t k_params[] = {
    { 0x20U, 0x00U, 1U,   0U,   254U },   /* 舵机ID */
    { 0x21U, 0x00U, 1U,   1U,     8U },   /* 波特率档位 */
    { 0x22U, 0x00U, 1U,   1U,    12U },   /* 工作模式 */
    { 0x23U, 0x00U, 1U,   1U,     3U },   /* 上电动作 */
    { 0x24U, 0x84U, 1U,   0U,   100U },   /* 扭矩上限 % */
    { 0x25U, 0x00U, 2U,   0U,  4095U },   /* 上电位置 位置码 */
    { 0x30U, 0x85U, 2U,   0U, 65535U },   /* 最大速度 °/s(上限[04]) */
    { 0x31U, 0x86U, 2U,   0U, 65535U },   /* 最大加速度 °/s²(上限[05]) */
    { 0x32U, 0x87U, 2U,   0U, 65535U },   /* 最大减速度 °/s²(上限[06]) */
    { 0x33U, 0x88U, 1U,   0U,   200U },   /* S曲线平滑时间 ms */
    { 0x34U, 0x89U, 2U,   0U,   500U },   /* 最短运动时间 ms */
    { 0x35U, 0x8AU, 1U,   0U,   100U },   /* 保持比例P */
    { 0x37U, 0x8EU, 2U,   0U,   500U },   /* 到位精度 0.01°(下限[08]×1.5) */
    { 0x38U, 0x8FU, 1U,   0U,   100U },   /* 保持积分I */
    { 0x40U, 0x00U, 2U, 500U,  5000U },   /* 过流阈值 mA */
    { 0x41U, 0x00U, 2U,  10U,  1000U },   /* 过流持续时间 ms */
    { 0x42U, 0x00U, 2U, 300U,  1000U },   /* 过温阈值 0.1℃ */
    { 0x43U, 0x00U, 2U, 250U, 10000U },   /* 过温持续时间 ms */
    { 0x44U, 0x00U, 2U,  10U, 65535U },   /* 堵转位移阈值(上限[04]/2) */
    { 0x45U, 0x00U, 1U,   1U,   100U },   /* 堵转连续次数 */
    { 0x46U, 0x00U, 1U,   0U,    15U },   /* 保护开关位掩码 */
    { 0x47U, 0x00U, 1U,  60U,   130U },   /* 最低工作电压 0.1V(范围随型号) */
    { 0x48U, 0x00U, 1U,  60U,   130U },   /* 最高工作电压 0.1V(范围随型号) */
    { 0x49U, 0x00U, 1U,   0U,   100U },   /* 最小启动扭矩 % */
};

static const ParamDef_t *Param_Find(Servo_Param_t p)
{
    uint8_t i;

    for (i = 0U; i < sizeof(k_params) / sizeof(k_params[0]); i++)
        if (k_params[i].flash == (uint8_t)p) return &k_params[i];
    return 0;
}

/* 按 store 选地址；没有 RAM 地址的参数选 RAM 返回0 */
static uint8_t Param_Addr(const ParamDef_t *d, Servo_Store_t store)
{
    return (store == SERVO_STORE_RAM) ? d->ram : d->flash;
}

int Servo_SetParam(Servo_Bus_t *bus, uint8_t id, Servo_Param_t p, uint16_t value, Servo_Store_t store)
{
    const ParamDef_t *d = Param_Find(p);
    uint8_t addr;

    if (d == 0 || (addr = Param_Addr(d, store)) == 0U) return SERVO_ERR_ARG;
    if (value < d->min || value > d->max) return SERVO_ERR_ARG;
    return (d->size == 1U) ? Servo_WriteU8(bus, id, addr, (uint8_t)value) : Servo_WriteU16(bus, id, addr, value);
}

int Servo_GetParam(Servo_Bus_t *bus, uint8_t id, Servo_Param_t p, uint16_t *value, Servo_Store_t store)
{
    const ParamDef_t *d = Param_Find(p);
    uint8_t addr, v8;
    int r;

    if (d == 0 || value == 0 || (addr = Param_Addr(d, store)) == 0U) return SERVO_ERR_ARG;
    if (d->size == 2U) return Servo_ReadU16(bus, id, addr, value);
    r = Servo_ReadU8(bus, id, addr, &v8);
    if (r == SERVO_OK) *value = v8;
    return r;
}

int Servo_SetVoltageRange(Servo_Bus_t *bus, uint8_t id, uint8_t min_dv, uint8_t max_dv)
{
    uint8_t cur_max;
    int r;

    if (min_dv >= max_dv) return SERVO_ERR_ARG;
    if (id == SERVO_BROADCAST_ID) cur_max = 0U;   /* 广播读不了，按"先写上限"处理 */
    else if ((r = Servo_ReadU8(bus, id, SERVO_P_VOLTAGE_MAX, &cur_max)) != SERVO_OK) return r;
    /* 舵机按"新值 + 另一端当前值"成对校验：上限升高先写上限，否则先写下限，中间态始终是更宽的区间 */
    if (max_dv > cur_max)
    {
        if ((r = Servo_SetParam(bus, id, SERVO_P_VOLTAGE_MAX, max_dv, SERVO_STORE_FLASH)) != SERVO_OK) return r;
        return Servo_SetParam(bus, id, SERVO_P_VOLTAGE_MIN, min_dv, SERVO_STORE_FLASH);
    }
    if ((r = Servo_SetParam(bus, id, SERVO_P_VOLTAGE_MIN, min_dv, SERVO_STORE_FLASH)) != SERVO_OK) return r;
    return Servo_SetParam(bus, id, SERVO_P_VOLTAGE_MAX, max_dv, SERVO_STORE_FLASH);
}

/* ============================== 7. 校准与系统动作 ============================== */

int Servo_Calibrate(Servo_Bus_t *bus, uint8_t id, Servo_Calib_t act)
{
    if ((unsigned)act < SERVO_CALIB_MID || (unsigned)act > SERVO_CALIB_SAVE_STARTUP) return SERVO_ERR_ARG;
    return Servo_WriteU8(bus, id, SERVO_ADDR_CALIB, (uint8_t)act);
}

int Servo_TrimMid(Servo_Bus_t *bus, uint8_t id, int16_t us)
{
    if (us < -999 || us > 999) return SERVO_ERR_ARG;
    return Servo_WriteU16(bus, id, SERVO_ADDR_CAL_TRIM, (uint16_t)us);
}

int Servo_SetSlave(Servo_Bus_t *bus, uint8_t id, uint8_t slave)
{
    uint8_t  f[SERVO_FRAME_MAX];
    uint16_t n;

    if (slave > 1U) return SERVO_ERR_ARG;
    if (!slave) return Servo_Transfer(bus, id, SERVO_CMD_SLAVE, &slave, 1U, 0, 0U, 0);
    if (bus == 0 || bus->port.write == 0) return SERVO_ERR_ARG;
    n = Servo_BuildFrame(f, id, SERVO_CMD_SLAVE, &slave, 1U);
    if (n == 0U) return SERVO_ERR_ARG;
    Bus_Send(bus, f, n);   /* 置从后舵机连这条也不应答：只发不等 */
    return SERVO_OK;
}

int Servo_ReloadRamParams(Servo_Bus_t *bus, uint8_t id)
{
    return Servo_WriteU8(bus, id, SERVO_ADDR_SYSTEM, 1U);
}

int Servo_FactoryReset(Servo_Bus_t *bus, uint8_t id, uint8_t keep_id)
{
    return Servo_Transfer(bus, id, keep_id ? SERVO_CMD_RESET_KEEP : SERVO_CMD_RESET_ALL, 0, 0U, 0, 0U, 0);
}
