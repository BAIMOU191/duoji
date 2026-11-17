/* A_Param.c —— 虚拟寄存器参数表：一张表管读、写校验、写后联动与上电修复；有联动的少数地址在 Param_WriteSpecial。
 * 参数表单位与内部单位的换算(位置码↔Q3、厘度/s↔0.1°/s)只在这里做。 */

#include "A_Param.h"
#include "A_Protect.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include "C_Fixed.h"
#include "iap.h"
#include <stddef.h>
#include <string.h>

_Static_assert(PARAM_POS_TO_Q3(0U) == Q3_MIN && PARAM_POS_TO_Q3(PARAM_POS_MAX) == Q3_MAX,
               "位置码端点须精确落在脉宽量程端点");
_Static_assert(Q3_TO_PARAM_POS(Q3_MIN) == 0U && Q3_TO_PARAM_POS(Q3_MAX) == PARAM_POS_MAX, "回程端点同理");
_Static_assert((PROT_ENABLE_IMPLEMENTED & (PROT_ENABLE_IMPLEMENTED + 1U)) == 0U,
               "保护开关位须从bit0连续，才能按 0~上限 校验");
_Static_assert(PARAM_TELEMETRY_BYTES <= PARAM_VALUE_MAX, "集合状态超出参数值缓冲");
_Static_assert(ENCODER_TRAVEL_LO == 0, "校准锚点按无符号存放，可转入区须从0起");

/* 存放位置 */
enum {
    P_CFG = 0,  /* Config_t 字段，只有 Flash 份 */
    P_RUN_F,    /* RunParams_t 字段的 Flash 地址：读 Flash 份，写 Flash+SRAM 两份 */
    P_RUN_R,    /* RunParams_t 字段的 SRAM 地址：只读写 SRAM 份 */
    P_RAM       /* RuntimeConfig_t 独有字段，只有 SRAM 份 */
};

#define PF_TRAJ    0x01U  /* 写后让轨迹参数生效(下一次规划) */
#define PF_PROT    0x02U  /* 写后保护模块立即重算 */
#define PF_BAUD    0x04U  /* 写后切波特率(协议层等应答发完) */
#define PF_ZERO    0x08U  /* 0 另外允许(表示自动/关闭)，其余须在 [min,max] */
#define PF_SPECIAL 0x10U  /* 写入与修复不走通用流程：见 Param_WriteSpecial / A_Param_Repair */

typedef struct {
    uint8_t  addr;
    uint8_t  where;    /* P_* */
    uint8_t  size;     /* 线上字节数 = 字段宽度 */
    uint8_t  flags;    /* PF_* */
    uint16_t off;      /* 字段偏移 */
    uint16_t min, max;
} ParamDef_t;

#define FSIZE(T, f) ((uint8_t)sizeof(((T *)0)->f))
#define P_CFG_(a, f, fl, lo, hi) { (a), P_CFG,   FSIZE(Config_t, f),        (fl), (uint16_t)offsetof(Config_t, f),        (lo), (hi) }
#define P_RUNF(a, f, fl, lo, hi) { (a), P_RUN_F, FSIZE(RunParams_t, f),     (fl), (uint16_t)offsetof(RunParams_t, f),     (lo), (hi) }
#define P_RUNR(a, f, fl, lo, hi) { (a), P_RUN_R, FSIZE(RunParams_t, f),     (fl), (uint16_t)offsetof(RunParams_t, f),     (lo), (hi) }
#define P_RAM_(a, f, fl, lo, hi) { (a), P_RAM,   FSIZE(RuntimeConfig_t, f), (fl), (uint16_t)offsetof(RuntimeConfig_t, f), (lo), (hi) }

static const ParamDef_t k_params[] = {
    /* ---- Flash 参数 ---- */
    P_CFG_(PARAM_SERVO_ID,             servo_id,                 0,                 0U, CFG_ID_MAX),
    P_CFG_(PARAM_BAUD_CODE,            baud_code,                PF_BAUD,           CFG_BAUD_MIN, CFG_BAUD_MAX),
    P_CFG_(PARAM_SERVO_MODE,           servo_mode,               PF_SPECIAL,        SERVO_MODE_270_CW, SERVO_MODE_CUSTOM),
    P_CFG_(PARAM_BOOT_MODE,            boot_mode,                0,                 SERVO_BOOT_GOTO_START, SERVO_BOOT_RELEASE),
    P_RUNF(PARAM_TORQUE_LIMIT,         torque_limit_pct,         PF_PROT,           0U, TORQUE_MAX_PERCENT),
    P_RUNF(PARAM_TRAJ_VMAX,            traj_vmax_dps,            PF_TRAJ,           0U, CFG_TRAJ_VMAX_MAX_DPS),
    P_RUNF(PARAM_TRAJ_AMAX,            traj_amax_dps2,           PF_TRAJ,           0U, CFG_TRAJ_AMAX_MAX_DPS2),
    P_RUNF(PARAM_TRAJ_DMAX,            traj_dmax_dps2,           PF_TRAJ,           0U, CFG_TRAJ_DMAX_MAX_DPS2),
    P_RUNF(PARAM_TRAJ_SMOOTH,          traj_smooth_ms,           PF_TRAJ | PF_ZERO, CFG_SMOOTH_MIN_MS, CFG_SMOOTH_MAX_MS),
    P_RUNF(PARAM_MOVE_MIN,             traj_move_min_ms,         PF_TRAJ,           0U, CFG_MOVE_MIN_MAX_MS),
    P_RUNF(PARAM_HOLD_KP,              hold_kp,                  0,                 0U, CFG_HOLD_KP_MAX),
    P_RUNF(PARAM_RESOLUTION,           position_resolution_cdeg, 0,                 CFG_RESOLUTION_MIN_CDEG, CFG_RESOLUTION_MAX_CDEG),
    P_RUNF(PARAM_HOLD_KI,              hold_ki,                  0,                 0U, CFG_HOLD_KI_MAX),
    P_RUNF(PARAM_PROT_CURRENT_LIMIT,   prot_current_limit_ma,    PF_PROT,           PROT_CURRENT_LIMIT_MIN_MA, PROT_CURRENT_LIMIT_MAX_MA),
    P_RUNF(PARAM_PROT_CURRENT_TIME,    prot_current_time_ms,     PF_PROT,           PROT_CURRENT_TIME_MIN_MS, PROT_CURRENT_TIME_MAX_MS),
    P_RUNF(PARAM_PROT_TEMP_LIMIT,      prot_temp_limit_dC,       PF_PROT,           PROT_TEMP_LIMIT_MIN_DC, PROT_TEMP_LIMIT_MAX_DC),
    P_RUNF(PARAM_PROT_TEMP_TIME,       prot_temp_time_ms,        PF_PROT,           PROT_TEMP_TIME_MIN_MS, PROT_TEMP_TIME_MAX_MS),
    P_RUNF(PARAM_PROT_STALL_THRESHOLD, prot_stall_act_cdeg,      PF_PROT,           PROT_STALL_THRESHOLD_MIN_CDEG, PROT_STALL_THRESHOLD_MAX_CDEG),
    P_RUNF(PARAM_PROT_STALL_TRIP,      prot_stall_trip,          PF_PROT,           PROT_STALL_TRIP_MIN, PROT_STALL_TRIP_MAX),
    P_RUNF(PARAM_PROT_ENABLE,          protect_enable,           0,                 0U, PROT_ENABLE_IMPLEMENTED),
    P_RUNF(PARAM_VOLTAGE_MIN,          voltage_min_dv,           PF_SPECIAL,        PROT_VOLTAGE_CFG_MIN_DV, PROT_VOLTAGE_CFG_MAX_DV),
    P_RUNF(PARAM_VOLTAGE_MAX,          voltage_max_dv,           PF_SPECIAL,        PROT_VOLTAGE_CFG_MIN_DV, PROT_VOLTAGE_CFG_MAX_DV),
    P_RUNF(PARAM_MIN_START_TORQUE,     min_start_torque_pct,     0,                 0U, TORQUE_MAX_PERCENT),
    P_RUNF(PARAM_LOAD_RATIO,           load_ratio_x10,           0,                 0U, CFG_LOAD_RATIO_MAX_X10),
    P_RUNF(PARAM_LOAD_LEARN,           load_learn,               0,                 0U, 1U),
    /* ---- SRAM 参数 ---- */
    P_RAM_(PARAM_TORQUE_MODE,          torque_mode,              PF_SPECIAL,        0U, 2U),
    P_RAM_(PARAM_DAMPING,              damping_percent,          PF_SPECIAL,        0U, 100U),
    P_RUNR(PARAM_RT_TORQUE_LIMIT,      torque_limit_pct,         PF_PROT,           0U, TORQUE_MAX_PERCENT),
    P_RUNR(PARAM_RT_TRAJ_VMAX,         traj_vmax_dps,            PF_TRAJ,           0U, CFG_TRAJ_VMAX_MAX_DPS),
    P_RUNR(PARAM_RT_TRAJ_AMAX,         traj_amax_dps2,           PF_TRAJ,           0U, CFG_TRAJ_AMAX_MAX_DPS2),
    P_RUNR(PARAM_RT_TRAJ_DMAX,         traj_dmax_dps2,           PF_TRAJ,           0U, CFG_TRAJ_DMAX_MAX_DPS2),
    P_RUNR(PARAM_RT_TRAJ_SMOOTH,       traj_smooth_ms,           PF_TRAJ | PF_ZERO, CFG_SMOOTH_MIN_MS, CFG_SMOOTH_MAX_MS),
    P_RUNR(PARAM_RT_MOVE_MIN,          traj_move_min_ms,         PF_TRAJ,           0U, CFG_MOVE_MIN_MAX_MS),
    P_RUNR(PARAM_RT_HOLD_KP,           hold_kp,                  0,                 0U, CFG_HOLD_KP_MAX),
    P_RUNR(PARAM_RT_RESOLUTION,        position_resolution_cdeg, 0,                 CFG_RESOLUTION_MIN_CDEG, CFG_RESOLUTION_MAX_CDEG),
    P_RUNR(PARAM_RT_HOLD_KI,           hold_ki,                  0,                 0U, CFG_HOLD_KI_MAX),
    P_RUNR(PARAM_RT_LOAD_RATIO,        load_ratio_x10,           0,                 0U, CFG_LOAD_RATIO_MAX_X10),
    P_RUNR(PARAM_RT_LOAD_LEARN,        load_learn,               0,                 0U, 1U),
};

/* 参数表上限写在2字节里，型号额定能力也必须放得进去 */
_Static_assert(CFG_TRAJ_VMAX_MAX_DPS <= UINT16_MAX && CFG_TRAJ_AMAX_MAX_DPS2 <= UINT16_MAX
               && CFG_TRAJ_DMAX_MAX_DPS2 <= UINT16_MAX, "额定能力超出参数表2字节");
_Static_assert(CFG_SMOOTH_MAX_MS <= UINT8_MAX && CFG_HOLD_KP_MAX <= UINT8_MAX && CFG_HOLD_KI_MAX <= UINT8_MAX
               && PROT_STALL_TRIP_MAX <= UINT8_MAX && PROT_VOLTAGE_CFG_MAX_DV <= UINT8_MAX, "1字节参数的上限超过255");

static const ParamDef_t *Param_Find(uint8_t addr)
{
    uint8_t i;

    for (i = 0U; i < sizeof(k_params) / sizeof(k_params[0]); i++)
        if (k_params[i].addr == addr) return &k_params[i];
    return 0;
}

/* 字段地址；cfg 为 Flash 份所在的 Config_t(修复时指向候选配置) */
static uint8_t *Param_Field(const ParamDef_t *d, Config_t *cfg)
{
    switch (d->where)
    {
    case P_CFG:   return (uint8_t *)cfg + d->off;
    case P_RUN_F: return (uint8_t *)&cfg->run + d->off;
    case P_RUN_R: return (uint8_t *)&g_runtime_config.run + d->off;
    default:      return (uint8_t *)&g_runtime_config + d->off;
    }
}

/* 偏移来自 offsetof，天然对齐 */
static uint32_t Param_Load(const uint8_t *p, uint8_t size) { return (size == 1U) ? *p : *(const uint16_t *)p; }
static void Param_Store(uint8_t *p, uint8_t size, uint32_t v)
{
    if (size == 1U) *p = (uint8_t)v;
    else            *(uint16_t *)p = (uint16_t)v;
}

static uint8_t Param_InRange(const ParamDef_t *d, uint32_t v)
{
    return (uint8_t)((v >= d->min || ((d->flags & PF_ZERO) && v == 0U)) && v <= d->max);
}

/* 小端写出，返回字节数 */
static uint8_t Param_Put(uint8_t *out, uint32_t v, uint8_t size)
{
    uint8_t i;

    for (i = 0U; i < size; i++) out[i] = (uint8_t)(v >> (8U * i));
    return size;
}

uint8_t A_Param_VoltageRangeValid(uint16_t lo_dv, uint16_t hi_dv)
{
    return (uint8_t)(lo_dv >= PROT_VOLTAGE_CFG_MIN_DV && hi_dv <= PROT_VOLTAGE_CFG_MAX_DV
                     && lo_dv + 2U * PROT_VOLTAGE_HYST_DV < hi_dv);
}

/* ============================== 实时状态 ============================== */

/* 速度：内部厘度/s(观测器缓存，不重读位置) → 0.1°/s，四舍五入；观测器不在更新报0 */
static int16_t Param_Speed(void)
{
    ServoTelemetry_t s;
    uint32_t mag;

    A_Servo_GetTelemetry(&s);
    if (s.speed_cdps == INT32_MIN) return 0;
    mag = Fx_UDivR((uint32_t)Fx_Abs32(s.speed_cdps) + 5U, FX_RECIP(10U));
    if (mag > 32767U) mag = 32767U;
    return (int16_t)((s.speed_cdps < 0) ? -(int32_t)mag : (int32_t)mag);
}

/* 扭矩：最近电流 + 是否采在驱动段，与集合状态同一口径 */
static int16_t Param_Torque(void)
{
    ProtectTelemetry_t p;

    A_Protect_GetTelemetry(&p);
    return A_Torque_Estimate(p.current_ma, p.current_valid);
}

/* 温度：读失败已保留上次值；内部 -32768 只剩上电后还没读成功过，对外报0 */
static int16_t Param_Temperature(int16_t temp_dc)
{
    return (temp_dc == (int16_t)-32768) ? 0 : temp_dc;
}

/* 七项实时状态，顺序与宽度是协议；扭矩、速度、电流带符号，位置值增大方向为正 */
static void Param_Telemetry(uint8_t *out)
{
    ServoTelemetry_t s;
    ProtectTelemetry_t p;

    A_Servo_GetTelemetry(&s);
    A_Protect_GetTelemetry(&p);
    (void)Param_Put(out,      Q3_TO_PARAM_POS(s.target_q3), 2U);
    (void)Param_Put(out + 2,  Q3_TO_PARAM_POS(s.position_q3), 2U);
    (void)Param_Put(out + 4,  (uint16_t)A_Torque_Estimate(p.current_ma, p.current_valid), 2U);
    (void)Param_Put(out + 6,  (uint16_t)Param_Speed(), 2U);
    (void)Param_Put(out + 8,  (uint16_t)p.current_ma, 2U);
    (void)Param_Put(out + 10, (uint16_t)Param_Temperature(p.temperature_dc), 2U);
    (void)Param_Put(out + 12, p.voltage_cv, 2U);
}

/* ============================== 读 ============================== */

uint8_t A_Param_Read(uint8_t addr, uint8_t *out, uint8_t *len)
{
    const ParamDef_t *d = Param_Find(addr);
    IAP_HwInfo_t hw;
    uint8_t n;

    if (d != 0)
    {
        *len = Param_Put(out, Param_Load(Param_Field(d, &g_config), d->size), d->size);
        return PARAM_ST_OK;
    }
    switch (addr)
    {
    /* 速度/加减速报标定值(标定电压下的能力)：与用户可设上限同源，不读 IAP 区 */
    case PARAM_HW_VMAX: n = Param_Put(out, MOTOR_RATED_VEL_DPS, 2U);  break;
    case PARAM_HW_AMAX: n = Param_Put(out, MOTOR_RATED_ACC_DPS2, 2U); break;
    case PARAM_HW_DMAX: n = Param_Put(out, MOTOR_RATED_DEC_DPS2, 2U); break;
    case PARAM_HW_TYPE: case PARAM_HW_VERSION: case PARAM_HW_VOLTAGE: case PARAM_HW_TORQUE:
    case PARAM_HW_SMOOTH: case PARAM_HW_MIN_STEP:
        if (!IAP_HwInfoRead(&hw)) return PARAM_ST_FAILED;
        n = Param_Put(out, (addr == PARAM_HW_TYPE)    ? hw.servo_type
                         : (addr == PARAM_HW_VERSION) ? hw.hw_version
                         : (addr == PARAM_HW_VOLTAGE) ? hw.voltage
                         : (addr == PARAM_HW_TORQUE)  ? hw.torque
                         : (addr == PARAM_HW_SMOOTH)  ? (uint8_t)hw.smooth_ms
                         :                              (uint8_t)hw.min_step_cdeg, 1U);
        break;
    case PARAM_FW_VERSION:
        out[0] = SERVO_VERSION_MAJOR; out[1] = SERVO_VERSION_MINOR; out[2] = SERVO_VERSION_PATCH; n = 3U; break;
    case PARAM_STARTUP_POS:  n = Param_Put(out, Q3_TO_PARAM_POS(g_config.startup_q3), 2U);       break;
    case PARAM_WORK_STATE:   n = Param_Put(out, A_Servo_GetWorkState(), 1U);                     break;
    case PARAM_FAULT:        n = Param_Put(out, A_Protect_Fault(), 2U);                          break;
    case PARAM_POSITION:     n = Param_Put(out, Q3_TO_PARAM_POS(A_Servo_GetPositionQ3()), 2U);  break;
    case PARAM_SPEED:        n = Param_Put(out, (uint16_t)Param_Speed(), 2U);                    break;
    case PARAM_TEMPERATURE:  n = Param_Put(out, (uint16_t)Param_Temperature(A_Protect_GetTemperature()), 2U);     break;
    case PARAM_VOLTAGE:      n = Param_Put(out, A_Voltage_Read(), 2U);                           break;
    case PARAM_CURRENT:      n = Param_Put(out, (uint16_t)A_Protect_GetCurrent(), 2U);           break;
    case PARAM_TORQUE:       n = Param_Put(out, (uint16_t)Param_Torque(), 2U);                   break;
    case PARAM_STALLED:      n = Param_Put(out, A_Protect_Stalled(), 1U);                        break;   /* 1=堵转保持中 */
    case PARAM_TELEMETRY:    Param_Telemetry(out); n = PARAM_TELEMETRY_BYTES;                    break;
    default: return PARAM_ST_VALUE;
    }
    *len = n;
    return PARAM_ST_OK;
}

/* ============================== 写 ============================== */

static uint8_t Cal_Mid(void)        { return A_Servo_CalibrateMid(); }
static uint8_t Cal_Zero(void)       { return A_Servo_CalibrateZero(); }
static uint8_t Cal_TravelMin(void)  { return A_Servo_SetTravelEnd(1U); }
static uint8_t Cal_TravelMax(void)  { return A_Servo_SetTravelEnd(0U); }
static uint8_t Cal_PulseMin(void)   { return A_Servo_SetPulseLimit(1U); }
static uint8_t Cal_PulseMax(void)   { return A_Servo_SetPulseLimit(0U); }
static uint8_t Cal_SaveStart(void)  { return A_Servo_SaveStartup(); }

/* 有联动或要调用其它模块的地址；返回 0xFF 表示不是特殊地址，走通用流程 */
static uint8_t Param_WriteSpecial(uint8_t addr, uint32_t v, uint8_t *fx)
{
    static uint8_t (*const k_cal[])(void) = {   /* 8B：1=SCK 2=SCZ 3=AMI 4=AMX 5=SMI 6=SMX 7=CSD */
        Cal_Mid, Cal_Zero, Cal_TravelMin, Cal_TravelMax, Cal_PulseMin, Cal_PulseMax, Cal_SaveStart
    };
    uint8_t lo, hi;

    switch (addr)
    {
    case PARAM_SERVO_MODE:
        return A_Servo_SetMode((uint8_t)((v > 0xFFU) ? 0U : v)) ? PARAM_ST_OK : PARAM_ST_VALUE;
    case PARAM_STARTUP_POS:   /* 位置码存成 Q3 */
        if (v > PARAM_POS_MAX) return PARAM_ST_VALUE;
        g_config.startup_q3 = PARAM_POS_TO_Q3(v);
        A_Config_MarkDirty();
        return PARAM_ST_OK;
    case PARAM_VOLTAGE_MIN:
    case PARAM_VOLTAGE_MAX:   /* 上下限成对校验：各留回差后仍有恢复区间 */
        lo = (addr == PARAM_VOLTAGE_MIN) ? (uint8_t)v : g_config.run.voltage_min_dv;
        hi = (addr == PARAM_VOLTAGE_MAX) ? (uint8_t)v : g_config.run.voltage_max_dv;
        if (v > 0xFFU || !A_Param_VoltageRangeValid(lo, hi)) return PARAM_ST_VALUE;
        g_config.run.voltage_min_dv = g_runtime_config.run.voltage_min_dv = lo;
        g_config.run.voltage_max_dv = g_runtime_config.run.voltage_max_dv = hi;
        A_Config_MarkDirty();
        return PARAM_ST_OK;
    case PARAM_TORQUE_MODE:
        if (v > 2U) return PARAM_ST_VALUE;
        return A_Servo_SetTorqueMode((uint8_t)v) ? PARAM_ST_OK : PARAM_ST_STATE;
    case PARAM_DAMPING:
        return (v <= 100U && A_Servo_SetDamping((uint8_t)v)) ? PARAM_ST_OK : PARAM_ST_VALUE;
    case PARAM_MOTION_ACTION:  /* 1=继续 2=暂停 3=停止 */
        if (v == 1U)      A_Servo_Resume();
        else if (v == 2U) A_Servo_Pause();
        else if (v == 3U) A_Servo_Stop();
        else return PARAM_ST_VALUE;
        return PARAM_ST_OK;
    case PARAM_CAL_ACTION:
        if (v < 1U || v > sizeof(k_cal) / sizeof(k_cal[0])) return PARAM_ST_VALUE;
        return k_cal[v - 1U]() ? PARAM_ST_OK : PARAM_ST_FAILED;
    case PARAM_SYSTEM_ACTION:  /* 1=SRAM参数重载，2=仅保留ID恢复默认，3=全部恢复默认 */
        if (v == 1U) { A_Config_ReloadRuntime(0U); A_Servo_ApplyTrajConfig(); A_Protect_Refresh(); return PARAM_ST_OK; }
        if (v != 2U && v != 3U) return PARAM_ST_VALUE;
        A_Servo_FactoryReset((uint8_t)(v == 2U));
        *fx |= PARAM_FX_BAUD;   /* 默认波特率在应答发完后生效 */
        return PARAM_ST_OK;
    default:
        return 0xFFU;
    }
}

uint8_t A_Param_WriteValue(uint8_t addr, uint32_t value, uint8_t *fx)
{
    const ParamDef_t *d;
    uint8_t st = Param_WriteSpecial(addr, value, fx);

    if (st != 0xFFU) return st;
    if ((d = Param_Find(addr)) == 0) return (addr < 0x20U || addr >= 0x80U) ? PARAM_ST_READONLY : PARAM_ST_VALUE;
    if (!Param_InRange(d, value)) return PARAM_ST_VALUE;
    Param_Store(Param_Field(d, &g_config), d->size, value);
    if (d->where == P_RUN_F) Param_Store((uint8_t *)&g_runtime_config.run + d->off, d->size, value);   /* Flash 地址两份都写 */
    if (d->where == P_CFG || d->where == P_RUN_F) A_Config_MarkDirty();
    if (d->flags & PF_TRAJ) A_Servo_ApplyTrajConfig();
    if (d->flags & PF_PROT) A_Protect_Refresh();
    if (d->flags & PF_BAUD) *fx |= PARAM_FX_BAUD;
    return PARAM_ST_OK;
}

/* 写入的线上字节数；0=不可写 */
static uint8_t Param_WriteSize(uint8_t addr)
{
    const ParamDef_t *d = Param_Find(addr);

    if (d != 0) return d->size;
    switch (addr)
    {
    case PARAM_STARTUP_POS:    return 2U;
    case PARAM_MOTION_REQUEST: return 4U;
    case PARAM_MOTION_ACTION: case PARAM_CAL_ACTION: case PARAM_SYSTEM_ACTION: return 1U;
    default:                   return 0U;
    }
}

uint8_t A_Param_Write(uint8_t addr, const uint8_t *data, uint8_t len, uint8_t *fx)
{
    uint8_t  size = Param_WriteSize(addr);
    uint16_t pos;

    if (size == 0U) return (addr < 0x20U || addr >= 0x80U) ? PARAM_ST_READONLY : PARAM_ST_VALUE;
    if (len != size) return PARAM_ST_LENGTH;
    if (addr == PARAM_MOTION_REQUEST)
    {   /* 位置码2B(0~4095)+时间2B；越界=值非法，故障/编码器失效/行程无效=状态不允许 */
        pos = (uint16_t)(data[0] | (uint16_t)data[1] << 8);
        if (pos > PARAM_POS_MAX) return PARAM_ST_VALUE;   /* 先拦再换算，防溢出 */
        switch (A_Servo_SubmitQ3(PARAM_POS_TO_Q3(pos), (uint16_t)(data[2] | (uint16_t)data[3] << 8)))
        {
        case SERVO_SUBMIT_OK:    return PARAM_ST_OK;
        case SERVO_SUBMIT_VALUE: return PARAM_ST_VALUE;
        default:                 return PARAM_ST_STATE;
        }
    }
    return A_Param_WriteValue(addr, (size == 1U) ? data[0] : (uint32_t)(data[0] | (uint32_t)data[1] << 8), fx);
}

/* ============================== 上电修复 ============================== */

/* 非法字段改回 def 中的值；坐标与限位、成对的电压上下限一起修，避免不匹配的组合残留 */
void A_Param_Repair(Config_t *c, const Config_t *def)
{
    uint8_t i;

    for (i = 0U; i < sizeof(k_params) / sizeof(k_params[0]); i++)
    {
        const ParamDef_t *d = &k_params[i];
        uint8_t *p;

        if ((d->where != P_CFG && d->where != P_RUN_F) || (d->flags & PF_SPECIAL)) continue;
        p = Param_Field(d, c);
        if (!Param_InRange(d, Param_Load(p, d->size)))
            memcpy(p, Param_Field(d, (Config_t *)def), d->size);
    }
    if (!SERVO_MODE_IS_SUPPORTED(c->servo_mode)) c->servo_mode = def->servo_mode;
    if (c->startup_q3 < Q3_MIN || c->startup_q3 > Q3_MAX) c->startup_q3 = def->startup_q3;
    if (c->pulse_lo_q3 < Q3_MIN || c->pulse_hi_q3 > Q3_MAX || c->pulse_lo_q3 >= c->pulse_hi_q3)
    { c->pulse_lo_q3 = def->pulse_lo_q3; c->pulse_hi_q3 = def->pulse_hi_q3; }
    if (c->cal_kind > SERVO_CAL_ZERO || (c->cal_kind != SERVO_CAL_NONE && (int32_t)c->cal_anchor_cdeg > ENCODER_TRAVEL_HI))
    { c->cal_kind = SERVO_CAL_NONE; c->cal_anchor_cdeg = 0U; }
    if (c->position_offset_cdeg >= CDEG_RANGE) c->position_offset_cdeg = def->position_offset_cdeg;
    if (c->custom_offset_cdeg >= CDEG_RANGE || c->custom_span_cdeg >= CDEG_RANGE || c->custom_reverse > 1U
        || (c->servo_mode == SERVO_MODE_CUSTOM && c->custom_span_cdeg < SERVO_CUSTOM_SPAN_MIN))
    {
        c->custom_offset_cdeg = c->custom_span_cdeg = 0U; c->custom_reverse = 0U;
        if (c->servo_mode == SERVO_MODE_CUSTOM) c->servo_mode = def->servo_mode;
    }
    if (!A_Param_VoltageRangeValid(c->run.voltage_min_dv, c->run.voltage_max_dv))
    { c->run.voltage_min_dv = def->run.voltage_min_dv; c->run.voltage_max_dv = def->run.voltage_max_dv; }
    c->run.reserved = 0U;
}
