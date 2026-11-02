/* A_Protect.c —— 保护与功率限制，工程里唯一会越过用户指令关电机的地方，只调公开API不碰寄存器。 */

#include "A_Protect.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include "C_Fixed.h"
#include <string.h>

/* 电流降额开关；过流故障保护独立生效。 */
#define PROT_CURRENT_DERATING_ENABLE 1

#define PROT_TICK_MS        10U                     /** 任务基拍(ms) */
#define PROT_TEMP_DIV       (PROT_TEMP_SAMPLE_MS / PROT_TICK_MS)      /** 过温检测分频 */
#define PROT_STALL_DIV      (PROT_STALL_PERIOD_MS / PROT_TICK_MS)      /** 堵转检测分频 */
#define PROT_COOLDOWN_TICKS (PROT_CURRENT_COOLDOWN_MS / PROT_TICK_MS)  /** 过流冷却拍数 */

/* 限流回路降额步长：超出量/4，半步收敛不振荡 */
#define PROT_LIMIT_DOWN_DIV     4                 /** 降额步长=超出电流/该值 */
#define PROT_LIMIT_DOWN_MAX   400                 /** 单拍最大降额，比机械时间常数快一个数量级 */
#define PROT_LIMIT_UP_STEP (CFG_PWM_FULL / 50)    /** 每10ms回升的步长，满量程约500ms */
#define PROT_LIMIT_MIN          1                 /** 最小限幅，控制器把0解释为"取消限幅" */

/* 缓存名、实时配置名共用一份清单，应用与变更检测不会漏掉某个参数。 */
#define PROTECT_SETTINGS(X) \
    X(torque_ckgcm, torque_limit_ckgcm) \
    X(current_limit_ma, prot_current_limit_ma) \
    X(current_time_ms, prot_current_time_ms) \
    X(temp_limit_dC, prot_temp_limit_dC) \
    X(temp_time_ms, prot_temp_time_ms) \
    X(stall_act_cdeg, prot_stall_act_cdeg) \
    X(stall_trip, prot_stall_trip)

typedef struct {
    uint16_t fault;      /** PROT_FAULT_* 位掩码 */
    uint16_t torque_ckgcm; /** 已生效的扭矩上限，与g_config不一致时重算 */
    uint16_t cur_cnt;    /** 连续过流拍数 */
    uint16_t temp_cnt;   /** 连续过温采样数 */
    uint8_t  temp_div;   /** 过温检测分频计数 */
    uint8_t  stall_div;  /** 堵转检测分频计数 */
    uint16_t stall_cnt;  /** 连续判定堵转成立的次数 */
    uint8_t  stalled;    /** 1=最近一次动作以堵转收场，供诊断 */
    uint16_t cooldown;   /** 过流冷却剩余拍数，0=不在冷却中 */
    uint16_t voltage_bad_ms;
    uint16_t voltage_good_ms;
    uint16_t current_limit_ma; /** 已生效的过流阈值 */
    uint16_t current_time_ms;  /** 已生效的过流持续时间 */
    uint16_t current_trip_ticks; /** 换算后的连续过流拍数 */
    uint16_t temp_limit_dC;    /** 已生效的过温阈值，0.1摄氏度 */
    uint16_t temp_time_ms;     /** 已生效的过温持续时间 */
    uint16_t temp_trip_samples;/** 换算后的连续过温采样数 */
    uint16_t stall_ref_cdeg;  /** 参考位移阈值 = 实际位移阈值 × PROT_STALL_REF_RATIO */
    uint16_t stall_act_cdeg;  /** 已生效的实际位移阈值 */
    uint16_t stall_trip;      /** 已生效的连续判定次数 */
    uint16_t i_limit_ma; /** 物理扭矩反算的限流值 */
    uint16_t hold_limit_ma; /** 堵转保持限流(mA)，0=不启用；覆盖 i_limit_ma */
    uint16_t last_ma;    /** 最近一块电流读数(mA)，供 RIV 查询上报 */
    uint16_t last_cv;    /** 最近一次电压读数，厘伏 */
    int16_t last_temp;   /** 最近一次温度读数，0.1摄氏度；-32768=无效 */
    uint8_t last_current_valid; /** 最近一块电流是否采在驱动段 */
    int16_t  pwm_limit;  /** 当前下发给舵机的PWM上限 */
} Protect_t;

static Protect_t s_prot;

/* ---- 编译期护栏 ---- */
_Static_assert((PROT_TEMP_HYST_DC > 0) && (PROT_TEMP_HYST_DC < PROT_TEMP_LIMIT_MIN_DC), "guard_prot_temp_hyst_invalid");
_Static_assert(PROT_CURRENT_LIMIT_MAX_MA < (int32_t)CURRENT_FULL_SCALE_MA, "guard_prot_current_exceeds_adc_range");
_Static_assert(TORQUE_STALL_CURRENT_MA < (int32_t)CURRENT_FULL_SCALE_MA, "guard_prot_torque_current_exceeds_adc_range");
_Static_assert((PROT_CURRENT_LIMIT_DEFAULT_MA >= PROT_CURRENT_LIMIT_MIN_MA)
               && (PROT_CURRENT_LIMIT_DEFAULT_MA <= PROT_CURRENT_LIMIT_MAX_MA)
               && (PROT_CURRENT_TIME_DEFAULT_MS >= PROT_CURRENT_TIME_MIN_MS)
               && (PROT_CURRENT_TIME_DEFAULT_MS <= PROT_CURRENT_TIME_MAX_MS)
               && (PROT_TEMP_LIMIT_DEFAULT_DC >= PROT_TEMP_LIMIT_MIN_DC)
               && (PROT_TEMP_LIMIT_DEFAULT_DC <= PROT_TEMP_LIMIT_MAX_DC)
               && (PROT_TEMP_TIME_DEFAULT_MS >= PROT_TEMP_TIME_MIN_MS)
               && (PROT_TEMP_TIME_DEFAULT_MS <= PROT_TEMP_TIME_MAX_MS), "guard_prot_defaults_invalid");
_Static_assert(((PROT_TEMP_SAMPLE_MS % PROT_TICK_MS) == 0U)
               && ((PROT_STALL_PERIOD_MS % PROT_TICK_MS) == 0U), "guard_prot_period_not_multiple_of_tick");
/* 两个位移阈值必须拉开避免含糊带；出厂阈值在对外范围内 */
_Static_assert((PROT_STALL_REF_RATIO > 1U) && (PROT_STALL_THRESHOLD_MIN_CDEG >= 1U)
               && (PROT_STALL_ACT_DEFAULT_CDEG >= PROT_STALL_THRESHOLD_MIN_CDEG)
               && (PROT_STALL_ACT_DEFAULT_CDEG <= PROT_STALL_THRESHOLD_MAX_CDEG), "guard_prot_stall_thresholds_invalid");
_Static_assert((PROT_ENABLE_DEFAULT & (uint8_t)~PROT_ENABLE_IMPLEMENTED) == 0U, "guard_prot_enable_default_unimplemented");
_Static_assert((PROT_STALL_TRIP_DEFAULT >= PROT_STALL_TRIP_MIN) && (PROT_STALL_TRIP_DEFAULT <= PROT_STALL_TRIP_MAX), "guard_prot_stall_trip_invalid");
/* 冻结判据要求的拍数不能超过一个检测周期的控制拍数 */
_Static_assert((PROT_STALL_HOLD_TICKS >= 1) && (PROT_STALL_HOLD_TICKS <= PROT_STALL_PERIOD_MS), "guard_prot_stall_hold_ticks_unreachable");

/* ============================ 扭矩上限设置 ============================ */

/* 限流启用时按扭矩反算电流/PWM上限；关闭时使用满量程，仍保留零扭矩语义。
 * 堵转保持限流(hold_limit_ma)优先，不被扭矩配置刷新覆盖。 */
static void Protect_ApplyTorqueSetting(void)
{
    uint16_t rated = A_Config_TorqueMax();
    uint16_t torque = g_runtime_config.torque_limit_ckgcm;

    if (torque > rated) torque = rated;
    if (s_prot.pwm_limit != 0 && s_prot.torque_ckgcm == torque) return;
    s_prot.torque_ckgcm = torque;
    s_prot.i_limit_ma = A_Torque_CurrentLimit(torque, rated);
#if PROT_CURRENT_DERATING_ENABLE
    /* 同值写不重置降额；真正改扭矩重建初始上限，避免零扭矩恢复后无有效采样而无法起转。 */
    if (s_prot.hold_limit_ma != 0U)
    {
        int32_t limit = (int32_t)s_prot.hold_limit_ma * CFG_PWM_FULL / TORQUE_STALL_CURRENT_MA;
        if (limit < PROT_LIMIT_MIN) limit = PROT_LIMIT_MIN;
        s_prot.pwm_limit = (int16_t)limit;
    }
    else
    {
        s_prot.pwm_limit = (int16_t)((uint32_t)s_prot.i_limit_ma * CFG_PWM_FULL
                                   / TORQUE_STALL_CURRENT_MA);
        if (s_prot.pwm_limit < PROT_LIMIT_MIN) s_prot.pwm_limit = PROT_LIMIT_MIN;
    }
#else
    s_prot.pwm_limit = torque == 0U ? PROT_LIMIT_MIN : CFG_PWM_FULL;
#endif
    A_Servo_SetOutputLimit(s_prot.pwm_limit);
}

/* 同步可配置保护参数。时间向上取整到实际任务/采样周期，保证不会早于用户设定值触发。 */
static void Protect_ApplyConfig(void)
{
    /* 提高阈值后旧历史不足以证明仍超限；改严保留已成立的证据。
     * 时间参数只改触发门槛；扭矩和其它检测器的改动不清本检测器历史。 */
    if (g_runtime_config.prot_current_limit_ma > s_prot.current_limit_ma) s_prot.cur_cnt = 0U;
    if (g_runtime_config.prot_temp_limit_dC > s_prot.temp_limit_dC) s_prot.temp_cnt = 0U;
    if (g_runtime_config.prot_stall_act_cdeg != s_prot.stall_act_cdeg) s_prot.stall_cnt = 0U;
    if (g_runtime_config.torque_limit_ckgcm != s_prot.torque_ckgcm || s_prot.pwm_limit == 0)
        Protect_ApplyTorqueSetting();
#define APPLY(cache, config) s_prot.cache = g_runtime_config.config;
    PROTECT_SETTINGS(APPLY)
#undef APPLY
    /* APPLY写的是配置原样值，扭矩缓存必须保持钳位后的生效值 */
    if (s_prot.torque_ckgcm > A_Config_TorqueMax()) s_prot.torque_ckgcm = A_Config_TorqueMax();
    s_prot.stall_ref_cdeg = (uint16_t)(s_prot.stall_act_cdeg * PROT_STALL_REF_RATIO);
    s_prot.current_trip_ticks = (uint16_t)((s_prot.current_time_ms + PROT_TICK_MS - 1U)
                                           / PROT_TICK_MS);
    s_prot.temp_trip_samples = (uint16_t)((s_prot.temp_time_ms + PROT_TEMP_SAMPLE_MS - 1U)
                                          / PROT_TEMP_SAMPLE_MS);
}

/* 设置物理扭矩上限并落盘，返回0=超过硬件上限 */
uint8_t A_Protect_SetTorque(uint16_t torque_ckgcm)
{
    if (torque_ckgcm > A_Config_TorqueMax()) return 0U;

    g_config.torque_limit_ckgcm = torque_ckgcm;
    g_runtime_config.torque_limit_ckgcm = torque_ckgcm;
    A_Config_MarkDirty();
    Protect_ApplyTorqueSetting();
    return 1U;
}

/* 读已生效扭矩上限，单位0.01 kgf·cm */
uint16_t A_Protect_GetTorque(void)
{ return s_prot.torque_ckgcm; }

/* 读故障位掩码，堵转不在其中 */
uint16_t A_Protect_Fault(void)
{ return s_prot.fault; }

/* 堵转进入保持后：限流目标改为 PROT_STALL_HOLD_LIMIT_MA，并立刻压低 PWM 上限 */
void A_Protect_EnterStallHold(void)
{
    int32_t limit;

    s_prot.hold_limit_ma = PROT_STALL_HOLD_LIMIT_MA;
    limit = (int32_t)s_prot.hold_limit_ma * CFG_PWM_FULL / TORQUE_STALL_CURRENT_MA;
    if (limit < PROT_LIMIT_MIN) limit = PROT_LIMIT_MIN;
    s_prot.pwm_limit = (int16_t)limit;
    A_Servo_SetOutputLimit(s_prot.pwm_limit);
}

/* 新指令等离开堵转保持：恢复扭矩限流 */
void A_Protect_ExitStallHold(void)
{
    if (s_prot.hold_limit_ma == 0U) return;
    s_prot.hold_limit_ma = 0U;
    s_prot.pwm_limit = 0; /* 强制重套扭矩限流初值 */
    Protect_ApplyTorqueSetting();
}

uint8_t A_Protect_CanDrive(void)
{
    uint16_t hard_fault = (uint16_t)(s_prot.fault & (uint16_t)~PROT_FAULT_STALL);
    if (hard_fault != PROT_FAULT_NONE) return 0U;
    if (g_runtime_config.torque_limit_ckgcm == 0U) return 0U;
    return (uint8_t)(g_runtime_config.min_start_torque_ckgcm == 0U
                  || g_runtime_config.torque_limit_ckgcm >= g_runtime_config.min_start_torque_ckgcm);
}

/* 1=最近一次动作以堵转收场(仅诊断)；只有看到机构真的动了才清除 */
uint8_t A_Protect_Stalled(void)
{ return s_prot.stalled; }

/* 最近一块绕组电流(mA)，供 RIV 查询；与限流回路用的是同一个数，采不到绕组电流时为0 */
uint16_t A_Protect_GetCurrent(void)
{ return s_prot.last_ma; }

/* 只复制各周期任务的缓存，关闭保护功能也仍会更新采样值。 */
void A_Protect_GetTelemetry(ProtectTelemetry_t *out)
{
    if (out == 0) return;
    out->current_ma = s_prot.last_ma;
    out->voltage_cv = s_prot.last_cv;
    out->temperature_dc = s_prot.last_temp;
    out->current_valid = s_prot.last_current_valid;
}

/* ============================== 功率限制 ============================== */

/* 限流回路：快降慢升，瞬时过流立刻按住，恢复平滑。
 * 堵转保持期间目标电流改为 hold_limit_ma，回升天花板也折到对应 PWM。 */
#if PROT_CURRENT_DERATING_ENABLE
static void Protect_PowerLimit(uint8_t valid, uint16_t ma)
{
    int32_t limit = s_prot.pwm_limit; /* 本拍算出的新上限 */
    int32_t ceiling;                  /* 回升目标         */
    int32_t step;                     /* 降额步长         */
    uint16_t i_lim = (s_prot.hold_limit_ma != 0U) ? s_prot.hold_limit_ma : s_prot.i_limit_ma;

    if (s_prot.torque_ckgcm == 0U)
    {
        /* 零扭矩不参与回升，保留最小限幅(0是控制器的取消限幅标记) */
        limit = PROT_LIMIT_MIN;
    }
    else if (valid && ma > i_lim)
    {
        step = (int32_t)(ma - i_lim) / PROT_LIMIT_DOWN_DIV;
        if (step < 1) step = 1;
        if (step > PROT_LIMIT_DOWN_MAX) step = PROT_LIMIT_DOWN_MAX;
        limit -= step;
    }
    else
    {
        /* 采样有效时可回升到满量程；采样无效(占空比太低或刹车段，电流不可测)时保持当前上限不动，
                 * 不回落到开环估计——否则停着时上限被持续压低，每条新指令都拿不到满功率。 */
        if (s_prot.hold_limit_ma != 0U)
        {
            ceiling = (int32_t)s_prot.hold_limit_ma * CFG_PWM_FULL / TORQUE_STALL_CURRENT_MA;
            if (ceiling < PROT_LIMIT_MIN) ceiling = PROT_LIMIT_MIN;
        }
        else ceiling = valid ? (int32_t)CFG_PWM_FULL : limit;

        if (limit < ceiling)
        {
            limit += PROT_LIMIT_UP_STEP;
            if (limit > ceiling) limit = ceiling;
        }
    }

    limit = Fx_ClampRange(limit, PROT_LIMIT_MIN, CFG_PWM_FULL);

    if ((int16_t)limit != s_prot.pwm_limit)
    {
        s_prot.pwm_limit = (int16_t)limit;
        A_Servo_SetOutputLimit(s_prot.pwm_limit);
    }
}
#endif

/* ============================= 检测链 ============================= */

/* 过流检测，每10ms一次：连续超阈值达到用户设定时间则卸力并进入冷却 */
static void Protect_OverCurrent(uint8_t valid, uint16_t ma)
{
    if ((g_runtime_config.protect_enable & PROT_ENABLE_CURRENT) == 0U)
    {
        s_prot.cur_cnt = s_prot.cooldown = 0U;
        s_prot.fault &= (uint16_t)~PROT_FAULT_OVERCUR;
        return;
    }
    if (s_prot.cooldown != 0U)
    {
        /* 冷却期间不判(卸力后绕组无电流) */
        if (--s_prot.cooldown == 0U)
            s_prot.fault &= (uint16_t)~PROT_FAULT_OVERCUR;
        return;
    }

    if (valid && ma > s_prot.current_limit_ma)
    {
        if (s_prot.cur_cnt < s_prot.current_trip_ticks) s_prot.cur_cnt++;
        if (s_prot.cur_cnt >= s_prot.current_trip_ticks)
        {
            s_prot.fault   |= PROT_FAULT_OVERCUR;
            s_prot.cooldown = PROT_COOLDOWN_TICKS;
            s_prot.cur_cnt  = 0U;
        }
    }
    else
    {
        s_prot.cur_cnt = 0U; /* 采样无效或已回落：尖峰不累计 */
    }
}

/* 堵转检测，20ms一次：比较本周期参考与机构的位移。每周期都要采样(采样会推进基准并清冻结计数)；
 * valid=0(卸力/暂停/电机模式/脱困/轨迹已走完)时连续计数清零。 */
static void Protect_Stall(void)
{
    ServoMotion_t m;      /* 本周期的运动采样 */
    int32_t d_ref, d_act; /* 参考/实际位移的绝对值，厘度 */

    A_Servo_SampleMotion(&m);

    if ((g_runtime_config.protect_enable & PROT_ENABLE_STALL) == 0U)
    {
        s_prot.stall_cnt = 0U;
        s_prot.stalled = 0U;
        s_prot.fault &= (uint16_t)~PROT_FAULT_STALL;
        return;
    }

    if (!m.valid)
    {
        s_prot.stall_cnt = 0U;
        return;
    }

    d_ref = Fx_Abs32(m.ref_delta);
    d_act = Fx_Abs32(m.act_delta);

    /* 参考在走，或参考被饱和冻结按在原地(顶死时正是这样) */
    if ((d_ref >= s_prot.stall_ref_cdeg || m.saturation_ticks >= PROT_STALL_HOLD_TICKS)
        && d_act <= s_prot.stall_act_cdeg)
    {
        if (++s_prot.stall_cnt >= s_prot.stall_trip)
        {
            s_prot.stall_cnt = 0U;
            A_Servo_HoldHere(); /* 当前角度设为目标，直接进保持；不卸力 */
            s_prot.stalled = 0U; /* 与普通到位一致，不留下堵转诊断标志 */
        }
        return;
    }

    s_prot.stall_cnt = 0U;
    /* 只有确实转起来才清诊断标志 */
    if (d_act > s_prot.stall_act_cdeg)
    { s_prot.stalled = 0U; }
}


/* 过温检测，250ms一次：持续超阈值达到设定时间才置故障位；回落到阈值-回差才清除。 */
static void Protect_OverTemp(void)
{
    int16_t temp = A_Temperature_Read(); /* 0.1摄氏度，-32768=读取失败 */

    s_prot.last_temp = temp;

    if ((g_runtime_config.protect_enable & PROT_ENABLE_TEMP) == 0U)
    {
        s_prot.temp_cnt = 0U;
        s_prot.fault &= (uint16_t)~PROT_FAULT_OVERTEMP;
        return;
    }
    if (temp == (int16_t)-32768) return; /* 保持上一次结论，不当成温度正常 */

    if (temp >= (int16_t)s_prot.temp_limit_dC)
    {
        if (s_prot.temp_cnt < s_prot.temp_trip_samples) s_prot.temp_cnt++;
        if (s_prot.temp_cnt >= s_prot.temp_trip_samples)
            s_prot.fault |= PROT_FAULT_OVERTEMP;
    }
    else
    {
        s_prot.temp_cnt = 0U;
        if (temp <= (int16_t)(s_prot.temp_limit_dC - PROT_TEMP_HYST_DC))
        s_prot.fault &= (uint16_t)~PROT_FAULT_OVERTEMP;
    }
}

/* 电压保护每10ms判断一次，越界50ms停机；回到阈值内并留0.2V回差后稳定500ms恢复。 */
static void Protect_Voltage(void)
{
    uint16_t cv = A_Voltage_Read();
    uint16_t lo = (uint16_t)g_runtime_config.voltage_min_dv * 10U;
    uint16_t hi = (uint16_t)g_runtime_config.voltage_max_dv * 10U;
    uint16_t recover_lo = (uint16_t)(g_runtime_config.voltage_min_dv + PROT_VOLTAGE_HYST_DV) * 10U;
    uint16_t recover_hi = (uint16_t)(g_runtime_config.voltage_max_dv - PROT_VOLTAGE_HYST_DV) * 10U;
    uint16_t voltage_faults = PROT_FAULT_UNDERVOLT | PROT_FAULT_OVERVOLT | PROT_FAULT_VOLT_INVALID;

    s_prot.last_cv = cv;
    if ((g_runtime_config.protect_enable & PROT_ENABLE_VOLTAGE) == 0U)
    {
        s_prot.voltage_bad_ms = s_prot.voltage_good_ms = 0U;
        s_prot.fault &= (uint16_t)~voltage_faults;
        return;
    }

    if (cv == 0U || cv < lo || cv > hi)
    {
        s_prot.voltage_good_ms = 0U;
        if (s_prot.voltage_bad_ms < PROT_VOLTAGE_TRIP_MS)
            s_prot.voltage_bad_ms += PROT_TICK_MS;
        if (s_prot.voltage_bad_ms >= PROT_VOLTAGE_TRIP_MS)
        {
            s_prot.fault &= (uint16_t)~voltage_faults;
            s_prot.fault |= (cv == 0U) ? PROT_FAULT_VOLT_INVALID
                         : (cv < lo)  ? PROT_FAULT_UNDERVOLT : PROT_FAULT_OVERVOLT;
        }
        return;
    }

    s_prot.voltage_bad_ms = 0U;
    if ((s_prot.fault & voltage_faults) != 0U && cv >= recover_lo && cv <= recover_hi)
    {
        if (s_prot.voltage_good_ms < PROT_VOLTAGE_RECOVER_MS)
            s_prot.voltage_good_ms += PROT_TICK_MS;
        if (s_prot.voltage_good_ms >= PROT_VOLTAGE_RECOVER_MS)
            s_prot.fault &= (uint16_t)~voltage_faults;
    }
    /* 恢复时间必须连续累计；即使只是进入回差区、尚未再次触发保护，
     * 也要清零恢复计时。 */
    else s_prot.voltage_good_ms = 0U;
}

/* ============================== 故障落实 ============================== */

/* =========================== 生命周期与任务 =========================== */

/* 初始化保护模块，须在 A_Servo_Init 之后调用 */
void A_Protect_Init(void)
{
    memset(&s_prot, 0, sizeof(s_prot));
    s_prot.last_temp = (int16_t)-32768;
    Protect_ApplyConfig();
}

/* 10ms任务入口：过流/限流每拍，堵转20ms，过温250ms，最后逐拍落实故障状态 */
void A_Protect_Task(void)
{
    uint16_t ma = 0U; /* 实测电流，毫安       */
    uint8_t  valid;   /* 本拍电流采样是否有效 */

    /* 配置被协议写入或 CLE 等路径改动时，最迟下一拍跟上 */
#define CHANGED(cache, config) || (s_prot.cache != g_runtime_config.config)
    if (0 PROTECT_SETTINGS(CHANGED))
        Protect_ApplyConfig();
#undef CHANGED

    /* valid 只表示采在驱动段：保护判据用 valid，诊断上报用最新读数 */
    valid = A_Current_Read(&ma);
    s_prot.last_ma = ma;
    s_prot.last_current_valid = valid;

    Protect_OverCurrent(valid, ma);
#if PROT_CURRENT_DERATING_ENABLE
    Protect_PowerLimit(valid, ma);
#endif
    Protect_Voltage();

    if (++s_prot.stall_div >= PROT_STALL_DIV)
    {
        s_prot.stall_div = 0U;
        Protect_Stall();
    }

    if (++s_prot.temp_div >= PROT_TEMP_DIV)
    {
        s_prot.temp_div = 0U;
        Protect_OverTemp();
    }

    A_Servo_SyncTorque(); /* 只同步实际输出，不改写用户扭矩请求。 */
}
