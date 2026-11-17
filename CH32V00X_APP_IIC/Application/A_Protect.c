/* A_Protect.c —— 保护与功率限制；本模块可越过用户指令关电机，只调公开API。
 * 可设参数(阈值、持续时间、开关)读 SRAM 运行参数，出厂值与范围见 A_Config.h 第5节；下面是内部常量。 */

#include "A_Protect.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include "C_Fixed.h"
#include <string.h>

#define PROT_TICK_MS        10U                     /** 任务基拍(ms) */
#define PROT_TEMP_SAMPLE_MS 250U                    /** 温度检测周期(ms) */
#define PROT_TEMP_HYST_DC    50                     /** 过温恢复回差，5.0℃ */
#define PROT_CURRENT_COOLDOWN_MS 500U               /** 过流后强制卸力冷却(ms) */
#define PROT_TEMP_DIV       (PROT_TEMP_SAMPLE_MS / PROT_TICK_MS)      /** 过温检测分频 */
#define PROT_STALL_DIV      (PROT_STALL_PERIOD_MS / PROT_TICK_MS)     /** 堵转检测分频 */
#define PROT_COOLDOWN_TICKS (PROT_CURRENT_COOLDOWN_MS / PROT_TICK_MS) /** 过流冷却拍数 */
#define PROT_STILL_CDEG     20                      /** 堵转周期内参考与实际位移都不超过它算静止(约10°/s) */
#define PROT_STALL_HOLD_TICKS 15                    /** 周期内轨迹被冻结这么多拍，同样算"轨迹在变" */
#define PROT_STALL_HOLD_LIMIT_MA 100U               /** 堵转保持限流(mA)，新指令恢复 */
#define PROT_VOLTAGE_TRIP_MS     100U               /** 静止时超限(或读数为0)持续此时长报电压故障 */
#define PROT_VOLTAGE_RECOVER_MS  500U               /** 回到恢复区间持续此时长清除 */

/* 限流回路：超出量/4降额，半步收敛不振荡；0被控制器解释为"取消限幅" */
#define PROT_LIMIT_DOWN_DIV     4                 /** 降额步长=超出电流/该值(移位) */
#define PROT_LIMIT_DOWN_MAX   400                 /** 单拍最大降额 */
#define PROT_LIMIT_UP_STEP (CFG_PWM_FULL / 50)    /** 每10ms回升步长，满量程约500ms */
#define PROT_LIMIT_MIN          1                 /** 最小限幅 */

/* 电流(mA) → PWM上限：满PWM对应额定堵转电流。只在配置变化时用(含除法) */
#define PROT_MA_TO_PWM(ma) ((int32_t)((uint32_t)(ma) * CFG_PWM_FULL / TORQUE_STALL_CURRENT_MA))
#define PROT_STALL_HOLD_PWM ((PROT_MA_TO_PWM(PROT_STALL_HOLD_LIMIT_MA) < PROT_LIMIT_MIN) \
                             ? PROT_LIMIT_MIN : PROT_MA_TO_PWM(PROT_STALL_HOLD_LIMIT_MA))  /** 编译期常量 */

/* 缓存名与运行参数名共用清单，变更检测不漏参数。 */
#define PROTECT_SETTINGS(X) \
    X(torque_pct, torque_limit_pct) \
    X(current_limit_ma, prot_current_limit_ma) \
    X(current_time_ms, prot_current_time_ms) \
    X(temp_limit_dC, prot_temp_limit_dC) \
    X(temp_time_ms, prot_temp_time_ms) \
    X(stall_act_cdeg, prot_stall_act_cdeg) \
    X(stall_trip, prot_stall_trip)

typedef struct {
    uint16_t fault;      /** PROT_FAULT_* 位掩码 */
    uint16_t torque_pct; /** 已生效扭矩上限(百分比0-100) */
    uint16_t cur_cnt;    /** 连续过流拍数 */
    uint16_t temp_cnt;   /** 连续过温采样数 */
    uint8_t  temp_div;   /** 过温检测分频计数 */
    uint8_t  stall_div;  /** 堵转检测分频计数 */
    uint16_t stall_cnt;  /** 连续判定堵转成立次数 */
    uint8_t  stall_hold; /** 1=堵转保持中(限流到保持电流)，也是协议的堵转状态 */
    uint16_t cooldown;   /** 过流冷却剩余拍数 */
    uint8_t  still;      /** 1=最近一个堵转周期参考与实际都没动；电压只在静止时判 */
    uint16_t voltage_bad_ms;
    uint16_t voltage_good_ms;
    uint16_t current_limit_ma; /** 已生效过流阈值(mA) */
    uint16_t current_time_ms;  /** 已生效过流持续时间(ms) */
    uint16_t current_trip_ticks; /** 换算后连续过流拍数 */
    uint16_t temp_limit_dC;    /** 已生效过温阈值，0.1℃ */
    uint16_t temp_time_ms;     /** 已生效过温持续时间(ms) */
    uint16_t temp_trip_samples;/** 换算后连续过温采样数 */
    uint16_t stall_ref_cdeg;  /** 参考位移阈值 = 实际×PROT_STALL_REF_RATIO，厘度/周期 */
    uint16_t stall_act_cdeg;  /** 已生效实际位移阈值，厘度/周期 */
    uint16_t stall_trip;      /** 已生效连续判定次数 */
    uint16_t i_limit_ma; /** 扭矩上限反算的限流(mA) */
    int16_t  i_limit_pwm;/** 同上换成PWM上限(回升目标) */
    int16_t  last_ma;    /** 最近电流(mA)，协议方向带符号，供上报 */
    uint16_t last_cv;    /** 最近电压，厘伏 */
    int16_t  last_temp;  /** 最近一次读成功的温度，0.1℃；-32768=上电后还没读成功过 */
    uint8_t  last_current_valid; /** 最近电流是否采在驱动段 */
    int16_t  pwm_limit;  /** 当前下发舵机的PWM上限 */
} Protect_t;

static Protect_t s_prot;

/* ---- 编译期护栏 ---- */
_Static_assert(TORQUE_STALL_CURRENT_MA > 0U && TORQUE_STALL_CURRENT_MA <= UINT16_MAX, "额定堵转电流必须落在电流字段量程内");
_Static_assert((PROT_TEMP_HYST_DC > 0) && (PROT_TEMP_HYST_DC < PROT_TEMP_LIMIT_MIN_DC), "guard_prot_temp_hyst_invalid");
_Static_assert(PROT_CURRENT_LIMIT_MAX_MA < (int32_t)CURRENT_FULL_SCALE_MA, "guard_prot_current_exceeds_adc_range");
_Static_assert(TORQUE_STALL_CURRENT_MA < (int32_t)CURRENT_FULL_SCALE_MA, "guard_prot_torque_current_exceeds_adc_range");
_Static_assert(((PROT_TEMP_SAMPLE_MS % PROT_TICK_MS) == 0U) && ((PROT_STALL_PERIOD_MS % PROT_TICK_MS) == 0U)
               && PROT_TEMP_TIME_MIN_MS >= PROT_TEMP_SAMPLE_MS, "guard_prot_period_not_multiple_of_tick");
_Static_assert((PROT_STALL_HOLD_TICKS >= 1) && (PROT_STALL_HOLD_TICKS <= PROT_STALL_PERIOD_MS), "guard_prot_stall_hold_ticks_unreachable");

/* ============================ 扭矩上限设置 ============================ */

/* 扭矩百分比反算限流电流/PWM上限并重建初始上限：避免零扭矩恢复后无有效采样而无法起转。
 * 0%禁止驱动(限流0、PWM最小)；堵转保持期间上限固定在保持电流。 */
static void Protect_ApplyTorqueSetting(void)
{
    uint16_t torque = g_runtime_config.run.torque_limit_pct;

    if (torque > TORQUE_MAX_PERCENT) torque = TORQUE_MAX_PERCENT;
    if (s_prot.pwm_limit != 0 && s_prot.torque_pct == torque) return;
    s_prot.torque_pct = torque;
    s_prot.i_limit_ma = (uint16_t)(((uint32_t)torque * TORQUE_STALL_CURRENT_MA + TORQUE_MAX_PERCENT / 2U) / TORQUE_MAX_PERCENT);
    s_prot.i_limit_pwm = (int16_t)Fx_ClampRange(PROT_MA_TO_PWM(s_prot.i_limit_ma), PROT_LIMIT_MIN, CFG_PWM_FULL);
    s_prot.pwm_limit = s_prot.stall_hold ? (int16_t)PROT_STALL_HOLD_PWM : s_prot.i_limit_pwm;
    A_Servo_SetOutputLimit(s_prot.pwm_limit);
}

/* 同步可配置保护参数。时间向上取整到实际周期，保证不早于设定值触发。 */
static void Protect_ApplyConfig(void)
{
    const RunParams_t *r = &g_runtime_config.run;

    /* 提高阈值后旧历史不足以证明仍超限；改严保留已成立的证据 */
    if (r->prot_current_limit_ma > s_prot.current_limit_ma) s_prot.cur_cnt = 0U;
    if (r->prot_temp_limit_dC > s_prot.temp_limit_dC) s_prot.temp_cnt = 0U;
    if (r->prot_stall_act_cdeg != s_prot.stall_act_cdeg) s_prot.stall_cnt = 0U;
    if (r->torque_limit_pct != s_prot.torque_pct || s_prot.pwm_limit == 0)
        Protect_ApplyTorqueSetting();
#define APPLY(cache, config) s_prot.cache = r->config;
    PROTECT_SETTINGS(APPLY)
#undef APPLY
    /* 扭矩缓存保持钳位后的生效值，不跟配置原样 */
    if (s_prot.torque_pct > TORQUE_MAX_PERCENT) s_prot.torque_pct = TORQUE_MAX_PERCENT;
    s_prot.stall_ref_cdeg = (uint16_t)(s_prot.stall_act_cdeg * PROT_STALL_REF_RATIO);
    s_prot.current_trip_ticks = (uint16_t)((s_prot.current_time_ms + PROT_TICK_MS - 1U) / PROT_TICK_MS);
    s_prot.temp_trip_samples = (uint16_t)((s_prot.temp_time_ms + PROT_TEMP_SAMPLE_MS - 1U) / PROT_TEMP_SAMPLE_MS);
}

void A_Protect_Refresh(void) { Protect_ApplyConfig(); }

/* 读已生效扭矩上限，百分比0-100 */
uint16_t A_Protect_GetTorque(void)
{ return s_prot.torque_pct; }

/* 读故障位掩码，堵转不在其中(堵转就地保持，不算故障) */
uint16_t A_Protect_Fault(void)
{ return s_prot.fault; }

/* 堵转进入保持：限流改为保持电流并立刻压低PWM上限 */
void A_Protect_EnterStallHold(void)
{
    s_prot.stall_hold = 1U;
    s_prot.pwm_limit = (int16_t)PROT_STALL_HOLD_PWM;
    A_Servo_SetOutputLimit(s_prot.pwm_limit);
}

/* 新指令等离开堵转保持：恢复扭矩限流 */
void A_Protect_ExitStallHold(void)
{
    if (!s_prot.stall_hold) return;
    s_prot.stall_hold = 0U;
    s_prot.pwm_limit = 0; /* 强制重套扭矩限流初值 */
    Protect_ApplyTorqueSetting();
}

/* 堵转状态就是"在堵转保持中"：堵转触发时进入，下一条运动指令退出，不另记标志 */
uint8_t A_Protect_Stalled(void)
{ return s_prot.stall_hold; }

uint8_t A_Protect_CanDrive(void)
{
    const RunParams_t *r = &g_runtime_config.run;

    if (s_prot.fault != PROT_FAULT_NONE) return 0U;
    if (r->torque_limit_pct == 0U) return 0U;
    return (uint8_t)(r->min_start_torque_pct == 0U || r->torque_limit_pct >= r->min_start_torque_pct);
}

/* 最近温度(0.1℃)：过温检测每250ms采一次的缓存，读失败保留上次值；查询不碰I2C；-32768=上电后还没读成功过 */
int16_t A_Protect_GetTemperature(void)
{ return s_prot.last_temp; }

/* 最近绕组电流(mA)，协议方向带符号；采不到时为0 */
int16_t A_Protect_GetCurrent(void)
{ return s_prot.last_ma; }

/* 只复制各周期任务的缓存；关闭保护功能也仍会更新采样值。 */
void A_Protect_GetTelemetry(ProtectTelemetry_t *out)
{
    if (out == 0) return;
    out->current_ma = s_prot.last_ma;
    out->voltage_cv = s_prot.last_cv;
    out->temperature_dc = s_prot.last_temp;
    out->current_valid = s_prot.last_current_valid;
}

/* ============================== 功率限制 ============================== */

/* 限流回路，每10ms：快降慢升；堵转保持期间目标为保持电流。 */
static void Protect_PowerLimit(uint8_t valid, uint16_t ma)
{
    int32_t limit = s_prot.pwm_limit; /* 本拍新上限 */
    int32_t ceiling;                  /* 回升目标 */
    int32_t step;                     /* 降额步长 */
    uint16_t i_lim = s_prot.stall_hold ? PROT_STALL_HOLD_LIMIT_MA : s_prot.i_limit_ma;

    if (s_prot.torque_pct == 0U)
    {
        /* 零扭矩不参与回升，保留最小限幅 */
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
        /* 采样无效(占空比太低或刹车段)时保持上限，不回落开环估计：
         * 否则停着时上限被持续压低，新指令拿不到满功率 */
        if (s_prot.stall_hold) ceiling = PROT_STALL_HOLD_PWM;
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

/* ============================= 检测链 ============================= */

/* 过流检测，每10ms：连续超阈值达设定时间则卸力并进入冷却 */
static void Protect_OverCurrent(uint8_t valid, uint16_t ma)
{
    if ((g_runtime_config.run.protect_enable & PROT_ENABLE_CURRENT) == 0U)
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

/* 堵转检测，20ms一次：比较参考与实际位移；每周期都要采样(采样会推进基准并清冻结计数)。
 * valid=0(卸力/暂停/电机模式等)时连续计数清零。 */
static void Protect_Stall(void)
{
    ServoMotion_t m;      /* 本周期运动采样 */
    int32_t d_ref, d_act; /* 参考/实际位移绝对值，厘度 */

    A_Servo_SampleMotion(&m);
    d_ref = Fx_Abs32(m.ref_delta);
    d_act = Fx_Abs32(m.act_delta);
    s_prot.still = (uint8_t)(d_ref <= PROT_STILL_CDEG && d_act <= PROT_STILL_CDEG);

    if ((g_runtime_config.run.protect_enable & PROT_ENABLE_STALL) == 0U || !m.valid)
    {
        s_prot.stall_cnt = 0U;
        return;
    }

    /* 参考在走，或参考被饱和冻结按在原地(顶死时正是这样) */
    if ((d_ref >= s_prot.stall_ref_cdeg || m.saturation_ticks >= PROT_STALL_HOLD_TICKS)
        && d_act <= s_prot.stall_act_cdeg)
    {
        if (++s_prot.stall_cnt >= s_prot.stall_trip)
        {
            s_prot.stall_cnt = 0U;
            A_Servo_HoldHere(); /* 当前角度设为目标，直接进保持；不卸力 */
        }
        return;
    }
    s_prot.stall_cnt = 0U;
}

/* 过温检测，250ms：持续超阈值达设定时间才置故障；回落到阈值-回差才清除。 */
static void Protect_OverTemp(void)
{
    int16_t temp = A_Temperature_Read(); /* 0.1℃，-32768=读取失败 */

    if (temp != (int16_t)-32768) s_prot.last_temp = temp;   /* 读失败保留上次值 */

    if ((g_runtime_config.run.protect_enable & PROT_ENABLE_TEMP) == 0U)
    {
        s_prot.temp_cnt = 0U;
        s_prot.fault &= (uint16_t)~PROT_FAULT_OVERTEMP;
        return;
    }
    if (temp == (int16_t)-32768) return; /* 保持上次结论，不当成温度正常 */

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

/* 电压保护每10ms：静止时超出设置值±容差持续100ms停机(运动中的起步压降、回馈泵升不算，由输出级限速和钳位处理)；
 * 读数为0随时判无效；回差内稳定500ms恢复。 */
static void Protect_Voltage(void)
{
    uint16_t cv = A_Voltage_Read();
    uint16_t lo = (uint16_t)(g_runtime_config.run.voltage_min_dv - PROT_VOLTAGE_TOL_LO_DV) * 10U;
    uint16_t hi = (uint16_t)(g_runtime_config.run.voltage_max_dv + PROT_VOLTAGE_TOL_HI_DV) * 10U;
    uint16_t recover_lo = (uint16_t)(lo + PROT_VOLTAGE_HYST_DV * 10U);
    uint16_t recover_hi = (uint16_t)(hi - PROT_VOLTAGE_HYST_DV * 10U);

    s_prot.last_cv = cv;
    if ((g_runtime_config.run.protect_enable & PROT_ENABLE_VOLTAGE) == 0U)
    {
        s_prot.voltage_bad_ms = s_prot.voltage_good_ms = 0U;
        s_prot.fault &= (uint16_t)~PROT_FAULT_VOLTAGE;
        return;
    }

    if (cv == 0U || (s_prot.still && (cv < lo || cv > hi)))
    {
        s_prot.voltage_good_ms = 0U;
        if (s_prot.voltage_bad_ms < PROT_VOLTAGE_TRIP_MS)
            s_prot.voltage_bad_ms += PROT_TICK_MS;
        if (s_prot.voltage_bad_ms >= PROT_VOLTAGE_TRIP_MS)
        {
            s_prot.fault &= (uint16_t)~PROT_FAULT_VOLTAGE;
            s_prot.fault |= (cv == 0U) ? PROT_FAULT_VOLT_INVALID
                         : (cv < lo)  ? PROT_FAULT_UNDERVOLT : PROT_FAULT_OVERVOLT;
        }
        return;
    }

    s_prot.voltage_bad_ms = 0U;
    if ((s_prot.fault & PROT_FAULT_VOLTAGE) != 0U && cv >= recover_lo && cv <= recover_hi)
    {
        if (s_prot.voltage_good_ms < PROT_VOLTAGE_RECOVER_MS)
            s_prot.voltage_good_ms += PROT_TICK_MS;
        if (s_prot.voltage_good_ms >= PROT_VOLTAGE_RECOVER_MS)
            s_prot.fault &= (uint16_t)~PROT_FAULT_VOLTAGE;
    }
    /* 恢复时间须连续累计；进入回差区也要清零恢复计时 */
    else s_prot.voltage_good_ms = 0U;
}

/* =========================== 生命周期与任务 =========================== */

/* 初始化保护模块，须在 A_Servo_Init 之后调用 */
void A_Protect_Init(void)
{
    memset(&s_prot, 0, sizeof(s_prot));
    s_prot.last_temp = (int16_t)-32768;
    Protect_ApplyConfig();
}

/* 10ms任务：过流/限流/电压每拍，堵转20ms，过温250ms，最后把故障同步到扭矩状态 */
void A_Protect_Task(void)
{
    int32_t  sma = 0; /* 实测电流(mA)，硬件方向带符号 */
    uint16_t ma;      /* 电流大小(mA)：保护判据，回馈电流同样计入 */
    uint8_t  valid;   /* 本拍电流采样是否有效 */

    /* 配置被协议等路径改动时，最迟下一拍跟上 */
#define CHANGED(cache, config) || (s_prot.cache != g_runtime_config.run.config)
    if (0 PROTECT_SETTINGS(CHANGED))
        Protect_ApplyConfig();
#undef CHANGED

    /* valid 只表示采在驱动段：保护判据用 valid，诊断上报用最新读数 */
    valid = A_Current_ReadSigned(&sma);
    ma = (uint16_t)((Fx_Abs32(sma) > 0xFFFF) ? 0xFFFF : Fx_Abs32(sma));
    s_prot.last_ma = (int16_t)Fx_Clamp32(sma * A_Servo_UserSign(), 32767);
    s_prot.last_current_valid = valid;

    Protect_OverCurrent(valid, ma);
    Protect_PowerLimit(valid, ma);
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

    A_Servo_SyncTorque(); /* 只同步实际输出，不改写用户扭矩请求 */
}
