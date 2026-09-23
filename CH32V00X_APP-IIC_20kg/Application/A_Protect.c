/*
 * A_Protect.c —— 保护与功率限制，工程里唯一会越过用户指令关电机的地方，只调公开API不碰寄存器。
 * 一个10ms基拍分频出堵转(20ms)和过温(250ms)，少注册任务以免挤占1ms控制拍。
 *   过温 —— 带回差防止贴着阈值反复通断；读失败保持上次结论。
 *   过流 —— 连续多拍超阈值才跳闸(滤起转/换向尖峰)；恢复按固定冷却时间(卸力后电流恒低，不能用回差)，
 *           故障仍在则形成 30ms通/500ms断 的打嗝式重试。
 *   堵转 —— 参考在走(或被饱和冻结)而实际不动，连续100ms才判定；位置反馈比电流更早发现小电机堵转。
 *   限功率 —— 扭矩%换成限流值，慢回路调PWM上限使实测电流落在限流值上(扭矩正比于电流而非占空比)。
 */

#include "A_Protect.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include <string.h>

#define PROT_TICK_MS        10U                     /** 任务基拍(ms) */
#define PROT_TEMP_DIV       (250U / PROT_TICK_MS)   /** 过温检测分频 */
#define PROT_STALL_DIV      (PROT_STALL_PERIOD_MS / PROT_TICK_MS)      /** 堵转检测分频 */
#define PROT_COOLDOWN_TICKS (PROT_CURRENT_COOLDOWN_MS / PROT_TICK_MS)  /** 过流冷却拍数 */

/* 限流回路降额步长：超出量/4，半步收敛不振荡 */
#define PROT_LIMIT_DOWN_DIV     4                 /** 降额步长=超出电流/该值 */
#define PROT_LIMIT_DOWN_MAX   400                 /** 单拍最大降额，比机械时间常数快一个数量级 */
#define PROT_LIMIT_UP_STEP (CFG_PWM_FULL / 50)    /** 每10ms回升的步长，满量程约500ms */
#define PROT_LIMIT_MIN          1                 /** 最小限幅，控制器把0解释为"取消限幅" */

typedef struct {
    uint8_t  fault;      /** PROT_FAULT_* 位掩码 */
    uint8_t  released;   /** 1=当前的卸力由本模块发出，只有它该由本模块解除 */
    uint8_t  torque_pct; /** 已生效的扭矩上限，与g_config不一致时重算 */
    uint8_t  cur_cnt;    /** 连续过流拍数 */
    uint8_t  temp_div;   /** 过温检测分频计数 */
    uint8_t  stall_div;  /** 堵转检测分频计数 */
    uint8_t  stall_cnt;  /** 连续判定堵转成立的次数 */
    uint8_t  stalled;    /** 1=最近一次动作以堵转收场，供诊断 */
    uint16_t cooldown;   /** 过流冷却剩余拍数，0=不在冷却中 */
    uint16_t i_limit_ma; /** 扭矩百分比换算出的限流值 */
    uint16_t last_ma;    /** 最近一块电流读数(mA)，供 RIV 查询上报 */
    int16_t  pwm_limit;  /** 当前下发给舵机的PWM上限 */
} Protect_t;

static Protect_t s_prot;

/* ---- 编译期护栏 ---- */
typedef char guard_prot_temp_hyst_invalid
    [((PROT_TEMP_HYST_C > 0) && (PROT_TEMP_HYST_C < PROT_TEMP_LIMIT_C)) ? 1 : -1];
typedef char guard_prot_current_exceeds_adc_range
    [(PROT_CURRENT_LIMIT_MA < (int32_t)CURRENT_FULL_SCALE_MA) ? 1 : -1];
typedef char guard_prot_torque_default_out_of_range
    [(PROT_TORQUE_DEFAULT <= PROT_TORQUE_MAX) ? 1 : -1];
typedef char guard_prot_trip_count_invalid
    [(PROT_CURRENT_TRIP >= 1) ? 1 : -1];
typedef char guard_prot_period_not_multiple_of_tick
    [(((250U % PROT_TICK_MS) == 0U)
   && ((PROT_STALL_PERIOD_MS % PROT_TICK_MS) == 0U)) ? 1 : -1];
/* 两个位移阈值必须拉开，避免含糊带 */
typedef char guard_prot_stall_thresholds_overlap
    [(PROT_STALL_REF_CDEG > PROT_STALL_ACT_CDEG) ? 1 : -1];
typedef char guard_prot_stall_trip_invalid
    [(PROT_STALL_TRIP >= 1) ? 1 : -1];
/* 冻结判据要求的拍数不能超过一个检测周期的控制拍数 */
typedef char guard_prot_stall_hold_ticks_unreachable
    [((PROT_STALL_HOLD_TICKS >= 1)
   && (PROT_STALL_HOLD_TICKS <= PROT_STALL_PERIOD_MS)) ? 1 : -1];

/* ============================ 扭矩上限设置 ============================ */

/* 把扭矩百分比换算成限流值和PWM上限起步值并下发给舵机 */
static void Protect_ApplyTorqueSetting(void)
{
    uint8_t pct = g_config.torque_limit; /* 配置里的扭矩百分比 */

    if (pct > PROT_TORQUE_MAX) pct = PROT_TORQUE_MAX;

    s_prot.torque_pct = pct;
    s_prot.i_limit_ma = (uint16_t)((uint32_t)pct * PROT_CURRENT_LIMIT_MA
                                 / PROT_TORQUE_MAX);
    s_prot.pwm_limit = (int16_t)((uint32_t)pct * CFG_PWM_FULL / PROT_TORQUE_MAX);
    if (s_prot.pwm_limit < PROT_LIMIT_MIN) s_prot.pwm_limit = PROT_LIMIT_MIN;
    A_Servo_SetOutputLimit(s_prot.pwm_limit);
}

/* 设置扭矩上限百分比并落盘，返回0=百分比越界 */
uint8_t A_Protect_SetTorque(uint8_t percent)
{
    if (percent > PROT_TORQUE_MAX) return 0U;

    g_config.torque_limit = percent;
    A_Config_MarkDirty();
    Protect_ApplyTorqueSetting();
    return 1U;
}

/* 读当前扭矩上限百分比 */
uint8_t A_Protect_GetTorque(void)
{
    return s_prot.torque_pct;
}

/* 读故障位掩码，堵转不在其中 */
uint8_t A_Protect_Fault(void)
{
    return s_prot.fault;
}

/* 1=最近一次动作以堵转收场(仅诊断)；只有看到机构真的动了才清除 */
uint8_t A_Protect_Stalled(void)
{
    return s_prot.stalled;
}

/* 最近一块绕组电流(mA)，供 RIV 查询；与限流回路用的是同一个数，采不到绕组电流时为0 */
uint16_t A_Protect_GetCurrent(void)
{
    return s_prot.last_ma;
}

/* ============================== 功率限制 ============================== */

/* 限流回路：快降慢升，瞬时过流立刻按住，恢复平滑 */
static void Protect_PowerLimit(uint8_t valid, uint16_t ma)
{
    int32_t limit = s_prot.pwm_limit; /* 本拍算出的新上限 */
    int32_t ceiling;                  /* 回升目标         */
    int32_t step;                     /* 降额步长         */

    if (s_prot.torque_pct == 0U)
    {
        /* 零扭矩不参与回升，保留最小限幅(0是控制器的取消限幅标记) */
        limit = PROT_LIMIT_MIN;
    }
    else if (valid && ma > s_prot.i_limit_ma)
    {
        step = (int32_t)(ma - s_prot.i_limit_ma) / PROT_LIMIT_DOWN_DIV;
        if (step < 1) step = 1;
        if (step > PROT_LIMIT_DOWN_MAX) step = PROT_LIMIT_DOWN_MAX;
        limit -= step;
    }
    else
    {
        /* 采样有效时可回升到满量程；采样无效(占空比太低或刹车段，电流不可测)时保持当前上限不动，
                 * 不回落到开环估计——否则停着时上限被持续压低，每条新指令都拿不到满功率。 */
        ceiling = valid ? (int32_t)CFG_PWM_FULL : limit;

        if (limit < ceiling)
        {
            limit += PROT_LIMIT_UP_STEP;
            if (limit > ceiling) limit = ceiling;
        }
    }

    if (limit < PROT_LIMIT_MIN) limit = PROT_LIMIT_MIN;
    if (limit > CFG_PWM_FULL)   limit = CFG_PWM_FULL;

    if ((int16_t)limit != s_prot.pwm_limit)
    {
        s_prot.pwm_limit = (int16_t)limit;
        A_Servo_SetOutputLimit(s_prot.pwm_limit);
    }
}

/* ============================= 检测链 ============================= */

/* 过流检测，每10ms一次：连续超阈值达到 PROT_CURRENT_TRIP 拍则卸力并进入冷却 */
static void Protect_OverCurrent(uint8_t valid, uint16_t ma)
{
    if (s_prot.cooldown != 0U)
    {
        /* 冷却期间不判(卸力后绕组无电流) */
        if (--s_prot.cooldown == 0U)
            s_prot.fault &= (uint8_t)~PROT_FAULT_OVERCUR;
        return;
    }

    if (valid && ma > PROT_CURRENT_LIMIT_MA)
    {
        if (s_prot.cur_cnt < PROT_CURRENT_TRIP) s_prot.cur_cnt++;
        if (s_prot.cur_cnt >= PROT_CURRENT_TRIP)
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

    if (!m.valid)
    {
        s_prot.stall_cnt = 0U;
        return;
    }

    d_ref = (m.ref_delta < 0) ? -m.ref_delta : m.ref_delta;
    d_act = (m.act_delta < 0) ? -m.act_delta : m.act_delta;

    /* 参考在走，或参考被饱和冻结按在原地(顶死时正是这样) */
    if ((d_ref >= PROT_STALL_REF_CDEG || m.hold_ticks >= PROT_STALL_HOLD_TICKS)
        && d_act <= PROT_STALL_ACT_CDEG)
    {
        if (++s_prot.stall_cnt >= PROT_STALL_TRIP)
        {
            s_prot.stall_cnt = 0U;
            s_prot.stalled   = 1U;
            A_Servo_HoldHere(); /* 目标改成当前实际位置，就地保持，不卸力 */
        }
        return;
    }

    s_prot.stall_cnt = 0U;
    /* 只有确实转起来才清诊断标志 */
    if (d_act > PROT_STALL_ACT_CDEG) s_prot.stalled = 0U;
}

/* 过温检测，250ms一次：超阈值置故障位，回落到 阈值-回差 才清除；读失败时保持原结论 */
static void Protect_OverTemp(void)
{
    int16_t temp = A_Temperature_Read(); /* 0.1摄氏度，-32768=读取失败 */

    if (temp == (int16_t)-32768) return; /* 保持上一次结论，不当成温度正常 */

    if (temp >= (int16_t)(PROT_TEMP_LIMIT_C * 10))
        s_prot.fault |= PROT_FAULT_OVERTEMP;
    else if (temp <= (int16_t)((PROT_TEMP_LIMIT_C - PROT_TEMP_HYST_C) * 10))
        s_prot.fault &= (uint8_t)~PROT_FAULT_OVERTEMP;
}

/* ============================== 故障落实 ============================== */

/* 把故障状态落实到舵机：逐拍复检(故障期间上位机可能发ULR或运动指令)；
 * 恢复只解除本模块发出的卸力，不撤销用户自己的ULK。 */
static void Protect_Enforce(void)
{
    if (s_prot.fault != PROT_FAULT_NONE)
    {
        if (A_Servo_TorqueOn())
        {
            A_Servo_Release(0U); /* 低阻力：自由转，功耗最低，降温最快 */
            s_prot.released = 1U;
        }
    }
    else if (s_prot.released)
    {
        s_prot.released = 0U;
        A_Servo_RestoreTorque(); /* 目标已在卸力时丢弃，恢复后原地保持 */
    }
}

/* =========================== 生命周期与任务 =========================== */

/* 初始化保护模块，须在 A_Servo_Init 之后调用 */
void A_Protect_Init(void)
{
    memset(&s_prot, 0, sizeof(s_prot));
    Protect_ApplyTorqueSetting();
}

/* 10ms任务入口：过流/限流每拍，堵转20ms，过温250ms，最后逐拍落实故障状态 */
void A_Protect_Task(void)
{
    uint16_t ma = 0U; /* 实测电流，毫安       */
    uint8_t  valid;   /* 本拍电流采样是否有效 */

    /* 配置被 CLE 等路径改动时跟上 */
    if (g_config.torque_limit != s_prot.torque_pct) Protect_ApplyTorqueSetting();

    /* valid 只表示采在驱动段：保护判据用 valid，诊断上报用最新读数 */
    valid = A_Current_Read(&ma);
    s_prot.last_ma = ma;

    Protect_OverCurrent(valid, ma);
    Protect_PowerLimit(valid, ma);

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

    Protect_Enforce();
}
