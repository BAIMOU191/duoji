/*
 * A_Servo.c —— 舵机编排层：一个控制拍的固定顺序 读编码器 -> 观测器 -> 轨迹 -> 控制器 -> H桥。
 * 位置模式(1~4, 11)走闭环；电机模式(7~10)开环给PWM、只用编码器数圈。
 * 反馈是电位器的线性行程：不回绕，两端之外有测不到角度的死区，掉进去时开环脱困。
 */

#include "A_Servo.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Protect.h"
#include "C_Speed_Observer.h"
#include "C_Plant_Adapt.h"
#include "C_Traj_Planner.h"
#include "C_Pos_Ctrl.h"
#include "C_Fixed.h"
#include "D_motor.h"
#include "D_tim.h"
#include "D_uart.h"   /* PA1(PWM捕获)与PC0(串口)是同一根信号线，上电二选一 */
#include "debug.h"
#include <string.h>

#define SERVO_ENC_FAIL_MAX      5U    /* 连续读失败这么多次才判编码器失效 */
#define SERVO_INPUT_DETECT_MS   60U   /* 上电嗅探PWM的时间窗，覆盖两个50Hz周期 */
#define SERVO_RETARGET_MIN_MS   5U    /* 运动中的普通目标最快每5ms合并重规划一次 */
#define SERVO_ESCAPE_DEPTH_CDEG 200   /* 脱困：读数回到可读区这么深才开始计内推 */
#define SERVO_ESCAPE_PWM        (CFG_PWM_FULL / 4)
#define SERVO_ESCAPE_TICKS      1500U /* 单方向最长脱困拍数 */
#define SERVO_ESCAPE_SETTLE     120U  /* 读数恢复后继续内推的拍数 */

/* 脉宽内部用 1/8us(Q3)，给PWM输入的一阶跟随留精度 */
#define Q3(us)       ((uint16_t)((uint16_t)(us) << 3))
#define Q3_MIN       Q3(SERVO_PWM_MIN)
#define Q3_MID       Q3(SERVO_PWM_MID)
#define Q3_MAX       Q3(SERVO_PWM_MAX)
#define Q3_SPAN      (Q3_MAX - Q3_MIN)
#define PWM_SNAP_Q3  Q3(7U)           /* PWM输入偏差超过7us判为真动作直接跳过去，否则1/8跟随 */
#define PWM_DB_US    2U               /* PWM输入的目标死区(us) */
#define SERVO_PWM_NOMINAL_CV  ((uint32_t)FW_VOLTAGE * 100U)   /* 电压补偿的标称电压，厘伏 */
#define VBUS_LIM_Q16 ((uint32_t)(((uint64_t)CFG_PWM_FULL << 16) / SERVO_PWM_NOMINAL_CV))

typedef char cfg_pwm_full_matches_motor[(CFG_PWM_FULL == MOTOR_PWM_MAX) ? 1 : -1];

enum { SERVO_ENC_OK = 0, SERVO_ENC_ESCAPING, SERVO_ENC_FAULT };        /* 编码器状态 */
enum { SERVO_TORQUE_ON = 0, SERVO_RELEASE_LOW, SERVO_RELEASE_HIGH };   /* 扭矩状态 */

typedef struct {
    ServoMode_t mode;
    uint8_t  reverse;           /* 1=脉宽增大角度减小 */
    uint8_t  range_valid;       /* 1=行程与可测范围有交集，否则禁止驱动 */
    uint8_t  input_source;      /* SERVO_INPUT_TX/PWM，上电定终身 */
    uint8_t  torque;            /* SERVO_TORQUE_ON/RELEASE_* */
    uint8_t  enc_state;         /* SERVO_ENC_* */
    uint8_t  enc_fail;          /* 连续读编码器失败次数 */
    uint8_t  paused;            /* 已暂停：刹车但保留目标 */
    uint8_t  resume_valid;      /* 有可继续的指令 */
    uint8_t  timed_position;    /* 当前是带时间参数的位置指令 */
    uint8_t  boot_pending;      /* 有指令(或上电动作)在等脱困结束 */
    uint8_t  retarget_pending;  /* 运动中收到的普通目标，等控制拍合并重规划 */
    uint8_t  retarget_age;      /* 距上次规划的拍数，饱和到 SERVO_RETARGET_MIN_MS */
    uint8_t  traj_done_prev;    /* 上一拍轨迹已走完：运动结束那一拍结算自适应 */
    uint8_t  pulse_ready;       /* PWM输入滤波器已有历史 */
    uint8_t  turn_sample_valid; /* raw_angle 是有效的定圈累计基准 */
    uint8_t  damping_acc;       /* 自定义阻尼的脉冲密度累加器 */
    uint8_t  motion_valid;      /* 上次堵转采样时正在跑轨迹 */
    uint8_t  escape_dir, escape_retried, vbus_n;
    uint16_t span_cdeg;         /* 有效行程，厘度；0=电机模式 */
    uint16_t angle;             /* 行程坐标[0,span]，只给目标和上报用 */
    uint16_t target_angle;      /* 当前目标 */
    uint16_t traj_lo, traj_hi;  /* 与规划器一致的目标范围 */
    uint16_t pulse_prev[2], pulse_filt;   /* PWM输入：三帧中值 + 一阶跟随，Q3 */
    uint16_t boot_pwm, boot_value;        /* 等脱困结束再执行的指令(Q3)，0=按上电动作 */
    uint16_t escape_ticks, escape_settle;
    uint16_t traj_hold;         /* 轨迹时钟被饱和冻结的拍数，堵转采样后清零 */
    uint16_t pwm_bus_cv;        /* 电压补偿用的母线电压(8样本均值)，0=还没有样本 */
    int16_t  motor_pwm;         /* 电机模式输出，含方向 */
    int16_t  out_limit;         /* 保护模块给的PWM上限 */
    int16_t  vbus_limit;        /* 当前电压下补偿后还能给出的等效PWM上限 */
    int16_t  ctrl_limit;        /* 已下发给控制器的上限 = min(out_limit, vbus_limit) */
    int16_t  last_pwm;          /* 上一拍实际输出(标称电压等效)，观测器要用 */
    int32_t  offset_cdeg;       /* 行程坐标0对应的编码器原始角度 */
    int32_t  angle_circ;        /* 未截断的反馈坐标，闭环只用它 */
    int32_t  raw_angle;         /* 编码器原始角度，定圈累计用 */
    int32_t  motion_ref, motion_act;
    uint32_t remaining;         /* 定圈=厘度，定时/带时间位置=拍，UINT32_MAX=无限 */
    uint32_t vbus_sum, comp_gain_q16, comp_back_q16;
    TrajRef_t     ref;
    SpeedObsOut_t obs;
    PosCtrlDbg_t  dbg;
} Servo_t;

static Servo_t s_servo;

static void Servo_SubmitQ3(uint16_t q3, uint16_t value);

/* ============================== 模式与坐标 ============================== */

static uint8_t Servo_IsPosition(void)
{
    return (uint8_t)(s_servo.mode <= SERVO_MODE_180_CCW || s_servo.mode == SERVO_MODE_CUSTOM);
}

static uint8_t Servo_IsTurns(void)
{
    return (uint8_t)(s_servo.mode == SERVO_MODE_TURNS_CW || s_servo.mode == SERVO_MODE_TURNS_CCW);
}

#define Servo_Offset() (s_servo.offset_cdeg)   /* 测试用 */

static uint16_t Servo_WrapAngle(int32_t a)
{
    while (a < 0) a += CDEG_RANGE;
    while (a >= CDEG_RANGE) a -= CDEG_RANGE;
    return (uint16_t)a;
}

/* 模式的标称行程(未经校准收缩)，0=电机模式 */
static uint16_t Servo_NominalSpan(ServoMode_t mode)
{
    if (mode == SERVO_MODE_CUSTOM) return g_config.custom_span_cdeg;
    return (mode <= SERVO_MODE_180_CCW) ? ((mode <= SERVO_MODE_270_CCW) ? 27000U : 18000U) : 0U;
}

/* 由校准锚点推有效行程与零点：MID 锚点是行程中点，ZERO 是500us端(反向时在高端)。
 * 可转入区只有 [TRAVEL_LO, TRAVEL_HI]，放不下标称行程就围绕锚点收缩；每次从标称行程重算，不叠加 */
static void Servo_DeriveCal(uint8_t kind, int32_t anchor, uint16_t nominal,
                            uint16_t *span_out, int32_t *offset_out)
{
    int32_t span = nominal, room, lo_room, hi_room;

    anchor  = Fx_ClampRange(anchor, ENCODER_TRAVEL_LO, ENCODER_TRAVEL_HI);
    lo_room = anchor - ENCODER_TRAVEL_LO;
    hi_room = ENCODER_TRAVEL_HI - anchor;
    if (kind == SERVO_CAL_MID) room = 2 * ((lo_room < hi_room) ? lo_room : hi_room);
    else                       room = s_servo.reverse ? lo_room : hi_room;
    if (span > room) span = room;
    *offset_out = (kind == SERVO_CAL_MID) ? anchor - span / 2 : (s_servo.reverse ? anchor - span : anchor);
    *span_out   = (uint16_t)span;
}

/* 按模式号展开行程、方向、零点；不支持的模式退回270度。奇数正向、偶数反向，CFG_DIR_INVERT 整体翻转 */
static void Servo_SetModeFields(ServoMode_t mode)
{
    uint16_t nominal, span;
    int32_t  anchor = g_config.cal_anchor_cdeg, offset;
    uint8_t  kind = g_config.cal_kind;

    if (!SERVO_MODE_IS_SUPPORTED(mode)) mode = SERVO_MODE_270_CW;
    s_servo.mode    = mode;
    s_servo.reverse = (uint8_t)(((mode == SERVO_MODE_CUSTOM) ? g_config.custom_reverse : ((mode & 1U) == 0U))
                                ^ CFG_DIR_INVERT);
    nominal = Servo_NominalSpan(mode);
    offset  = (mode == SERVO_MODE_CUSTOM) ? g_config.custom_offset_cdeg : g_config.position_offset_cdeg;
    if (offset > ENCODER_POT_ANGLE_MAX) offset -= CDEG_RANGE;   /* Flash 按[0,36000)存，负零点存成36000+偏移 */
    s_servo.span_cdeg   = nominal;
    s_servo.offset_cdeg = offset;
    if (nominal == 0U) return;
    if (kind == SERVO_CAL_NONE)
    {
        if (mode == SERVO_MODE_CUSTOM) return;   /* 自定义行程由 AMI/AMX 直接标出 */
        /* 旧版 SCK 的零点加半个行程还原成中点锚点；0=从未校准，取可转入区中点 */
        kind   = SERVO_CAL_MID;
        anchor = offset ? offset + nominal / 2 : (ENCODER_TRAVEL_LO + ENCODER_TRAVEL_HI) / 2;
    }
    Servo_DeriveCal(kind, anchor, nominal, &span, &offset);
    if (span < SERVO_CUSTOM_SPAN_MIN) return;    /* 推不出可用行程就保留标称值 */
    s_servo.span_cdeg   = span;
    s_servo.offset_cdeg = offset;
}

/* 读编码器，刷新原始角度、反馈坐标、行程坐标；读失败返回0，角度保持上次值 */
static uint8_t Servo_ReadPosition(void)
{
    int32_t raw = A_Encoder_Read();

    if (raw == ENCODER_ANGLE_ERROR) return 0U;
    s_servo.raw_angle  = raw;
    s_servo.angle_circ = raw - s_servo.offset_cdeg;   /* 闭环用，不截断：截断会在顶住端点时把误差抹成0 */
    s_servo.angle      = (uint16_t)Fx_ClampRange(s_servo.angle_circ, 0, s_servo.span_cdeg);
    s_servo.enc_fail   = 0U;
    return 1U;
}

/* 脉宽(Q3) -> 行程坐标，就近取整，与 Servo_AngleToPwm 往返一致 */
static uint16_t Servo_PulseToAngle(uint16_t q3)
{
    uint32_t a = ((uint32_t)(Fx_ClampRange(q3, Q3_MIN, Q3_MAX) - Q3_MIN) * s_servo.span_cdeg + Q3_SPAN / 2U) / Q3_SPAN;
    return (uint16_t)(s_servo.reverse ? s_servo.span_cdeg - a : a);
}

/* 行程坐标 -> 协议脉宽；电机模式统一返回中位 */
static uint16_t Servo_AngleToPwm(uint16_t angle)
{
    uint32_t a = s_servo.reverse ? (uint32_t)s_servo.span_cdeg - angle : angle;

    if (!Servo_IsPosition() || s_servo.span_cdeg == 0U) return SERVO_PWM_MID;
    return (uint16_t)(SERVO_PWM_MIN + (a * (SERVO_PWM_MAX - SERVO_PWM_MIN) + s_servo.span_cdeg / 2U) / s_servo.span_cdeg);
}

/* 轨迹可用范围 = 行程 ∩ 可转入区；端点外扩的可读余量就是兜超调的安全带 */
static void Servo_SetTrajRange(void)
{
    int32_t lo = ENCODER_TRAVEL_LO - s_servo.offset_cdeg, hi = ENCODER_TRAVEL_HI - s_servo.offset_cdeg;

    if (lo < 0) lo = 0;
    if (hi > s_servo.span_cdeg) hi = s_servo.span_cdeg;
    s_servo.range_valid = (uint8_t)(lo <= hi);
    s_servo.traj_lo = s_servo.range_valid ? (uint16_t)lo : 0U;
    s_servo.traj_hi = s_servo.range_valid ? (uint16_t)hi : 0U;
    if (s_servo.range_valid) C_Traj_Set_Range(lo, hi);
}

/* ============================== 输出 ============================== */

static void Servo_ApplyCtrlLimit(void)
{
    int16_t lim = (s_servo.vbus_limit < s_servo.out_limit) ? s_servo.vbus_limit : s_servo.out_limit;

    if (lim != s_servo.ctrl_limit) { s_servo.ctrl_limit = lim; C_PosCtrl_Set_Output_Limit(lim); }
}

/* 按新母线电压更新补偿系数；电压低到补偿后顶满时，等效上限一起告诉控制器(否则积分会在饱和里继续攒) */
static void Servo_VbusUpdate(uint16_t cv)
{
    uint32_t lim = (cv * VBUS_LIM_Q16) >> 16;

    s_servo.pwm_bus_cv    = cv;
    s_servo.comp_gain_q16 = ((SERVO_PWM_NOMINAL_CV << 16) + cv / 2U) / cv;
    s_servo.comp_back_q16 = ((uint32_t)cv << 16) / SERVO_PWM_NOMINAL_CV;
    s_servo.vbus_limit    = (int16_t)((lim > CFG_PWM_FULL) ? CFG_PWM_FULL : lim);
    Servo_ApplyCtrlLimit();
}

/* 带符号PWM乘Q16系数(四舍五入)，夹到满量程 */
static int16_t Servo_ScalePwm(int16_t pwm, uint32_t k_q16)
{
    uint32_t m = ((uint32_t)((pwm < 0) ? -pwm : pwm) * k_q16 + 0x8000U) >> 16;

    if (m > CFG_PWM_FULL) m = CFG_PWM_FULL;
    return (int16_t)((pwm < 0) ? -(int32_t)m : (int32_t)m);
}

/* 控制器给的是标称电压等效PWM：实际占空比 = 指令 x 标称电压 / 母线电压(8样本均值)，返回值折回标称等效给观测器。
 * 电压读数越过硬件安全边界时不补偿也不进均值，交给电压保护停机 */
static int16_t Servo_MotorSetCompensated(int16_t pwm)
{
    uint16_t cv;

    if (pwm == 0) return D_Motor_Set(0);
    cv = A_Voltage_Read();
    if (cv < PROT_VOLTAGE_ABS_MIN_DV * 10U || cv > PROT_VOLTAGE_ABS_MAX_DV * 10U) return D_Motor_Set(pwm);
    if (s_servo.pwm_bus_cv == 0U)
    {
        s_servo.vbus_sum = 0U; s_servo.vbus_n = 0U;
        Servo_VbusUpdate(cv);
    }
    else if ((s_servo.vbus_sum += cv, ++s_servo.vbus_n >= 8U))
    {
        Servo_VbusUpdate((uint16_t)((s_servo.vbus_sum + 4U) >> 3));
        s_servo.vbus_sum = 0U; s_servo.vbus_n = 0U;
    }
    return Servo_ScalePwm(D_Motor_Set(Servo_ScalePwm(pwm, s_servo.comp_gain_q16)), s_servo.comp_back_q16);
}

/* 按扭矩状态把H桥摆到静止态：有扭矩刹车，卸力按阻力档释放 */
static void Servo_ApplyTorqueOutput(void)
{
    if (s_servo.torque == SERVO_TORQUE_ON) s_servo.last_pwm = D_Motor_Set(0);
    else { D_Motor_Release((uint8_t)(s_servo.torque == SERVO_RELEASE_HIGH)); s_servo.last_pwm = 0; }
}

/* 学到的对象参数比同步给观测器、控制器、规划器：加速度增益比 = 惯量比 / 速度增益比 */
static void Servo_ApplyPlant(void)
{
    int32_t kv = C_PlantAdapt_Kv();
    int32_t ka = (int32_t)(((int64_t)C_PlantAdapt_Rho() << 12) / kv);

    C_SpeedObs_SetPlant(C_PlantAdapt_Rho(), kv);
    C_PosCtrl_SetPlant(ka, kv);
    C_Traj_SetPlant(ka);
}

/* 以当前行程坐标为起点重建轨迹与控制器；reinit_obs=1 连观测器一起重置(坐标跳变或卸力恢复后)。
 * 目标基准一起重锚，否则紧接着"回到旧目标"的指令会被目标死区吃掉 */
static void Servo_Resync(uint8_t reinit_obs)
{
    if (reinit_obs) C_SpeedObs_Init(s_servo.angle_circ);
    C_Traj_Hold(s_servo.angle);
    C_PosCtrl_Reset();
    C_Traj_Step(&s_servo.ref, 0U);
    memset(&s_servo.dbg, 0, sizeof(s_servo.dbg));
    C_PlantAdapt_Abort();      /* 被打断的运动不参与自学习 */
    s_servo.traj_done_prev   = 1U;
    s_servo.retarget_pending = 0U;
    s_servo.retarget_age     = SERVO_RETARGET_MIN_MS;
    s_servo.target_angle     = s_servo.angle;
}

/* 按 target_angle 重新规划；带时间的指令用剩余拍数 */
static void Servo_Replan(void)
{
    s_servo.retarget_pending = 0U;
    s_servo.retarget_age     = 0U;
    C_Traj_Plan(s_servo.target_angle,
                s_servo.timed_position ? (uint16_t)((s_servo.remaining > 0xFFFFU) ? 0xFFFFU : s_servo.remaining) : 0U);
}

/* 丢掉当前指令的全部记账(停止/改配置共用)；脱困中时置暂停，等新指令 */
static void Servo_ClearCommand(void)
{
    s_servo.paused       = (uint8_t)(s_servo.enc_state == SERVO_ENC_ESCAPING);
    s_servo.resume_valid = s_servo.boot_pending = s_servo.timed_position = s_servo.retarget_pending = 0U;
    s_servo.remaining    = 0U;
    s_servo.motor_pwm    = 0;
    s_servo.boot_pwm     = s_servo.boot_value = 0U;
    s_servo.retarget_age = SERVO_RETARGET_MIN_MS;
}

/* ============================== 脱困与上电 ============================== */

/* 上电动作：外部PWM指令优先，其次按配置的上电模式 */
static void Servo_ApplyBootAction(void)
{
    if (s_servo.boot_pwm != 0U) Servo_SubmitQ3(s_servo.boot_pwm, s_servo.boot_value);
    else if (g_config.boot_mode == SERVO_BOOT_GOTO_START)   /* 上电目标夹进脉冲边界照常执行 */
        A_Servo_Submit((uint16_t)Fx_ClampRange(g_config.startup_pwm, g_config.pulse_lo, g_config.pulse_hi), 0U);
    else if (g_config.boot_mode == SERVO_BOOT_RELEASE) A_Servo_Release(0U);
    else if (Servo_IsPosition()) C_Traj_Hold(s_servo.angle);
}

static void Servo_EscapeStart(void)
{
    s_servo.enc_state = SERVO_ENC_ESCAPING;
    s_servo.escape_dir = 1U;
    s_servo.escape_retried = s_servo.enc_fail = 0U;
    s_servo.escape_ticks = s_servo.escape_settle = 0U;
}

/* 脱困成功：停下重建环路，补上欠着的上电动作或没走完的指令 */
static void Servo_EscapeFinish(void)
{
    uint16_t target = s_servo.target_angle;

    s_servo.last_pwm  = D_Motor_Set(0);
    s_servo.enc_state = SERVO_ENC_OK;
    s_servo.enc_fail  = 0U;
    Servo_Resync(1U);
    if (s_servo.boot_pending) { s_servo.boot_pending = 0U; Servo_ApplyBootAction(); }
    else if (s_servo.resume_valid && Servo_IsPosition()) { s_servo.target_angle = target; Servo_Replan(); }
}

/* 脱困一拍：低速开环单向转，读数回来并转进行程够深够久才交回闭环；一个方向超时掉头，再超时卸力报故障 */
static void Servo_EscapeTick(void)
{
    int16_t pwm;

    if (++s_servo.escape_ticks >= (s_servo.escape_retried ? 2U : 1U) * SERVO_ESCAPE_TICKS)
    {
        if (s_servo.escape_retried)
        {
            s_servo.enc_state = SERVO_ENC_FAULT;
            s_servo.boot_pending = 0U;
            A_Servo_Release(0U);
            return;
        }
        s_servo.escape_retried = 1U;
        s_servo.escape_dir ^= 1U;
        s_servo.escape_ticks = s_servo.escape_settle = 0U;
    }
    if (!Servo_ReadPosition()) s_servo.escape_settle = 0U;
    else
    {   /* 有读数后按实测所在的物理端选内推方向 */
        s_servo.escape_dir = (uint8_t)(s_servo.raw_angle < ENCODER_ANGLE_MID);
        if (s_servo.raw_angle >= ENCODER_ANGLE_LO + SERVO_ESCAPE_DEPTH_CDEG
            && s_servo.raw_angle <= ENCODER_ANGLE_HI - SERVO_ESCAPE_DEPTH_CDEG) s_servo.escape_settle++;
        else s_servo.escape_settle = 0U;
        if (s_servo.escape_settle >= SERVO_ESCAPE_SETTLE) { Servo_EscapeFinish(); return; }
    }
    pwm = (int16_t)((s_servo.escape_dir ? SERVO_ESCAPE_PWM : -SERVO_ESCAPE_PWM) * CAL_MOTOR_SIGN);
    s_servo.last_pwm = Servo_MotorSetCompensated((int16_t)Fx_Clamp32(pwm, s_servo.out_limit));
}

/* 上电嗅探信号线上有没有PWM，有就以PWM为输入并返回脉宽(Q3)，否则交给串口返回0 */
static uint16_t Servo_DetectInput(void)
{
    uint16_t pulse = 0U;
    uint8_t  t;

    D_UART_Enable(0U);
    D_PWM_Input_Enable(1U);
    for (t = 0U; t < SERVO_INPUT_DETECT_MS && pulse == 0U; t++)
        if ((pulse = D_PWM_Read()) == 0U) Delay_Ms(1U);
    if (pulse == 0U)
    {
        D_PWM_Input_Enable(0U);
        D_UART_Enable(1U);
        s_servo.input_source = SERVO_INPUT_TX;
        return 0U;
    }
    s_servo.input_source = SERVO_INPUT_PWM;   /* 嗅探到的帧直接作为滤波器初值，避免开机抖动 */
    pulse = Q3(Fx_ClampRange(pulse, SERVO_PWM_MIN, SERVO_PWM_MAX));
    s_servo.pulse_prev[0] = s_servo.pulse_prev[1] = s_servo.pulse_filt = pulse;
    s_servo.pulse_ready = 1U;
    return pulse;
}

/* PWM输入两级滤波(Q3)：三帧中值去孤立毛刺，再1/8一阶跟随压抖动 */
static uint16_t Servo_FilterPulse(uint16_t us)
{
    uint16_t c = Q3(Fx_ClampRange(us, SERVO_PWM_MIN, SERVO_PWM_MAX)), a, b, m;
    int32_t  err;

    if (!s_servo.pulse_ready)
    {
        s_servo.pulse_prev[0] = s_servo.pulse_prev[1] = s_servo.pulse_filt = c;
        s_servo.pulse_ready = 1U;
    }
    a = s_servo.pulse_prev[0]; b = s_servo.pulse_prev[1];
    s_servo.pulse_prev[0] = b; s_servo.pulse_prev[1] = c;
    if (a > b) { m = a; a = b; b = m; }
    if (b > c) b = c;
    m   = (a > b) ? a : b;
    err = (int32_t)m - s_servo.pulse_filt;
    s_servo.pulse_filt = (uint16_t)((err >= PWM_SNAP_Q3 || err <= -(int32_t)PWM_SNAP_Q3) ? m : s_servo.pulse_filt + err / 8);
    return s_servo.pulse_filt;
}

/* ============================== 生命周期 ============================== */

void A_Servo_ApplyTrajConfig(void)
{
    C_Traj_Set_Limits((int32_t)g_runtime_config.traj_vmax_dps * 100, (int32_t)g_runtime_config.traj_amax_dps2 * 100,
                      (int32_t)g_runtime_config.traj_dmax_dps2 * 100, (int32_t)g_runtime_config.traj_jmax_dps3 * 100);
    C_Traj_Set_MoveMinMs(g_runtime_config.traj_move_min_ms);
}

/* 让新的模式/校准立即生效，所有环路对齐到当前实测位置 */
void A_Servo_ApplyConfig(void)
{
    A_Servo_ApplyTrajConfig();
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    (void)Servo_ReadPosition();
    Servo_ClearCommand();
    if (Servo_IsPosition()) { Servo_SetTrajRange(); Servo_Resync(1U); }
    else { s_servo.enc_state = SERVO_ENC_OK; s_servo.paused = 0U; C_PosCtrl_Reset(); }
    Servo_ApplyTorqueOutput();
}

void A_Servo_Init(void)
{
    uint8_t enc_ok;

    memset(&s_servo, 0, sizeof(s_servo));
    s_servo.out_limit = s_servo.vbus_limit = s_servo.ctrl_limit = CFG_PWM_FULL;
    s_servo.boot_pwm  = Servo_DetectInput();
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    enc_ok = Servo_ReadPosition();
    s_servo.target_angle = s_servo.angle;   /* 目标基准=当前位置，否则上电第一条 P0500 可能被死区吃掉 */
    C_SpeedObs_Init(s_servo.angle_circ);
    C_Traj_Init(s_servo.angle);
    A_Servo_ApplyTrajConfig();
    if (Servo_IsPosition()) Servo_SetTrajRange();
    C_PosCtrl_Init();
    C_PlantAdapt_Reset();                   /* 上电从标定模型开始；运行中的重同步保留已学到的负载 */
    Servo_ApplyPlant();
    s_servo.torque   = SERVO_TORQUE_ON;
    s_servo.last_pwm = D_Motor_Set(0);
    C_Traj_Step(&s_servo.ref, 0U);
    if (enc_ok) Servo_ApplyBootAction();
    else { s_servo.boot_pending = 1U; Servo_EscapeStart(); }   /* 上电就在死区：先脱困再执行上电动作 */
}

/* ============================== 运动指令 ============================== */

/* 运动指令内核(脉宽Q3)，总线与PWM输入共用。越界或故障整条不响应 */
static void Servo_SubmitQ3(uint16_t q3, uint16_t value)
{
    uint8_t  was_paused = s_servo.paused;
    uint16_t lo = Q3(g_config.pulse_lo), hi = Q3(g_config.pulse_hi), target;
    int32_t  delta, db;

    if (lo < Q3_MIN) lo = Q3_MIN;
    if (hi > Q3_MAX) hi = Q3_MAX;
    if (lo > hi || q3 < lo || q3 > hi || s_servo.enc_state == SERVO_ENC_FAULT
        || A_Protect_Fault() != PROT_FAULT_NONE || (Servo_IsPosition() && !s_servo.range_valid)) return;
    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();
    if (s_servo.enc_state == SERVO_ENC_ESCAPING)
    {   /* 脱困结束后再执行，只留最新一条 */
        s_servo.boot_pwm = q3;
        s_servo.boot_value = value;
        s_servo.boot_pending = s_servo.resume_valid = 1U;
        s_servo.paused = 0U;
        return;
    }
    if (was_paused && Servo_IsPosition())
    {   /* 暂停后的新目标从停稳位置起步 */
        if (!Servo_ReadPosition()) return;
        Servo_Resync(1U);
    }
    s_servo.paused = 0U;
    s_servo.resume_valid = 1U;

    if (!Servo_IsPosition())
    {   /* 电机模式：中位为零速，两侧线性映射到正负满PWM；value 定圈=圈数、定时=秒，0=无限 */
        s_servo.motor_pwm = (int16_t)((q3 >= Q3_MID) ? (int32_t)((uint32_t)(q3 - Q3_MID) * MOTOR_PWM_MAX / (Q3_MAX - Q3_MID))
                                                     : -(int32_t)((uint32_t)(Q3_MID - q3) * MOTOR_PWM_MAX / (Q3_MID - Q3_MIN)));
        if (s_servo.reverse != (CAL_MOTOR_SIGN < 0)) s_servo.motor_pwm = (int16_t)-s_servo.motor_pwm;
        s_servo.remaining = value ? (uint32_t)value * (Servo_IsTurns() ? CDEG_RANGE : CFG_TICK_HZ) : UINT32_MAX;
        s_servo.enc_fail  = 0U;
        if (value && Servo_IsTurns())
        {
            int32_t raw = A_Encoder_Read();
            s_servo.turn_sample_valid = (uint8_t)(raw != ENCODER_ANGLE_ERROR);
            if (s_servo.turn_sample_valid) s_servo.raw_angle = raw;
        }
        return;
    }

    /* 位置模式。总线目标逐条接收；PWM输入只滤掉入口抖动(2us)；带时间参数的指令一律放行 */
    target = (uint16_t)Fx_ClampRange(Servo_PulseToAngle(q3), s_servo.traj_lo, s_servo.traj_hi);
    delta  = (int32_t)target - s_servo.target_angle;
    db     = (s_servo.input_source == SERVO_INPUT_PWM)
           ? (int32_t)(((uint32_t)s_servo.span_cdeg * PWM_DB_US + (SERVO_PWM_MAX - SERVO_PWM_MIN) - 1U)
                       / (SERVO_PWM_MAX - SERVO_PWM_MIN)) : 0;
    if (!was_paused && value == 0U && delta <= db && delta >= -db) return;
    s_servo.target_angle   = target;
    s_servo.timed_position = (uint8_t)(value != 0U);
    s_servo.remaining      = 0U;
    if (value != 0U || C_Traj_Is_Done())
    {   /* 带时间的指令自己定义整段时间、不合并；空闲时普通指令立即起步 */
        s_servo.retarget_pending = 0U;
        s_servo.retarget_age = 0U;
        C_Traj_Plan(target, value);
        if (value != 0U) s_servo.remaining = C_Traj_Get_Ticks();
    }
    else s_servo.retarget_pending = 1U;   /* 运动中只覆盖邮箱，由控制拍合并重规划 */
}

/* 协议入口：pwm=目标脉宽(电机模式为速度与方向)，value=时间(ms；定圈为圈数；0=最快/无限) */
void A_Servo_Submit(uint16_t pwm, uint16_t value)
{
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX || !A_Protect_CanDrive()) return;   /* 先拦再升Q3，防溢出 */
    Servo_SubmitQ3(Q3(pwm), value);
}

void A_Servo_Pause(void)
{
    uint16_t target = s_servo.target_angle;

    if (s_servo.torque != SERVO_TORQUE_ON || s_servo.paused || (!s_servo.resume_valid && !s_servo.boot_pending)) return;
    s_servo.paused = s_servo.resume_valid = 1U;
    if (Servo_IsPosition()) { (void)Servo_ReadPosition(); Servo_Resync(0U); s_servo.target_angle = target; }
    s_servo.last_pwm = D_Motor_Set(0);
}

/* 继续：从当前位置按剩余时间重新规划到原目标 */
void A_Servo_Resume(void)
{
    uint16_t target = s_servo.target_angle;

    if (!s_servo.paused || !s_servo.resume_valid) return;
    if (Servo_IsPosition() && s_servo.enc_state != SERVO_ENC_ESCAPING)
    {   /* 暂停期间可能滑过，以最新反馈重建起点；读不到就保持暂停，允许重发 */
        if (!Servo_ReadPosition()) return;
        Servo_Resync(1U);
        s_servo.target_angle = target;
    }
    s_servo.paused = 0U;
    if (s_servo.enc_state != SERVO_ENC_ESCAPING && Servo_IsPosition()) Servo_Replan();
}

/* 停止：丢弃目标(不可继续)，按扭矩状态收尾 */
void A_Servo_Stop(void)
{
    Servo_ClearCommand();
    if (Servo_IsPosition()) { (void)Servo_ReadPosition(); Servo_Resync(0U); }
    Servo_ApplyTorqueOutput();
}

/* 卸力：0=低阻力自由转，1=高阻力短路制动；先改状态再停，避免先刹一下 */
void A_Servo_Release(uint8_t high_resistance)
{
    g_runtime_config.torque_mode = high_resistance ? 2U : 0U;
    s_servo.torque      = high_resistance ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
    s_servo.damping_acc = 0U;
    A_Servo_Stop();
}

void A_Servo_RestoreTorque(void)
{
    if (!A_Protect_CanDrive() || s_servo.torque == SERVO_TORQUE_ON) return;
    g_runtime_config.torque_mode = 1U;
    s_servo.torque = SERVO_TORQUE_ON;
    (void)Servo_ReadPosition();
    if (Servo_IsPosition()) Servo_Resync(1U);
    s_servo.last_pwm = D_Motor_Set(0);
}

uint8_t A_Servo_SetTorqueMode(uint8_t mode)
{
    if (mode > 2U || (mode == 1U && !A_Protect_CanDrive())) return 0U;
    if (mode == 1U) A_Servo_RestoreTorque();
    else A_Servo_Release((uint8_t)(mode == 2U));
    return 1U;
}

uint8_t A_Servo_SetDamping(uint8_t percent)
{
    if (percent > 100U) return 0U;
    g_runtime_config.damping_percent = percent;
    s_servo.damping_acc = 0U;
    return 1U;
}

/* ============================== 参数类指令 ============================== */

/* 切换工作模式并落盘；11(自定义)只能由 AMI/AMX 进入 */
uint8_t A_Servo_SetMode(uint8_t mode)
{
    if (!SERVO_MODE_IS_SUPPORTED(mode) || mode > SERVO_MODE_TIMED_CCW) return 0U;
    A_Servo_Stop();
    g_config.servo_mode = mode;
    A_Config_MarkDirty();
    A_Servo_ApplyConfig();
    return 1U;
}

/* SCK/SCZ：当前位置记为锚点(中点/500us端)，有效行程与零点由 Servo_SetModeFields 现推 */
static uint8_t Servo_Calibrate(uint8_t kind)
{
    int32_t  raw = A_Encoder_Read(), offset;
    uint16_t nominal = Servo_NominalSpan(s_servo.mode), span;

    if (!Servo_IsPosition() || raw == ENCODER_ANGLE_ERROR || raw < ENCODER_TRAVEL_LO
        || raw > ENCODER_TRAVEL_HI || nominal == 0U) return 0U;
    Servo_DeriveCal(kind, raw, nominal, &span, &offset);
    if (span < SERVO_CUSTOM_SPAN_MIN) return 0U;   /* 推不出至少1度的行程：本次校准作废 */
    g_config.cal_kind        = kind;
    g_config.cal_anchor_cdeg = (uint16_t)raw;
    A_Servo_Stop();
    A_Config_MarkDirty();
    Servo_SetModeFields(s_servo.mode);
    (void)Servo_ReadPosition();
    Servo_SetTrajRange();
    Servo_Resync(1U);
    return 1U;
}

uint8_t A_Servo_CalibrateMid(void)  { return Servo_Calibrate((uint8_t)SERVO_CAL_MID); }
uint8_t A_Servo_CalibrateZero(void) { return Servo_Calibrate((uint8_t)SERVO_CAL_ZERO); }

/* AMI/AMX：把行程一端收到当前位置并进入自定义模式(is_min=1为500us端，在哪头由方向决定)，可连续收窄 */
uint8_t A_Servo_SetTravelEnd(uint8_t is_min)
{
    int32_t  raw, pos;
    uint16_t span;
    uint8_t  raise_low = (uint8_t)(is_min != s_servo.reverse);   /* 1=抬低端(零点跟着挪)，0=压高端 */

    if (!Servo_IsPosition() || (raw = A_Encoder_Read()) == ENCODER_ANGLE_ERROR) return 0U;
    pos  = Fx_ClampRange(raw - s_servo.offset_cdeg, 0, s_servo.span_cdeg);
    span = (uint16_t)(raise_low ? s_servo.span_cdeg - pos : pos);
    if (span < SERVO_CUSTOM_SPAN_MIN) return 0U;   /* 先验证再动手：被拒的不停舵机、不写Flash */
    A_Servo_Stop();
    g_config.custom_offset_cdeg = Servo_WrapAngle(s_servo.offset_cdeg + (raise_low ? pos : 0) + CDEG_RANGE);
    g_config.custom_span_cdeg   = span;
    g_config.custom_reverse     = (uint8_t)(s_servo.reverse ^ CFG_DIR_INVERT);   /* 存翻转前的原始方向 */
    g_config.servo_mode         = SERVO_MODE_CUSTOM;
    g_config.cal_kind           = (uint8_t)SERVO_CAL_NONE;                       /* 旧校准锚点作废 */
    A_Config_MarkDirty();
    A_Servo_ApplyConfig();
    return 1U;
}

/* SMI/SMX：当前位置的脉宽设为可响应边界(is_min=1下界)，不改行程映射；两端不得交叉 */
uint8_t A_Servo_SetPulseLimit(uint8_t is_min)
{
    uint16_t pulse;

    if (!Servo_IsPosition() || s_servo.span_cdeg == 0U || !Servo_ReadPosition()) return 0U;
    pulse = Servo_AngleToPwm(s_servo.angle);
    if (is_min ? pulse >= g_config.pulse_hi : pulse <= g_config.pulse_lo) return 0U;
    if (is_min) g_config.pulse_lo = pulse;
    else        g_config.pulse_hi = pulse;
    A_Config_MarkDirty();
    return 1U;
}

uint8_t A_Servo_SaveStartup(void)
{
    if (!Servo_IsPosition() || !Servo_ReadPosition()) return 0U;
    g_config.startup_pwm = Servo_AngleToPwm(s_servo.angle);
    A_Config_MarkDirty();
    return 1U;
}

/* ============================== 查询 ============================== */

uint8_t A_Servo_TorqueOn(void) { return (uint8_t)(s_servo.torque == SERVO_TORQUE_ON); }

uint8_t A_Servo_GetWorkState(void)
{
    if (s_servo.enc_state == SERVO_ENC_FAULT || A_Protect_Fault() != PROT_FAULT_NONE) return SERVO_WORK_FAULT;
    if (s_servo.enc_state == SERVO_ENC_ESCAPING) return SERVO_WORK_RECOVERING;
    if (s_servo.torque == SERVO_RELEASE_LOW)     return SERVO_WORK_RELEASE_LOW;
    if (s_servo.torque == SERVO_RELEASE_HIGH)    return SERVO_WORK_RELEASE_HIGH;
    if (s_servo.paused)                          return SERVO_WORK_PAUSED;
    if (Servo_IsPosition() ? !C_Traj_Is_Done() : s_servo.remaining != 0U) return SERVO_WORK_MOVING;
    return SERVO_WORK_IDLE;
}

/* 只读缓存，不读编码器、不推进状态 */
void A_Servo_GetTelemetry(ServoTelemetry_t *out)
{
    out->target_pwm   = Servo_AngleToPwm(s_servo.target_angle);
    out->position_pwm = Servo_AngleToPwm(s_servo.angle);
    out->speed_cdps   = (Servo_IsPosition() && s_servo.range_valid && !s_servo.paused
                         && s_servo.torque == SERVO_TORQUE_ON && s_servo.enc_state == SERVO_ENC_OK
                         && s_servo.enc_fail == 0U && s_servo.obs.vel != INT32_MIN)
                      ? (s_servo.reverse ? -s_servo.obs.vel : s_servo.obs.vel) : INT32_MIN;
}

/* 重读一次位置再换算成协议脉宽(RAD 查询) */
uint16_t A_Servo_GetPositionPwm(void)
{
    if (Servo_IsPosition()) (void)Servo_ReadPosition();
    return Servo_AngleToPwm(s_servo.angle);
}

/* ============================== 对保护模块的接口 ============================== */

void A_Servo_SetOutputLimit(int16_t limit)
{
    s_servo.out_limit = (int16_t)Fx_ClampRange(limit, 1, CFG_PWM_FULL);
    Servo_ApplyCtrlLimit();
}

/* 堵转采样：本区间参考与机构各走了多少，并把基准推到本拍；valid 要求两端都在跑同一条未走完的轨迹 */
void A_Servo_SampleMotion(ServoMotion_t *out)
{
    uint8_t driving = (uint8_t)(Servo_IsPosition() && s_servo.torque == SERVO_TORQUE_ON && !s_servo.paused
                                && s_servo.range_valid && s_servo.enc_state == SERVO_ENC_OK && !C_Traj_Is_Done());

    out->ref_delta  = s_servo.ref.pos - s_servo.motion_ref;
    out->act_delta  = s_servo.angle_circ - s_servo.motion_act;
    out->hold_ticks = s_servo.traj_hold;
    out->valid      = (uint8_t)(driving && s_servo.motion_valid);
    s_servo.motion_ref   = s_servo.ref.pos;
    s_servo.motion_act   = s_servo.angle_circ;
    s_servo.motion_valid = driving;
    s_servo.traj_hold    = 0U;
}

/* 堵转处理：放弃当前动作就地保持，不卸力 */
void A_Servo_HoldHere(void)
{
    if (Servo_IsPosition() && s_servo.torque == SERVO_TORQUE_ON) A_Servo_Stop();
}

/* ============================== 控制拍 ============================== */

/* 电机模式一拍：开环输出 + 定圈/定时倒计时 */
static void Servo_MotorControl(void)
{
    int32_t raw, delta;

    if (s_servo.remaining == 0U || s_servo.motor_pwm == 0) { s_servo.last_pwm = D_Motor_Set(0); return; }
    /* 定圈：按编码器累计沿指令方向的位移倒计时；穿过死区读不到时保留上次采样，下次读数会包含这段位移 */
    if (Servo_IsTurns() && s_servo.remaining != UINT32_MAX && (raw = A_Encoder_Read()) != ENCODER_ANGLE_ERROR)
    {
        if (s_servo.turn_sample_valid)
        {
            delta = raw - s_servo.raw_angle;
            if (delta > CDEG_RANGE / 2) delta -= CDEG_RANGE;
            else if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
            if (s_servo.motor_pwm * CAL_MOTOR_SIGN < 0) delta = -delta;
            if (delta > 0) s_servo.remaining = ((uint32_t)delta < s_servo.remaining) ? s_servo.remaining - delta : 0U;
        }
        s_servo.raw_angle = raw;
        s_servo.turn_sample_valid = 1U;
        s_servo.enc_fail = 0U;
    }
    if (s_servo.remaining == 0U) { s_servo.last_pwm = D_Motor_Set(0); return; }
    s_servo.last_pwm = Servo_MotorSetCompensated((int16_t)Fx_Clamp32(s_servo.motor_pwm, s_servo.out_limit));
    if (!Servo_IsTurns() && s_servo.remaining != UINT32_MAX) s_servo.remaining--;
}

/* 1ms控制拍 */
void A_Servo_Control(void)
{
    uint16_t pulse;
    uint8_t  hold;

    if (s_servo.input_source == SERVO_INPUT_PWM && (pulse = D_PWM_Read()) != 0U)
        Servo_SubmitQ3(Servo_FilterPulse(pulse), 0U);   /* PWM输入：每帧当一条新指令 */

    if (s_servo.enc_state == SERVO_ENC_FAULT)
    {   /* 脱困失败后不再自己动，轴被拨回可测范围即解除，扭矩保持卸力 */
        if (Servo_ReadPosition()) { s_servo.enc_state = SERVO_ENC_OK; Servo_Resync(1U); }
        return;
    }
    if (s_servo.torque != SERVO_TORQUE_ON)
    {   /* 自定义阻尼：在全短路制动和全自由之间按百分比交替，不产生方向电压 */
        if (s_servo.torque == SERVO_RELEASE_HIGH)
        {
            uint16_t sum = (uint16_t)s_servo.damping_acc + g_runtime_config.damping_percent;
            s_servo.damping_acc = (uint8_t)((sum >= 100U) ? sum - 100U : sum);
            D_Motor_Release((uint8_t)(sum >= 100U));
            s_servo.last_pwm = 0;
        }
        return;
    }
    if (s_servo.paused || (Servo_IsPosition() && !s_servo.range_valid)) { s_servo.last_pwm = D_Motor_Set(0); return; }
    if (s_servo.enc_state == SERVO_ENC_ESCAPING) { Servo_EscapeTick(); return; }
    if (!Servo_IsPosition()) { Servo_MotorControl(); return; }

    if (!Servo_ReadPosition() && ++s_servo.enc_fail >= SERVO_ENC_FAIL_MAX)
    {   /* 编码器连续失效：先停，再交给脱困 */
        s_servo.last_pwm = D_Motor_Set(0);
        C_PosCtrl_Reset();
        Servo_EscapeStart();
        return;
    }
    C_SpeedObs_Update(s_servo.angle_circ, s_servo.last_pwm, &s_servo.obs);
    C_PlantAdapt_Sample(&s_servo.obs, (uint8_t)!C_Traj_Is_Done());

    /* 运动中的普通目标只在控制拍重规划，最快每5ms一次；旧轨迹已走完就不再等 */
    if (s_servo.retarget_age < SERVO_RETARGET_MIN_MS) s_servo.retarget_age++;
    if (s_servo.retarget_pending && !s_servo.timed_position
        && (s_servo.retarget_age >= SERVO_RETARGET_MIN_MS || C_Traj_Is_Done())) Servo_Replan();

    /* 输出饱和且确实跟不上(误差与参考速度同号)时冻结轨迹时钟，冻结拍数留给堵转检测 */
    hold = (uint8_t)(s_servo.dbg.sat && (int64_t)s_servo.dbg.e_pos * s_servo.ref.vel > 0);
    if (hold && s_servo.traj_hold < 0xFFFFU) s_servo.traj_hold++;
    C_Traj_Step(&s_servo.ref, hold);
    s_servo.last_pwm = Servo_MotorSetCompensated(C_PosCtrl_Update(&s_servo.ref, &s_servo.obs, C_Traj_Is_Done(), &s_servo.dbg));

    /* 一段运动刚走完：结算自适应，学到的比值下一段运动起生效 */
    if (C_Traj_Is_Done() && !s_servo.traj_done_prev && C_PlantAdapt_MoveEnd()) Servo_ApplyPlant();
    s_servo.traj_done_prev = C_Traj_Is_Done();
    if (s_servo.timed_position && s_servo.remaining != 0U) s_servo.remaining--;
}
