/*
 * A_Servo.c —— 舵机编排层：一个控制拍的固定顺序 读编码器 -> 观测器 -> 轨迹 -> 控制器 -> H桥。
 * 位置模式(1~4, 11)走闭环；电机模式(7~10)开环给PWM、只用编码器数圈。
 * 反馈是电位器的线性行程：不回绕，两端之外有测不到角度的死区，掉进去时开环脱困。
 */

#include "A_Servo.h"
#include "A_Config.h"
#include "C_Motor_Model.h"
#include "A_Sensor.h"
#include "A_Protect.h"
#include "C_Speed_Observer.h"
#include "C_Traj_Planner.h"
#include "C_Pos_Ctrl.h"
#include "C_Fixed.h"
#include "D_motor.h"
#include "D_adc.h"
#include "D_tim.h"
#include "D_uart.h"   /* PA1(PWM捕获)与PC0(串口)是同一根信号线，上电二选一 */
#include "debug.h"
#include <string.h>

#define SERVO_ENC_FAIL_MAX      5U    /* 连续读失败这么多次才判编码器失效 */
#define SERVO_INPUT_DETECT_MS   60U   /* 上电嗅探PWM的时间窗，覆盖两个50Hz周期 */
#define SERVO_MERGE_MS          4U    /* 更密的普通目标合并成一条(只留最新) */
#define SERVO_HOLD_KP_SLEW_MS   10U   /* 运行中改保持 P：每10ms最多变 1‰/°，避免力矩阶跃 */
#define SERVO_BOOT_MOVE_MS      1500U /* 上电位置复位传给规划器的请求时间，ms */
#define SERVO_ESCAPE_DEPTH_CDEG 200   /* 脱困：读数回到可读区这么深才开始计内推 */
#define SERVO_ESCAPE_DUTY       (CFG_PWM_FULL / 4)
#define SERVO_ESCAPE_TICKS      1500U /* 单方向最长脱困拍数 */
#define SERVO_ESCAPE_SETTLE     120U  /* 读数恢复后继续内推的拍数 */

/* 脉宽内部用 1/8us(Q3)，给PWM输入的一阶跟随留精度 */
#define Q3(us)       ((uint16_t)((uint16_t)(us) << 3))
#define Q3_MIN       Q3(SERVO_PWM_MIN)
#define Q3_MID       Q3(SERVO_PWM_MID)
#define Q3_MAX       Q3(SERVO_PWM_MAX)
#define Q3_SPAN      (Q3_MAX - Q3_MIN)
#define PWM_DB_Q3    Q3(2U)           /* PWM输入回差死区：静止吸收±2us抖动，移动时连续跟随 */

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
    uint8_t  paused;            /* 位置模式保持暂停点，保留原目标 */
    uint8_t  arrived;           /* 1=本条指令已首次到位；外力推动不改变，下一条指令清零 */
    uint8_t  move_pending;      /* 有一条被合并、等最小间隔到了再下发的普通目标 */
    uint16_t hold_kp;           /* 当前生效的保持 P(‰/°)，缓慢跟随运行参数 */
    uint8_t  stiff_slew_ms;
    uint8_t  resume_valid;      /* 有可继续的指令 */
    uint8_t  timed_position;    /* 当前是带时间参数的位置指令 */
    uint8_t  boot_pending;      /* 有指令(或上电动作)在等脱困结束 */
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
    uint16_t saturation_ticks; /* 输出饱和且跟踪落后、轨迹时钟被冻结的拍数，供堵转采样 */
    uint16_t move_age;          /* 距上一条已下发普通目标的毫秒数，合并过密目标用；饱和在0xFFFF */
    uint16_t pwm_bus_cv;        /* 母线电压(驱动期间8样本均值)，厘伏 */
    int32_t  to_duty_q16;       /* 占空比/mV，Q16：电压与占空比的唯一换算处 */
    int32_t  to_mv_q16;         /* mV/占空比，Q16 */
    int32_t  motor_mv;          /* 电机模式输出电压，含方向；满量程为标定电压 */
    int16_t  out_limit;         /* 保护模块给的PWM上限 */
    int16_t  applied_pwm;       /* 诊断：D_Motor_Set返回的实际PWM */
    uint8_t  debug_control_valid; /* 诊断：最近一次输出来自本拍闭环 */
    uint8_t  debug_position_valid; /* 诊断：最近一次位置读取是否成功 */
    int32_t  ctrl_limit_mv;     /* 保护上限对应的可用电压，控制器硬限幅 */
    int32_t  applied_mv;        /* 上一拍实际施加电压(角度增大为正)，供观测器 */
    int32_t  offset_cdeg;       /* 行程坐标0对应的编码器原始角度 */
    int32_t  angle_circ;        /* 未截断的反馈坐标，闭环只用它 */
    int32_t  raw_angle;         /* 编码器原始角度，定圈累计用 */
    int32_t  motion_ref, motion_act;
    uint32_t remaining;         /* 定圈=厘度，定时/带时间位置=拍，UINT32_MAX=无限 */
    uint32_t vbus_sum;
    TrajRef_t     ref;
    SpeedObserver_t observer;
    SpeedObsOut_t obs;
    PosCtrlDbg_t  dbg;
} Servo_t;

static Servo_t s_servo;

static uint8_t Servo_SubmitQ3(uint16_t q3, uint16_t value);

/* ============================== 模式与坐标 ============================== */

static uint8_t Servo_IsPosition(void)
{
    return (uint8_t)(s_servo.mode <= SERVO_MODE_180_CCW || s_servo.mode == SERVO_MODE_CUSTOM);
}

static uint8_t Servo_IsTurns(void)
{
    return (uint8_t)(s_servo.mode == SERVO_MODE_TURNS_CW || s_servo.mode == SERVO_MODE_TURNS_CCW);
}

#define Servo_Offset() (s_servo.offset_cdeg)   /* 当前行程零点 */

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

    s_servo.debug_position_valid = (uint8_t)(raw != ENCODER_ANGLE_ERROR);
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
    if (s_servo.range_valid) C_Traj_SetRange(lo, hi);
}

/* ============================== 输出 ============================== */

/* 电压 <-> 占空比：全链路以 mV 计，只在这里按实测母线电压换算 */
static int16_t Servo_MvToDuty(int32_t mv) { return (int16_t)Fx_Clamp32(Fx_MulShift(mv, s_servo.to_duty_q16, 16), CFG_PWM_FULL); }
static int32_t Servo_DutyToMv(int16_t duty) { return Fx_MulShift(duty, s_servo.to_mv_q16, 16); }

/* 保护给的占空比上限在当前母线电压下对应的电压 */
static void Servo_ApplyCtrlLimit(void)
{
    s_servo.ctrl_limit_mv = Servo_DutyToMv(s_servo.out_limit);
}

static void Servo_VbusUpdate(uint16_t cv)
{
    uint32_t mv = (uint32_t)cv * 10U;

    if (cv < PROT_VOLTAGE_ABS_MIN_DV * 10U || cv > PROT_VOLTAGE_ABS_MAX_DV * 10U) return;
    s_servo.pwm_bus_cv  = cv;
    s_servo.to_duty_q16 = (int32_t)(((uint32_t)CFG_PWM_FULL << 16) / mv);
    s_servo.to_mv_q16   = (int32_t)((mv << 16) / CFG_PWM_FULL);
    Servo_ApplyCtrlLimit();
}

/* 驱动H桥(物理方向占空比)并记下实际施加电压；驱动期间母线电压8拍平均后更新换算(不跟随单拍压降) */
static void Servo_Drive(int16_t duty)
{
    uint16_t cv = A_Voltage_Read();

    if (s_servo.pwm_bus_cv == 0U) Servo_VbusUpdate(cv); /* 首份有效电压不依赖驱动已出力。 */
    else if (duty != 0 && cv >= PROT_VOLTAGE_ABS_MIN_DV * 10U && cv <= PROT_VOLTAGE_ABS_MAX_DV * 10U) {
        s_servo.vbus_sum += cv;
        if (++s_servo.vbus_n >= 8U) {
            cv = (uint16_t)((s_servo.vbus_sum + 4U) >> 3);
            if (cv != s_servo.pwm_bus_cv) Servo_VbusUpdate(cv);
            s_servo.vbus_sum = 0U; s_servo.vbus_n = 0U;
        }
    }
    s_servo.applied_pwm = D_Motor_Set(duty);
    s_servo.applied_mv = Servo_DutyToMv(s_servo.applied_pwm) * MOTOR_SIGN;
    s_servo.debug_control_valid = 0U;
}

/* 上电时的母线电压：取8次平均，作为电压换算的初值 */
static uint16_t Servo_BootBusCv(void)
{
    uint32_t sum = 0U;
    uint8_t  i;

    for (i = 0U; i < 8U; i++) { sum += A_Voltage_Read(); Delay_Ms(1U); }
    return (uint16_t)((sum + 4U) >> 3);
}

/* 按扭矩状态把H桥摆到静止态：有扭矩刹车，卸力按阻力档释放 */
static void Servo_ApplyTorqueOutput(void)
{
    if (s_servo.torque == SERVO_TORQUE_ON) Servo_Drive(0);
    else {
        D_Motor_Release((uint8_t)(s_servo.torque == SERVO_RELEASE_HIGH));
        s_servo.applied_mv = 0;
        s_servo.applied_pwm = 0;
        s_servo.debug_control_valid = 0U;
    }
}

/* 连续普通目标的接收时钟独立于规划合并时钟；显式T不改变时间语义。 */
#define SERVO_STREAM_MIN_MS       10U
#define SERVO_STREAM_MAX_MS       300U
#define SERVO_STREAM_CONFIRM      2U

_Static_assert(TUNE_LOAD_RATIO_X10 <= POS_CTRL_LOAD_RATIO_MAX_X10, "Invalid fixed load inertia ratio");

static struct {
    uint16_t age, interval;
    uint8_t consistent;
    int8_t direction;
} s_stream;

static void Servo_StreamReset(void)
{
    s_stream.age = 0xFFFFU;
    s_stream.interval = 0U;
    s_stream.consistent = 0U;
    s_stream.direction = 0;
}

static void Servo_StreamObserve(uint16_t requested_ms, uint8_t was_paused, int32_t target)
{
    uint16_t interval = s_stream.age;
    int8_t direction = (int8_t)Fx_Sign32(target - s_servo.target_angle);

    if (requested_ms != 0U || was_paused) { Servo_StreamReset(); return; }
    if (interval >= SERVO_STREAM_MIN_MS && interval <= SERVO_STREAM_MAX_MS
        && s_stream.interval != 0U && direction == s_stream.direction
        && (uint32_t)interval * 2U >= s_stream.interval && interval <= 2U * s_stream.interval)
    {
        if (s_stream.consistent < SERVO_STREAM_CONFIRM) s_stream.consistent++;
    }
    else s_stream.consistent = 0U;
    s_stream.interval = interval;
    s_stream.direction = direction;
    s_stream.age = 0U;
}

/* 观测器从当前实测位置重新起步，输出也同步过去(起步重锚在下一个控制拍之前就可能读它) */
static void Servo_ObsInit(void)
{
    C_SpeedObs_Init(&s_servo.observer, s_servo.angle_circ);
    memset(&s_servo.obs, 0, sizeof(s_servo.obs));
    s_servo.obs.pos = s_servo.angle_circ;
}

/* 以当前实测位置为起点重建参考；reinit=1 连观测器、控制器状态一起重置(坐标跳变或卸力恢复后)。
 * 目标基准一起重锚，否则紧接着"回到旧目标"的指令会被当成重复目标忽略 */
static void Servo_Resync(uint8_t reinit)
{
    Servo_StreamReset();
    if (reinit) {
        Servo_ObsInit();
        C_PosCtrl_Reset();
    }
    C_Traj_Hold(s_servo.angle);
    C_Traj_Step(&s_servo.ref, 0U);
    s_servo.arrived      = 1U;
    s_servo.move_pending = 0U;
    s_servo.move_age     = 0xFFFFU;
    memset(&s_servo.dbg, 0, sizeof(s_servo.dbg));
    s_servo.debug_control_valid = 0U;
    s_servo.target_angle = s_servo.angle;
}

/* 按 target_angle 下发运动：所有目标都按单条规划，带时间的按剩余时间拉伸(规划器另套最短运动时间)。
 * 参考还在动时从当前参考状态接着规划，连续发来的目标不会先停下 */
static void Servo_Move(void)
{
    uint16_t time_ms = s_servo.timed_position ? (uint16_t)((s_servo.remaining > 0xFFFFU) ? 0xFFFFU : s_servo.remaining) : 0U;

    C_PosCtrl_BeginMotion();
    s_servo.arrived      = 0U;
    s_servo.move_pending = 0U;
    s_servo.move_age     = 0U;
    /* 参考已停：从实测位置起步。轴可停在捕获窗内任意处，从旧目标起步会让第一拍误差反向、起转补偿朝反方向顶 */
    if (C_Traj_Is_Done()) C_Traj_Hold(Fx_ClampRange(s_servo.obs.pos, s_servo.traj_lo, s_servo.traj_hi));
    if (!s_servo.timed_position && s_stream.consistent >= SERVO_STREAM_CONFIRM)
    {
        uint16_t horizon_ms = (uint16_t)(2U * s_stream.interval);
        if (time_ms < horizon_ms) time_ms = horizon_ms;
    }
    C_Traj_Plan(s_servo.target_angle, time_ms);
}

/* 丢掉当前指令的全部记账(停止/改配置共用)；脱困中时置暂停，等新指令 */
static void Servo_ClearCommand(void)
{
    Servo_StreamReset();
    s_servo.arrived      = 1U;
    s_servo.move_pending = 0U;
    s_servo.move_age     = 0xFFFFU;
    s_servo.paused       = (uint8_t)(s_servo.enc_state == SERVO_ENC_ESCAPING);
    s_servo.resume_valid = s_servo.boot_pending = s_servo.timed_position = 0U;
    s_servo.remaining    = 0U;
    s_servo.motor_mv     = 0;
    s_servo.boot_pwm     = s_servo.boot_value = 0U;
}

/* ============================== 脱困与上电 ============================== */

/* 上电动作：外部PWM指令优先，其次按配置的上电模式 */
static void Servo_ApplyBootAction(void)
{
    if (s_servo.boot_pwm != 0U) (void)Servo_SubmitQ3(s_servo.boot_pwm, s_servo.boot_value);
    else if (g_config.boot_mode == SERVO_BOOT_GOTO_START)   /* 上电目标夹进脉冲边界照常执行 */
        (void)A_Servo_Submit((uint16_t)Fx_ClampRange(g_config.startup_pwm, g_config.pulse_lo, g_config.pulse_hi),
                             Servo_IsPosition() ? SERVO_BOOT_MOVE_MS : 0U);
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

    Servo_Drive(0);
    s_servo.enc_state = SERVO_ENC_OK;
    s_servo.enc_fail  = 0U;
    Servo_Resync(1U);
    if (s_servo.boot_pending) { s_servo.boot_pending = 0U; Servo_ApplyBootAction(); }
    else if (s_servo.resume_valid && Servo_IsPosition()) { s_servo.target_angle = target; s_servo.move_age = 0xFFFFU; Servo_Move(); }
}

/* 脱困一拍：低速开环单向转，读数回来并转进行程够深够久才交回闭环；一个方向超时掉头，再超时卸力报故障 */
static void Servo_EscapeTick(void)
{
    int16_t pwm;

    if (++s_servo.escape_ticks >= SERVO_ESCAPE_TICKS)
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
    pwm = (int16_t)((s_servo.escape_dir ? SERVO_ESCAPE_DUTY : -SERVO_ESCAPE_DUTY) * MOTOR_SIGN);
    Servo_Drive((int16_t)Fx_Clamp32(pwm, s_servo.out_limit));
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

/* PWM输入两级滤波(Q3)：三帧中值去孤立毛刺，再回差死区——静止吸收抖动，移动时每帧连续跟随(输出=输入∓回差) */
static uint16_t Servo_FilterPulse(uint16_t us)
{
    uint16_t c = Q3(Fx_ClampRange(us, SERVO_PWM_MIN, SERVO_PWM_MAX)), a, b, m;

    if (!s_servo.pulse_ready)
    {
        s_servo.pulse_prev[0] = s_servo.pulse_prev[1] = s_servo.pulse_filt = c;
        s_servo.pulse_ready = 1U;
    }
    a = s_servo.pulse_prev[0]; b = s_servo.pulse_prev[1];
    s_servo.pulse_prev[0] = b; s_servo.pulse_prev[1] = c;
    if (a > b) { m = a; a = b; b = m; }
    if (b > c) b = c;
    m = (a > b) ? a : b;
    if (m > s_servo.pulse_filt + PWM_DB_Q3) s_servo.pulse_filt = (uint16_t)(m - PWM_DB_Q3);
    else if (m + PWM_DB_Q3 < s_servo.pulse_filt) s_servo.pulse_filt = (uint16_t)(m + PWM_DB_Q3);
    return s_servo.pulse_filt;
}

/* ============================== 生命周期 ============================== */

/* 设定值为0或超过额定能力时取额定(额定已含余量) */
static int32_t Servo_LimitOf(uint32_t user, uint32_t rated)
{
    return (int32_t)((user > 0U && user < rated) ? user : rated);
}

/* 生效限值 = min(设定值, 额定)；加加速度 = max(A,D)/平滑时间，平滑时间为0时取 τ；另设最短运动时间。下一次规划生效，不打断当前轨迹 */
void A_Servo_ApplyTrajConfig(void)
{
    const MotorCal_t *cal = C_MotorModel_Cal();
    int32_t acc = Servo_LimitOf(g_runtime_config.traj_amax_dps2 * 100U, cal->rated_acc_dps2 * 100U);
    int32_t dec = Servo_LimitOf(g_runtime_config.traj_dmax_dps2 * 100U, cal->rated_dec_dps2 * 100U);
    int64_t peak = (acc > dec) ? acc : dec;
    int64_t jerk = g_runtime_config.traj_smooth_ms
                 ? peak * 1000 / g_runtime_config.traj_smooth_ms
                 : peak * 1000000 / C_MotorModel_TauUs();

    C_Traj_SetLimits(Servo_LimitOf((uint32_t)g_runtime_config.traj_vmax_dps * 100U, (uint32_t)cal->rated_vel_dps * 100U),
                     acc, dec, (int32_t)((jerk > 2000000000LL) ? 2000000000LL : jerk));
    C_Traj_SetMoveMinMs(g_runtime_config.traj_move_min_ms);
}

/* 控制带宽是固件整定常量(A_Config.h 第3节)，不对外开放 */
static void Servo_ApplyGains(void)
{
    PosCtrlGains_t g = {
        .pos_bw = TUNE_POS_BW_RADS, .vel_bw = TUNE_VEL_BW_RADS,
        .int_bw = TUNE_INT_BW_RADS, .load_ratio_x10 = TUNE_LOAD_RATIO_X10
    };

    (void)C_PosCtrl_SetGains(&g);
}

/* 保持参数：精度、齿隙、积分时间立即生效(改积分时间不阶跃输出)；保持位置环增益每10ms最多变 1/s */
static void Servo_HoldParams(void)
{
    uint16_t target = g_runtime_config.hold_kp;

    if (target != s_servo.hold_kp && ++s_servo.stiff_slew_ms >= SERVO_HOLD_KP_SLEW_MS)
    {
        s_servo.stiff_slew_ms = 0U;
        s_servo.hold_kp = (uint16_t)((target > s_servo.hold_kp) ? s_servo.hold_kp + 1U : s_servo.hold_kp - 1U);
    }
    C_PosCtrl_SetHold(g_runtime_config.position_resolution_cdeg, s_servo.hold_kp, g_runtime_config.hold_ki);
}

/* 让新的模式/校准立即生效，所有环路对齐到当前实测位置 */
void A_Servo_ApplyConfig(void)
{
    A_Servo_ApplyTrajConfig();
    Servo_ApplyGains();
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    (void)Servo_ReadPosition();
    Servo_ClearCommand();
    if (Servo_IsPosition()) { Servo_SetTrajRange(); Servo_Resync(1U); }
    else { s_servo.enc_state = SERVO_ENC_OK; s_servo.paused = 0U; C_PosCtrl_Reset(); }
    A_Servo_SyncTorque();
    Servo_ApplyTorqueOutput();
}

void A_Servo_Init(void)
{
    uint8_t enc_ok;

    Servo_StreamReset();
    memset(&s_servo, 0, sizeof(s_servo));
    s_servo.arrived   = 1U;
    s_servo.out_limit = CFG_PWM_FULL;
    s_servo.boot_pwm  = Servo_DetectInput();
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    s_servo.boot_value = Servo_IsPosition() ? SERVO_BOOT_MOVE_MS : 0U; /* 首帧PWM上电目标同样按1.5s请求 */
    enc_ok = Servo_ReadPosition();
    s_servo.target_angle = s_servo.angle;   /* 目标基准=当前位置，否则上电第一条 P0500 可能被当成重复目标 */
    s_servo.move_age     = 0xFFFFU;
    Servo_VbusUpdate(Servo_BootBusCv());
    {
        MotorCal_t observer_cal = *C_MotorModel_Cal();
        observer_cal.ka_nv = (int32_t)((int64_t)observer_cal.ka_nv * (10U + TUNE_LOAD_RATIO_X10) / 10);
        C_SpeedObs_Configure(&observer_cal, TUNE_OBS_BW_HZ, CTRL_VEL_NOISE_CDPS);
    }
    Servo_ObsInit();
    C_PosCtrl_Init(C_MotorModel_Cal(), CTRL_VEL_NOISE_CDPS, TUNE_OBS_BW_HZ);
    Servo_ApplyGains();
    s_servo.hold_kp = g_runtime_config.hold_kp;
    Servo_HoldParams();
    C_Traj_Init(s_servo.angle);
    A_Servo_ApplyTrajConfig();
    if (Servo_IsPosition()) Servo_SetTrajRange();
    C_Traj_Hold(s_servo.angle);
    s_servo.torque = SERVO_TORQUE_ON;
    Servo_Drive(0);
    C_Traj_Step(&s_servo.ref, 0U);
    if (enc_ok) Servo_ApplyBootAction();
    else { s_servo.boot_pending = 1U; Servo_EscapeStart(); }   /* 上电就在死区：先脱困再执行上电动作 */
}

/* ============================== 运动指令 ============================== */

/* 运动指令内核(脉宽Q3)，总线与PWM输入共用。越界或故障整条不响应，返回 ServoSubmit_t */
static uint8_t Servo_SubmitQ3(uint16_t q3, uint16_t value)
{
    uint8_t  was_paused = s_servo.paused;
    uint16_t lo = Q3(g_config.pulse_lo), hi = Q3(g_config.pulse_hi), target;

    if (lo < Q3_MIN) lo = Q3_MIN;
    if (hi > Q3_MAX) hi = Q3_MAX;
    if (s_servo.enc_state == SERVO_ENC_FAULT || !A_Protect_CanDrive()
        || (Servo_IsPosition() && !s_servo.range_valid)) return SERVO_SUBMIT_STATE;
    if (lo > hi || q3 < lo || q3 > hi) return SERVO_SUBMIT_VALUE;
    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();
    if (s_servo.enc_state == SERVO_ENC_ESCAPING)
    {   /* 脱困结束后再执行，只留最新一条 */
        s_servo.boot_pwm = q3;
        s_servo.boot_value = value;
        s_servo.boot_pending = s_servo.resume_valid = 1U;
        s_servo.paused = 0U;
        return SERVO_SUBMIT_OK;
    }
    s_servo.paused = 0U;
    s_servo.resume_valid = 1U;

    if (!Servo_IsPosition())
    {   /* 电机模式：中位为零速，两侧线性映射到正负满PWM；value 定圈=圈数、定时=秒，0=无限 */
        s_servo.motor_mv = (q3 >= Q3_MID) ? (int32_t)((uint32_t)(q3 - Q3_MID) * MOTOR_CAL_MV / (Q3_MAX - Q3_MID))
                                          : -(int32_t)((uint32_t)(Q3_MID - q3) * MOTOR_CAL_MV / (Q3_MID - Q3_MIN));
        if (s_servo.reverse != (MOTOR_SIGN < 0)) s_servo.motor_mv = -s_servo.motor_mv;
        s_servo.remaining = value ? (uint32_t)value * (Servo_IsTurns() ? CDEG_RANGE : CFG_TICK_HZ) : UINT32_MAX;
        s_servo.enc_fail  = 0U;
        if (value && Servo_IsTurns())
        {
            int32_t raw = A_Encoder_Read();
            s_servo.turn_sample_valid = (uint8_t)(raw != ENCODER_ANGLE_ERROR);
            if (s_servo.turn_sample_valid) s_servo.raw_angle = raw;
        }
        return SERVO_SUBMIT_OK;
    }

    /* 位置模式：重复的普通目标忽略(PWM输入的抖动已由回差滤波吸收)；带时间参数的指令一律放行 */
    target = (uint16_t)Fx_ClampRange(Servo_PulseToAngle(q3), s_servo.traj_lo, s_servo.traj_hi);
    if (!was_paused && value == 0U && target == s_servo.target_angle) return SERVO_SUBMIT_OK;
    Servo_StreamObserve(value, was_paused, target);
    s_servo.target_angle   = target;
    s_servo.timed_position = (uint8_t)(value != 0U);
    s_servo.remaining      = value;
    if (was_paused) s_servo.move_age = 0xFFFFU;   /* 暂停后立即下发 */
    else if (s_servo.move_age < SERVO_MERGE_MS)
    {   /* 太密：先记下最新目标，到最小间隔再下发 */
        s_servo.move_pending = 1U;
        s_servo.arrived      = 0U;
        return SERVO_SUBMIT_OK;
    }
    Servo_Move();
    return SERVO_SUBMIT_OK;
}

/* 协议入口：pwm=目标脉宽(电机模式为速度与方向)，value=时间(ms；定圈为圈数；0=最快/无限) */
uint8_t A_Servo_Submit(uint16_t pwm, uint16_t value)
{
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX) return SERVO_SUBMIT_VALUE;   /* 先拦再升Q3，防溢出 */
    if (!A_Protect_CanDrive()) return SERVO_SUBMIT_STATE;
    return Servo_SubmitQ3(Q3(pwm), value);
}

void A_Servo_Pause(void)
{
    uint16_t target = s_servo.target_angle;

    if (s_servo.torque != SERVO_TORQUE_ON || s_servo.paused || (!s_servo.resume_valid && !s_servo.boot_pending)) return;
    Servo_StreamReset();
    s_servo.paused = s_servo.resume_valid = 1U;
    if (Servo_IsPosition() && s_servo.enc_state == SERVO_ENC_OK) {
        C_Traj_Stop();                      /* 沿当前方向平滑刹停，保留原目标供继续 */
        s_servo.target_angle = target;
        s_servo.arrived      = 1U;
        s_servo.move_pending = 0U;
    }
    else Servo_Drive(0);
}

/* 继续：从暂停点按剩余时间重新下发到原目标 */
void A_Servo_Resume(void)
{
    if (!s_servo.paused || !s_servo.resume_valid) return;
    s_servo.paused = 0U;
    if (s_servo.enc_state != SERVO_ENC_ESCAPING && Servo_IsPosition()) { s_servo.move_age = 0xFFFFU; Servo_Move(); }
}

/* 停止：丢弃目标，就地保持；卸力/电机模式立即按扭矩状态收尾。已在保持中就停在原保持点、不重锚：
 * 重锚会让窗外托着负载的 P 一拍消失，负载再掉一截 */
void A_Servo_Stop(void)
{
    uint8_t measured;
    Servo_ClearCommand();
    if (Servo_IsPosition() && s_servo.enc_state == SERVO_ENC_OK) {
        measured = Servo_ReadPosition();
        if (!(C_Traj_Is_Done() && C_PosCtrl_Reached())) Servo_Resync(0U);
        if (measured && s_servo.torque == SERVO_TORQUE_ON && A_Protect_CanDrive()) return;
    }
    Servo_ApplyTorqueOutput();
}

/* torque_mode只表示用户请求；故障只改变实际状态，恢复不复活旧目标。 */
void A_Servo_SyncTorque(void)
{
    uint8_t torque = !A_Protect_CanDrive() ? SERVO_RELEASE_LOW
                   : g_runtime_config.torque_mode == 1U ? SERVO_TORQUE_ON
                   : g_runtime_config.torque_mode == 2U ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
    if (torque == s_servo.torque) return;
    s_servo.torque = torque;
    s_servo.damping_acc = 0U;
    if (torque == SERVO_TORQUE_ON) {
        (void)Servo_ReadPosition();
        if (Servo_IsPosition()) Servo_Resync(1U);
        Servo_Drive(0);
    }
    else A_Servo_Stop();
}

/* 卸力：0=低阻力自由转，1=高阻力短路制动。 */
void A_Servo_Release(uint8_t high_resistance)
{
    g_runtime_config.torque_mode = high_resistance ? 2U : 0U;
    A_Servo_SyncTorque();
    A_Servo_Stop(); /* 同档重复卸力也取消脱困中排队的命令。 */
}

void A_Servo_RestoreTorque(void)
{
    if (!A_Protect_CanDrive()) return;
    g_runtime_config.torque_mode = 1U;
    A_Servo_SyncTorque();
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
    if (Servo_IsPosition() ? !s_servo.arrived : s_servo.remaining != 0U) return SERVO_WORK_MOVING;
    return SERVO_WORK_IDLE;
}

/* 只读缓存，不读编码器、不推进状态 */
void A_Servo_GetTelemetry(ServoTelemetry_t *out)
{
    out->target_pwm   = Servo_AngleToPwm(s_servo.target_angle);
    out->position_pwm = Servo_AngleToPwm(s_servo.angle);
    out->speed_cdps   = (Servo_IsPosition() && s_servo.range_valid
                         && s_servo.torque == SERVO_TORQUE_ON && s_servo.enc_state == SERVO_ENC_OK
                         && s_servo.enc_fail == 0U && s_servo.obs.vel != INT32_MIN)
                      ? (s_servo.reverse ? -s_servo.obs.vel : s_servo.obs.vel) : INT32_MIN;
}

/* 控制与打印都是协作式任务；ISR不改s_servo，无需关中断或重读传感器。 */
void A_Servo_GetDebugTelemetry(ServoDebugTelemetry_t *out)
{
    uint8_t active = s_servo.debug_control_valid;
    int32_t cur_ma = 0;
    uint8_t cur_ok;
    if (out == 0) return;
    cur_ok = A_Current_ReadSigned(&cur_ma);
    out->target_cdeg = s_servo.target_angle;
    out->ref_cdeg = s_servo.ref.pos;
    out->actual_cdeg = s_servo.angle_circ; /* 不截断，保留端点超调 */
    out->ref_cdps = active ? s_servo.ref.vel : 0;
    out->actual_cdps = active ? s_servo.obs.vel : 0;
    out->pwm = s_servo.applied_pwm;
    out->ff_mv = active ? s_servo.dbg.u_ff : 0;
    out->fb_mv = active ? s_servo.dbg.u_fb : 0;
    out->i_mv = active ? s_servo.dbg.u_i : 0;
    out->stick_mv = active ? s_servo.dbg.u_stick : 0;
    out->out_mv = active ? s_servo.dbg.u_out : 0;
    out->dist_mv = active ? s_servo.obs.dist : 0;
    out->saturated = active ? s_servo.dbg.sat : 0U;
    out->state = active ? s_servo.dbg.state : 3U;   /* 控制器状态；3 = 本拍没闭环 */
    out->cur_ma = (int16_t)Fx_Clamp32(cur_ma, 32767);
    if (!D_ADC_Encoder_Read(&out->pot_q4)) out->pot_q4 = 0U;
    out->flags = (uint8_t)((active ? 1U : 0U) | (s_servo.debug_position_valid ? 2U : 0U)
                         | ((active && s_servo.obs.stuck) ? 4U : 0U) | (cur_ok ? 8U : 0U));
    out->input_source = s_servo.input_source;
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
    out->saturation_ticks = s_servo.saturation_ticks;
    out->valid      = (uint8_t)(driving && s_servo.motion_valid);
    s_servo.motion_ref   = s_servo.ref.pos;
    s_servo.motion_act   = s_servo.angle_circ;
    s_servo.motion_valid = driving;
    s_servo.saturation_ticks    = 0U;
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

    if (s_servo.remaining == 0U || s_servo.motor_mv == 0) { Servo_Drive(0); return; }
    /* 定圈：按编码器累计沿指令方向的位移倒计时；穿过死区读不到时保留上次采样，下次读数会包含这段位移 */
    if (Servo_IsTurns() && s_servo.remaining != UINT32_MAX && (raw = A_Encoder_Read()) != ENCODER_ANGLE_ERROR)
    {
        if (s_servo.turn_sample_valid)
        {
            delta = raw - s_servo.raw_angle;
            if (delta > CDEG_RANGE / 2) delta -= CDEG_RANGE;
            else if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
            if (s_servo.motor_mv * MOTOR_SIGN < 0) delta = -delta;
            if (delta > 0) s_servo.remaining = ((uint32_t)delta < s_servo.remaining) ? s_servo.remaining - delta : 0U;
        }
        s_servo.raw_angle = raw;
        s_servo.turn_sample_valid = 1U;
        s_servo.enc_fail = 0U;
    }
    if (s_servo.remaining == 0U) { Servo_Drive(0); return; }
    Servo_Drive((int16_t)Fx_Clamp32(Servo_MvToDuty(s_servo.motor_mv), s_servo.out_limit));
    if (!Servo_IsTurns() && s_servo.remaining != UINT32_MAX) s_servo.remaining--;
}

/* 位置控制固定顺序：反馈 -> 观测 -> 轨迹 -> 控制 -> 驱动 */
static void Servo_PositionControl(void)
{
    uint8_t measured = Servo_ReadPosition();
    uint8_t hold;
    PosCtrlFeedback_t feedback;
    int32_t u_mv;

    if (!measured && ++s_servo.enc_fail >= SERVO_ENC_FAIL_MAX)
    {   /* 编码器连续失效：先停，再交给脱困 */
        Servo_Drive(0);
        C_PosCtrl_Reset();
        Servo_EscapeStart();
        return;
    }
    C_SpeedObs_Update(&s_servo.observer, s_servo.angle_circ, measured, s_servo.applied_mv, &s_servo.obs);

    /* 被合并的目标到最小间隔就下发；上一拍输出饱和且跟踪落后时冻结轨迹时钟(参考不再领先)，冻结拍数交给堵转检测 */
    if (s_servo.move_age < 0xFFFFU) s_servo.move_age++;
    if (s_servo.move_pending && s_servo.move_age >= SERVO_MERGE_MS) Servo_Move();
    hold = (uint8_t)(s_servo.dbg.sat && (int64_t)s_servo.dbg.e_pos * s_servo.ref.vel > 0);
    if (hold && s_servo.saturation_ticks < 0xFFFFU) s_servo.saturation_ticks++;
    C_Traj_Step(&s_servo.ref, hold);

    Servo_HoldParams();
    feedback.pos   = s_servo.obs.pos;
    feedback.vel   = s_servo.obs.vel;
    feedback.valid = measured;
    u_mv = C_PosCtrl_Update(&s_servo.ref, &feedback, s_servo.ctrl_limit_mv, &s_servo.dbg);
    Servo_Drive(Servo_MvToDuty(u_mv * MOTOR_SIGN));
    s_servo.debug_control_valid = 1U;
    /* 首次到位：参考已停且控制器按到位精度判定到位；锁存到下一条指令，外力推动不改变上报状态 */
    if (!s_servo.arrived && !s_servo.move_pending && s_servo.ref.done && C_PosCtrl_ArrivalQualified()) s_servo.arrived = 1U;
    if (!s_servo.paused && s_servo.timed_position && s_servo.remaining != 0U) s_servo.remaining--;
}

/* 1ms控制拍：异常/失能先处理，正常模式各自计算。暂停位置仍闭环保持暂停点。 */
void A_Servo_Control(void)
{
    uint16_t pulse;
    A_Servo_SyncTorque();
    if (s_stream.age < 0xFFFFU) s_stream.age++;
    if (s_stream.age > SERVO_STREAM_MAX_MS) s_stream.consistent = 0U;
    s_servo.debug_control_valid = 0U;
    if (s_servo.input_source == SERVO_INPUT_PWM && (pulse = D_PWM_Read()) != 0U)
        (void)Servo_SubmitQ3(Servo_FilterPulse(pulse), 0U);
    if (s_servo.enc_state == SERVO_ENC_FAULT) {
        if (Servo_ReadPosition()) { s_servo.enc_state = SERVO_ENC_OK; Servo_Resync(1U); }
        return;
    }
    if (s_servo.torque != SERVO_TORQUE_ON) {
        (void)Servo_ReadPosition();   /* 卸力时照常读位置：手转时诊断打印和查询都能跟上 */
        /* 自定义阻尼：在短路制动和自由释放间按百分比交替，不产生方向电压。 */
        if (s_servo.torque == SERVO_RELEASE_HIGH) {
            uint16_t sum = (uint16_t)s_servo.damping_acc + g_runtime_config.damping_percent;
            s_servo.damping_acc = (uint8_t)((sum >= 100U) ? sum - 100U : sum);
            D_Motor_Release((uint8_t)(sum >= 100U));
            s_servo.applied_mv = 0;
            s_servo.applied_pwm = 0;
        }
        return;
    }
    if ((Servo_IsPosition() && !s_servo.range_valid)
        || (s_servo.paused && (!Servo_IsPosition() || s_servo.enc_state == SERVO_ENC_ESCAPING))) {
        Servo_Drive(0);
        return;
    }
    if (s_servo.enc_state == SERVO_ENC_ESCAPING) { Servo_EscapeTick(); return; }
    if (Servo_IsPosition()) Servo_PositionControl();
    else                    Servo_MotorControl();
}
