/*
 * A_Servo_Ctrl.c —— 舵机执行层：坐标 -> 算法链 -> 输出级。
 * 改控制算法改 Common 层；改"用哪些参数、怎么串、电压怎么换算"改这里；指令与状态逻辑在 A_Servo.c。
 * 反馈是电位器的线性行程：不回绕，两端之外有测不到角度的死区。
 */

#include "A_Servo_Ctrl.h"
#include "A_Sensor.h"
#include "A_Protect.h"
#include "C_Motor_Model.h"
#include "C_Fixed.h"
#include "D_motor.h"
#include "debug.h"
#include <string.h>

#define HOLD_KP_SLEW_MS   10U   /* 运行中改保持 P：每10ms最多变 1‰/°，避免力矩阶跃 */

typedef char cfg_pwm_full_matches_motor[(CFG_PWM_FULL == MOTOR_PWM_MAX) ? 1 : -1];
_Static_assert(TUNE_LOAD_RATIO_X10 <= POS_CTRL_LOAD_RATIO_MAX_X10, "Invalid fixed load inertia ratio");

/* 电机标定(A_Config.h 第2节，由标定固件生成) */
static const MotorCal_t k_motor_cal = {
    .kv_uv = MOTOR_KV_UV, .ka_nv = MOTOR_KA_NV, .fc_mv = MOTOR_FC_MV, .fs_mv = MOTOR_FS_MV,
    .delay_us = MOTOR_DELAY_US, .cal_mv = MOTOR_CAL_MV, .noise_cdeg = MOTOR_NOISE_CDEG,
    .min_step_cdeg = MOTOR_MIN_STEP_CDEG, .sign = MOTOR_SIGN,
    .rated_vel_dps = MOTOR_RATED_VEL_DPS, .rated_acc_dps2 = MOTOR_RATED_ACC_DPS2,
    .rated_dec_dps2 = MOTOR_RATED_DEC_DPS2
};

static ServoGeom_t s_geom;                         /* 1. 坐标 */
const ServoGeom_t *const g_servo_geom = &s_geom;

static struct {                                    /* 2. 算法链 */
    TrajRef_t       ref;
    SpeedObserver_t observer;
    SpeedObsOut_t   obs;
    PosCtrlDbg_t    dbg;
    uint16_t hold_kp;           /* 当前生效的保持 P(‰/°)，缓慢跟随运行参数 */
    uint16_t saturation_ticks;  /* 输出饱和且跟踪落后、轨迹时钟被冻结的拍数，供堵转采样 */
    uint8_t  hold_kp_slew_ms;
} s_loop;

static struct {                                    /* 3. 输出级 */
    uint16_t bus_cv;        /* 母线电压(驱动期间8样本均值)，厘伏 */
    uint8_t  vbus_n;
    uint8_t  damping_acc;   /* 自定义阻尼的脉冲密度累加器 */
    uint32_t vbus_sum;
    int32_t  to_duty_q16;   /* 占空比/mV，Q16：电压与占空比的唯一换算处 */
    int32_t  to_mv_q16;     /* mV/占空比，Q16 */
    int16_t  out_limit;     /* 保护模块给的PWM上限 */
    int16_t  applied_pwm;   /* D_Motor_Set 返回的实际PWM */
    int32_t  limit_mv;      /* 上限在当前母线下对应的电压，控制器硬限幅 */
    int32_t  applied_mv;    /* 上一拍实际施加电压(角度增大为正)，供观测器 */
} s_out;

static void Out_VbusUpdate(uint16_t cv);
static uint16_t Out_BootBusCv(void);
static void Loop_HoldParams(void);
static void Loop_ObsInit(int32_t pos_circ);

/* ============================== 上电 ============================== */

uint8_t A_ServoCtrl_Init(ServoMode_t mode)
{
    uint8_t enc_ok;

    memset(&s_geom, 0, sizeof(s_geom));
    memset(&s_loop, 0, sizeof(s_loop));
    memset(&s_out, 0, sizeof(s_out));
    A_ServoCtrl_SetMode(mode);
    enc_ok = A_ServoCtrl_ReadPosition();
    s_out.out_limit = CFG_PWM_FULL;
    Out_VbusUpdate(Out_BootBusCv());
    C_MotorModel_Init(&k_motor_cal);
    {   /* 观测器按固定负载惯量比放大加速度系数 */
        MotorCal_t observer_cal = *C_MotorModel_Cal();
        observer_cal.ka_nv = (int32_t)((int64_t)observer_cal.ka_nv * (10U + TUNE_LOAD_RATIO_X10) / 10);
        C_SpeedObs_Configure(&observer_cal, TUNE_OBS_BW_HZ, TUNE_OBS_DIST_HZ, CTRL_VEL_NOISE_CDPS);
    }
    Loop_ObsInit(s_geom.angle_circ);
    C_PosCtrl_Init(C_MotorModel_Cal(), CTRL_VEL_NOISE_CDPS, TUNE_OBS_BW_HZ);
    A_ServoCtrl_ApplyGains();
    s_loop.hold_kp = g_runtime_config.hold_kp;
    Loop_HoldParams();
    C_Traj_Init(s_geom.angle);
    A_ServoCtrl_ApplyTrajConfig();
    if (A_ServoCtrl_IsPosition()) A_ServoCtrl_UpdateRange();
    A_ServoCtrl_Rebase();
    return enc_ok;
}

/* ============================== 1. 模式与坐标 ============================== */

uint8_t A_ServoCtrl_IsPosition(void)
{
    return (uint8_t)(s_geom.mode <= SERVO_MODE_180_CCW || s_geom.mode == SERVO_MODE_CUSTOM);
}

uint8_t A_ServoCtrl_IsTurns(void)
{
    return (uint8_t)(s_geom.mode == SERVO_MODE_TURNS_CW || s_geom.mode == SERVO_MODE_TURNS_CCW);
}

static uint16_t Geom_WrapAngle(int32_t a)
{
    while (a < 0) a += CDEG_RANGE;
    while (a >= CDEG_RANGE) a -= CDEG_RANGE;
    return (uint16_t)a;
}

/* 模式的标称行程(未经校准收缩)，0=电机模式 */
static uint16_t Geom_NominalSpan(ServoMode_t mode)
{
    if (mode == SERVO_MODE_CUSTOM) return g_config.custom_span_cdeg;
    return (mode <= SERVO_MODE_180_CCW) ? ((mode <= SERVO_MODE_270_CCW) ? 27000U : 18000U) : 0U;
}

/* 由校准锚点推有效行程与零点：MID 锚点是行程中点，ZERO 是500us端(反向时在高端)。
 * 可转入区只有 [TRAVEL_LO, TRAVEL_HI]，放不下标称行程就围绕锚点收缩；每次从标称行程重算，不叠加 */
static void Geom_DeriveCal(uint8_t kind, int32_t anchor, uint16_t nominal,
                           uint16_t *span_out, int32_t *offset_out)
{
    int32_t span = nominal, room, lo_room, hi_room;

    anchor  = Fx_ClampRange(anchor, ENCODER_TRAVEL_LO, ENCODER_TRAVEL_HI);
    lo_room = anchor - ENCODER_TRAVEL_LO;
    hi_room = ENCODER_TRAVEL_HI - anchor;
    if (kind == SERVO_CAL_MID) room = 2 * ((lo_room < hi_room) ? lo_room : hi_room);
    else                       room = s_geom.reverse ? lo_room : hi_room;
    if (span > room) span = room;
    *offset_out = (kind == SERVO_CAL_MID) ? anchor - span / 2 : (s_geom.reverse ? anchor - span : anchor);
    *span_out   = (uint16_t)span;
}

/* 按模式号展开行程、方向、零点；不支持的模式退回270度。奇数正向、偶数反向，CFG_DIR_INVERT 整体翻转 */
void A_ServoCtrl_SetMode(ServoMode_t mode)
{
    uint16_t nominal, span;
    int32_t  anchor = g_config.cal_anchor_cdeg, offset;
    uint8_t  kind = g_config.cal_kind;

    if (!SERVO_MODE_IS_SUPPORTED(mode)) mode = SERVO_MODE_270_CW;
    s_geom.mode    = mode;
    s_geom.reverse = (uint8_t)(((mode == SERVO_MODE_CUSTOM) ? g_config.custom_reverse : ((mode & 1U) == 0U))
                               ^ CFG_DIR_INVERT);
    nominal = Geom_NominalSpan(mode);
    offset  = (mode == SERVO_MODE_CUSTOM) ? g_config.custom_offset_cdeg : g_config.position_offset_cdeg;
    if (offset > ENCODER_POT_ANGLE_MAX) offset -= CDEG_RANGE;   /* Flash 按[0,36000)存，负零点存成36000+偏移 */
    s_geom.span_cdeg   = nominal;
    s_geom.offset_cdeg = offset;
    if (nominal == 0U) return;
    if (kind == SERVO_CAL_NONE)
    {
        if (mode == SERVO_MODE_CUSTOM) return;   /* 自定义行程由 AMI/AMX 直接标出 */
        /* 旧版 SCK 的零点加半个行程还原成中点锚点；0=从未校准，取可转入区中点 */
        kind   = SERVO_CAL_MID;
        anchor = offset ? offset + nominal / 2 : (ENCODER_TRAVEL_LO + ENCODER_TRAVEL_HI) / 2;
    }
    Geom_DeriveCal(kind, anchor, nominal, &span, &offset);
    if (span < SERVO_CUSTOM_SPAN_MIN) return;    /* 推不出可用行程就保留标称值 */
    s_geom.span_cdeg   = span;
    s_geom.offset_cdeg = offset;
}

/* 读编码器，刷新原始角度、反馈坐标、行程坐标；读失败返回0，角度保持上次值 */
uint8_t A_ServoCtrl_ReadPosition(void)
{
    int32_t raw = A_Encoder_Read();

    s_geom.read_ok = (uint8_t)(raw != ENCODER_ANGLE_ERROR);
    if (raw == ENCODER_ANGLE_ERROR) return 0U;
    s_geom.raw_angle  = raw;
    s_geom.angle_circ = raw - s_geom.offset_cdeg;   /* 闭环用，不截断：截断会在顶住端点时把误差抹成0 */
    s_geom.angle      = (uint16_t)Fx_ClampRange(s_geom.angle_circ, 0, s_geom.span_cdeg);
    return 1U;
}

/* 脉宽(Q3) -> 行程坐标，就近取整，与 A_ServoCtrl_AngleToPwm 往返一致 */
uint16_t A_ServoCtrl_PulseToAngle(uint16_t q3)
{
    uint32_t a = ((uint32_t)(Fx_ClampRange(q3, Q3_MIN, Q3_MAX) - Q3_MIN) * s_geom.span_cdeg + Q3_SPAN / 2U) / Q3_SPAN;
    return (uint16_t)(s_geom.reverse ? s_geom.span_cdeg - a : a);
}

/* 行程坐标 -> 协议脉宽；电机模式统一返回中位 */
uint16_t A_ServoCtrl_AngleToPwm(uint16_t angle)
{
    uint32_t a = s_geom.reverse ? (uint32_t)s_geom.span_cdeg - angle : angle;

    if (!A_ServoCtrl_IsPosition() || s_geom.span_cdeg == 0U) return SERVO_PWM_MID;
    return (uint16_t)(SERVO_PWM_MIN + (a * (SERVO_PWM_MAX - SERVO_PWM_MIN) + s_geom.span_cdeg / 2U) / s_geom.span_cdeg);
}

/* 轨迹可用范围 = 行程 ∩ 可转入区；端点外扩的可读余量就是兜超调的安全带 */
void A_ServoCtrl_UpdateRange(void)
{
    int32_t lo = ENCODER_TRAVEL_LO - s_geom.offset_cdeg, hi = ENCODER_TRAVEL_HI - s_geom.offset_cdeg;

    if (lo < 0) lo = 0;
    if (hi > s_geom.span_cdeg) hi = s_geom.span_cdeg;
    s_geom.range_valid = (uint8_t)(lo <= hi);
    s_geom.traj_lo = s_geom.range_valid ? (uint16_t)lo : 0U;
    s_geom.traj_hi = s_geom.range_valid ? (uint16_t)hi : 0U;
    if (s_geom.range_valid) C_Traj_SetRange(lo, hi);
}

/* SCK/SCZ：当前位置记为锚点(中点/500us端)；有效行程与零点下次 A_ServoCtrl_SetMode 时现推。
 * 自定义模式没有锚点这套真值，零点直接写进 custom_offset(与AMI/AMX同一套)并作废旧锚点。 */
uint8_t A_ServoCtrl_SetCalAnchor(uint8_t kind)
{
    int32_t  raw = A_Encoder_Read(), offset;
    uint16_t nominal = Geom_NominalSpan(s_geom.mode), span;

    if (!A_ServoCtrl_IsPosition() || raw == ENCODER_ANGLE_ERROR || raw < ENCODER_TRAVEL_LO
        || raw > ENCODER_TRAVEL_HI || nominal == 0U) return 0U;
    Geom_DeriveCal(kind, raw, nominal, &span, &offset);
    if (span < SERVO_CUSTOM_SPAN_MIN) return 0U;   /* 推不出至少1度的行程：本次校准作废 */
    if (s_geom.mode == SERVO_MODE_CUSTOM)
    {
        if (span != g_config.custom_span_cdeg) return 0U;   /* 行程被收缩，custom_span表达不了 */
        g_config.custom_offset_cdeg = Geom_WrapAngle(offset + CDEG_RANGE);
        g_config.cal_kind           = (uint8_t)SERVO_CAL_NONE;
        return 1U;
    }
    g_config.cal_kind        = kind;
    g_config.cal_anchor_cdeg = (uint16_t)raw;
    return 1U;
}

/* AMI/AMX：把行程一端收到当前位置并转为自定义模式(is_min=1为500us端，在哪头由方向决定)，可连续收窄 */
uint8_t A_ServoCtrl_SetTravelEnd(uint8_t is_min)
{
    int32_t  raw, pos;
    uint16_t span;
    uint8_t  raise_low = (uint8_t)(is_min != s_geom.reverse);   /* 1=抬低端(零点跟着挪)，0=压高端 */

    if (!A_ServoCtrl_IsPosition() || (raw = A_Encoder_Read()) == ENCODER_ANGLE_ERROR) return 0U;
    pos  = Fx_ClampRange(raw - s_geom.offset_cdeg, 0, s_geom.span_cdeg);
    span = (uint16_t)(raise_low ? s_geom.span_cdeg - pos : pos);
    if (span < SERVO_CUSTOM_SPAN_MIN) return 0U;   /* 先验证再动手：被拒的不改配置 */
    g_config.custom_offset_cdeg = Geom_WrapAngle(s_geom.offset_cdeg + (raise_low ? pos : 0) + CDEG_RANGE);
    g_config.custom_span_cdeg   = span;
    g_config.custom_reverse     = (uint8_t)(s_geom.reverse ^ CFG_DIR_INVERT);   /* 存翻转前的原始方向 */
    g_config.servo_mode         = SERVO_MODE_CUSTOM;
    g_config.cal_kind           = (uint8_t)SERVO_CAL_NONE;                      /* 旧校准锚点作废 */
    return 1U;
}

/* ============================== 2. 算法链 ============================== */

/* 设定值为0或超过额定能力时取额定(额定已含余量) */
static int32_t Loop_LimitOf(uint32_t user, uint32_t rated)
{
    return (int32_t)((user > 0U && user < rated) ? user : rated);
}

/* 生效限值 = min(设定值, 额定)；平滑时间为0时取机械时间常数 τ；另设最短运动时间。下一次规划生效，不打断当前轨迹 */
void A_ServoCtrl_ApplyTrajConfig(void)
{
    const MotorCal_t *cal = C_MotorModel_Cal();
    uint32_t smooth = g_runtime_config.traj_smooth_ms ? g_runtime_config.traj_smooth_ms
                                                      : ((uint32_t)C_MotorModel_TauUs() + 500U) / 1000U;

    C_Traj_SetLimits(Loop_LimitOf((uint32_t)g_runtime_config.traj_vmax_dps * 100U, (uint32_t)cal->rated_vel_dps * 100U),
                     Loop_LimitOf(g_runtime_config.traj_amax_dps2 * 100U, cal->rated_acc_dps2 * 100U),
                     Loop_LimitOf(g_runtime_config.traj_dmax_dps2 * 100U, cal->rated_dec_dps2 * 100U),
                     (uint16_t)((smooth > TRAJ_SMOOTH_MS_MAX) ? TRAJ_SMOOTH_MS_MAX : smooth));
    C_Traj_SetMoveMinMs(g_runtime_config.traj_move_min_ms);
}

/* 控制带宽是固件整定常量(A_Config.h 第3节)，不对外开放 */
void A_ServoCtrl_ApplyGains(void)
{
    PosCtrlGains_t g = {
        .pos_bw = TUNE_POS_BW_RADS, .vel_bw = TUNE_VEL_BW_RADS,
        .int_bw = TUNE_INT_BW_RADS, .load_ratio_x10 = TUNE_LOAD_RATIO_X10
    };

    (void)C_PosCtrl_SetGains(&g);
}

/* 保持参数：精度、积分时间立即生效(改积分时间不阶跃输出)；保持位置环增益每10ms最多变 1/s */
static void Loop_HoldParams(void)
{
    uint16_t target = g_runtime_config.hold_kp;

    if (target != s_loop.hold_kp && ++s_loop.hold_kp_slew_ms >= HOLD_KP_SLEW_MS)
    {
        s_loop.hold_kp_slew_ms = 0U;
        s_loop.hold_kp = (uint16_t)((target > s_loop.hold_kp) ? s_loop.hold_kp + 1U : s_loop.hold_kp - 1U);
    }
    C_PosCtrl_SetHold(g_runtime_config.position_resolution_cdeg, s_loop.hold_kp, g_runtime_config.hold_ki);
}

/* 观测器从当前实测位置重新起步，输出也同步过去(起步重锚在下一个控制拍之前就可能读它) */
static void Loop_ObsInit(int32_t pos_circ)
{
    C_SpeedObs_Init(&s_loop.observer, pos_circ);
    memset(&s_loop.obs, 0, sizeof(s_loop.obs));
    s_loop.obs.pos = pos_circ;
}

void A_ServoCtrl_Reset(void)
{
    Loop_ObsInit(s_geom.angle_circ);
    C_PosCtrl_Reset();
}

void A_ServoCtrl_ResetLoop(void) { C_PosCtrl_Reset(); }

void A_ServoCtrl_Rebase(void)
{
    C_Traj_Hold(s_geom.angle);
    C_Traj_Step(&s_loop.ref, 0U);
    memset(&s_loop.dbg, 0, sizeof(s_loop.dbg));
}

void A_ServoCtrl_HoldRef(void) { C_Traj_Hold(s_geom.angle); }

/* 参考已停：从实测位置起步。轴可停在捕获窗内任意处，从旧目标起步会让第一拍误差反向、起转补偿朝反方向顶 */
void A_ServoCtrl_Begin(void)
{
    C_PosCtrl_BeginMotion();
    if (C_Traj_Is_Done()) C_Traj_Hold(Fx_ClampRange(s_loop.obs.pos, s_geom.traj_lo, s_geom.traj_hi));
}

/* 堵转触发的保持：清积分托举力，保持态只靠窗外弹簧 */
void A_ServoCtrl_StallHold(void) { C_PosCtrl_StallHold(); }

/* 所有位置指令的唯一入口：从参考当前位置用 max(T, 最短运动时间) 到达；新目标从当前状态接着走 */
void A_ServoCtrl_Command(uint16_t target, uint16_t time_ms) { C_Traj_Command(target, time_ms); }

void    A_ServoCtrl_Brake(void)    { C_Traj_Stop(); }
uint8_t A_ServoCtrl_TrajDone(void) { return C_Traj_Is_Done(); }
uint8_t A_ServoCtrl_Holding(void)  { return (uint8_t)(C_Traj_Is_Done() && C_PosCtrl_Reached()); }
uint8_t A_ServoCtrl_Arrived(void)  { return (uint8_t)(s_loop.ref.done && C_PosCtrl_ArrivalQualified()); }

void A_ServoCtrl_Observe(uint8_t measured)
{
    C_SpeedObs_Update(&s_loop.observer, s_geom.angle_circ, measured, s_out.applied_mv, &s_loop.obs);
}

/* 上一拍输出饱和且跟踪落后时冻结轨迹时钟(参考不再领先)，冻结拍数交给堵转检测 */
void A_ServoCtrl_Update(uint8_t measured)
{
    uint8_t hold = (uint8_t)(s_loop.dbg.sat && (int64_t)s_loop.dbg.e_pos * s_loop.ref.vel > 0);
    PosCtrlFeedback_t feedback;
    int32_t u_mv;

    if (hold && s_loop.saturation_ticks < 0xFFFFU) s_loop.saturation_ticks++;
    C_Traj_Step(&s_loop.ref, hold);
    Loop_HoldParams();
    feedback.pos   = s_loop.obs.pos;
    feedback.vel   = s_loop.obs.vel;
    feedback.valid = measured;
    u_mv = C_PosCtrl_Update(&s_loop.ref, &feedback, s_out.limit_mv, &s_loop.dbg);
    A_ServoCtrl_Output(A_ServoCtrl_MvToDuty(u_mv * MOTOR_SIGN));
}

uint16_t A_ServoCtrl_TakeSaturation(void)
{
    uint16_t n = s_loop.saturation_ticks;

    s_loop.saturation_ticks = 0U;
    return n;
}

const TrajRef_t     *A_ServoCtrl_Ref(void) { return &s_loop.ref; }
const SpeedObsOut_t *A_ServoCtrl_Obs(void) { return &s_loop.obs; }
const PosCtrlDbg_t  *A_ServoCtrl_Dbg(void) { return &s_loop.dbg; }

/* ============================== 3. 输出级 ============================== */

/* 电压 <-> 占空比：全链路以 mV 计，只在这里按实测母线电压换算 */
int16_t A_ServoCtrl_MvToDuty(int32_t mv) { return (int16_t)Fx_Clamp32(Fx_MulShift(mv, s_out.to_duty_q16, 16), CFG_PWM_FULL); }
static int32_t Out_DutyToMv(int16_t duty) { return Fx_MulShift(duty, s_out.to_mv_q16, 16); }

int16_t A_ServoCtrl_Limit(void)      { return s_out.out_limit; }
int16_t A_ServoCtrl_AppliedPwm(void) { return s_out.applied_pwm; }

static void Out_UpdateLimitMv(void)
{
    s_out.limit_mv = Out_DutyToMv(s_out.out_limit);
}

static void Out_VbusUpdate(uint16_t cv)
{
    uint32_t mv = (uint32_t)cv * 10U;

    if (cv < PROT_VOLTAGE_ABS_MIN_DV * 10U || cv > PROT_VOLTAGE_ABS_MAX_DV * 10U) return;
    s_out.bus_cv      = cv;
    s_out.to_duty_q16 = (int32_t)(((uint32_t)CFG_PWM_FULL << 16) / mv);
    s_out.to_mv_q16   = (int32_t)((mv << 16) / CFG_PWM_FULL);
    Out_UpdateLimitMv();
}

/* 上电时的母线电压：取8次平均，作为电压换算的初值 */
static uint16_t Out_BootBusCv(void)
{
    uint32_t sum = 0U;
    uint8_t  i;

    for (i = 0U; i < 8U; i++) { sum += A_Voltage_Read(); Delay_Ms(1U); }
    return (uint16_t)((sum + 4U) >> 3);
}

/* 驱动H桥并记下实际施加电压；驱动期间母线电压8拍平均后更新换算(不跟随单拍压降) */
void A_ServoCtrl_Output(int16_t duty)
{
    uint16_t cv = A_Voltage_Read();

    if (s_out.bus_cv == 0U) Out_VbusUpdate(cv); /* 首份有效电压不依赖驱动已出力。 */
    else if (duty != 0 && cv >= PROT_VOLTAGE_ABS_MIN_DV * 10U && cv <= PROT_VOLTAGE_ABS_MAX_DV * 10U) {
        s_out.vbus_sum += cv;
        if (++s_out.vbus_n >= 8U) {
            cv = (uint16_t)((s_out.vbus_sum + 4U) >> 3);
            if (cv != s_out.bus_cv) Out_VbusUpdate(cv);
            s_out.vbus_sum = 0U; s_out.vbus_n = 0U;
        }
    }
    s_out.applied_pwm = D_Motor_Set(duty);
    s_out.applied_mv  = Out_DutyToMv(s_out.applied_pwm) * MOTOR_SIGN;
}

void A_ServoCtrl_Release(uint8_t high)
{
    D_Motor_Release(high);
    s_out.applied_mv  = 0;
    s_out.applied_pwm = 0;
}

/* 不产生方向电压，只调制短路制动的占空比 */
void A_ServoCtrl_Damping(uint8_t percent)
{
    uint16_t sum = (uint16_t)s_out.damping_acc + percent;

    s_out.damping_acc = (uint8_t)((sum >= 100U) ? sum - 100U : sum);
    A_ServoCtrl_Release((uint8_t)(sum >= 100U));
}

void A_ServoCtrl_ResetDamping(void) { s_out.damping_acc = 0U; }

void A_ServoCtrl_SetLimit(int16_t limit)
{
    s_out.out_limit = (int16_t)Fx_ClampRange(limit, 1, CFG_PWM_FULL);
    Out_UpdateLimitMv();
}
