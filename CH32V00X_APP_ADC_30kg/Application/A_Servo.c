/* A_Servo.c —— 舵机编排层：输入源、指令受理、扭矩/暂停/脱困状态机与控制拍顺序。
 * 底层"怎么转"在 A_Servo_Ctrl.c；电位器两端死区掉入时开环脱困。 */

#include "A_Servo.h"
#include "A_Servo_Ctrl.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Protect.h"
#include "C_Fixed.h"
#include "D_adc.h"
#include "D_tim.h"
#include "D_uart.h"   /* PA1(PWM捕获)与PC0(串口)是同一根信号线，上电二选一 */
#include "debug.h"
#include <string.h>

#define SERVO_ENC_FAIL_MAX      5U    /* 连续读失败这么多次才判编码器失效 */
#define SERVO_INPUT_DETECT_MS   60U   /* 上电嗅探PWM的时间窗，覆盖两个50Hz周期 */
#define SERVO_MERGE_MS          4U    /* 更密的普通目标合并成一条(只留最新) */
#define SERVO_BOOT_MOVE_MS      2000U /* 上电位置复位传给规划器的请求时间，ms */
#define SERVO_ESCAPE_DEPTH_CDEG 200   /* 脱困：读数回到可读区这么深才开始计内推 */
#define SERVO_ESCAPE_DUTY       (CFG_PWM_FULL / 4)
#define SERVO_ESCAPE_TICKS      1500U /* 单方向最长脱困拍数 */
#define SERVO_ESCAPE_SETTLE     120U  /* 读数恢复后继续内推的拍数 */
#define PWM_DB_Q3               Q3(2U) /* PWM输入回差死区：静止吸收±2us抖动，移动时连续跟随 */


#define GEO g_servo_geom              /* 只读几何与位置，由 A_Servo_Ctrl 维护 */

enum { SERVO_ENC_OK = 0, SERVO_ENC_ESCAPING, SERVO_ENC_FAULT };        /* 编码器状态 */
enum { SERVO_TORQUE_ON = 0, SERVO_RELEASE_LOW, SERVO_RELEASE_HIGH };   /* 扭矩状态 */

typedef struct {
    uint8_t  input_source;      /* SERVO_INPUT_TX/PWM，上电定终身 */
    uint8_t  torque;            /* SERVO_TORQUE_ON/RELEASE_* */
    uint8_t  enc_state;         /* SERVO_ENC_* */
    uint8_t  enc_fail;          /* 连续读编码器失败次数 */
    uint8_t  paused;            /* 位置模式保持暂停点，保留原目标 */
    uint8_t  arrived;           /* 1=本条指令已首次到位；外力推动不改变，下一条指令清零 */
    uint8_t  move_pending;      /* 有一条被合并、等最小间隔到了再下发的普通目标 */
    uint8_t  resume_valid;      /* 有可继续的指令 */
    uint8_t  timed_position;    /* 当前是带时间参数的位置指令 */
    uint8_t  boot_pending;      /* 有指令(或上电动作)在等脱困结束 */
    uint8_t  pulse_ready;       /* PWM输入滤波器已有历史 */
    uint8_t  turn_valid;        /* turn_base 是有效的定圈累计基准 */
    uint8_t  motion_valid;      /* 上次堵转采样时正在跑轨迹 */
    uint8_t  control_valid;     /* 诊断：最近一次输出来自本拍闭环 */
    uint8_t  escape_dir, escape_retried;
    uint16_t target_angle;      /* 当前目标 */
    uint16_t pulse_prev[2], pulse_filt;   /* PWM输入：三帧中值 + 回差跟随，Q3 */
    uint16_t boot_pwm, boot_value;        /* 等脱困结束再执行的指令(Q3)，0=按上电动作 */
    uint16_t escape_ticks, escape_settle;
    uint16_t move_age;          /* 距上一条已下发普通目标的毫秒数，合并过密目标用；饱和在0xFFFF */
    int32_t  motor_mv;          /* 电机模式输出电压，含方向；满量程为标定电压 */
    int32_t  turn_base;         /* 定圈累计的上次编码器原始角 */
    uint32_t remaining;         /* 定圈=厘度，定时/带时间位置=拍，UINT32_MAX=无限 */
    int32_t  motion_ref, motion_act;
} Servo_t;

static Servo_t s_servo;


static uint8_t Servo_SubmitQ3(uint16_t q3, uint16_t value);

/* ============================== 输入源 ============================== */

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


/* ============================== 基础动作 ============================== */

/* 读编码器刷新坐标；成功时清连续失败计数 */
static uint8_t Servo_ReadPosition(void)
{
    if (!A_ServoCtrl_ReadPosition()) return 0U;
    s_servo.enc_fail = 0U;
    return 1U;
}

static void Servo_Drive(int16_t duty)
{
    A_ServoCtrl_Output(duty);
    s_servo.control_valid = 0U;
}

/* 按扭矩状态把H桥摆到静止态：有扭矩刹车，卸力按阻力档释放 */
static void Servo_ApplyTorqueOutput(void)
{
    if (s_servo.torque == SERVO_TORQUE_ON) Servo_Drive(0);
    else {
        A_ServoCtrl_Release((uint8_t)(s_servo.torque == SERVO_RELEASE_HIGH));
        s_servo.control_valid = 0U;
    }
}

/* 以实测位置重建参考；reinit=1 连观测器/控制器一起重置。目标基准须一并重锚，否则"回旧目标"会被当重复忽略 */
static void Servo_Resync(uint8_t reinit)
{
    if (reinit) A_ServoCtrl_Reset();
    A_ServoCtrl_Rebase();
    s_servo.arrived       = 1U;
    s_servo.move_pending  = 0U;
    s_servo.move_age      = 0xFFFFU;
    s_servo.control_valid = 0U;
    s_servo.target_angle  = GEO->angle;
}

/* 按 target_angle 下发：所有位置指令共用此入口，从当前状态以 max(剩余时间,最短时间) 到达，新指令覆盖旧的 */
static void Servo_Move(void)
{
    uint16_t time_ms = s_servo.timed_position ? (uint16_t)((s_servo.remaining > 0xFFFFU) ? 0xFFFFU : s_servo.remaining) : 0U;

    A_Protect_ExitStallHold(); /* 新指令恢复扭矩限流 */
    A_ServoCtrl_Begin();
    s_servo.arrived      = 0U;
    s_servo.move_pending = 0U;
    s_servo.move_age     = 0U;
    A_ServoCtrl_Command(s_servo.target_angle, time_ms);
}

/* 丢掉当前指令的全部记账(停止/改配置共用)；脱困中时置暂停，等新指令 */
static void Servo_ClearCommand(void)
{
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
                             A_ServoCtrl_IsPosition() ? SERVO_BOOT_MOVE_MS : 0U);
    else if (g_config.boot_mode == SERVO_BOOT_RELEASE) A_Servo_Release(0U);
    else if (A_ServoCtrl_IsPosition()) A_ServoCtrl_HoldRef();
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
    else if (s_servo.resume_valid && A_ServoCtrl_IsPosition()) { s_servo.target_angle = target; s_servo.move_age = 0xFFFFU; Servo_Move(); }
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
        s_servo.escape_dir = (uint8_t)(GEO->raw_angle < ENCODER_ANGLE_MID);
        if (GEO->raw_angle >= ENCODER_ANGLE_LO + SERVO_ESCAPE_DEPTH_CDEG
            && GEO->raw_angle <= ENCODER_ANGLE_HI - SERVO_ESCAPE_DEPTH_CDEG) s_servo.escape_settle++;
        else s_servo.escape_settle = 0U;
        if (s_servo.escape_settle >= SERVO_ESCAPE_SETTLE) { Servo_EscapeFinish(); return; }
    }
    pwm = (int16_t)((s_servo.escape_dir ? SERVO_ESCAPE_DUTY : -SERVO_ESCAPE_DUTY) * MOTOR_SIGN);
    Servo_Drive((int16_t)Fx_Clamp32(pwm, A_ServoCtrl_Limit()));
}

/* ============================== 生命周期 ============================== */

void A_Servo_ApplyTrajConfig(void)
{
    A_ServoCtrl_ApplyTrajConfig();
}

/* 让新的模式/校准立即生效，所有环路对齐到当前实测位置 */
void A_Servo_ApplyConfig(void)
{
    A_ServoCtrl_ApplyTrajConfig();
    A_ServoCtrl_ApplyGains();
    A_ServoCtrl_SetMode((ServoMode_t)g_config.servo_mode);
    (void)Servo_ReadPosition();
    Servo_ClearCommand();
    if (A_ServoCtrl_IsPosition()) { A_ServoCtrl_UpdateRange(); Servo_Resync(1U); }
    else { s_servo.enc_state = SERVO_ENC_OK; s_servo.paused = 0U; A_ServoCtrl_ResetLoop(); }
    A_Servo_SyncTorque();
    Servo_ApplyTorqueOutput();
}

/* 恢复出厂(keep_id非0仅保留ID)并复刻上电动作：参数归零、坐标重建后按boot_mode再走一次 */
void A_Servo_FactoryReset(uint8_t keep_id)
{
    A_Config_Default(keep_id);
    A_Servo_ApplyConfig();
    Servo_ApplyBootAction();
}

void A_Servo_Init(void)
{
    uint8_t enc_ok;

    memset(&s_servo, 0, sizeof(s_servo));
    s_servo.arrived  = 1U;
    s_servo.boot_pwm = Servo_DetectInput();
    enc_ok = A_ServoCtrl_Init((ServoMode_t)g_config.servo_mode);
    s_servo.boot_value   = A_ServoCtrl_IsPosition() ? SERVO_BOOT_MOVE_MS : 0U; /* 首帧PWM上电目标同样按2s请求 */
    s_servo.target_angle = GEO->angle;   /* 目标基准=当前位置，否则上电第一条 P0500 可能被当成重复目标 */
    s_servo.move_age     = 0xFFFFU;
    s_servo.torque       = SERVO_TORQUE_ON;
    Servo_Drive(0);
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
        || (A_ServoCtrl_IsPosition() && !GEO->range_valid)) return SERVO_SUBMIT_STATE;
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

    if (!A_ServoCtrl_IsPosition())
    {   /* 电机模式：中位为零速，两侧线性映射到正负标定电压；value 定圈=圈数、定时=秒，0=无限 */
        s_servo.motor_mv = (q3 >= Q3_MID) ? (int32_t)((uint32_t)(q3 - Q3_MID) * MOTOR_CAL_MV / (Q3_MAX - Q3_MID))
                                          : -(int32_t)((uint32_t)(Q3_MID - q3) * MOTOR_CAL_MV / (Q3_MID - Q3_MIN));
        if (GEO->reverse != (MOTOR_SIGN < 0)) s_servo.motor_mv = -s_servo.motor_mv;
        s_servo.remaining = value ? (uint32_t)value * (A_ServoCtrl_IsTurns() ? CDEG_RANGE : CFG_TICK_HZ) : UINT32_MAX;
        s_servo.enc_fail  = 0U;
        if (value && A_ServoCtrl_IsTurns())
        {
            int32_t raw = A_Encoder_Read();
            s_servo.turn_valid = (uint8_t)(raw != ENCODER_ANGLE_ERROR);
            if (s_servo.turn_valid) s_servo.turn_base = raw;
        }
        return SERVO_SUBMIT_OK;
    }

    /* 位置模式：同目标指令忽略(带时间同目标若放行，上位机重发会不断后推到达时刻，参考走不完) */
    target = (uint16_t)Fx_ClampRange(A_ServoCtrl_PulseToAngle(q3), GEO->traj_lo, GEO->traj_hi);
    if (!was_paused && target == s_servo.target_angle) return SERVO_SUBMIT_OK;
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
    s_servo.paused = s_servo.resume_valid = 1U;
    if (A_ServoCtrl_IsPosition() && s_servo.enc_state == SERVO_ENC_OK) {
        A_ServoCtrl_Brake();                /* 沿当前方向平滑刹停，保留原目标供继续 */
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
    if (s_servo.enc_state != SERVO_ENC_ESCAPING && A_ServoCtrl_IsPosition()) { s_servo.move_age = 0xFFFFU; Servo_Move(); }
}

/* 停止：丢弃目标就地保持；卸力/电机模式按扭矩收尾。保持中不重锚——重锚会让托着负载的输出一拍消失 */
void A_Servo_Stop(void)
{
    uint8_t measured;
    Servo_ClearCommand();
    if (A_ServoCtrl_IsPosition() && s_servo.enc_state == SERVO_ENC_OK) {
        measured = Servo_ReadPosition();
        if (!A_ServoCtrl_Holding()) Servo_Resync(0U);
        if (measured && s_servo.torque == SERVO_TORQUE_ON && A_Protect_CanDrive()) return;
    }
    Servo_ApplyTorqueOutput();
}

/* 仅电压类故障时短路制动：短路让回馈电流在绕组耗掉、阻住负载；放开会回灌且负载自由落下。过流/过温仍放开 */
static uint8_t Servo_FaultTorque(void)
{
    const uint16_t volt = PROT_FAULT_OVERVOLT | PROT_FAULT_UNDERVOLT | PROT_FAULT_VOLT_INVALID;
    uint16_t fault = (uint16_t)(A_Protect_Fault() & (uint16_t)~PROT_FAULT_STALL);

    return ((fault & volt) != 0U && (fault & (uint16_t)~volt) == 0U) ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
}

/* torque_mode只表示用户请求；故障只改变实际状态，恢复不复活旧目标。 */
void A_Servo_SyncTorque(void)
{
    uint8_t torque = !A_Protect_CanDrive() ? Servo_FaultTorque()
                   : g_runtime_config.torque_mode == 1U ? SERVO_TORQUE_ON
                   : g_runtime_config.torque_mode == 2U ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
    if (torque == s_servo.torque) return;
    s_servo.torque = torque;
    A_ServoCtrl_ResetDamping();
    if (torque == SERVO_TORQUE_ON) {
        (void)Servo_ReadPosition();
        if (A_ServoCtrl_IsPosition()) Servo_Resync(1U);
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
    A_ServoCtrl_ResetDamping();
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

/* SCK/SCZ：当前位置记为锚点(中点/500us端)，有效行程与零点由 A_ServoCtrl_SetMode 现推 */
static uint8_t Servo_Calibrate(uint8_t kind)
{
    if (!A_ServoCtrl_SetCalAnchor(kind)) return 0U;
    A_Servo_Stop();
    A_Config_MarkDirty();
    A_ServoCtrl_SetMode(GEO->mode);
    (void)Servo_ReadPosition();
    A_ServoCtrl_UpdateRange();
    Servo_Resync(1U);
    return 1U;
}

uint8_t A_Servo_CalibrateMid(void)  { return Servo_Calibrate((uint8_t)SERVO_CAL_MID); }
uint8_t A_Servo_CalibrateZero(void) { return Servo_Calibrate((uint8_t)SERVO_CAL_ZERO); }

/* AMI/AMX：把行程一端收到当前位置并进入自定义模式，可连续收窄；被拒的不停舵机、不写Flash */
uint8_t A_Servo_SetTravelEnd(uint8_t is_min)
{
    if (!A_ServoCtrl_SetTravelEnd(is_min)) return 0U;
    A_Servo_Stop();
    A_Config_MarkDirty();
    A_Servo_ApplyConfig();
    return 1U;
}

/* SMI/SMX：当前位置的脉宽设为可响应边界(is_min=1下界)，不改行程映射；两端不得交叉 */
uint8_t A_Servo_SetPulseLimit(uint8_t is_min)
{
    uint16_t pulse;

    if (!A_ServoCtrl_IsPosition() || GEO->span_cdeg == 0U || !Servo_ReadPosition()) return 0U;
    pulse = A_ServoCtrl_AngleToPwm(GEO->angle);
    if (is_min ? pulse >= g_config.pulse_hi : pulse <= g_config.pulse_lo) return 0U;
    if (is_min) g_config.pulse_lo = pulse;
    else        g_config.pulse_hi = pulse;
    A_Config_MarkDirty();
    return 1U;
}

uint8_t A_Servo_SaveStartup(void)
{
    if (!A_ServoCtrl_IsPosition() || !Servo_ReadPosition()) return 0U;
    g_config.startup_pwm = A_ServoCtrl_AngleToPwm(GEO->angle);
    A_Config_MarkDirty();
    return 1U;
}

/* ============================== 查询 ============================== */

uint8_t A_Servo_TorqueOn(void) { return (uint8_t)(s_servo.torque == SERVO_TORQUE_ON); }

/* 1=现在写Flash安全：擦写期间CPU停顿数ms，运动中(参考在走/电机在转)先不写 */
uint8_t A_Servo_CanSave(void)
{
    if (A_ServoCtrl_IsPosition())
        return (uint8_t)(s_servo.torque != SERVO_TORQUE_ON
                         || (A_ServoCtrl_TrajDone() && !s_servo.move_pending));
    return (uint8_t)(s_servo.remaining == 0U || s_servo.motor_mv == 0);
}

uint8_t A_Servo_GetWorkState(void)
{
    if (s_servo.enc_state == SERVO_ENC_FAULT || A_Protect_Fault() != PROT_FAULT_NONE) return SERVO_WORK_FAULT;
    if (s_servo.enc_state == SERVO_ENC_ESCAPING) return SERVO_WORK_RECOVERING;
    if (s_servo.torque == SERVO_RELEASE_LOW)     return SERVO_WORK_RELEASE_LOW;
    if (s_servo.torque == SERVO_RELEASE_HIGH)    return SERVO_WORK_RELEASE_HIGH;
    if (s_servo.paused)                          return SERVO_WORK_PAUSED;
    if (A_ServoCtrl_IsPosition() ? !s_servo.arrived : s_servo.remaining != 0U) return SERVO_WORK_MOVING;
    return SERVO_WORK_IDLE;
}

/* 只读缓存，不读编码器、不推进状态 */
void A_Servo_GetTelemetry(ServoTelemetry_t *out)
{
    const SpeedObsOut_t *obs = A_ServoCtrl_Obs();

    out->target_pwm   = A_ServoCtrl_AngleToPwm(s_servo.target_angle);
    out->position_pwm = A_ServoCtrl_AngleToPwm(GEO->angle);
    out->speed_cdps   = (A_ServoCtrl_IsPosition() && GEO->range_valid
                         && s_servo.torque == SERVO_TORQUE_ON && s_servo.enc_state == SERVO_ENC_OK
                         && s_servo.enc_fail == 0U && obs->vel != INT32_MIN)
                      ? (GEO->reverse ? -obs->vel : obs->vel) : INT32_MIN;
}

/* 控制与打印都是协作式任务；ISR不改舵机状态，无需关中断或重读传感器。 */
void A_Servo_GetDebugTelemetry(ServoDebugTelemetry_t *out)
{
    const TrajRef_t     *ref = A_ServoCtrl_Ref();
    const SpeedObsOut_t *obs = A_ServoCtrl_Obs();
    const PosCtrlDbg_t  *dbg = A_ServoCtrl_Dbg();
    uint8_t active = s_servo.control_valid;
    int32_t cur_ma = 0;
    uint8_t cur_ok;
    if (out == 0) return;
    cur_ok = A_Current_ReadSigned(&cur_ma);
    out->target_cdeg = s_servo.target_angle;
    out->ref_cdeg = ref->pos;
    out->actual_cdeg = GEO->angle_circ; /* 不截断，保留端点超调 */
    out->ref_cdps = active ? ref->vel : 0;
    out->actual_cdps = active ? obs->vel : 0;
    out->pwm = A_ServoCtrl_AppliedPwm();
    out->ff_mv = active ? dbg->u_ff : 0;
    out->fb_mv = active ? dbg->u_fb : 0;
    out->i_mv = active ? dbg->u_i : 0;
    out->stick_mv = active ? dbg->u_stick : 0;
    out->out_mv = active ? dbg->u_out : 0;
    out->dist_mv = active ? obs->dist : 0;
    out->saturated = active ? dbg->sat : 0U;
    out->state = active ? dbg->state : 3U;   /* 控制器状态；3 = 本拍没闭环 */
    out->cur_ma = (int16_t)Fx_Clamp32(cur_ma, 32767);
    if (!D_ADC_Encoder_Read(&out->pot_q4)) out->pot_q4 = 0U;
    out->flags = (uint8_t)((active ? 1U : 0U) | (GEO->read_ok ? 2U : 0U)
                         | ((active && obs->stuck) ? 4U : 0U) | (cur_ok ? 8U : 0U));
    out->input_source = s_servo.input_source;
}

/* 重读一次位置再换算成协议脉宽(RAD 查询) */
uint16_t A_Servo_GetPositionPwm(void)
{
    if (A_ServoCtrl_IsPosition()) (void)Servo_ReadPosition();
    return A_ServoCtrl_AngleToPwm(GEO->angle);
}

/* ============================== 对保护模块的接口 ============================== */

void A_Servo_SetOutputLimit(int16_t limit)
{
    A_ServoCtrl_SetLimit(limit);
}

/* 堵转采样：本区间参考与机构各走了多少，并把基准推到本拍；valid 要求两端都在跑同一条未走完的轨迹 */
void A_Servo_SampleMotion(ServoMotion_t *out)
{
    int32_t ref_pos = A_ServoCtrl_Ref()->pos;
    uint8_t driving = (uint8_t)(A_ServoCtrl_IsPosition() && s_servo.torque == SERVO_TORQUE_ON && !s_servo.paused
                                && GEO->range_valid && s_servo.enc_state == SERVO_ENC_OK && !A_ServoCtrl_TrajDone());

    out->ref_delta  = ref_pos - s_servo.motion_ref;
    out->act_delta  = GEO->angle_circ - s_servo.motion_act;
    out->saturation_ticks = A_ServoCtrl_TakeSaturation();
    out->valid      = (uint8_t)(driving && s_servo.motion_valid);
    s_servo.motion_ref   = ref_pos;
    s_servo.motion_act   = GEO->angle_circ;
    s_servo.motion_valid = driving;
}

/* 堵转处理：当前角度设为目标，直接进保持(跳过捕获/交接)，不卸力。
 * 结束状态与普通完成一次指令后一致，仅少了积分托举力，并限流到0.1A(15kg)。 */
void A_Servo_HoldHere(void)
{
    if (!(A_ServoCtrl_IsPosition() && s_servo.torque == SERVO_TORQUE_ON)) return;
    Servo_ClearCommand();
    (void)Servo_ReadPosition();
    s_servo.target_angle = GEO->angle;
    A_ServoCtrl_Rebase();
    A_ServoCtrl_StallHold();
    A_Protect_EnterStallHold(); /* 保持期电流限到0.1A */
}

/* ============================== 控制拍 ============================== */

/* 电机模式一拍：开环输出 + 定圈/定时倒计时 */
static void Servo_MotorControl(void)
{
    int32_t raw, delta;

    if (s_servo.remaining == 0U || s_servo.motor_mv == 0) { Servo_Drive(0); return; }
    /* 定圈：按编码器累计沿指令方向的位移倒计时；穿过死区读不到时保留上次采样，下次读数会包含这段位移 */
    if (A_ServoCtrl_IsTurns() && s_servo.remaining != UINT32_MAX && (raw = A_Encoder_Read()) != ENCODER_ANGLE_ERROR)
    {
        if (s_servo.turn_valid)
        {
            delta = raw - s_servo.turn_base;
            if (delta > CDEG_RANGE / 2) delta -= CDEG_RANGE;
            else if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
            if (s_servo.motor_mv * MOTOR_SIGN < 0) delta = -delta;
            if (delta > 0) s_servo.remaining = ((uint32_t)delta < s_servo.remaining) ? s_servo.remaining - delta : 0U;
        }
        s_servo.turn_base = raw;
        s_servo.turn_valid = 1U;
        s_servo.enc_fail = 0U;
    }
    if (s_servo.remaining == 0U) { Servo_Drive(0); return; }
    Servo_Drive((int16_t)Fx_Clamp32(A_ServoCtrl_MvToDuty(s_servo.motor_mv), A_ServoCtrl_Limit()));
    if (!A_ServoCtrl_IsTurns() && s_servo.remaining != UINT32_MAX) s_servo.remaining--;
}

/* 位置控制固定顺序：反馈 -> 观测 -> (合并目标下发) -> 轨迹 -> 控制 -> 驱动 */
static void Servo_PositionControl(void)
{
    uint8_t measured = Servo_ReadPosition();

    if (!measured && ++s_servo.enc_fail >= SERVO_ENC_FAIL_MAX)
    {   /* 编码器连续失效：先停，再交给脱困 */
        Servo_Drive(0);
        A_ServoCtrl_ResetLoop();
        Servo_EscapeStart();
        return;
    }
    A_ServoCtrl_Observe(measured);

    /* 被合并的目标到最小间隔就下发 */
    if (s_servo.move_age < 0xFFFFU) s_servo.move_age++;
    if (s_servo.move_pending && s_servo.move_age >= SERVO_MERGE_MS) Servo_Move();

    A_ServoCtrl_Update(measured);
    s_servo.control_valid = 1U;
    /* 首次到位：参考已停且控制器按到位精度判定到位；锁存到下一条指令，外力推动不改变上报状态 */
    if (!s_servo.arrived && !s_servo.move_pending && A_ServoCtrl_Arrived()) s_servo.arrived = 1U;
    if (!s_servo.paused && s_servo.timed_position && s_servo.remaining != 0U) s_servo.remaining--;
}

/* 1ms控制拍：异常/失能先处理，正常模式各自计算。暂停位置仍闭环保持暂停点。 */
void A_Servo_Control(void)
{
    uint16_t pulse;

    A_Servo_SyncTorque();
    s_servo.control_valid = 0U;
    if (s_servo.input_source == SERVO_INPUT_PWM && (pulse = D_PWM_Read()) != 0U)
        (void)Servo_SubmitQ3(Servo_FilterPulse(pulse), 0U);
    if (s_servo.enc_state == SERVO_ENC_FAULT) {
        if (Servo_ReadPosition()) { s_servo.enc_state = SERVO_ENC_OK; Servo_Resync(1U); }
        return;
    }
    if (s_servo.torque != SERVO_TORQUE_ON) {
        (void)Servo_ReadPosition();   /* 卸力时照常读位置：手转时诊断打印和查询都能跟上 */
        if (s_servo.torque == SERVO_RELEASE_HIGH) A_ServoCtrl_Damping(A_Protect_CanDrive() ? g_runtime_config.damping_percent : 100U);
        return;
    }
    if ((A_ServoCtrl_IsPosition() && !GEO->range_valid)
        || (s_servo.paused && (!A_ServoCtrl_IsPosition() || s_servo.enc_state == SERVO_ENC_ESCAPING))) {
        Servo_Drive(0);
        return;
    }
    if (s_servo.enc_state == SERVO_ENC_ESCAPING) { Servo_EscapeTick(); return; }
    if (A_ServoCtrl_IsPosition()) Servo_PositionControl();
    else                          Servo_MotorControl();
}
