/*
 * A_Servo.c —— 舵机编排层：唯一决定一个控制拍顺序的地方
 *     读编码器 -> 观测器 -> 轨迹规划器 -> 位置控制器 -> H桥
 * 模式分两类(判据 Servo_IsPosition)：位置模式(1~6, 11)走闭环；电机模式(7~10)开环给PWM、只用编码器数圈。
 * 编码器差异(圆周/线性坐标、有无死区、360度可用性)全部由 A_Sensor.h 的 ENCODER_MODE 派生。
 */

#include "A_Servo.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Protect.h"
#include "C_Speed_Observer.h"
#include "C_Traj_Planner.h"
#include "C_Pos_Ctrl.h"
#include "D_motor.h"
#include "D_tim.h"
#include "D_uart.h"   /* 输入源仲裁：PA1(PWM捕获)与PC0(串口)是同一根信号线 */
#include "debug.h"    /* Delay_Ms */
#include <limits.h>
#include <string.h>

#define SERVO_ENC_FAIL_MAX    5U   /** 连续读编码器失败达到该次数才停机，滤掉偶发读取错误 */
#define SERVO_INPUT_DETECT_MS 60U  /** 上电嗅探PWM的时间窗，覆盖最坏相位下至少两个50Hz周期 */

/* 多圈指令只在整圈可测的编码器上成立(电位器转过死区就数不清圈)；为0时 A_Servo_SubmitTurns 静默丢弃 */
#if ENCODER_IS_CIRCULAR
#define SERVO_MULTITURN 1
#else
#define SERVO_MULTITURN 0
#endif
#define SERVO_MT_MAX_TURNS 9U      /** 协议里 N 只有一位十进制 */
#define SERVO_MT_TURN_CDEG CDEG_RANGE /** 一圈 = 360度 */
/* C_Traj_Plan 时间参数只有16位，超过此值按时间比例分段，每段角速度不变 */
#define SERVO_MT_SEG_MAX_MS 60000UL

/* PWM输入：脉宽在入口升到 1/8us(Q3)，给一阶跟随滤波留工作精度，下游全程用 Q3 */
#define SERVO_PULSE_Q3_SHIFT 3U
#define SERVO_PULSE_Q3(us)  ((uint16_t)((uint16_t)(us) << SERVO_PULSE_Q3_SHIFT))
#define SERVO_PULSE_Q3_MIN  SERVO_PULSE_Q3(SERVO_PWM_MIN)
#define SERVO_PULSE_Q3_MID  SERVO_PULSE_Q3(SERVO_PWM_MID)
#define SERVO_PULSE_Q3_MAX  SERVO_PULSE_Q3(SERVO_PWM_MAX)
#define SERVO_PULSE_Q3_SPAN (SERVO_PULSE_Q3_MAX - SERVO_PULSE_Q3_MIN)

/* 三帧中值去孤立毛刺，再 1/8 一阶跟随压随机抖动；偏差超过 SERVO_PWM_SNAP_Q3 判为真动作直接跳过去。
 * 两个常数由仿真 jitter 场景扫出：1/8 + 7us 对 ±5us 线上抖动保持静止。 */
#define SERVO_PWM_IIR_SHIFT 3U
#define SERVO_PWM_SNAP_Q3   SERVO_PULSE_Q3(7U)

/* 仅PWM输入：目标死区额外留给滤波残差的余量(us) */
#define SERVO_PWM_INPUT_DB_US 2U

/* 以下四项只对有死区的编码器有意义 */
#define SERVO_ESCAPE_DEPTH_CDEG 200U        /** 读数回到可读范围内至少这么深才开始计内推，见 Servo_EscapeTick */
#define SERVO_ESCAPE_PWM (CFG_PWM_FULL / 4) /** 脱困开环幅值：够克服摩擦，又不会甩过头 */
#define SERVO_ESCAPE_TICKS  1500U           /** 单方向最长脱困时间，控制拍=ms */
#define SERVO_ESCAPE_SETTLE  120U           /** 读数恢复后继续内推的控制拍 */

/* 控制器的PWM满量程必须和电机驱动的实际上限一致，否则限幅逻辑会失真 */
typedef char cfg_pwm_full_matches_motor[(CFG_PWM_FULL == MOTOR_PWM_MAX) ? 1 : -1];
/* A_Parameter.h 的回绕量程必须和 A_Sensor.h 选的编码器类型一致，混用会满舵 */
typedef char cfg_feedback_coordinates_match_sensor[
    (CFG_WRAP_RANGE_CDEG == (ENCODER_IS_CIRCULAR ? CDEG_RANGE : 0)) ? 1 : -1];

/* 编码器可用性。只有带死区的传感器会出现"轴还在但反馈没了"。 */
typedef enum {
    SERVO_ENC_OK = 0,   /* 反馈可用 */
    SERVO_ENC_ESCAPING, /* 轴在死区里，正开环往外转 */
    SERVO_ENC_FAULT     /* 两个方向都没转出来，已卸力等人处理 */
} ServoEncState_t;

/* 扭矩状态。卸力分低阻力(两路低，自由转)和高阻力(两路高，短路制动阻尼)。 */
typedef enum {
    SERVO_TORQUE_ON = 0,  /* 正常闭环/开环输出 */
    SERVO_RELEASE_LOW,    /* 卸力，低阻力 */
    SERVO_RELEASE_HIGH    /* 卸力，高阻力 */
} ServoTorque_t;

typedef struct {
    ServoMode_t mode;           /** 当前工作模式，1~11 */
    uint16_t angle;             /** 行程坐标[0,span]，死区里的位置已投影到端点，只给目标和上报用 */
    int32_t  angle_circ;        /** 未截断反馈坐标，闭环只能用它，理由见 Servo_ClampToTravel */
    int32_t  raw_angle;         /** 编码器原始角度(保留负值)，用于定圈累计位移 */
    uint16_t span_cdeg;         /** 位置模式**有效**行程，厘度；0=电机模式。校准后可能小于标称量程 */
    int32_t  offset_cdeg;       /** 行程坐标0对应的编码器原始角度；已校准时由锚点现推 */
    uint16_t target_angle;      /** 当前目标位置，暂停后继续时按它重新规划 */
    uint16_t traj_lo, traj_hi;  /** 与规划器一致的有效目标范围 */
    uint16_t pulse_prev[2];     /** PWM输入的前两帧脉宽(1/8us)，三帧中值过滤孤立毛刺 */
    uint16_t pulse_filt;        /** 中值之后的一阶跟随输出(1/8us)，真正下发的就是它 */
    uint8_t  pulse_ready;       /** 1=中值/跟随滤波器已有历史 */
    uint8_t  range_valid;       /** 1=当前行程与物理可测范围有交集 */
    uint8_t  reverse;           /** 1=脉宽增大对应角度减小(偶数模式) */
    uint8_t  enc_fail;          /** 连续读编码器失败次数 */
    uint8_t  input_source;      /** SERVO_INPUT_TX / SERVO_INPUT_PWM，上电定终身 */
    uint8_t  paused;            /** 1=已暂停，输出刹车但保留目标 */
    uint8_t  resume_valid;      /** 1=有可恢复的目标(收到过运动指令且未被停止) */
    uint8_t  timed_position;    /** 1=位置模式带时间参数，需要按拍倒计时 */
    uint8_t  turn_sample_valid; /** 1=raw_angle是有效的定圈累计基准 */
    uint8_t  torque;            /** ServoTorque_t */
    uint8_t  enc_state;         /** ServoEncState_t */
    uint8_t  escape_dir;        /** 当前脱困方向，1=正向 */
    uint8_t  escape_retried;    /** 1=已经掉过一次头 */
    uint8_t  boot_pending;      /** 1=上电动作在等脱困完成 */
    uint16_t escape_ticks;      /** 本方向已经驱动的控制拍 */
    uint16_t escape_settle;     /** 读数恢复后继续内推的控制拍 */
    uint16_t traj_hold;         /** 轨迹时钟被饱和冻结的控制拍数，采样后清零 */
    uint8_t  motion_valid;      /** 1=上次运动采样时正在跑轨迹，位移可做差 */
    int32_t  motion_ref;        /** 上次运动采样的参考位置 */
    int32_t  motion_act;        /** 上次运动采样的实测位置 */
    uint16_t boot_pwm;          /** 待执行的上电脉宽(1/8us)，0=按boot_mode */
    uint16_t boot_value;        /** 脱困期间收到的最新指令的时间参数 */
    uint32_t remaining;         /** 剩余量：定圈=厘度，定时/位置=控制拍，UINT32_MAX=无限 */
    int16_t  motor_pwm;         /** 电机模式下的固定输出PWM(含方向符号) */
    int16_t  out_limit;         /** 对称PWM上限，由保护模块的功率限制回路设定 */
    int16_t  last_pwm;          /** 上一拍实际施加的PWM，观测器要用它做模型预测 */
    uint8_t  mt_active;         /** 1=正在执行多圈指令，参考位于未回绕的扩展坐标 */
    uint8_t  mt_final;          /** 1=当前这段轨迹的终点就是多圈指令的最终目标 */
    uint16_t mt_turn_ms;        /** 多圈指令每转一圈的时间(ms)，0=最快 */
    int32_t  mt_target;         /** 多圈指令的最终目标，扩展坐标，厘度 */
    int32_t  mt_fold;           /** 交给位置环之前要从参考里扣掉的整圈数，见 Servo_RefForCtrl */
    TrajRef_t     ref;          /** 本拍参考位置/速度/加速度 */
    SpeedObsOut_t obs;          /** 本拍观测器输出 */
    PosCtrlDbg_t  dbg;          /** 控制器遥测，同时给轨迹冻结判据提供饱和信息 */
} Servo_t;

static Servo_t s_servo;

/* ======================== 小工具：模式判别与角度换算 ======================== */

/* 位置模式(1~6和11)走闭环；电机模式(7~10)只开环给PWM */
static uint8_t Servo_IsPosition(void)
{
    return (uint8_t)(s_servo.mode <= SERVO_MODE_360_CCW
                  || s_servo.mode == SERVO_MODE_CUSTOM);
}

/* 定圈模式(7~8)按累计角度倒计时，定时模式(9~10)按控制拍倒计时 */
static uint8_t Servo_IsTurns(void)
{
    return (uint8_t)(s_servo.mode >= SERVO_MODE_TURNS_CW
                  && s_servo.mode <= SERVO_MODE_TURNS_CCW);
}

/* 角度折回 [0, CDEG_RANGE)，入参允许 (-CDEG_RANGE, 3*CDEG_RANGE)；用条件减代替取模(无硬件除法) */
static uint16_t Servo_WrapAngle(int32_t angle)
{
    if (angle < 0)           angle += CDEG_RANGE;
    if (angle >= CDEG_RANGE) angle -= CDEG_RANGE;
    if (angle >= CDEG_RANGE) angle -= CDEG_RANGE;
    return (uint16_t)angle;
}

/* 角度差折算最短路径：圆周坐标跨0点需折算；线性坐标不折(捷径要穿过死区) */
static int32_t Servo_WrapDiff(int32_t delta)
{
#if ENCODER_IS_CIRCULAR
    if (delta >  CDEG_RANGE / 2) delta -= CDEG_RANGE;
    if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
#endif
    return delta;
}

/* 当前模式的标称行程(未经校准收缩)，0=电机模式；校准永远以它为基准 */
static uint16_t Servo_NominalSpan(void)
{
    /* 第三格(360度)已不可选，保留以维持下标 (mode-1)/2 */
    static const uint16_t span[] = {27000U, 18000U, 0U}; /* 270/180度行程 */

    if (s_servo.mode == SERVO_MODE_CUSTOM) return g_config.custom_span_cdeg;
    if (s_servo.mode <= SERVO_MODE_360_CCW)
        return span[((uint8_t)s_servo.mode - 1U) / 2U];
    return 0U; /* 电机模式没有行程 */
}

/* 未校准时的坐标零点：直接取Flash里存的历史偏移。 */
static int32_t Servo_StoredOffset(void)
{
    int32_t offset = (s_servo.mode == SERVO_MODE_CUSTOM) ? g_config.custom_offset_cdeg
                                                       : g_config.position_offset_cdeg;
#if !ENCODER_IS_CIRCULAR
    /* Flash 沿用[0,36000)编码，负零点存成 36000+offset */
    if (offset > ENCODER_POT_ANGLE_MAX) offset -= CDEG_RANGE;
#endif
    return offset;
}

/* 由校准锚点现推有效行程与零点。kind=SERVO_CAL_MID 锚点为行程中点(1500us)，SERVO_CAL_ZERO 为500us端
 * (正向在低端，反向在高端)。整圈可测只平移零点；有死区时可转入区只有 [ENCODER_TRAVEL_LO, HI]，
 * 放不下标称行程就围绕锚点收缩(如50度校中位得0~100度)。每次从标称行程重算，不叠加。 */
static void Servo_DeriveCal(uint8_t kind, int32_t anchor, uint16_t nominal,
                            uint16_t *span_out, int32_t *offset_out)
{
    int32_t span = (int32_t)nominal;

#if !ENCODER_IS_CIRCULAR
    /* 锚点可能来自旧版参数换算，先夹进可转入区，免得推出一段转不进去的行程 */
    if (anchor < ENCODER_TRAVEL_LO) anchor = ENCODER_TRAVEL_LO;
    if (anchor > ENCODER_TRAVEL_HI) anchor = ENCODER_TRAVEL_HI;
#endif

#if ENCODER_IS_CIRCULAR
    if (kind == SERVO_CAL_MID)
        *offset_out = (int32_t)Servo_WrapAngle(anchor - span / 2 + CDEG_RANGE);
    else
        *offset_out = (int32_t)Servo_WrapAngle(
                          anchor - (s_servo.reverse ? span : 0) + CDEG_RANGE);
#else
    {
        int32_t room; /* 锚点到受限那一端还剩多少角度 */

        if (kind == SERVO_CAL_MID)
        {
            int32_t lo_room = anchor - ENCODER_TRAVEL_LO;
            int32_t hi_room = ENCODER_TRAVEL_HI - anchor;
            room = 2 * ((lo_room < hi_room) ? lo_room : hi_room);
            if (span > room) span = room;
            if (span < 0) span = 0;
            *offset_out = anchor - span / 2;
        }
        else
        {
            room = s_servo.reverse ? (anchor - ENCODER_TRAVEL_LO)
                                   : (ENCODER_TRAVEL_HI - anchor);
            if (span > room) span = room;
            if (span < 0) span = 0;
            *offset_out = s_servo.reverse ? (anchor - span) : anchor;
        }
    }
#endif
    *span_out = (uint16_t)span;
}

/* 从未校准时的默认中点锚点：整圈可测取半个标称量程(零点即编码器零点)；有死区时取可转入区中点，使180度模式居中 */
static int32_t Servo_DefaultAnchor(uint16_t nominal)
{
#if ENCODER_IS_CIRCULAR
    return (int32_t)nominal / 2;
#else
    (void)nominal;
    return (ENCODER_TRAVEL_LO + ENCODER_TRAVEL_HI) / 2;
#endif
}

/* 按模式号展开行程、方向、零点，不支持时退回270度正向。奇数正向、偶数反向；CFG_DIR_INVERT 在此整体翻转旋向 */
static void Servo_SetModeFields(ServoMode_t mode)
{
    uint16_t nominal; /* 本模式的标称行程     */
    uint8_t  kind;    /* 实际生效的校准种类   */
    int32_t  anchor;  /* 实际生效的校准锚点   */

    if (!SERVO_MODE_IS_SUPPORTED(mode))
        mode = SERVO_MODE_270_CW;

    s_servo.mode = mode;
    s_servo.reverse = (mode == SERVO_MODE_CUSTOM)
                    ? (uint8_t)(g_config.custom_reverse ^ CFG_DIR_INVERT)
                    : (uint8_t)((((uint8_t)mode & 1U) == 0U) ^ CFG_DIR_INVERT);

    nominal = Servo_NominalSpan();
    s_servo.span_cdeg   = nominal;
    s_servo.offset_cdeg = Servo_StoredOffset();
    if (nominal == 0U) return;                    /* 电机模式没有行程 */

    kind   = g_config.cal_kind;
    anchor = (int32_t)g_config.cal_anchor_cdeg;
    if (kind == SERVO_CAL_NONE)
    {
        /* 自定义模式的窗口由 AMI/AMX 标出，不套默认锚点 */
        if (mode == SERVO_MODE_CUSTOM) return;

        {
            int32_t legacy = Servo_StoredOffset();

            kind = SERVO_CAL_MID;
            /* 旧版 SCK 把零点存在 position_offset_cdeg，加半个标称量程即还原为中点锚点，升级不丢中位；0=从未校准 */
            anchor = (legacy != 0) ? (legacy + (int32_t)nominal / 2)
                                   : Servo_DefaultAnchor(nominal);
        }
    }

    {
        uint16_t derived; /* 推出来的有效行程 */
        int32_t  offset;  /* 推出来的坐标零点 */

        Servo_DeriveCal(kind, anchor, nominal, &derived, &offset);
        /* 推出的行程短到无意义时保留标称行程，好过变成几乎动不了的舵机 */
        if (derived < SERVO_CUSTOM_SPAN_MIN) return;
        s_servo.span_cdeg   = derived;
        s_servo.offset_cdeg = offset;
    }
}

/* 当前模式的坐标零点，由 Servo_SetModeFields 算好缓存 */
static int32_t Servo_Offset(void)
{
    return s_servo.offset_cdeg;
}

/* 编码器原始角度 -> 零点校正后的反馈坐标，不做任何截断、不丢符号 */
static int32_t Servo_RawToCircular(int32_t raw)
{
#if ENCODER_IS_CIRCULAR
    return Servo_WrapAngle(raw + CDEG_RANGE - Servo_Offset());
#else
    return raw - Servo_Offset();
#endif
}

/* 行程坐标投影，只给轨迹目标和上报用。闭环必须用未截断的 angle_circ：投影会把推出端点的误差抹成0，舵机在最该顶住时松手 */
static uint16_t Servo_ClampToTravel(int32_t pos)
{
#if !ENCODER_IS_CIRCULAR
    if (pos < 0) return 0U;
    if (pos > s_servo.span_cdeg) return s_servo.span_cdeg;
    return (uint16_t)pos;
#else
    if (s_servo.span_cdeg == 0U || pos <= s_servo.span_cdeg) return pos;

    return ((uint32_t)(pos - s_servo.span_cdeg) <= (uint32_t)(CDEG_RANGE - pos))
         ? s_servo.span_cdeg  /* 离行程上端更近 */
         : 0U;                /* 离行程下端更近 */
#endif
}

/* 读一次编码器并刷新原始角度、反馈坐标、行程坐标；返回0=读失败(三个角度保持上次值) */
static uint8_t Servo_ReadPosition(void)
{
    int32_t raw = A_Encoder_Read(); /* 编码器原始角度 */

    if (raw == ENCODER_ANGLE_ERROR) return 0U;

    s_servo.raw_angle  = raw;
    s_servo.angle_circ = Servo_RawToCircular(raw);  /* 反馈用，不截断、不丢符号 */
    s_servo.angle      = Servo_ClampToTravel(s_servo.angle_circ); /* 目标/上报用 */
    s_servo.enc_fail   = 0U;
    return 1U;
}

/* 脉宽(1/8us) -> 行程坐标，反向模式再翻转；就近取整以与 Servo_PositionToPwm 往返一致 */
static uint16_t Servo_PulseToAngle(uint16_t pulse_q3)
{
    uint32_t angle; /* 换算结果，厘度 */

    if (pulse_q3 < SERVO_PULSE_Q3_MIN) pulse_q3 = SERVO_PULSE_Q3_MIN;
    if (pulse_q3 > SERVO_PULSE_Q3_MAX) pulse_q3 = SERVO_PULSE_Q3_MAX;

    angle = ((uint32_t)(pulse_q3 - SERVO_PULSE_Q3_MIN) * s_servo.span_cdeg
             + SERVO_PULSE_Q3_SPAN / 2U) / SERVO_PULSE_Q3_SPAN;
    if (s_servo.reverse) angle = (uint32_t)s_servo.span_cdeg - angle;
    return (uint16_t)angle;
}

/* ======================== 公共动作：输出与环路复位 ======================== */

/* 按扭矩状态把H桥摆到静止态：有扭矩刹车，卸力按阻力档释放 */
static void Servo_ApplyTorqueOutput(void)
{
    if (s_servo.torque == SERVO_TORQUE_ON)
    {
        s_servo.last_pwm = D_Motor_Set(0);
    }
    else
    {
        D_Motor_Release((uint8_t)(s_servo.torque == SERVO_RELEASE_HIGH));
        s_servo.last_pwm = 0;
    }
}

/* 以 anchor 为新起点重建轨迹与控制器，reinit_obs=1 时重置观测器(坐标跳变或卸力恢复后必须)；调用前须已刷新位置 */
static void Servo_ResyncLoopsAt(int32_t anchor, uint8_t reinit_obs)
{
    if (reinit_obs) C_SpeedObs_Init(s_servo.angle_circ);
    C_Traj_Hold(anchor);        /* 轨迹立即停在这里，不再产生新的参考速度 */
    C_PosCtrl_Reset();          /* 清速度环积分和到位状态机               */
    C_Traj_Step(&s_servo.ref, 0U); /* Hold后的静止参考，避免沿用重规划前的饱和判据 */
    memset(&s_servo.dbg, 0, sizeof(s_servo.dbg));
    /* 目标死区的比较基准一起重锚，否则紧接着"回到旧目标"的指令会被死区吃掉 */
    s_servo.target_angle = s_servo.angle;
}

/* 常用形式：以当前实测的行程坐标为新起点 */
static void Servo_ResyncLoops(uint8_t reinit_obs)
{
    Servo_ResyncLoopsAt((int32_t)s_servo.angle, reinit_obs);
}

/* ==================== 编码器死区：禁入余量与脱困状态机 ==================== */

/* 设置轨迹可用范围：只收轨迹范围不改 span(span 决定脉宽到角度的映射) */
static void Servo_SetTrajRange(void)
{
    int32_t lo = 0;
    int32_t hi = (int32_t)s_servo.span_cdeg;

#if !ENCODER_IS_CIRCULAR
    /* 与可转入区[0, 270度]求交，端点本身可达：两端外扩的可读余量(各10度)就是兜超调的安全带，
     * 在区内再收一道余量只会让 P0500/P2500 停在差2度(约15us)的地方。 */
    {
        int32_t offset = Servo_Offset();
        int32_t physical_lo = ENCODER_TRAVEL_LO - offset;
        int32_t physical_hi = ENCODER_TRAVEL_HI - offset;
        if (lo < physical_lo) lo = physical_lo;
        if (hi > physical_hi) hi = physical_hi;
    }
#endif
    s_servo.range_valid = (uint8_t)(lo <= hi);
    if (!s_servo.range_valid)
    {
        s_servo.traj_lo = s_servo.traj_hi = 0U;
        return; /* 无可测行程时禁止驱动，等有效配置恢复 */
    }
    s_servo.traj_lo = (uint16_t)lo;
    s_servo.traj_hi = (uint16_t)hi;
    C_Traj_Set_Range(lo, hi);
}

/* 捕获脉宽唯一入口：先夹进协议量程再升Q3，避免越界值进入滤波器状态 */
static uint16_t Servo_PulseIn(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PWM_MIN) pulse_us = SERVO_PWM_MIN;
    if (pulse_us > SERVO_PWM_MAX) pulse_us = SERVO_PWM_MAX;
    return SERVO_PULSE_Q3(pulse_us);
}

/* 可响应的脉冲窗口 = 协议量程 ∩ SMI/SMX 边界。越界指令整条不响应而不是夹到边界：夹会掩盖上位机的配置错误 */
static uint8_t Servo_PulseAllowedQ3(uint16_t pulse_q3)
{
    uint16_t lo = SERVO_PULSE_Q3(g_config.pulse_lo);
    uint16_t hi = SERVO_PULSE_Q3(g_config.pulse_hi);

    if (lo < SERVO_PULSE_Q3_MIN) lo = SERVO_PULSE_Q3_MIN;
    if (hi > SERVO_PULSE_Q3_MAX) hi = SERVO_PULSE_Q3_MAX;
    return (uint8_t)(lo <= hi && pulse_q3 >= lo && pulse_q3 <= hi);
}

/* PWM输入两级滤波(1/8us)：三帧中值去毛刺 + 一阶跟随压抖动；|偏差|<1us 时自然停住，残差在目标死区内 */
static uint16_t Servo_FilterPulse(uint16_t pulse)
{
    uint16_t a, b, c, temp;
    int32_t  err;

    if (!s_servo.pulse_ready)
    {
        s_servo.pulse_prev[0] = s_servo.pulse_prev[1] = pulse;
        s_servo.pulse_filt    = pulse;
        s_servo.pulse_ready   = 1U;
    }
    a = s_servo.pulse_prev[0]; b = s_servo.pulse_prev[1]; c = pulse;
    s_servo.pulse_prev[0] = b; s_servo.pulse_prev[1] = c;
    if (a > b) { temp = a; a = b; b = temp; }
    if (b > c) { b = c; }
    c = (a > b) ? a : b;   /* 三帧中值 */

    err = (int32_t)c - (int32_t)s_servo.pulse_filt;
    if (err >= (int32_t)SERVO_PWM_SNAP_Q3 || err <= -(int32_t)SERVO_PWM_SNAP_Q3)
        s_servo.pulse_filt = c;
    else
        s_servo.pulse_filt = (uint16_t)((int32_t)s_servo.pulse_filt
                                        + err / (int32_t)(1 << SERVO_PWM_IIR_SHIFT));
    return s_servo.pulse_filt;
}

static void Servo_SubmitQ3(uint16_t pulse_q3, uint16_t value); /* 见"运动指令"一节 */

/* 执行上电动作，优先级：外部PWM指令 > 配置的上电模式；轴在死区里时须先脱困 */
static void Servo_ApplyBootAction(void)
{
    if (s_servo.boot_pwm != 0U)
        Servo_SubmitQ3(s_servo.boot_pwm, s_servo.boot_value);
    else if (g_config.boot_mode == SERVO_BOOT_GOTO_START)
    {
        /* 上电目标夹进脉冲边界照常执行，边界只用来拒收越界指令 */
        uint16_t pwm = g_config.startup_pwm;
        if (pwm < g_config.pulse_lo) pwm = g_config.pulse_lo;
        if (pwm > g_config.pulse_hi) pwm = g_config.pulse_hi;
        A_Servo_Submit(pwm, 0U);
    }
    else if (g_config.boot_mode == SERVO_BOOT_RELEASE)
        A_Servo_Release(0U);
    else if (Servo_IsPosition())
        C_Traj_Hold(s_servo.angle); /* SERVO_BOOT_HOLD：原地保持 */
}

/* 进入脱困状态，从正方向开始试 */
static void Servo_EscapeStart(void)
{
    s_servo.enc_state      = SERVO_ENC_ESCAPING;
    s_servo.escape_dir     = 1U;
    s_servo.escape_retried = 0U;
    s_servo.escape_ticks   = 0U;
    s_servo.escape_settle  = 0U;
    s_servo.enc_fail       = 0U;
}

/* 两个方向都没转出来：卸力等人处理(机构卡死或传感器/接线坏) */
static void Servo_EscapeFail(void)
{
    s_servo.enc_state    = SERVO_ENC_FAULT;
    s_servo.boot_pending = 0U;
    A_Servo_Release(0U);
}

/* 脱困成功：停下、重建环路、补上欠着的上电动作或未走完的指令 */
static void Servo_EscapeFinish(void)
{
    uint16_t saved_target = s_servo.target_angle;
    s_servo.last_pwm  = D_Motor_Set(0);
    s_servo.enc_state = SERVO_ENC_OK;
    s_servo.enc_fail  = 0U;
    Servo_ResyncLoops(1U); /* 刚被开环推过一段，观测器的模型预测已经脱节 */

    if (s_servo.boot_pending)
    {
        s_servo.boot_pending = 0U;
        Servo_ApplyBootAction();
    }
    else if (s_servo.resume_valid && Servo_IsPosition())
    {
        uint16_t ticks = (s_servo.remaining > 0xFFFFU) ? 0xFFFFU : (uint16_t)s_servo.remaining;
        s_servo.target_angle = saved_target;
        C_Traj_Plan(saved_target, s_servo.timed_position ? ticks : 0U);
    }
}

/* 脱困的一拍：低速开环单向转，直到读数回来并且已经转进行程里 */
static void Servo_EscapeTick(void)
{
    uint16_t limit;
    int16_t  pwm;

    /* 所有脱困拍都计时，包括读数时有时无或有反馈却推不动的情况。 */
    limit = s_servo.escape_retried
          ? (uint16_t)(2U * SERVO_ESCAPE_TICKS) : (uint16_t)SERVO_ESCAPE_TICKS;
    if (++s_servo.escape_ticks >= limit)
    {
        if (s_servo.escape_retried) { Servo_EscapeFail(); return; }
        s_servo.escape_retried = 1U;
        s_servo.escape_dir = (uint8_t)(!s_servo.escape_dir);
        s_servo.escape_ticks = 0U;
        s_servo.escape_settle = 0U;
    }

    if (Servo_ReadPosition())
    {
        /* 读数刚回来时抽头贴着碳膜边缘，同方向再推一小段进到行程里再交回闭环 */
        /* 有反馈后按实测所在的物理端选择内推方向，不能继续赌旧方向。 */
        s_servo.escape_dir = (uint8_t)(s_servo.raw_angle < ENCODER_ANGLE_MID);
        if (s_servo.raw_angle >= ENCODER_ANGLE_LO + (int32_t)SERVO_ESCAPE_DEPTH_CDEG
            && s_servo.raw_angle <= ENCODER_ANGLE_HI - (int32_t)SERVO_ESCAPE_DEPTH_CDEG)
            s_servo.escape_settle++;
        else s_servo.escape_settle = 0U;
        if (s_servo.escape_settle >= SERVO_ESCAPE_SETTLE)
        {
            Servo_EscapeFinish();
            return;
        }
    }
    else
    {
        s_servo.escape_settle = 0U; /* 又掉回去了，内推重新计时 */

    }

    pwm = s_servo.escape_dir ? (int16_t)SERVO_ESCAPE_PWM
                             : (int16_t)(-SERVO_ESCAPE_PWM);
    pwm = (int16_t)(pwm * CTRL_MOTOR_SIGN);
    if (pwm >  s_servo.out_limit) pwm =  s_servo.out_limit;
    if (pwm < -s_servo.out_limit) pwm = -s_servo.out_limit;
    s_servo.last_pwm = D_Motor_Set(pwm);
}

/* 上电嗅探信号线是否有PWM，返回脉宽(1/8us)，0=走串口。PA1与PC0共线须独占，结果上电定终身 */
static uint16_t Servo_DetectInput(void)
{
    uint8_t  elapsed;    /* 已等待的毫秒数   */
    uint16_t pulse = 0U; /* 捕获到的脉宽(us) */

    D_UART_Enable(0U);      /* 松开PC0，交出信号线 */
    D_PWM_Input_Enable(1U); /* PA1接管，开始听     */

    for (elapsed = 0U; elapsed < SERVO_INPUT_DETECT_MS && pulse == 0U; elapsed++)
    {
        pulse = D_PWM_Read();
        if (pulse == 0U) Delay_Ms(1U);
    }

    if (pulse != 0U)
    {
        s_servo.input_source = SERVO_INPUT_PWM; /* 捕获保持开启，串口保持释放 */
        /* 嗅探到的帧直接作为滤波器初值，避免从0起步的开机抖动 */
        pulse = Servo_PulseIn(pulse);
        s_servo.pulse_prev[0] = s_servo.pulse_prev[1] = pulse;
        s_servo.pulse_filt    = pulse;
        s_servo.pulse_ready   = 1U;
    }
    else
    {
        D_PWM_Input_Enable(0U); /* PA1开漏置高，松开信号线 */
        D_UART_Enable(1U);      /* 串口接管PC0             */
        s_servo.input_source = SERVO_INPUT_TX;
    }
    return pulse;
}

/* 读本次上电选定的输入源，上电定终身 */
uint8_t A_Servo_InputSource(void)
{
    return s_servo.input_source;
}

/* ========================= 对保护模块的接口 ========================= */

/* 设置对称PWM上限，夹到[1, CFG_PWM_FULL]；电机模式开环直给，在 Servo_MotorControl 里另夹 */
void A_Servo_SetOutputLimit(int16_t limit)
{
    if (limit < 1) limit = 1;
    if (limit > CFG_PWM_FULL) limit = CFG_PWM_FULL;

    s_servo.out_limit = limit;
    C_PosCtrl_Set_Output_Limit(limit);
}

/* 1=有扭矩输出，0=卸力中；保护模块每拍据此复检 */
uint8_t A_Servo_TorqueOn(void)
{
    return (uint8_t)(s_servo.torque == SERVO_TORQUE_ON);
}

/* 取一次运动采样并把基准推到本拍：参考与机构各走了多少(跨0折算只有本模块知道)。
 * valid 要求区间两端都在跑同一条未走完的轨迹；hold_ticks 每次采样后清零。 */
void A_Servo_SampleMotion(ServoMotion_t *out)
{
    uint8_t driving; /* 本拍是否正在跑一条未走完的位置轨迹 */

    if (out == 0) return;

    driving = (uint8_t)(Servo_IsPosition()
                     && s_servo.torque == SERVO_TORQUE_ON
                     && !s_servo.paused
                     && s_servo.range_valid
                     && s_servo.enc_state == SERVO_ENC_OK
                     && !C_Traj_Is_Done());

    out->ref_delta  = Servo_WrapDiff(s_servo.ref.pos - s_servo.motion_ref);
    out->act_delta  = Servo_WrapDiff(s_servo.angle_circ - s_servo.motion_act);
    out->hold_ticks = s_servo.traj_hold;
    out->valid      = (uint8_t)(driving && s_servo.motion_valid);

    s_servo.motion_ref   = s_servo.ref.pos;
    s_servo.motion_act   = s_servo.angle_circ;
    s_servo.motion_valid = driving;
    s_servo.traj_hold    = 0U;
}

/* 堵转处理：放弃当前动作，就地保持，不卸力 */
void A_Servo_HoldHere(void)
{
    if (!Servo_IsPosition() || s_servo.torque != SERVO_TORQUE_ON) return;
    A_Servo_Stop();
}

/* ============================ 生命周期与配置 ============================ */

/* 让新的模式/中位参数立即生效，并把所有环路对齐到当前实测位置 */
void A_Servo_ApplyConfig(void)
{
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    (void)Servo_ReadPosition(); /* 读失败就沿用上一次的角度，不影响后续流程 */

    s_servo.remaining    = 0U;
    s_servo.motor_pwm    = 0;
    s_servo.paused       = (uint8_t)(s_servo.enc_state == SERVO_ENC_ESCAPING);
    s_servo.resume_valid = 0U;
    s_servo.boot_pending = 0U;
    s_servo.boot_pwm     = 0U;
    s_servo.boot_value   = 0U;
    s_servo.timed_position = 0U;
    s_servo.mt_active      = 0U; /* 换模式/换参数，正在跑的多圈一律作废 */
#if SERVO_MULTITURN
    s_servo.mt_fold        = 0;
#endif

    if (Servo_IsPosition())
    {
        Servo_SetTrajRange(); /* 行程可能随模式变了 */
        Servo_ResyncLoops(1U);
    }
    else
    {
        s_servo.enc_state = SERVO_ENC_OK;
        s_servo.paused = 0U;
        C_PosCtrl_Reset();
    }

    Servo_ApplyTorqueOutput();
}

/* 上电初始化：选输入源、初始化三个算法模块、执行上电动作 */
void A_Servo_Init(void)
{
    uint8_t enc_ok; /* 上电这一刻编码器读数是否可信 */

    memset(&s_servo, 0, sizeof(s_servo));
    s_servo.out_limit = CFG_PWM_FULL; /* 保护模块随后会按配置的扭矩上限压下来 */

    s_servo.boot_pwm = Servo_DetectInput();
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    enc_ok = Servo_ReadPosition();
    if (!enc_ok)
        s_servo.angle = s_servo.angle_circ = s_servo.raw_angle = 0U;
    /* 目标死区基准初始化为当前位置，否则上电第一条 P0500 可能被吃掉 */
    s_servo.target_angle = s_servo.angle;

    C_SpeedObs_Init(s_servo.angle_circ); /* 反馈坐标类型由CFG_WRAP_RANGE_CDEG确定 */
    C_Traj_Init(s_servo.angle, TUNE_SMOOTH_ACC, TUNE_SMOOTH_DEC);
    if (Servo_IsPosition()) Servo_SetTrajRange();
    C_PosCtrl_Init();

    s_servo.torque   = SERVO_TORQUE_ON;
    s_servo.last_pwm = D_Motor_Set(0);
    /* 补一次采样，否则上电第一拍前读 s_servo.ref 拿到的都是0 */
    C_Traj_Step(&s_servo.ref, 0U);

    /* 上电就在死区：读到的位置是假的，先开环脱困再执行上电动作 */
    if (!enc_ok && ENCODER_HAS_DEADZONE)
    {
        s_servo.boot_pending = 1U;
        Servo_EscapeStart();
    }
    else
    {
        Servo_ApplyBootAction();
    }
}

/* ============================== 多圈指令 ==============================
 * 目标用未回绕的扩展坐标表示(3圈即参考涨到 +108000 厘度)，规划器位移直线计算，放开行程范围就会转满整圈。
 * 反馈仍是回绕的，交给位置环前由 Servo_RefForCtrl 把参考折回实测所在那一圈；结束时参考折回行程坐标。 */
#if SERVO_MULTITURN

/* 把回绕坐标折算到离 near 不超过半圈的那一圈上；循环有硬上限，不卡控制拍 */
static int32_t Servo_Unwrap(int32_t wrapped, int32_t near)
{
    uint8_t n;

    for (n = 0U; n < 16U && wrapped - near > CDEG_RANGE / 2; n++)
        wrapped -= CDEG_RANGE;
    for (n = 0U; n < 16U && near - wrapped > CDEG_RANGE / 2; n++)
        wrapped += CDEG_RANGE;
    return wrapped;
}

/* 规划(或续规划)多圈轨迹的一段，起点取规划器自身状态，续段时位置速度不跳变。
 * turn_ms 为每圈时间，剩余时间由剩余位移现算，暂停/继续后角速度不变；超长指令按 SERVO_MT_SEG_MAX_MS 分段。 */
static void Servo_PlanMultiTurn(void)
{
    /* 起点问规划器本人：s_servo.ref 是上一拍快照，刚Hold完时是陈的 */
    int32_t  here  = C_Traj_Get_Pos();
    int32_t  dist  = s_servo.mt_target - here;   /* 扩展坐标下的剩余位移 */
    int32_t  adist = (dist >= 0) ? dist : -dist;
    int32_t  waypoint = s_servo.mt_target;
    uint32_t ms;      /* 本段请求时间(ms)，0=最快 */

    s_servo.mt_final = 1U;
    if (s_servo.mt_turn_ms == 0U) { C_Traj_Plan(waypoint, 0U); return; }

    ms = (uint32_t)(((int64_t)s_servo.mt_turn_ms * adist) / SERVO_MT_TURN_CDEG);
    if (ms == 0U) ms = 1U;
    if (ms > SERVO_MT_SEG_MAX_MS)
    {
        int32_t seg = (int32_t)(((int64_t)adist * (int32_t)SERVO_MT_SEG_MAX_MS)
                                / (int32_t)ms);
        waypoint = here + ((dist >= 0) ? seg : -seg);
        ms = SERVO_MT_SEG_MAX_MS;
        s_servo.mt_final = 0U;
    }
    C_Traj_Plan(waypoint, (uint16_t)ms);
}

/* 交给位置环之前的参考折算，漏掉会在第二圈开始飞车：C_Pos_Ctrl 的折算只能消一圈。
 * 每拍位移远小于半圈，一次条件加减即可。只折交给控制器的副本，规划器仍留在扩展坐标上。 */
static int32_t Servo_RefForCtrl(void)
{
    int32_t pos = s_servo.ref.pos - s_servo.mt_fold;
    int32_t err = pos - s_servo.obs.pos; /* 位置环真正要相减的就是这两个量 */

    if (err > CDEG_RANGE / 2)
    {
        s_servo.mt_fold += CDEG_RANGE;
        pos -= CDEG_RANGE;
    }
    else if (err < -CDEG_RANGE / 2)
    {
        s_servo.mt_fold -= CDEG_RANGE;
        pos += CDEG_RANGE;
    }
    return pos;
}

/* 全部圈数走完：参考折回行程坐标(差整数圈，无扰动)并恢复行程范围 */
static void Servo_MultiTurnFinish(void)
{
    uint16_t target = s_servo.target_angle;

    s_servo.mt_active = 0U;
    s_servo.mt_fold   = 0;
    (void)Servo_ReadPosition();
    C_Traj_Hold((int32_t)target); /* 先折回行程坐标，再恢复范围才不会被夹错 */
    Servo_SetTrajRange();
    C_Traj_Step(&s_servo.ref, 0U);
    s_servo.target_angle = target;
}

/* 中止多圈：把参考和行程范围换回行程坐标，停在哪由调用者重锚决定 */
static void Servo_MultiTurnAbort(void)
{
    if (!s_servo.mt_active) return;
    s_servo.mt_active = 0U;
    s_servo.mt_fold   = 0;
    (void)Servo_ReadPosition();
    C_Traj_Hold((int32_t)s_servo.angle);
    Servo_SetTrajRange();
    C_Traj_Step(&s_servo.ref, 0U);
    /* 目标死区基准一起重锚，否则打断后再发同一目标会被吃掉 */
    s_servo.target_angle = s_servo.angle;
}
#else
#define Servo_MultiTurnAbort() ((void)0)
#endif /* SERVO_MULTITURN */

/* ============================== 运动指令 ============================== */

/* 下发运动指令内核，脉宽以 1/8us 计；协议与PWM输入共用，差别只在入口分辨率 */
static void Servo_SubmitQ3(uint16_t pulse_q3, uint16_t value)
{
    int32_t raw; /* 定圈模式建立累计基准时读到的原始角度 */
    uint8_t was_paused = s_servo.paused;

    /* 越界整条丢弃 */
    if (!Servo_PulseAllowedQ3(pulse_q3)) return;

    if (s_servo.enc_state == SERVO_ENC_FAULT || A_Protect_Fault() != PROT_FAULT_NONE
        || (Servo_IsPosition() && !s_servo.range_valid)) return;
    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();
    /* 普通指令打断正在执行的多圈，先换回行程坐标 */
    Servo_MultiTurnAbort();
    if (s_servo.enc_state == SERVO_ENC_ESCAPING)
    {
        s_servo.boot_pwm = pulse_q3;
        s_servo.boot_value = value;
        s_servo.boot_pending = 1U;
        s_servo.paused = 0U;
        s_servo.resume_valid = 1U;
        return; /* 脱困结束后再规划，保留最新指令 */
    }

    if (was_paused && Servo_IsPosition())
    {
        /* 暂停后的新目标从停稳位置起步 */
        if (!Servo_ReadPosition()) return;
        Servo_ResyncLoops(1U);
    }
    s_servo.paused       = 0U;
    s_servo.resume_valid = 1U;

    /* ---- 位置模式：交给轨迹规划器 ---- */
    if (Servo_IsPosition())
    {
        uint16_t target = Servo_PulseToAngle(pulse_q3);
        int32_t delta, target_db = CTRL_TARGET_DB_CDEG;
        if (target < s_servo.traj_lo) target = s_servo.traj_lo;
        if (target > s_servo.traj_hi) target = s_servo.traj_hi;
        delta = (int32_t)target - (int32_t)s_servo.target_angle;
        if (s_servo.input_source == SERVO_INPUT_PWM)
        {
            int32_t pwm_db = ((int32_t)s_servo.span_cdeg * SERVO_PWM_INPUT_DB_US
                             + SERVO_PWM_MAX - SERVO_PWM_MIN - 1)
                            / (SERVO_PWM_MAX - SERVO_PWM_MIN);
            if (pwm_db > target_db) target_db = pwm_db;
        }

        /* 目标死区：变化不超过最小可靠步长时不重规划，PWM输入再加滤波残差余量。PWM输入每20ms调一次，
         * 小位移轨迹走不完就被打断会导致永远进不了保持态、静止时持续出力，所以必需。
         * 基准是上一次被接受的目标，连续小步会累积生效；带时间参数的指令一律放行。 */
        if (!was_paused && value == 0U && delta <= target_db && delta >= -target_db)
            return;

        s_servo.target_angle   = target;
        C_Traj_Plan(target, value);
        s_servo.timed_position = (uint8_t)(value != 0U);
        s_servo.remaining      = s_servo.timed_position ? C_Traj_Get_Ticks() : 0U;
        return;
    }

    /* ---- 电机模式：以中位为零速，两侧线性映射到正负满PWM ---- */
    if (pulse_q3 >= SERVO_PULSE_Q3_MID)
        s_servo.motor_pwm = (int16_t)((uint32_t)(pulse_q3 - SERVO_PULSE_Q3_MID)
                * MOTOR_PWM_MAX / (SERVO_PULSE_Q3_MAX - SERVO_PULSE_Q3_MID));
    else
        s_servo.motor_pwm = (int16_t)-(int32_t)((uint32_t)(SERVO_PULSE_Q3_MID - pulse_q3)
                * MOTOR_PWM_MAX / (SERVO_PULSE_Q3_MID - SERVO_PULSE_Q3_MIN));

    if (s_servo.reverse) s_servo.motor_pwm = (int16_t)-s_servo.motor_pwm;
    s_servo.motor_pwm = (int16_t)(s_servo.motor_pwm * CTRL_MOTOR_SIGN); /* 换到H桥物理方向 */

    /* 定圈按厘度倒计时，定时按控制拍倒计时，value=0都表示无限 */
    s_servo.remaining = value ? (uint32_t)value
                              * (Servo_IsTurns() ? CDEG_RANGE : CFG_TICK_HZ)
                             : UINT32_MAX;
    s_servo.enc_fail  = 0U;

    if (value && Servo_IsTurns())
    {
        raw = A_Encoder_Read();
        s_servo.turn_sample_valid = (uint8_t)(raw != ENCODER_ANGLE_ERROR);
        if (s_servo.turn_sample_valid) s_servo.raw_angle = raw;
    }
}

/* 协议入口：pwm=目标脉宽(位置模式为角度，电机模式为速度与方向)，value=时间参数(ms；定圈模式为圈数；0=最快/无限) */
void A_Servo_Submit(uint16_t pwm, uint16_t value)
{
    /* 越界不响应；须先拦再升Q3，9999<<3 会溢出16位 */
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX) return;
    Servo_SubmitQ3(SERVO_PULSE_Q3(pwm), value);
}

/* 多圈运动指令：先转 turns 整圈再到 pwm 位置，turn_ms 为每圈时间(0=最快)。方向同普通指令，重合时取正方向 */
void A_Servo_SubmitTurns(uint16_t pwm, uint8_t turns, uint16_t turn_ms)
{
#if SERVO_MULTITURN
    uint16_t target;     /* 目标的行程坐标           */
    int32_t  here;       /* 规划器当前的参考位置     */
    int32_t  delta;      /* 到目标的行程位移，定方向 */
    int32_t  total;      /* 整条指令的总位移(带符号) */
    int32_t  lo, hi;     /* 放开后的轨迹范围         */
    uint8_t  was_paused = s_servo.paused;

    if (turns > SERVO_MT_MAX_TURNS) return;
    if (!Servo_PulseAllowedQ3(SERVO_PULSE_Q3(pwm))) return;
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX) return;
    if (!Servo_IsPosition() || !s_servo.range_valid) return;
    if (s_servo.enc_state != SERVO_ENC_OK
        || A_Protect_Fault() != PROT_FAULT_NONE) return;

    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();

    /* 打断未走完的多圈或暂停后重发：换回行程坐标并以实测位置重建起点(等同DST打断) */
    if (s_servo.mt_active || was_paused)
    {
        Servo_MultiTurnAbort();
        if (!Servo_ReadPosition()) return;
        Servo_ResyncLoops(1U);
    }
    s_servo.paused       = 0U;
    s_servo.resume_valid = 1U;

    target = Servo_PulseToAngle(SERVO_PULSE_Q3(pwm));
    if (target < s_servo.traj_lo) target = s_servo.traj_lo;
    if (target > s_servo.traj_hi) target = s_servo.traj_hi;

    /* 方向与总位移以规划器当前参考为准 */
    here  = C_Traj_Get_Pos();
    delta = (int32_t)target - here;
    total = (int32_t)turns * SERVO_MT_TURN_CDEG + ((delta >= 0) ? delta : -delta);
    if (delta < 0) total = -total;
    /* 已在目标上且0圈：按普通指令处理 */
    if (total == 0)
    {
        Servo_SubmitQ3(SERVO_PULSE_Q3(pwm), 0U);
        return;
    }

    s_servo.mt_target      = here + total;
    s_servo.mt_turn_ms     = turn_ms;
    s_servo.mt_active      = 1U;
    s_servo.mt_fold        = 0;
    s_servo.target_angle   = target;
    s_servo.timed_position = 0U;
    s_servo.remaining      = 0U;

    /* 行程范围放开到整段扩展坐标，否则目标会被夹回行程里 */
    lo = (total < 0) ? s_servo.mt_target : here;
    hi = (total < 0) ? here               : s_servo.mt_target;
    C_Traj_Set_Range_Open(lo, hi); /* 放开范围但不打断已有轨迹，见规划器同名函数 */
    Servo_PlanMultiTurn();
#else
    (void)pwm; (void)turns; (void)turn_ms;
#endif
}

/* 暂停：输出刹车，但保留目标以便继续 */
void A_Servo_Pause(void)
{
    if (s_servo.torque != SERVO_TORQUE_ON
        || s_servo.paused || (!s_servo.resume_valid && !s_servo.boot_pending)) return;

    s_servo.paused = 1U;
    s_servo.resume_valid = 1U;
    if (Servo_IsPosition())
    {
        uint16_t saved_target = s_servo.target_angle;
        (void)Servo_ReadPosition();
#if SERVO_MULTITURN
        /* 多圈进行中：参考停在扩展坐标的实测位置，行程范围保留(用行程坐标重锚会倒转回去) */
        if (s_servo.mt_active)
            Servo_ResyncLoopsAt(Servo_Unwrap(s_servo.angle_circ, s_servo.ref.pos), 0U);
        else
#endif
        Servo_ResyncLoops(0U);
        s_servo.target_angle = saved_target; /* 暂停不能丢掉继续时的目的地 */
    }
    s_servo.last_pwm = D_Motor_Set(0);
}

/* 继续：以当前位置为起点，用剩余时间重新规划到原目标 */
void A_Servo_Resume(void)
{
    uint16_t ticks; /* 剩余控制拍数，饱和到16位 */
    uint16_t target;

    if (!s_servo.paused || !s_servo.resume_valid) return;
    if (Servo_IsPosition() && s_servo.enc_state != SERVO_ENC_ESCAPING)
    {
        /* 刹车后可能滑行且暂停期间观测器未更新，以最新反馈重建起点 */
        if (!Servo_ReadPosition()) return; /* 无有效反馈时保持暂停，允许重发继续。 */
        target = s_servo.target_angle;
#if SERVO_MULTITURN
        if (s_servo.mt_active)
        {
            /* 多圈继续：以扩展坐标实测位置为起点，角速度与原指令一致 */
            Servo_ResyncLoopsAt(Servo_Unwrap(s_servo.angle_circ, s_servo.ref.pos), 1U);
            s_servo.target_angle = target;
            s_servo.paused = 0U;
            Servo_PlanMultiTurn();
            return;
        }
#endif
        Servo_ResyncLoops(1U);
        s_servo.target_angle = target;
    }
    s_servo.paused = 0U;
    if (s_servo.enc_state == SERVO_ENC_ESCAPING) return;
    if (!Servo_IsPosition()) return;

    ticks = (s_servo.remaining > 0xFFFFU) ? 0xFFFFU : (uint16_t)s_servo.remaining;
    C_Traj_Plan(s_servo.target_angle, s_servo.timed_position ? ticks : 0U);
}

/* 停止：丢弃目标，按当前扭矩状态收尾(不可继续) */
void A_Servo_Stop(void)
{
    Servo_MultiTurnAbort(); /* 先把坐标系从扩展坐标换回来，再按行程坐标收尾 */

    /* 无反馈时停止须挂起脱困，等新运动指令 */
    s_servo.paused       = (uint8_t)(s_servo.enc_state == SERVO_ENC_ESCAPING);
    s_servo.resume_valid = 0U;
    s_servo.remaining    = 0U;
    s_servo.motor_pwm    = 0;
    s_servo.boot_pending = 0U;
    s_servo.boot_pwm     = 0U;
    s_servo.boot_value   = 0U;
    s_servo.timed_position = 0U;

    if (Servo_IsPosition())
    {
        (void)Servo_ReadPosition();
        Servo_ResyncLoops(0U);
    }
    Servo_ApplyTorqueOutput();
}

/* 卸力，0=低阻力自由转，1=高阻力短路制动；先改扭矩状态再Stop，避免先刹一下车 */
void A_Servo_Release(uint8_t high_resistance)
{
    s_servo.torque = high_resistance ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
    A_Servo_Stop();
}

/* 从卸力恢复扭矩，并以当前位置重建全部环路(连观测器一起重置) */
void A_Servo_RestoreTorque(void)
{
    if (A_Protect_Fault() != PROT_FAULT_NONE) return;
    if (s_servo.torque == SERVO_TORQUE_ON) return;

    s_servo.torque = SERVO_TORQUE_ON;
    (void)Servo_ReadPosition();
    if (Servo_IsPosition()) Servo_ResyncLoops(1U);
    s_servo.last_pwm = D_Motor_Set(0);
}

/* ============================ 参数类指令 ============================ */

/* 切换工作模式并落盘，返回0=模式号非法 */
uint8_t A_Servo_SetMode(uint8_t mode)
{
    /* 不能手动切入11(自定义行程只能由 AMI/AMX 标定进入)；5/6由 SERVO_MODE_IS_SUPPORTED 挡掉 */
    if (!SERVO_MODE_IS_SUPPORTED(mode) || mode > SERVO_MODE_TIMED_CCW)
        return 0U;

    A_Servo_Stop();            /* 先停稳，避免带着旧模式的目标切过去 */
    g_config.servo_mode = mode;
    A_Config_MarkDirty();
    A_Servo_ApplyConfig();
    return 1U;
}

/* 读当前位置并换算回协议脉宽；电机模式没有位置概念，统一返回中位 */
static uint16_t Servo_PositionToPwm(void)
{
    uint32_t angle; /* 行程坐标，反向模式下已翻转 */

    if (!Servo_IsPosition() || s_servo.span_cdeg == 0U) return SERVO_PWM_MID;

    angle = s_servo.angle;
    if (s_servo.reverse) angle = s_servo.span_cdeg - angle;

    return (uint16_t)(SERVO_PWM_MIN
         + (angle * (SERVO_PWM_MAX - SERVO_PWM_MIN) + s_servo.span_cdeg / 2U)
           / s_servo.span_cdeg);
}

/* 重读一次位置再换算成协议脉宽，供 RAD 查询 */
uint16_t A_Servo_GetPositionPwm(void)
{
    if (Servo_IsPosition()) (void)Servo_ReadPosition();
    return Servo_PositionToPwm();
}

/* 角度校准共同实现：记下锚点，有效行程与零点由 Servo_SetModeFields 现推。
 * kind=SERVO_CAL_MID(SCK，1500us) / SERVO_CAL_ZERO(SCZ，500us)；返回0=电机模式/读失败/锚点不可转入/推不出可用行程 */
static uint8_t Servo_Calibrate(uint8_t kind)
{
    int32_t  raw = A_Encoder_Read(); /* 当前编码器原始角度 */
    uint16_t nominal;                /* 本模式标称行程     */
    uint16_t derived;                /* 推出来的有效行程   */
    int32_t  offset;                 /* 推出来的坐标零点   */
    uint8_t  saved_kind   = g_config.cal_kind;
    uint16_t saved_anchor = g_config.cal_anchor_cdeg;

    if (!Servo_IsPosition() || raw == ENCODER_ANGLE_ERROR) return 0U;

    /* 锚点须在可主动转入区内，外扩段只用来兜超调 */
    if (raw < ENCODER_TRAVEL_LO || raw > ENCODER_TRAVEL_HI) return 0U;

    nominal = Servo_NominalSpan();
    if (nominal == 0U) return 0U; /* 电机模式没有行程可校 */

    g_config.cal_kind        = kind;
    g_config.cal_anchor_cdeg = (uint16_t)raw;
    Servo_DeriveCal(kind, raw, nominal, &derived, &offset);
    if (derived < SERVO_CUSTOM_SPAN_MIN)
    {
        /* 推出的行程不足1度：原样退回，本次校准失败 */
        g_config.cal_kind        = saved_kind;
        g_config.cal_anchor_cdeg = saved_anchor;
        return 0U;
    }

    A_Servo_Stop();                    /* 用旧坐标系干净收尾，再换坐标系 */
    A_Config_MarkDirty();
    Servo_SetModeFields(s_servo.mode); /* 有效行程与零点按新锚点现推 */
    (void)Servo_ReadPosition();        /* 坐标系整体变了，三个角度全部重算 */
    Servo_SetTrajRange();
    Servo_ResyncLoops(1U);
    return 1U;
}

/* SCK：当前位置标定为行程中点；有死区时按两端余量对称收缩行程 */
uint8_t A_Servo_CalibrateMid(void)
{
    return Servo_Calibrate((uint8_t)SERVO_CAL_MID);
}

/* SCZ：当前位置标定为500us端("0度")；反向模式下500us在行程高端 */
uint8_t A_Servo_CalibrateZero(void)
{
    return Servo_Calibrate((uint8_t)SERVO_CAL_ZERO);
}

/* AMI/AMX：把行程一端收到当前位置并进入自定义模式，is_min=1为500us端；返回0=电机模式/读失败/行程太短。
 * 只动指定一端，可连续逐步收窄；500us端在行程哪头由方向决定，见 raise_low。 */
uint8_t A_Servo_SetTravelEnd(uint8_t is_min)
{
    int32_t  raw;       /* 当前编码器原始角度               */
    uint16_t pos;       /* 当前位置的行程坐标               */
    uint16_t offset;    /* 新的坐标零点                     */
    uint16_t span;      /* 新的行程长度                     */
    uint8_t  raise_low; /* 1=把行程低端抬到当前位置，0=压高端 */

    if (!Servo_IsPosition()) return 0U; /* 电机模式没有行程可收 */

    raw = A_Encoder_Read();
    if (raw == ENCODER_ANGLE_ERROR) return 0U;

    /* 用截断后的行程坐标，端点必须落在现有行程内 */
    pos = Servo_ClampToTravel(Servo_RawToCircular(raw));

    raise_low = (uint8_t)(is_min != s_servo.reverse);

    if (raise_low)
    {
        /* 低端抬到当前位置：零点跟着挪，行程少掉挪过的那一段 */
        offset = Servo_WrapAngle(Servo_Offset() + (int32_t)pos + CDEG_RANGE);
        span   = (uint16_t)(s_servo.span_cdeg - pos);
    }
    else
    {
        /* 高端压到当前位置：零点不动，行程就是当前位置的坐标 */
        offset = Servo_WrapAngle(Servo_Offset() + CDEG_RANGE);
        span   = pos;
    }

    /* 先验证再动手：被拒的指令不停舵机、不写Flash */
    if (span < SERVO_CUSTOM_SPAN_MIN) return 0U;

    A_Servo_Stop(); /* 用旧行程干净收尾，再换坐标系 */

    g_config.custom_offset_cdeg = offset;
    g_config.custom_span_cdeg   = span;
    /* 存未经 CFG_DIR_INVERT 翻转的原始方向，读出时再异或 */
    g_config.custom_reverse     = (uint8_t)(s_servo.reverse ^ CFG_DIR_INVERT);
    g_config.servo_mode         = SERVO_MODE_CUSTOM;
    /* 行程窗口刚被显式标定，旧校准锚点作废 */
    g_config.cal_kind           = (uint8_t)SERVO_CAL_NONE;
    A_Config_MarkDirty();

    A_Servo_ApplyConfig(); /* 按新行程重建轨迹范围、观测器和控制器 */
    return 1U;
}

/* SMI/SMX：把当前角度对应的脉宽设为可响应边界，is_min=1为下界；返回0=电机模式/读失败/两端交叉。
 * 只限制可用脉宽，不改行程映射(改行程用 AMI/AMX)；不允许交叉，否则窗口为空、拒收一切指令。 */
uint8_t A_Servo_SetPulseLimit(uint8_t is_min)
{
    uint16_t pulse; /* 当前位置换算回来的协议脉宽 */

    if (!Servo_IsPosition() || s_servo.span_cdeg == 0U) return 0U;
    if (!Servo_ReadPosition()) return 0U;

    pulse = Servo_PositionToPwm();
    if (is_min)
    {
        if (pulse >= g_config.pulse_hi) return 0U;
        g_config.pulse_lo = pulse;
    }
    else
    {
        if (pulse <= g_config.pulse_lo) return 0U;
        g_config.pulse_hi = pulse;
    }
    A_Config_MarkDirty();
    return 1U;
}

/* 把当前位置存为上电目标位置，返回0=电机模式或读失败 */
uint8_t A_Servo_SaveStartup(void)
{
    if (!Servo_IsPosition() || !Servo_ReadPosition()) return 0U;

    g_config.startup_pwm = Servo_PositionToPwm();
    A_Config_MarkDirty();
    return 1U;
}

/* ============================== 遥测输出 ==============================
 * FireWater(VOFA+)明文：逗号分隔，换行结帧。不用 printf(省代码)；必须走 D_UART1_Tx_Write(半双工方向切换在驱动内)。
 * 通道：ref_pos, meas_pos, ref_vel, obs_vel, pwm, u_vel, u_fb, u_i, u_fric, sat */

#define SERVO_PLOT_CH  10U   /* 通道数 */
#define SERVO_PLOT_MAX 96U   /* 帧缓冲上限：10通道最坏约 70 字节 */

/* 把有符号十进制追加进帧缓冲，返回写入后的位置 */
static uint8_t Plot_SInt(uint8_t *buf, uint8_t pos, int32_t value)
{
    uint8_t  digit[10]; /* 倒序暂存的数字字符 */
    uint8_t  n = 0U;
    uint32_t v;

    if (value < 0)
    {
        if (pos < SERVO_PLOT_MAX) buf[pos++] = '-';
        /* 先转无符号再取负，INT32_MIN 也不会溢出 */
        v = (uint32_t)(0U - (uint32_t)value);
    }
    else v = (uint32_t)value;

    do {
        digit[n++] = (uint8_t)('0' + v % 10U);
        v /= 10U;
    } while (v && n < sizeof(digit));

    while (n && pos < SERVO_PLOT_MAX) buf[pos++] = digit[--n];
    return pos;
}

/* 发一帧遥测，只被State任务调用(协作式，无需临界区)；队列满整帧丢弃 */
void A_Servo_printf(void)
{
    uint8_t buf[SERVO_PLOT_MAX];
    uint8_t n = 0U;
    uint8_t i;
    int32_t ch[SERVO_PLOT_CH];

    ch[0] = s_servo.ref.pos;
    ch[1] = (int32_t)s_servo.angle_circ;
    ch[2] = s_servo.ref.vel;
    ch[3] = s_servo.obs.vel;
    ch[4] = (int32_t)s_servo.last_pwm;
    ch[5] = (int32_t)s_servo.dbg.u_vel;
    ch[6] = (int32_t)s_servo.dbg.u_fb;
    ch[7] = (int32_t)s_servo.dbg.u_i;
    ch[8] = (int32_t)s_servo.dbg.u_fric;
    ch[9] = (int32_t)s_servo.dbg.sat;

    for (i = 0U; i < SERVO_PLOT_CH; i++)
    {
        if (i != 0U && n < SERVO_PLOT_MAX) buf[n++] = ',';
        n = Plot_SInt(buf, n, ch[i]);
    }
    if (n < SERVO_PLOT_MAX) buf[n++] = '\n';

    (void)D_UART1_Tx_Write(buf, n);
}

/* ============================== 控制拍 ============================== */

/* 电机模式的一拍：开环输出 + 定圈/定时倒计时 */
static void Servo_MotorControl(void)
{
    int32_t  raw;   /* 本拍编码器原始角度       */
    int32_t  delta; /* 相对上一拍的位移，厘度   */
    uint32_t moved; /* 沿指令方向的有效位移     */
    int16_t  pwm;   /* 按扭矩上限夹过的输出     */

    if (s_servo.remaining == 0U || s_servo.motor_pwm == 0)
    {
        s_servo.last_pwm = D_Motor_Set(0);
        return;
    }

    /* 定圈模式靠编码器累计位移倒计时；无限圈(T=0)不需要计数 */
    if (Servo_IsTurns() && s_servo.remaining != UINT32_MAX)
    {
        raw = A_Encoder_Read();
        if (raw == ENCODER_ANGLE_ERROR)
        {
            /* 读数缺口前后不连续，累计基准作废 */
            s_servo.turn_sample_valid = 0U;

            /* 电位器多半只是转过死区缺口，必须继续驱动(缺口段不计数)；磁编码器读失败连续到阈值才停机 */
            if (!ENCODER_HAS_DEADZONE)
            {
                if (s_servo.enc_fail < SERVO_ENC_FAIL_MAX) s_servo.enc_fail++;
                if (s_servo.enc_fail == SERVO_ENC_FAIL_MAX)
                {
                    s_servo.last_pwm = D_Motor_Set(0);
                    return;
                }
            }
        }
        else
        {
            if (s_servo.turn_sample_valid)
            {
                delta = Servo_WrapDiff((int32_t)raw - s_servo.raw_angle);
                /* 反向转动时位移取反，保证"沿指令方向走的量"恒为正 */
                if ((int32_t)s_servo.motor_pwm * CTRL_MOTOR_SIGN < 0) delta = -delta;
                moved = (uint32_t)((delta > 0) ? delta : 0);
                s_servo.remaining = (moved < s_servo.remaining)
                                  ? s_servo.remaining - moved : 0U;
            }
            s_servo.raw_angle         = raw;
            s_servo.turn_sample_valid = 1U;
            s_servo.enc_fail          = 0U;
        }
    }

    if (s_servo.remaining == 0U)
    {
        s_servo.last_pwm = D_Motor_Set(0);
    }
    else
    {
        /* 开环链路不过控制器，扭矩上限只能在这里夹 */
        pwm = s_servo.motor_pwm;
        if (pwm >  s_servo.out_limit) pwm =  s_servo.out_limit;
        if (pwm < -s_servo.out_limit) pwm = -s_servo.out_limit;

        s_servo.last_pwm = D_Motor_Set(pwm);
        /* 定时模式按拍减；定圈模式上面已经按位移减过了 */
        if (!Servo_IsTurns() && s_servo.remaining != UINT32_MAX)
            s_servo.remaining--;
    }
}

/* 1ms控制拍入口，位置模式固定顺序：读编码器 -> 观测器 -> 轨迹 -> 控制器 -> H桥(观测器要用上一拍PWM) */
void A_Servo_Control(void)
{
    uint16_t pulse; /* PWM输入模式下捕获到的脉宽(us) */
    int16_t  pwm;   /* 本拍控制器输出            */
    uint8_t  hold;  /* 1=本拍冻结轨迹时钟        */

    /* PWM输入模式：每拍把最新脉宽当成一条新的运动指令 */
    if (s_servo.input_source == SERVO_INPUT_PWM)
    {
        pulse = D_PWM_Read();
        if (pulse != 0U)
            Servo_SubmitQ3(Servo_FilterPulse(Servo_PulseIn(pulse)), 0U);
    }

    /* 脱困失败后不再自己动，轴被拨回可测范围即自动解除，扭矩保持卸力 */
    if (s_servo.enc_state == SERVO_ENC_FAULT)
    {
        if (Servo_ReadPosition())
        {
            s_servo.enc_state = SERVO_ENC_OK;
            s_servo.enc_fail  = 0U;
            Servo_ResyncLoops(1U);
        }
        return;
    }

    if (s_servo.torque != SERVO_TORQUE_ON) return; /* 卸力中，输出已由Release设定 */

    if (s_servo.paused || (Servo_IsPosition() && !s_servo.range_valid))
    {
        s_servo.last_pwm = D_Motor_Set(0);
        return;
    }

    if (s_servo.enc_state == SERVO_ENC_ESCAPING)
    {
        Servo_EscapeTick();
        return;
    }

    if (!Servo_IsPosition())
    {
        Servo_MotorControl();
        return;
    }

    /* ---- 位置模式闭环 ---- */
    if (!Servo_ReadPosition())
    {
        if (s_servo.enc_fail < SERVO_ENC_FAIL_MAX) s_servo.enc_fail++;
        if (s_servo.enc_fail == SERVO_ENC_FAIL_MAX)
        {
            /* 编码器彻底失效先停：有死区交给脱困，无死区停着等读成功 */
            s_servo.last_pwm = D_Motor_Set(0);
            C_PosCtrl_Reset();
            if (ENCODER_HAS_DEADZONE) Servo_EscapeStart();
            return;
        }
    }

    /* 反馈用未截断坐标，见 Servo_ClampToTravel */
    C_SpeedObs_Update(s_servo.angle_circ, s_servo.last_pwm, &s_servo.obs);

    /* 输出饱和且误差与参考速度同号(确实跟不上)时冻结轨迹时钟 */
    hold = (uint8_t)(s_servo.dbg.sat
                  && (int64_t)s_servo.dbg.e_pos * s_servo.ref.vel > 0);

    /* 冻结拍数留给堵转检测(顶死时参考不动，只看位移会漏判) */
    if (hold && s_servo.traj_hold < 0xFFFFU) s_servo.traj_hold++;

    C_Traj_Step(&s_servo.ref, hold);

#if SERVO_MULTITURN
    /* 多圈续段与收尾：分段走完续下一段，最后一段走完再折回行程坐标；用 mt_final 判断，不比较定点位置 */
    if (s_servo.mt_active && C_Traj_Is_Done())
    {
        if (s_servo.mt_final) Servo_MultiTurnFinish();
        else                  Servo_PlanMultiTurn();
        C_Traj_Step(&s_servo.ref, 0U); /* 用新段的参考算本拍输出，不空等一拍 */
    }
#endif

#if SERVO_MULTITURN
    if (s_servo.mt_active)
    {
        /* 多圈：参考在扩展坐标上，交给位置环之前必须折回实测所在的那一圈 */
        TrajRef_t ref_ctrl = s_servo.ref;
        ref_ctrl.pos = Servo_RefForCtrl();
        pwm = C_PosCtrl_Update(&ref_ctrl, &s_servo.obs,
                               C_Traj_Is_Done(), &s_servo.dbg);
    }
    else
#endif
    pwm = C_PosCtrl_Update(&s_servo.ref, &s_servo.obs,
                           C_Traj_Is_Done(), &s_servo.dbg);
    s_servo.last_pwm = D_Motor_Set(pwm);


    if (s_servo.timed_position && s_servo.remaining != 0U) s_servo.remaining--;
}
