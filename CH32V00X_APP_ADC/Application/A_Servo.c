/*
 * A_Servo.c —— 舵机编排层
 *
 * 本文件是整个工程里唯一知道"一个控制拍该按什么顺序做事"的地方：
 *     读编码器 -> 观测器 -> 轨迹规划器 -> 位置控制器 -> H桥
 * 上面四个算法模块(C_*)都不知道彼此存在，也不碰硬件；下面的驱动(D_*)只认
 * 寄存器。中间的粘合、模式切换、卸力/暂停/停止这些状态机全部收在这里。
 *
 * 十一种工作模式分成两大类，判据只有 Servo_IsPosition() 一个：
 *     位置模式(1~6, 11) 270/180/360度和自定义行程，走完整的闭环链路
 *     电机模式(7~10)    定圈/定时，开环给固定PWM，只用编码器数圈数
 * 自定义模式(11)与1~6的唯一差别是行程、方向和坐标零点从配置里取，进了
 * Servo_SetModeFields 之后下游逻辑完全共用。
 *
 * 编码器差异集中在三处，全部由 A_Sensor.h 的 ENCODER_MODE 派生：反馈坐标是
 * 圆周还是带符号线性、有没有"测不到角度"的死区、以及360度模式可不可用。
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
#define SERVO_PWM_INPUT_DB_US 6U   /** 仅PWM输入：目标死区额外覆盖静止时±3us的峰峰抖动 */

/* 下面四项只对有死区的编码器有意义，见 A_Sensor.h 的 ENCODER_HAS_DEADZONE。 */
#define SERVO_TRAVEL_GUARD_CDEG 200U        /** 行程两端各留2度禁入，见 Servo_SetTrajRange */
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
    uint16_t span_cdeg;         /** 位置模式行程，厘度；0=电机模式 */
    uint16_t target_angle;      /** 当前目标位置，暂停后继续时按它重新规划 */
    uint16_t traj_lo, traj_hi;  /** 与规划器一致的有效目标范围 */
    uint16_t pulse_prev[2];     /** PWM输入的前两帧脉宽，三帧中值过滤孤立毛刺 */
    uint8_t  pulse_ready;       /** 1=中值滤波器已有历史 */
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
    uint16_t boot_pwm;          /** 待执行的上电脉宽，0=按boot_mode */
    uint16_t boot_value;        /** 脱困期间收到的最新指令的时间参数 */
    uint32_t remaining;         /** 剩余量：定圈=厘度，定时/位置=控制拍，UINT32_MAX=无限 */
    int16_t  motor_pwm;         /** 电机模式下的固定输出PWM(含方向符号) */
    int16_t  out_limit;         /** 对称PWM上限，由保护模块的功率限制回路设定 */
    int16_t  last_pwm;          /** 上一拍实际施加的PWM，观测器要用它做模型预测 */
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

/* 把角度折回 [0, CDEG_RANGE)，入参允许落在 (-CDEG_RANGE, 3*CDEG_RANGE)。
 * 用条件减代替取模：RV32EC 没有硬件除法，一个 % 会展开成 __umodsi3 调用。
 * 三次条件减不是冗余：SMI/SMX 那条路径传进来的是 零点+行程坐标+CDEG_RANGE，
 * 零点接近满量程时可以到 3 倍量程，只减一次会留下一个超出量程的假零点。 */
static uint16_t Servo_WrapAngle(int32_t angle)
{
    if (angle < 0)           angle += CDEG_RANGE;
    if (angle >= CDEG_RANGE) angle -= CDEG_RANGE;
    if (angle >= CDEG_RANGE) angle -= CDEG_RANGE;
    return (uint16_t)angle;
}

/* 角度差折算到最短路径。圆周坐标跨0点(359.99->0)的原始差值会是一整圈，不折算
 * 会让定圈计数瞬间少算/多算一圈；线性坐标不折算，那条捷径要穿过测不到的死区。 */
static int32_t Servo_WrapDiff(int32_t delta)
{
#if ENCODER_IS_CIRCULAR
    if (delta >  CDEG_RANGE / 2) delta -= CDEG_RANGE;
    if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
#endif
    return delta;
}

/* 按模式号展开出行程、方向等派生字段，越界或本板不支持时退回270度正向。
 * 奇数=正向(脉宽增大角度增大)，偶数=反向；1/2->270度，3/4->180度，5/6->360度，
 * 7~10是电机模式(行程为0)；11是自定义模式，行程和方向直接取 SMI/SMX 标定的配置。 */
static void Servo_SetModeFields(ServoMode_t mode)
{
    /* 第三格(360度)已经取不到了，留着只是让下标 (mode-1)/2 保持原样 */
    static const uint16_t span[] = {27000U, 18000U, 0U}; /* 270/180度行程 */

    if (!SERVO_MODE_IS_SUPPORTED(mode))
        mode = SERVO_MODE_270_CW;

    s_servo.mode = mode;

    if (mode == SERVO_MODE_CUSTOM)
    {
        s_servo.reverse   = g_config.custom_reverse;
        s_servo.span_cdeg = g_config.custom_span_cdeg;
        return;
    }

    s_servo.reverse   = (uint8_t)(((uint8_t)mode & 1U) == 0U);
    s_servo.span_cdeg = (mode <= SERVO_MODE_360_CCW)
                      ? span[((uint8_t)mode - 1U) / 2U] : 0U;
}

/* 取当前模式的坐标零点(行程坐标0对应的编码器原始角度)。标准模式用SCK标出来的中值
 * 偏移，自定义模式用SMI/SMX标出来的行程零点，两套分开存，切模式时互不覆盖。 */
static int32_t Servo_Offset(void)
{
    int32_t offset = (s_servo.mode == SERVO_MODE_CUSTOM) ? g_config.custom_offset_cdeg
                                                       : g_config.position_offset_cdeg;
#if !ENCODER_IS_CIRCULAR
    /* Flash仍沿用[0,36000)编码，负零点以36000+offset保存，避免破坏已有配置。 */
    if (offset > ENCODER_POT_ANGLE_MAX) offset -= CDEG_RANGE;
#endif
    return offset;
}

/* 写当前模式的坐标零点 */
static void Servo_SetOffset(uint16_t offset)
{
    if (s_servo.mode == SERVO_MODE_CUSTOM) g_config.custom_offset_cdeg = offset;
    else                                   g_config.position_offset_cdeg = offset;
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

/* 行程坐标投影，**只给轨迹目标和协议上报用**。线性坐标直接夹在[0,span]，圆周坐标
 * 投影到最近的那一端。闭环反馈一律走未截断、未丢符号的 angle_circ：轴被外力推到
 * 端点之外时，投影会把测量值钉在端点上，位置误差恒为0，控制器于是判定"已到位"、
 * 关掉摩擦前馈、泄放积分——舵机在最该顶住的位置反而彻底松手，一推就能推穿死区。 */
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

/* 协议脉宽[500,2500] -> 行程坐标，反向模式下再翻一次 */
static uint16_t Servo_PwmToAngle(uint16_t pwm)
{
    uint32_t angle; /* 换算结果，厘度 */

    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;

    angle = (uint32_t)(pwm - SERVO_PWM_MIN) * s_servo.span_cdeg
          / (SERVO_PWM_MAX - SERVO_PWM_MIN);
    if (s_servo.reverse) angle = (uint32_t)s_servo.span_cdeg - angle;
    return (uint16_t)angle;
}

/* ======================== 公共动作：输出与环路复位 ======================== */

/* 按当前扭矩状态把H桥摆到静止态：有扭矩就刹车(两路全高)，卸力就按阻力档释放。
 * 停止、模式切换、参数生效三条路径的收尾动作完全一样，集中在这里避免三份拷贝。 */
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

/* 让轨迹和控制器以当前实测位置为新起点并清掉历史状态，reinit_obs=1时连观测器一起重置。
 * 调用前必须已刷新过位置。轨迹目标用截断后的 angle，观测器用未截断的 angle_circ。
 * 位置坐标跳变(中位校正)或刚从卸力恢复时必须重置观测器，否则模型预测已完全脱节。 */
static void Servo_ResyncLoops(uint8_t reinit_obs)
{
    if (reinit_obs) C_SpeedObs_Init(s_servo.angle_circ);
    C_Traj_Hold(s_servo.angle); /* 轨迹立即停在这里，不再产生新的参考速度 */
    C_PosCtrl_Reset();          /* 清速度环积分和到位状态机               */
    C_Traj_Step(&s_servo.ref, 0U); /* Hold后的静止参考，避免沿用重规划前的饱和判据 */
    memset(&s_servo.dbg, 0, sizeof(s_servo.dbg));
    /* 目标死区的比较基准必须跟着一起重锚。停止/暂停/换模式/恢复扭矩/脱困完成
     * 这几条路径都会把轨迹拉回当前位置，如果基准还停在旧目标上，紧接着发来的
     * 那条"回到旧目标"的指令就会被死区当成"没变化"吃掉。 */
    s_servo.target_angle = s_servo.angle;
}

/* ==================== 编码器死区：禁入余量与脱困状态机 ====================
 * 传感器有死区时(电位器抽头转出碳膜)，那一段完全没有反馈，读到的不是"很小的
 * 角度"而是"没有角度"。两道防线：
 *   进不去：轨迹范围两端各收 SERVO_TRAVEL_GUARD_CDEG，闭环永远不往那儿走；
 *   出得来：万一还是进去了(上电就在里面、被外力推进去)，开环转出来。
 * 整圈可测的编码器不需要，两道防线在编译期被 ENCODER_HAS_DEADZONE 关掉。 */

/* 按当前模式设置轨迹可用范围，有死区的编码器在两端各留一段禁入余量。
 * 收的是轨迹范围而不是 span：span 还决定协议脉宽到角度的映射，改了会让500~2500us
 * 的含义跟着变。这里只让参考走不到最后那一小段，上报和映射都保持原样。 */
static void Servo_SetTrajRange(void)
{
    int32_t lo = 0;
    int32_t hi = (int32_t)s_servo.span_cdeg;

    if (ENCODER_HAS_DEADZONE && hi > (int32_t)(2U * SERVO_TRAVEL_GUARD_CDEG))
    {
        lo += (int32_t)SERVO_TRAVEL_GUARD_CDEG;
        hi -= (int32_t)SERVO_TRAVEL_GUARD_CDEG;
    }
#if !ENCODER_IS_CIRCULAR
    /* 标定/换模式会移动坐标零点，逻辑行程仍须与真实碳膜安全区求交。 */
    {
        int32_t offset = Servo_Offset();
        int32_t physical_lo = ENCODER_POT_ANGLE_MIN + (int32_t)SERVO_TRAVEL_GUARD_CDEG - offset;
        int32_t physical_hi = ENCODER_POT_ANGLE_MAX - (int32_t)SERVO_TRAVEL_GUARD_CDEG - offset;
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

/* 三帧中值：PWM输入线上偶发的孤立毛刺不该变成一条运动指令 */
static uint16_t Servo_FilterPulse(uint16_t pulse)
{
    uint16_t a, b, c, temp;
    if (!s_servo.pulse_ready)
    {
        s_servo.pulse_prev[0] = s_servo.pulse_prev[1] = pulse;
        s_servo.pulse_ready = 1U;
    }
    a = s_servo.pulse_prev[0]; b = s_servo.pulse_prev[1]; c = pulse;
    s_servo.pulse_prev[0] = b; s_servo.pulse_prev[1] = c;
    if (a > b) { temp = a; a = b; b = temp; }
    if (b > c) { b = c; }
    return (a > b) ? a : b;
}

/* 执行上电动作，优先级：外部PWM指令 > 配置的上电模式。
 * 单独成函数是因为上电时轴可能就停在死区里，那时读到的位置是假的，必须先脱困。 */
static void Servo_ApplyBootAction(void)
{
    if (s_servo.boot_pwm != 0U)
        A_Servo_Submit(s_servo.boot_pwm, s_servo.boot_value);
    else if (g_config.boot_mode == SERVO_BOOT_GOTO_START)
        A_Servo_Submit(g_config.startup_pwm, 0U);
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

/* 两个方向都没转出来：卸力停下等人处理。到这一步说明机构卡死或传感器/接线坏了，
 * 继续转只会顶着障碍发热，不如松手把问题暴露出来。 */
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
        /* 读数回来了还不能立刻交回闭环：此刻抽头就贴在碳膜边缘，一点回弹
         * 就掉回死区。同方向再推一小段，进到行程里面再说。 */
        /* 有反馈后按实测所在的物理端选择内推方向，不能继续赌旧方向。 */
        s_servo.escape_dir = (uint8_t)(s_servo.raw_angle < ENCODER_ANGLE_MID);
        if (s_servo.raw_angle >= ENCODER_ANGLE_LO + (int32_t)SERVO_TRAVEL_GUARD_CDEG
            && s_servo.raw_angle <= ENCODER_ANGLE_HI - (int32_t)SERVO_TRAVEL_GUARD_CDEG)
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

/* 上电嗅探信号线上是不是PWM，返回捕获到的脉宽(us)，0=没有PWM走串口总线。
 * PA1(TIM1输入捕获)和PC0(USART1半双工)接的是同一根信号线，两者必须独占；嗅探期间
 * 先把串口完全释放。判定结果上电后不再改变，进了PWM模式串口就保持关闭到下次上电。 */
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

/* ========================= 对保护模块的几个接口 ========================= */

/* 设置对称PWM输出上限，越界自动夹到[1, CFG_PWM_FULL]。位置模式转发给控制器(抗积分
 * 饱和会跟着走)；电机模式是开环直给，还要在 Servo_MotorControl 里自己夹一次。 */
void A_Servo_SetOutputLimit(int16_t limit)
{
    if (limit < 1) limit = 1;
    if (limit > CFG_PWM_FULL) limit = CFG_PWM_FULL;

    s_servo.out_limit = limit;
    C_PosCtrl_Set_Output_Limit(limit);
}

/* 当前是否带扭矩输出，1=正常输出(闭环或开环)，0=卸力中。
 * 保护模块每拍靠它复检：故障期间上位机发来的指令若把扭矩恢复，要能立刻发现并压回去。 */
uint8_t A_Servo_TorqueOn(void)
{
    return (uint8_t)(s_servo.torque == SERVO_TORQUE_ON);
}

/* 取一次运动采样：上次采样到现在，参考走了多少、机构走了多少，同时把基准推到本拍。
 * 位移在这里做差而不是把坐标抛出去——跨0折算只有本模块知道怎么算，而保护模块自己
 * 攒基准的话，中位校正或换模式那一拍的坐标跳变就会被当成一次真实位移。
 * valid 要求区间**两端**都在跑同一条未走完的轨迹，否则指令刚下发的第一个区间会把
 * 上一条轨迹结束到这条开始之间的静止段算进来。hold_ticks 每次采样后清零，所以调用
 * 周期就是它的统计窗口，漏调一次下次拿到的是两个周期的累计值。 */
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

/* 堵转处理：放弃当前动作，把目标改成此刻的实际位置并保持。要的正是 A_Servo_Stop 的
 * 语义(重读位置、把轨迹和目标锚到实测位置、清速度环积分)，扭矩不动——堵转不卸力，
 * 下一个控制拍立刻以新目标闭环保持。resume_valid 一并清掉：这条动作是被放弃的。 */
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
    /* memset 之后 target_angle 是 0，而 0 正好是行程低端——不初始化的话上电
     * 第一条 #000P0500! 会被目标死区当成"目标没变"吃掉。 */
    s_servo.target_angle = s_servo.angle;

    C_SpeedObs_Init(s_servo.angle_circ); /* 反馈坐标类型由CFG_WRAP_RANGE_CDEG确定 */
    C_Traj_Init(s_servo.angle, TUNE_SMOOTH_ACC, TUNE_SMOOTH_DEC);
    if (Servo_IsPosition()) Servo_SetTrajRange();
    C_PosCtrl_Init();

    s_servo.torque   = SERVO_TORQUE_ON;
    s_servo.last_pwm = D_Motor_Set(0);

    /* 上电就落在死区：读到的0度是假的，此时执行任何上电动作都会朝着一个
     * 编造出来的位置使劲。先开环脱困，转出来再执行——包括"上电释放"，
     * 它也得等轴回到碳膜上才有意义，否则一松手就再也不知道自己在哪。 */
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

/* ============================== 运动指令 ============================== */

/* 下发一条运动指令。pwm=目标脉宽(位置模式=目标角度，电机模式=速度与方向)，
 * value=时间参数(位置/定时模式为毫秒，定圈模式为圈数，0=最快/无限) */
void A_Servo_Submit(uint16_t pwm, uint16_t value)
{
    int32_t raw; /* 定圈模式建立累计基准时读到的原始角度 */
    uint8_t was_paused = s_servo.paused;

    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;

    if (s_servo.enc_state == SERVO_ENC_FAULT || A_Protect_Fault() != PROT_FAULT_NONE
        || (Servo_IsPosition() && !s_servo.range_valid)) return;
    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();
    if (s_servo.enc_state == SERVO_ENC_ESCAPING)
    {
        s_servo.boot_pwm = pwm;
        s_servo.boot_value = value;
        s_servo.boot_pending = 1U;
        s_servo.paused = 0U;
        s_servo.resume_valid = 1U;
        return; /* 脱困结束后再规划，保留最新指令 */
    }

    if (was_paused && Servo_IsPosition())
    {
        /* 暂停后的新目标也须从停稳位置起步，不能沿用暂停瞬间的旧起点。 */
        if (!Servo_ReadPosition()) return;
        Servo_ResyncLoops(1U);
    }
    s_servo.paused       = 0U;
    s_servo.resume_valid = 1U;

    /* ---- 位置模式：交给轨迹规划器，闭环跟随 ---- */
    if (Servo_IsPosition())
    {
        uint16_t target = Servo_PwmToAngle(pwm);
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

        /* ==================== 目标死区 ====================
         * 目标变化不超过 CTRL_TARGET_DB_CDEG(最小可靠步长)时不重新规划，PWM输入
         * 再额外覆盖 SERVO_PWM_INPUT_DB_US 的脉宽抖动。比较前先夹到实际轨迹范围，
         * 端点附近同一有效目标不会反复重规划。
         *
         * 这不是"顺手加的滤波"，而是 PWM 输入模式的必需品：那条路径每收到一个
         * 输入脉冲(50Hz)就调一次本函数，而 Traj_VelCap 会把几厘度的小位移按
         * TUNE_MOVE_MIN_MS 规划成上百拍的慢动作，20ms 走不完就被下一次重规划
         * 打断。后果是连锁的：轨迹永远 done=0 -> 控制器永远进不了 HOLD ->
         * 保持态静音和速度死区全部失效 -> 静止时持续输出上百计数，而且参考永远
         * 追不上目标，留一个固定偏差。
         *
         * 比较基准是**上一次被接受的目标**而不是当前位置，所以连续的小幅指令会
         * 累积：十次 +2us 累计 +20us，一旦超过死区就正常响应，不会被吃掉。
         *
         * 带时间参数的指令(value != 0)一律放行：那是明确的"用这么长时间走过去"
         * 的意图，即使目标没变也应该重新规划。 */
        if (!was_paused && value == 0U && delta <= target_db && delta >= -target_db)
            return;

        s_servo.target_angle   = target;
        C_Traj_Plan(target, value);
        s_servo.timed_position = (uint8_t)(value != 0U);
        s_servo.remaining      = s_servo.timed_position ? C_Traj_Get_Ticks() : 0U;
        return;
    }

    /* ---- 电机模式：以中位为零速，两侧线性映射到正负满PWM ---- */
    if (pwm >= SERVO_PWM_MID)
        s_servo.motor_pwm = (int16_t)((uint32_t)(pwm - SERVO_PWM_MID)
                * MOTOR_PWM_MAX / (SERVO_PWM_MAX - SERVO_PWM_MID));
    else
        s_servo.motor_pwm = (int16_t)-(int32_t)((uint32_t)(SERVO_PWM_MID - pwm)
                * MOTOR_PWM_MAX / (SERVO_PWM_MID - SERVO_PWM_MIN));

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
        /* 刹车后仍可能滑行，且暂停期间观测器未更新。用最新反馈重建起点，
         * 清除旧速度、负载估计及驱动延迟历史，防止继续时先追回暂停位置。 */
        if (!Servo_ReadPosition()) return; /* 无有效反馈时保持暂停，允许重发继续。 */
        target = s_servo.target_angle;
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
    /* 无反馈时无法闭环保持，停止必须挂起脱困，直到新的运动指令到来。 */
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

/* 卸力，high_resistance: 0=低阻力自由转，1=高阻力短路制动阻尼。
 * 先改扭矩状态再调Stop，让Stop末尾的收尾动作直接落到释放上；反过来写会先刹一下车。 */
void A_Servo_Release(uint8_t high_resistance)
{
    s_servo.torque = high_resistance ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
    A_Servo_Stop();
}

/* 从卸力恢复扭矩，并以当前实际位置重建全部环路。卸力期间轴可能被外力转到任意位置，
 * 观测器的模型预测已经完全脱节，所以必须连观测器一起重置。 */
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

/* 切换工作模式并落盘，返回0=模式号非法。上限卡在10而不是11：自定义模式的行程要现场
 * 标定，手动切进去只会得到上次遗留的(甚至为0的)行程，想进11只有 SMI/SMX 一条路。 */
uint8_t A_Servo_SetMode(uint8_t mode)
{
    /* 上限卡在10而不是11：自定义模式的行程要现场标定，手动切进去只会
     * 得到一个上次遗留的(甚至是0的)行程。想进11只有 SMI/SMX 一条路。
     * 5/6(360度)已停用，由 SERVO_MODE_IS_SUPPORTED 一并挡掉。 */
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

/* SCK：把当前物理位置标定为行程中点，返回0=电机模式/读失败/标定后会超出可测范围。
 * 自定义模式下语义不变，只是改写自定义行程的零点：行程长度和方向都不动，整段行程
 * 平移到"当前位置落在1500us"的位置上。1500us无论正反向都映射到中点，公式是同一条。 */
uint8_t A_Servo_CalibrateMid(void)
{
    uint16_t center;                 /* 行程中点，厘度       */
    int32_t raw = A_Encoder_Read();  /* 当前编码器原始角度   */

    if (!Servo_IsPosition() || raw == ENCODER_ANGLE_ERROR) return 0U;

    center = (uint16_t)(s_servo.span_cdeg / 2U);
#if !ENCODER_IS_CIRCULAR
    if (raw - center < ENCODER_POT_ANGLE_MIN
        || raw - center + s_servo.span_cdeg > ENCODER_POT_ANGLE_MAX) return 0U;
#endif
    Servo_SetOffset(Servo_WrapAngle(raw + CDEG_RANGE - center));
    A_Config_MarkDirty();

    /* 坐标系整体平移了，所有环路都要按新坐标重来 */
    s_servo.raw_angle  = raw;
    s_servo.angle_circ = center; /* 偏移刚按center标定，两个坐标此刻相等 */
    s_servo.angle      = center;
    Servo_SetTrajRange();
    Servo_ResyncLoops(1U);
    return 1U;
}

/* SMI/SMX：把行程的一端收到当前位置，进入(或更新)自定义模式。is_min=1为SMI(500us端)，
 * 0为SMX(2500us端)；返回0=电机模式/读失败/剩下的行程太短。
 * 只动指定的那一端，另一端和方向都不变，所以每发一次行程只会向内收窄，基准是当前
 * 模式的行程，于是可以连着发几次逐步收窄。
 * "500us端"在行程坐标里是哪一头由方向决定：正向时在低端，反向时在高端，所以真正要
 * 动的是低端还是高端要把指令和方向异或起来看(见 raise_low)，直接映射会把两条指令对调。 */
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

    /* 端点必须落在现有行程之内，所以用截断后的行程坐标：轴被推到死区里
     * 时投影到最近的端点，收出来的行程要么不变要么为0，由下面的下限兜住。 */
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

    /* 先验证再动手：被拒的指令不该把舵机停下来，更不该写进Flash */
    if (span < SERVO_CUSTOM_SPAN_MIN) return 0U;

    A_Servo_Stop(); /* 用旧行程干净收尾，再换坐标系 */

    g_config.custom_offset_cdeg = offset;
    g_config.custom_span_cdeg   = span;
    g_config.custom_reverse     = s_servo.reverse; /* 收窄不改方向 */
    g_config.servo_mode         = SERVO_MODE_CUSTOM;
    A_Config_MarkDirty();

    A_Servo_ApplyConfig(); /* 按新行程重建轨迹范围、观测器和控制器 */
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
 *
 * FireWater(VOFA+ 的明文协议)：ASCII，逗号分隔，'\n' 结帧，上位机按浮点解析。
 *
 * 两个实现约束，都不是随便选的：
 *
 * 1) **不用 printf**。正常固件根本没链接它(只有 APP_MODE_CALIB 那版用)，为
 *    一条调试输出把 newlib 的格式化拖进来要多花好几 KB。这里自己转整数，
 *    和 A_UartCmd.c 的 Reply_UInt 是同一套做法。反正遥测量全是整数(厘度、
 *    厘度/秒、PWM计数)，上位机当浮点读一样画。
 *
 * 2) **必须走 D_UART1_Tx_Write，不能直写 USART**。这条线是半双工的，收发
 *    方向切换、TXE中断搬字节、TC中断确认末字节移出——整套都在 D_uart.c 里
 *    闭环。绕过它直写 DR 会和 TXE 中断抢寄存器，还会把自己发的字节当成收到
 *    的数据。顺带一个好处：PWM输入模式下 PC0 已经交还给共线信号，那时
 *    D_UART1_Tx_Write 自己会拒发(s_enabled=0)，这里不需要再判一次。
 *
 * 通道顺序(改了记得同步改上位机的图例)：
 *     1 ref_pos   规划位置    厘度
 *     2 meas_pos  实测位置    厘度，未滤波，看量化和噪声就看它
 *     3 ref_vel   规划速度    厘度/秒
 *     4 obs_vel   观测速度    厘度/秒
 *     5 pwm       实际输出    PWM计数(含方向符号，已限幅)
 *     6 u_vel     速度前馈    PWM
 *     7 u_fb      速度环P反馈 PWM
 *     8 u_i       速度环积分  PWM
 *     9 u_fric    摩擦前馈    PWM
 *    10 sat       本拍是否饱和 0/1
 *
 * 分项而不是只看总输出：整机纹波可能来自完全不同的地方——前馈跟着 ref_vel 走
 * (规划的问题)、P反馈跟着速度估计噪声走(传感器的问题)、积分自己爬升(静摩擦顶
 * 不动)，只看总输出这三种长得一模一样。 */

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

/* 按 FireWater 格式发一帧遥测，供上位机实时绘图。调度器是协作式的，本函数只被 State
 * 任务调用，与1ms控制拍不会互相抢占，所以直接读 s_servo 不需要临界区。
 * 队列放不下就整帧丢弃，绝不阻塞——这是调试输出，不值得为它拖慢控制拍。 */
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
            /* 缺口两侧的两个读数之间隔着一段没测到的位移，不是连续量，
             * 基准必须作废，否则下一拍会把整段缺口当成一次跳变算进去。 */
            s_servo.turn_sample_valid = 0U;

            /* 电位器模式下这多半只是转过了死区那段缺口，**必须继续驱动**：
             * 停在缺口里就再没有反馈能把自己转出来了。代价是缺口那一段不
             * 计数，定圈每转会少算这个角度——电位器的物理限制，不是bug。
             * 磁编码器没有缺口，读失败就是真失败，连续到阈值照旧停机。 */
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

/* 1ms控制拍入口，由调度器最高优先级任务调用。位置模式的一拍固定是这条链，顺序不能换：
 *     读编码器 -> 观测器更新 -> 轨迹推进 -> 控制器计算 -> 下发H桥
 * 观测器要用上一拍实际施加的PWM做模型预测，所以必须先更新观测器再算新的PWM。 */
void A_Servo_Control(void)
{
    uint16_t pulse; /* PWM输入模式下捕获到的脉宽 */
    int16_t  pwm;   /* 本拍控制器输出            */
    uint8_t  hold;  /* 1=本拍冻结轨迹时钟        */

    /* PWM输入模式：每拍把最新脉宽当成一条新的运动指令 */
    if (s_servo.input_source == SERVO_INPUT_PWM)
    {
        pulse = D_PWM_Read();
        if (pulse != 0U) A_Servo_Submit(Servo_FilterPulse(pulse), 0U);
    }

    /* 脱困失败后停在这里：不再自己动，但有人把轴拨回碳膜上就自动解除。
     * 扭矩保持卸力，等下一条运动指令按正常路径恢复。 */
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
            /* 编码器彻底失效：先停，带着错误反馈继续跑最危险。
             * 电位器模式下这通常是抽头被推出了碳膜，交给脱困状态机转回来；
             * 磁编码器没有死区，失效就是器件坏了，停着等下一次读成功。 */
            s_servo.last_pwm = D_Motor_Set(0);
            C_PosCtrl_Reset();
            if (ENCODER_HAS_DEADZONE) Servo_EscapeStart();
            return;
        }
    }

    /* 反馈必须用未截断的实际坐标：轴被推到行程端点之外时，截断值会把误差
     * 抹成0，舵机反而在最该顶住的位置松手。详见 Servo_ClampToTravel。 */
    C_SpeedObs_Update(s_servo.angle_circ, s_servo.last_pwm, &s_servo.obs);

    /* 输出饱和且实际落后于参考时冻结轨迹时钟(参考调节器)，
     * 避免轨迹一路跑远、误差越积越大。判据要求误差和参考速度同号，
     * 也就是"确实是跟不上"，而不是刚换向的正常瞬态。 */
    hold = (uint8_t)(s_servo.dbg.sat
                  && (int64_t)s_servo.dbg.e_pos * s_servo.ref.vel > 0);

    /* 冻结的拍数要留给堵转检测。顶死的时候正是这条支路一直成立，参考被钉在
     * 原地不动，"参考在变"的判据反而看不见最典型的堵转，见 A_Protect.c。 */
    if (hold && s_servo.traj_hold < 0xFFFFU) s_servo.traj_hold++;

    C_Traj_Step(&s_servo.ref, hold);

    pwm = C_PosCtrl_Update(&s_servo.ref, &s_servo.obs,
                           C_Traj_Is_Done(), &s_servo.dbg);
    s_servo.last_pwm = D_Motor_Set(pwm);


    if (s_servo.timed_position && s_servo.remaining != 0U) s_servo.remaining--;
}
