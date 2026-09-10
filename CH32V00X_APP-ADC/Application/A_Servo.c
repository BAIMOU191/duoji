/*
 * A_Servo.c —— 舵机编排层
 *
 * 本文件是整个工程里唯一知道"一个控制拍该按什么顺序做事"的地方：
 *
 *     读编码器 -> 观测器 -> 轨迹规划器 -> 位置控制器 -> H桥
 *
 * 上面四个算法模块(C_*)都不知道彼此存在，也不碰硬件；下面的驱动(D_*)只认
 * 寄存器。中间的粘合、模式切换、卸力/暂停/停止这些状态机全部收在这里。
 *
 * 十一种工作模式分成两大类，判据只有 Servo_IsPosition() 一个：
 *     位置模式(1~6, 11) 270/180/360度和自定义行程，走完整的闭环链路
 *     电机模式(7~10)    定圈/定时，开环给固定PWM，只用编码器数圈数
 *
 * 自定义模式(11)与1~6的唯一差别是行程、方向和坐标零点从配置里取，而不是
 * 由模式号推导；进了 Servo_SetModeFields 之后下游全部逻辑完全共用。
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

#define SERVO_ENC_FAIL_MAX    5U  /* 连续读编码器失败达到该次数才停机，滤掉偶发I2C错误 */
#define SERVO_INPUT_DETECT_MS 60U /* 上电嗅探PWM的时间窗，覆盖最坏相位下至少两个50Hz周期 */
#define SERVO_PWM_INPUT_DB_US 6U /* 仅PWM输入：覆盖静止时±3us的峰峰抖动 */

/* ==================== 电位器死区相关(见 A_Sensor.h) ==================== */
#define SERVO_TRAVEL_GUARD_CDEG 200U /* 行程两端各留2度禁入，见 Servo_SetTrajRange */
#define SERVO_ESCAPE_PWM  (CFG_PWM_FULL / 4) /* 脱困开环幅值：够克服摩擦，又不会甩过头 */
#define SERVO_ESCAPE_TICKS   1500U   /* 单方向最长脱困时间，控制拍=ms */
#define SERVO_ESCAPE_SETTLE   120U   /* 读数恢复后继续内推的控制拍 */

/* 控制器的PWM满量程必须和电机驱动的实际上限一致，否则限幅逻辑会失真 */
typedef char cfg_pwm_full_matches_motor[(CFG_PWM_FULL == MOTOR_PWM_MAX) ? 1 : -1];
typedef char cfg_feedback_coordinates_match_sensor[
    (CFG_WRAP_RANGE_CDEG == (ENCODER_IS_CIRCULAR ? CDEG_RANGE : 0)) ? 1 : -1];




/* 编码器可用性。只有电位器会出现"轴还在但反馈没了"，见 ENCODER_HAS_DEADZONE。 */
typedef enum {
    SERVO_ENC_OK = 0,   /* 反馈可用                           */
    SERVO_ENC_ESCAPING, /* 抽头在死区里，正开环往外转         */
    SERVO_ENC_FAULT     /* 两个方向都没转出来，已卸力等人处理 */
} ServoEncState_t;

/* 扭矩状态。卸力分低阻力(两路低，自由转)和高阻力(两路高，短路制动阻尼)。 */
typedef enum {
    SERVO_TORQUE_ON = 0,  /* 正常闭环/开环输出 */
    SERVO_RELEASE_LOW,    /* 卸力，低阻力      */
    SERVO_RELEASE_HIGH    /* 卸力，高阻力      */
} ServoTorque_t;

typedef struct {
    ServoMode_t mode;           /* 当前工作模式，1~10                          */
    uint16_t angle;             /* 行程坐标[0,span]，死区里的位置已投影到端点；
                                 * 只给轨迹目标和位置上报用                    */
    int32_t  angle_circ;        /* 未截断反馈：电位器为带符号线性坐标，磁编码器为圆周坐标；
                                 * 闭环反馈只能用它，理由见Servo_ClampToTravel */
    int32_t  raw_angle;         /* 原始反馈保留负角度，用于定圈累计位移 */
    uint16_t span_cdeg;         /* 位置模式行程，厘度；0=电机模式              */
    uint16_t target_angle;      /* 当前目标位置，暂停后继续时按它重新规划      */
    uint16_t traj_lo, traj_hi;  /* 与规划器一致的有效目标范围 */
    uint16_t pulse_prev[2];     /* 三帧中值过滤孤立脉宽毛刺 */
    uint8_t  pulse_ready;
    uint8_t  range_valid;       /* 当前行程与物理可测范围有交集 */
    uint8_t  reverse;           /* 1=脉宽增大对应角度减小(偶数模式)            */
    uint8_t  enc_fail;          /* 连续读编码器失败次数                        */
    uint8_t  input_source;      /* SERVO_INPUT_TX / SERVO_INPUT_PWM，上电定终身 */
    uint8_t  paused;            /* 1=已暂停，输出刹车但保留目标                */
    uint8_t  resume_valid;      /* 1=有可恢复的目标(收到过运动指令且未被停止)  */
    uint8_t  timed_position;    /* 1=位置模式带时间参数，需要按拍倒计时        */
    uint8_t  turn_sample_valid; /* 1=raw_angle是有效的定圈累计基准             */
    uint8_t  torque;            /* ServoTorque_t                               */
    uint8_t  enc_state;         /* ServoEncState_t                             */
    uint8_t  escape_dir;        /* 当前脱困方向，1=正向                        */
    uint8_t  escape_retried;    /* 1=已经掉过一次头                            */
    uint8_t  boot_pending;      /* 1=上电动作在等脱困完成                      */
    uint16_t escape_ticks;      /* 本方向已经驱动的控制拍                      */
    uint16_t escape_settle;     /* 读数恢复后继续内推的控制拍                  */
    uint16_t traj_hold;         /* 轨迹时钟被饱和冻结的控制拍数，采样后清零    */
    uint8_t  motion_valid;      /* 1=上次运动采样时正在跑轨迹，位移可做差      */
    int32_t  motion_ref;        /* 上次运动采样的参考位置                      */
    int32_t  motion_act;        /* 上次运动采样的实测位置                      */
    uint16_t boot_pwm;          /* 待执行的上电脉宽，0=按boot_mode             */
    uint16_t boot_value;        /* 脱困期间收到的最新指令的时间参数 */
    uint32_t remaining;         /* 剩余量：定圈=厘度，定时/位置=控制拍；
                                 * UINT32_MAX=无限(T=0)                        */
    int16_t  motor_pwm;         /* 电机模式下的固定输出PWM(含方向符号)         */
    int16_t  out_limit;         /* 对称PWM上限，由保护模块的功率限制回路设定   */
    int16_t  last_pwm;          /* 上一拍实际施加的PWM，观测器要用它做模型预测 */
    TrajRef_t     ref;          /* 本拍参考位置/速度/加速度                    */
    SpeedObsOut_t obs;          /* 本拍观测器输出                              */
    PosCtrlDbg_t  dbg;          /* 控制器遥测，同时给轨迹冻结判据提供饱和信息  */
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

/*
 * @fn      Servo_WrapAngle
 * @brief   把 [0, 2*CDEG_RANGE) 的角度折回 [0, CDEG_RANGE)
 * @param   angle 待折算角度，调用方须保证不超过两倍量程
 * @return  折算后的角度，厘度
 *
 * 用一次条件减代替取模：CH32V006是RV32EC，没有硬件除法指令，
 * 一个 % 会展开成 __umodsi3 函数调用，在1ms控制拍里不值得。
 */
static uint16_t Servo_WrapAngle(uint32_t angle)
{
    return (uint16_t)((angle >= CDEG_RANGE) ? (angle - CDEG_RANGE) : angle);
}

/*
 * @fn      Servo_WrapDiff
 * @brief   把角度差折算到最短路径 (-量程/2, 量程/2]
 * @param   delta 两次角度采样之差，厘度
 * @return  折算后的增量，厘度
 *
 * 电机连续转动跨0点(359.99度 -> 0度)时原始差值会是一整圈，不折算的话
 * 定圈计数会瞬间少算/多算一圈。前提是每拍真实位移远小于半圈，1kHz恒满足。
 */
static int32_t Servo_WrapDiff(int32_t delta)
{
#if ENCODER_IS_CIRCULAR
    if (delta >  CDEG_RANGE / 2) delta -= CDEG_RANGE;
    if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
#endif
    return delta;
}

/*
 * @fn      Servo_SetModeFields
 * @brief   按模式号展开出行程、方向等派生字段
 * @param   mode 目标模式，越界时退回270度正向
 * @return  无
 *
 * 模式编号约定：奇数=正向(脉宽增大角度增大)，偶数=反向；
 * 1/2->270度，3/4->180度，7~10是电机模式(行程为0)。
 * 11是自定义模式，行程和方向不由编号推导，直接取 SMI/SMX 标定出来的配置。
 *
 * 5/6(360度)已停用，见 A_Config.h 的 SERVO_MODE_IS_SUPPORTED。这里连同越界
 * 一起退回270度，好让Flash里遗留的5/6还能开机，而不是整份参数作废。
 */
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

/*
 * @fn      Servo_Offset
 * @brief   当前模式的坐标零点：行程坐标0对应的编码器原始角度
 * @param   无
 * @return  零点角度，厘度
 *
 * 标准模式(1~6)用SCK标出来的中值偏移，自定义模式用SMI/SMX标出来的行程零点。
 * 两套偏移分开存，切模式时互不覆盖：标过自定义行程再切回模式1，原来的中位
 * 校正还在；反过来也一样。
 */
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

/*
 * @fn      Servo_SetOffset
 * @brief   写当前模式的坐标零点
 * @param   offset 新零点，厘度
 * @return  无
 */
static void Servo_SetOffset(uint16_t offset)
{
    if (s_servo.mode == SERVO_MODE_CUSTOM) g_config.custom_offset_cdeg = offset;
    else                                   g_config.position_offset_cdeg = offset;
}

/*
 * @fn      Servo_RawToCircular
 * @brief   编码器原始角度 -> 零点校正后的反馈坐标，不做任何截断
 * @param   raw 编码器角度，厘度
 * @return  磁编码器为圆周坐标；电位器为可正可负的线性坐标
 */
static int32_t Servo_RawToCircular(int32_t raw)
{
#if ENCODER_IS_CIRCULAR
    return Servo_WrapAngle((uint32_t)raw + CDEG_RANGE - Servo_Offset());
#else
    return raw - Servo_Offset();
#endif
}

/* 仅供目标/协议上报使用的投影。电位器线性夹在[0,span]，磁编码器
 * 投影到最近端点。闭环始终使用未截断、未丢符号的angle_circ反馈。 */
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

/*
 * @fn      Servo_ReadPosition
 * @brief   读一次编码器，刷新原始角度、圆周坐标和行程坐标
 * @param   无
 * @return  1=成功，0=I2C失败(此时三个角度都保持上一次的值)
 */
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

/*
 * @fn      Servo_ApplyTorqueOutput
 * @brief   按当前扭矩状态把H桥摆到静止态
 * @param   无
 * @return  无
 *
 * 有扭矩就刹车(两路全高)，卸力就按阻力档释放。停止、模式切换、参数生效
 * 这三条路径的收尾动作完全一样，集中在这里，避免三份拷贝各改各的。
 */
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

/*
 * @fn      Servo_ResyncLoops
 * @brief   让轨迹和控制器以当前实测位置为新起点，清掉历史状态
 * @param   reinit_obs 1=连观测器一起重置
 * @return  无
 *
 * 调用前必须已经刷新过位置(Servo_ReadPosition 或直接赋值)。两个坐标各司其职：
 * 轨迹目标用截断后的 angle(落在死区时自动指向最近端点)，观测器用未截断的
 * angle_circ(电位器为线性坐标，只有磁编码器按圆周折算残差)。
 *
 * 什么时候要重置观测器：位置坐标发生了跳变(中位校正)，或者刚从卸力恢复
 * (卸力期间轴被人转过，观测器的模型预测已经完全脱节)。暂停/停止当拍
 * 保留速度估计；暂停期间不推进观测器，继续时须按最新位置重新初始化。
 */
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

/* ==================== 电位器死区：禁入余量与脱困状态机 ====================
 *
 * 电位器的抽头一旦转出碳膜就完全没有反馈了，而且下拉电阻把读数钳到地——
 * 那是"没有角度"，不是"角度很小"。所以这一段有两道防线：
 *   进不去：轨迹范围两端各收 SERVO_TRAVEL_GUARD_CDEG，闭环永远不往那儿走；
 *   出得来：万一还是进去了(上电就在里面、被外力推进去)，开环转出来。
 * 磁编码器整圈都可测，两道防线在编译期就被 ENCODER_HAS_DEADZONE 关掉。
 */

/*
 * @fn      Servo_SetTrajRange
 * @brief   按当前模式设置轨迹可用范围，电位器模式两端各留一段禁入余量
 * @param   无
 * @return  无
 *
 * 收的是轨迹范围而不是 span：span 还决定协议脉宽到角度的映射，改了会让
 * 500~2500us 的含义跟着变。这里只是让参考永远走不到最后那2度，上报和
 * 映射都保持原样。
 */
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

/*
 * @fn      Servo_ApplyBootAction
 * @brief   执行上电动作，优先级：外部PWM指令 > 配置的上电模式
 * @param   无
 * @return  无
 *
 * 从 A_Servo_Init 里拆出来单独成函数，是因为上电时轴可能就停在死区里：
 * 那时候读到的位置是假的，必须先脱困，脱出来再回头执行这里。
 */
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

/*
 * @fn      Servo_EscapeStart
 * @brief   进入脱困状态，从正方向开始试
 * @param   无
 * @return  无
 */
static void Servo_EscapeStart(void)
{
    s_servo.enc_state      = SERVO_ENC_ESCAPING;
    s_servo.escape_dir     = 1U;
    s_servo.escape_retried = 0U;
    s_servo.escape_ticks   = 0U;
    s_servo.escape_settle  = 0U;
    s_servo.enc_fail       = 0U;
}

/*
 * @fn      Servo_EscapeFail
 * @brief   两个方向都没转出来：卸力停下，等人处理
 * @param   无
 * @return  无
 *
 * 到这一步说明不是"停在缺口里"这么简单——机构卡死，或者电位器/接线坏了。
 * 继续转只会顶着障碍发热，不如松手把问题暴露出来。
 */
static void Servo_EscapeFail(void)
{
    s_servo.enc_state    = SERVO_ENC_FAULT;
    s_servo.boot_pending = 0U;
    A_Servo_Release(0U);
}

/*
 * @fn      Servo_EscapeFinish
 * @brief   脱困成功：停下、重建环路、补上欠着的上电动作
 * @param   无
 * @return  无
 */
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

/*
 * @fn      Servo_EscapeTick
 * @brief   脱困的一拍：低速开环单向转，直到读数回来并且已经转进行程里
 * @param   无
 * @return  无
 */
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
        s_servo.escape_dir = (uint8_t)(s_servo.raw_angle < (int32_t)ENCODER_POT_SPAN_CDEG / 2);
        if (s_servo.raw_angle >= ENCODER_POT_ANGLE_MIN + (int32_t)SERVO_TRAVEL_GUARD_CDEG
            && s_servo.raw_angle <= ENCODER_POT_ANGLE_MAX - (int32_t)SERVO_TRAVEL_GUARD_CDEG)
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

/*
 * @fn      Servo_DetectInput
 * @brief   上电嗅探信号线上是不是PWM，决定本次上电用哪种输入
 * @param   无
 * @return  捕获到的脉宽(us)；0=没有PWM，走串口总线
 *
 * PA1(TIM1输入捕获)和PC0(USART1半双工)接的是同一根信号线，两者必须独占。
 * 嗅探期间先把串口完全释放，只让捕获通道听；D_PWM_Read内部已经校验过
 * 20ms周期，串口数据不会被误判成PWM。
 *
 * 判定结果上电后不再改变：一旦进了PWM模式，串口会保持关闭直到下次上电。
 */
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

uint8_t A_Servo_InputSource(void)
{
    return s_servo.input_source;
}

/* ========================= 对保护模块的几个接口 ========================= */

/*
 * @fn      A_Servo_SetOutputLimit
 * @brief   设置对称PWM输出上限
 * @param   limit 上限，[1, CFG_PWM_FULL]，越界自动夹住
 * @return  无
 *
 * 位置模式转发给控制器(它的抗积分饱和会跟着这个上限走，限功率时积分不会
 * 继续充电)；电机模式是开环直给，控制器不在链路上，所以还要在
 * Servo_MotorControl 里自己夹一次，否则限扭矩对定圈/定时模式完全无效。
 */
void A_Servo_SetOutputLimit(int16_t limit)
{
    if (limit < 1) limit = 1;
    if (limit > CFG_PWM_FULL) limit = CFG_PWM_FULL;

    s_servo.out_limit = limit;
    C_PosCtrl_Set_Output_Limit(limit);
}

/*
 * @fn      A_Servo_TorqueOn
 * @brief   当前是否带扭矩输出
 * @param   无
 * @return  1=正常输出(闭环或开环)，0=卸力中
 *
 * 保护模块每拍靠它复检：故障期间上位机发来的ULR或运动指令会把扭矩恢复，
 * 要能立刻发现并压回去。
 */
uint8_t A_Servo_TorqueOn(void)
{
    return (uint8_t)(s_servo.torque == SERVO_TORQUE_ON);
}

/*
 * @fn      A_Servo_SampleMotion
 * @brief   取一次运动采样：上次采样到现在，参考走了多少、实际走了多少
 * @param   out 采样结果，不可为空
 * @return  无
 *
 * 保护模块靠"参考在走而实际不走"判堵转。位移在这里做差而不是把两个坐标
 * 抛出去，有两个理由：圆周坐标跨0点的折算(Servo_WrapDiff)只有本模块知道
 * 怎么算；而且保护模块拿到裸坐标就得自己攒基准，一旦它和本模块对"什么时候
 * 该重新锚定"的理解不一致，中位校正或换模式那一拍的坐标跳变就会被当成
 * 一次真实位移。
 *
 * valid 要求区间的**两端**都在跑同一条未走完的轨迹：只看当前拍的话，指令
 * 刚下发的第一个区间会把"上一条轨迹结束到这条轨迹开始"之间的静止段算进来。
 *
 * hold_ticks 每次采样后清零，所以调用周期就是它的统计窗口；漏调一次，
 * 下一次拿到的是两个周期的累计值。保护模块每20ms固定调一次。
 */
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

/*
 * @fn      A_Servo_HoldHere
 * @brief   放弃当前动作，把目标改成此刻的实际位置并保持
 * @param   无
 * @return  无
 *
 * 堵转的处理动作。要的正是 A_Servo_Stop 的语义：它会重读位置、把轨迹和
 * 目标一起锚到当前实测位置、清掉速度环里已经充满的积分，然后按当前扭矩
 * 状态收尾。扭矩不动——堵转不卸力，下一个1ms控制拍立刻以新目标闭环保持。
 *
 * 顺带把 resume_valid 清掉也是对的：这条动作是被放弃的，没有"继续"可言，
 * 上位机想再试必须重发一条指令。
 */
void A_Servo_HoldHere(void)
{
    if (!Servo_IsPosition() || s_servo.torque != SERVO_TORQUE_ON) return;
    A_Servo_Stop();
}

/* ============================ 生命周期与配置 ============================ */

/*
 * @fn      A_Servo_ApplyConfig
 * @brief   让新的模式/中位参数立即生效，并把所有环路对齐到当前实测位置
 * @param   无
 * @return  无
 */
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

/*
 * @fn      A_Servo_Init
 * @brief   上电初始化：选输入源、初始化三个算法模块、执行上电动作
 * @param   无
 * @return  无
 */
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

/*
 * @fn      A_Servo_Submit
 * @brief   下发一条运动指令
 * @param   pwm   目标脉宽(us)，位置模式=目标角度，电机模式=速度与方向
 * @param   value 时间参数：位置/定时模式=毫秒，定圈模式=圈数；0=最快/无限
 * @return  无
 */
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
         * 串口目标变化不超过最小步长时不重新规划；PWM输入额外覆盖6us峰峰抖动。
         * 比较前先夹到实际轨迹范围，端点附近同一有效目标不反复重规划。
         *
         * 这不是"顺手加的滤波"，而是 PWM 输入模式的必需品：那条路径每收到
         * 一个输入脉冲(50Hz)就调一次本函数，而 Traj_VelCap 会把几厘度的小
         * 位移按 TUNE_MOVE_MIN_MS 规划成 100 拍以上的慢动作——20ms 走不完就
         * 被下一次重规划打断。后果是连锁的：
         *     轨迹永远 done=0  ->  控制器永远进不了 HOLD
         *                      ->  保持态静音和速度死区全部失效
         *                      ->  静止时持续输出约 160 计数，且参考永远
         *                          追不上目标，留一个几厘度的固定偏差
         * 主机仿真实测：输入完全不抖时 traj_done 占比就是 0%，|pwm| 均值 153。
         *
         * 比较基准是**上一次被接受的目标**而不是当前位置，所以连续的小幅指令
         * 会累积：十次 +2us 累计 +20us，一旦超过死区就正常响应，不会被吃掉。
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

/*
 * @fn      A_Servo_Pause
 * @brief   暂停：输出刹车，但保留目标以便继续
 * @param   无
 * @return  无
 */
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

/*
 * @fn      A_Servo_Resume
 * @brief   继续：以当前位置为起点，用剩余时间重新规划到原目标
 * @param   无
 * @return  无
 */
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

/*
 * @fn      A_Servo_Stop
 * @brief   停止：丢弃目标，按当前扭矩状态收尾(不可继续)
 * @param   无
 * @return  无
 */
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

/*
 * @fn      A_Servo_Release
 * @brief   卸力
 * @param   high_resistance 0=低阻力(自由转)，1=高阻力(短路制动阻尼)
 * @return  无
 *
 * 先改扭矩状态再调Stop，让Stop末尾的收尾动作直接落到释放上；
 * 反过来写会先刹一下车再释放，多一次无意义的输出跳变。
 */
void A_Servo_Release(uint8_t high_resistance)
{
    s_servo.torque = high_resistance ? SERVO_RELEASE_HIGH : SERVO_RELEASE_LOW;
    A_Servo_Stop();
}

/*
 * @fn      A_Servo_RestoreTorque
 * @brief   从卸力恢复扭矩，并以当前实际位置重建全部环路
 * @param   无
 * @return  无
 *
 * 卸力期间轴可能被外力转到任意位置，观测器的模型预测已经完全脱节，
 * 所以这里必须连观测器一起重置，否则恢复瞬间会算出一个巨大的残差。
 */
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

/*
 * @fn      A_Servo_SetMode
 * @brief   切换工作模式并落盘
 * @param   mode 目标模式，1~10
 * @return  1=已切换，0=模式号非法
 */
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

/*
 * @fn      A_Servo_GetPositionPwm
 * @brief   读当前位置并换算回协议脉宽
 * @param   无
 * @return  脉宽(us)；电机模式没有位置概念，统一返回中位
 */
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

uint16_t A_Servo_GetPositionPwm(void)
{
    if (Servo_IsPosition()) (void)Servo_ReadPosition();
    return Servo_PositionToPwm();
}

/*
 * @fn      A_Servo_CalibrateMid
 * @brief   把当前物理位置标定为行程中点
 * @param   无
 * @return  1=成功，0=电机模式或编码器读取失败
 *
 * 自定义模式下语义不变，只是改写的是自定义行程的零点：行程长度和方向都
 * 保持不动，整段行程平移到"当前位置落在1500us"的位置上。1500us无论正反向
 * 都映射到行程中点，所以两个方向的公式是同一条。
 */
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
    Servo_SetOffset(Servo_WrapAngle((uint32_t)raw + CDEG_RANGE - center));
    A_Config_MarkDirty();

    /* 坐标系整体平移了，所有环路都要按新坐标重来 */
    s_servo.raw_angle  = raw;
    s_servo.angle_circ = center; /* 偏移刚按center标定，两个坐标此刻相等 */
    s_servo.angle      = center;
    Servo_SetTrajRange();
    Servo_ResyncLoops(1U);
    return 1U;
}

/*
 * @fn      A_Servo_SetTravelEnd
 * @brief   把行程的一端收到当前位置，进入(或更新)自定义模式
 * @param   is_min 1=SMI，把500us端设到当前位置；0=SMX，把2500us端设到当前位置
 * @return  1=成功，0=电机模式/编码器读失败/剩下的行程太短
 *
 * ============================ 语义 ============================
 * 只动你指定的那一端，另一端留在原地，方向也不变，所以每发一次行程只会
 * 向内收窄。基准是**当前模式的行程**：在1~6里是该模式的标准行程，在11里
 * 就是上一次标出来的自定义行程，于是可以连着发几次逐步收窄。
 *
 * 例：模式1(270度正向，500us在0度、2500us在270度)，轴停在200度
 *     SMX -> 500us仍在0度，2500us移到200度，行程200度
 *     SMI -> 500us移到200度，2500us仍在270度，行程70度
 *
 * ====================== 收哪一端不能只看指令 ======================
 * "500us端"在行程坐标里是哪一头由方向决定：正向(reverse=0)时500us在行程
 * 低端，反向时在高端。所以真正要动的是低端还是高端，得把指令和方向异或
 * 起来看，见下面的 raise_low。写成 is_min 直接映射低端在反向模式下会把
 * 两条指令的作用对调。
 */
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
        offset = Servo_WrapAngle((uint32_t)(Servo_Offset() + pos + CDEG_RANGE));
        span   = (uint16_t)(s_servo.span_cdeg - pos);
    }
    else
    {
        /* 高端压到当前位置：零点不动，行程就是当前位置的坐标 */
        offset = Servo_WrapAngle((uint32_t)(Servo_Offset() + CDEG_RANGE));
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

/*
 * @fn      A_Servo_SaveStartup
 * @brief   把当前位置存为上电目标位置
 * @param   无
 * @return  1=成功，0=电机模式不支持
 */
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
 *     1 ref_pos   规划位置        厘度
 *     2 meas_pos  实测位置        厘度  未滤波，看量化和噪声就看它
 *     3 ref_vel   规划速度        厘度/秒
 *     4 obs_vel   观测速度        厘度/秒
 *     5 pwm       实际输出        PWM计数(含方向符号，已限幅)
 *     6 u_vel     速度前馈        PWM   \
 *     7 u_fb      速度环P反馈     PWM    | 这四项之和(再加u_acc)就是控制器
 *     8 u_i       速度环积分      PWM    | 的原始输出，分开看才知道纹波
 *     9 u_fric    摩擦前馈        PWM   /  是前馈算出来的还是反馈抖出来的
 *    10 sat       本拍是否饱和    0/1
 *
 * 去掉了 obs_pos：实测它和 meas_pos 差不了几个厘度，观测器在位置这一路上
 * 没有引入问题，那一格留给更有诊断价值的分项。
 *
 * 为什么要分项：整机纹波可能来自完全不同的地方——前馈跟着 ref_vel 走(那是
 * 规划的问题)、P反馈跟着速度估计噪声走(那是传感器的问题)、积分自己爬升
 * (那是静摩擦顶不动)。只看总输出这三种长得一模一样。
 */

#define SERVO_PLOT_CH  10U   /* 通道数 */
#define SERVO_PLOT_MAX 96U   /* 帧缓冲上限：10通道最坏约 70 字节 */

/*
 * @fn      Plot_SInt
 * @brief   把有符号十进制追加进帧缓冲
 * @param   buf 帧缓冲
 * @param   pos 当前写入位置
 * @param   value 数值
 * @return  写入后的位置
 */
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

/*
 * @fn      A_Servo_printf
 * @brief   按 FireWater 格式发一帧遥测，供上位机实时绘图
 * @param   无
 * @return  无
 *
 * 调度器是协作式的，本函数只会被 State 任务调用，与 1ms 控制拍不会互相抢占，
 * 所以直接读 s_servo 不需要临界区(这些字段也不被任何中断改写)。
 *
 * 队列放不下就整帧丢弃，绝不阻塞——这是调试输出，不值得为它拖慢控制拍。
 * 低波特率下要留意：本帧约 48 字节，9600 下就要 42ms，比 State 任务 30ms
 * 的周期还长，队列会一直是满的，把串口指令的回复挤掉。那种情况下把这个
 * 调用去掉，或者仿照 App_LedHeartbeat 的 LED_BLINK_DIV 加个分频。
 */


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

/*
 * @fn      Servo_MotorControl
 * @brief   电机模式的一拍：开环输出 + 定圈/定时倒计时
 * @param   无
 * @return  无
 */
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

/*
 * @fn      A_Servo_Control
 * @brief   1ms控制拍入口，由调度器最高优先级任务调用
 * @param   无
 * @return  无
 *
 * 位置模式的一拍固定是这条链，顺序不能换：
 *     读编码器 -> 观测器更新 -> 轨迹推进 -> 控制器计算 -> 下发H桥
 * 观测器要用上一拍实际施加的PWM(last_pwm)做模型预测，所以必须先更新
 * 观测器再算新的PWM；反过来会把本拍还没生效的输出当成已经作用过。
 */
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
