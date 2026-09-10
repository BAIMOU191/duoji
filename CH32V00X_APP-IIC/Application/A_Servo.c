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

/* 控制器的PWM满量程必须和电机驱动的实际上限一致，否则限幅逻辑会失真 */
typedef char cfg_pwm_full_matches_motor[(CFG_PWM_FULL == MOTOR_PWM_MAX) ? 1 : -1];

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
    uint16_t angle_circ;        /* 中值校正后的圆周坐标[0,36000)，**不截断**；
                                 * 闭环反馈只能用它，理由见Servo_ClampToTravel */
    uint16_t raw_angle;         /* 编码器原始角度，厘度，用于定圈累计位移      */
    uint16_t span_cdeg;         /* 位置模式行程，厘度；0=电机模式              */
    uint16_t target_angle;      /* 当前目标位置，暂停后继续时按它重新规划      */
    uint8_t  reverse;           /* 1=脉宽增大对应角度减小(偶数模式)            */
    uint8_t  enc_fail;          /* 连续读编码器失败次数                        */
    uint8_t  input_source;      /* SERVO_INPUT_TX / SERVO_INPUT_PWM，上电定终身 */
    uint8_t  paused;            /* 1=已暂停，输出刹车但保留目标                */
    uint8_t  resume_valid;      /* 1=有可恢复的目标(收到过运动指令且未被停止)  */
    uint8_t  timed_position;    /* 1=位置模式带时间参数，需要按拍倒计时        */
    uint8_t  turn_sample_valid; /* 1=raw_angle是有效的定圈累计基准             */
    uint8_t  torque;            /* ServoTorque_t                               */
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
    if (delta >  CDEG_RANGE / 2) delta -= CDEG_RANGE;
    if (delta < -CDEG_RANGE / 2) delta += CDEG_RANGE;
    return delta;
}

/*
 * @fn      Servo_SetModeFields
 * @brief   按模式号展开出行程、方向等派生字段
 * @param   mode 目标模式，越界时退回270度正向
 * @return  无
 *
 * 模式编号约定：奇数=正向(脉宽增大角度增大)，偶数=反向；
 * 1/2->270度，3/4->180度，5/6->360度，7~10是电机模式(行程为0)。
 * 11是自定义模式，行程和方向不由编号推导，直接取 SMI/SMX 标定出来的配置。
 */
static void Servo_SetModeFields(ServoMode_t mode)
{
    static const uint16_t span[] = {27000U, 18000U, 35999U}; /* 270/180/360度行程 */

    if (mode < SERVO_MODE_270_CW || mode > SERVO_MODE_CUSTOM)
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
static uint16_t Servo_Offset(void)
{
    return (s_servo.mode == SERVO_MODE_CUSTOM) ? g_config.custom_offset_cdeg
                                               : g_config.position_offset_cdeg;
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
 * @brief   编码器原始角度 -> 零点校正后的圆周坐标，不做任何截断
 * @param   raw 编码器角度，厘度
 * @return  圆周坐标[0, CDEG_RANGE)
 */
static uint16_t Servo_RawToCircular(uint16_t raw)
{
    return Servo_WrapAngle((uint32_t)raw + CDEG_RANGE - Servo_Offset());
}

/*
 * @fn      Servo_ClampToTravel
 * @brief   圆周坐标 -> 行程坐标，死区里的位置投影到最近的那一端
 * @param   pos 圆周坐标[0, CDEG_RANGE)
 * @return  行程坐标[0, span]
 *
 * ============================ 这个投影只能用在两个地方 ============================
 * 位置模式下 [0, span] 是行程，剩下的 (span, 36000) 是机械死区。投影的用途是
 * 回答"这个位置最近的合法端点是哪个"，只有两个正当消费者：
 *     1) C_Traj_Hold 的目标 —— 停在死区里时应该往最近的端点回；
 *     2) 位置上报 —— 协议规定必须落在 500~2500us 之内。
 *
 * **绝对不能拿去做闭环反馈。** 一旦轴被外力推到端点之外，投影会把测量值
 * 死死钉在端点上，控制器算出的位置误差恒为0，于是判定"已到位"进入HOLD、
 * 关掉摩擦前馈、积分项还按死区规则慢慢泄放——舵机在最该顶住的位置反而
 * 彻底松手，用手一推就能一路推穿整个死区。
 *
 * 主机实测(把轴按在端点外不动，测稳态输出)：
 *     推过头     投影做反馈      圆周坐标做反馈
 *     0.2度        0 PWM            137 PWM
 *     1.0度        0 PWM           1269 PWM
 *     5.0度        0 PWM           1958 PWM
 * 所以反馈一律走 angle_circ；控制器的 PosCtrl_WrapDiff 会按最短路径算误差，
 * 端点外的回推方向天然就是对的，不需要额外判断在死区的哪一半。
 */
static uint16_t Servo_ClampToTravel(uint16_t pos)
{
    if (s_servo.span_cdeg == 0U || pos <= s_servo.span_cdeg) return pos;

    return ((uint32_t)(pos - s_servo.span_cdeg) <= (uint32_t)(CDEG_RANGE - pos))
         ? s_servo.span_cdeg  /* 离行程上端更近 */
         : 0U;                /* 离行程下端更近 */
}

/*
 * @fn      Servo_ReadPosition
 * @brief   读一次编码器，刷新原始角度、圆周坐标和行程坐标
 * @param   无
 * @return  1=成功，0=I2C失败(此时三个角度都保持上一次的值)
 */
static uint8_t Servo_ReadPosition(void)
{
    uint16_t raw = A_Encoder_Read(); /* 编码器原始角度 */

    if (raw == ENCODER_ANGLE_ERROR) return 0U;

    s_servo.raw_angle  = raw;
    s_servo.angle_circ = Servo_RawToCircular(raw);            /* 反馈用，不截断 */
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
 * angle_circ(它内部按圆周做残差折算，喂截断值会让它以为轴卡住了)。
 *
 * 什么时候要重置观测器：位置坐标发生了跳变(中位校正)，或者刚从卸力恢复
 * (卸力期间轴被人转过，观测器的模型预测已经完全脱节)。普通的暂停/停止
 * 不重置，保留观测器的速度估计，恢复时不会有一拍的估计空窗。
 */
static void Servo_ResyncLoops(uint8_t reinit_obs)
{
    if (reinit_obs) C_SpeedObs_Init(s_servo.angle_circ);
    C_Traj_Hold(s_servo.angle); /* 轨迹立即停在这里，不再产生新的参考速度 */
    C_PosCtrl_Reset();          /* 清速度环积分和到位状态机               */
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

/* ========================= 对保护模块的两个接口 ========================= */

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
    s_servo.paused       = 0U;
    s_servo.resume_valid = 0U;

    if (Servo_IsPosition())
    {
        C_Traj_Set_Range(0, s_servo.span_cdeg); /* 行程可能随模式变了 */
        Servo_ResyncLoops(1U);
    }
    else
    {
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
    uint16_t boot_pwm; /* 嗅探到的PWM脉宽，非0表示本次上电走PWM输入 */

    memset(&s_servo, 0, sizeof(s_servo));
    s_servo.out_limit = CFG_PWM_FULL; /* 保护模块随后会按配置的扭矩上限压下来 */

    boot_pwm = Servo_DetectInput();
    Servo_SetModeFields((ServoMode_t)g_config.servo_mode);
    if (!Servo_ReadPosition())
        s_servo.angle = s_servo.angle_circ = s_servo.raw_angle = 0U;

    C_SpeedObs_Init(s_servo.angle_circ); /* 观测器始终跑圆周坐标 */
    C_Traj_Init(s_servo.angle, TUNE_SMOOTH_ACC, TUNE_SMOOTH_DEC);
    if (Servo_IsPosition()) C_Traj_Set_Range(0, s_servo.span_cdeg);
    C_PosCtrl_Init();

    s_servo.torque   = SERVO_TORQUE_ON;
    s_servo.last_pwm = D_Motor_Set(0);

    /* 上电动作优先级：外部PWM指令 > 配置的上电模式 */
    if (boot_pwm != 0U)
        A_Servo_Submit(boot_pwm, 0U);
    else if (g_config.boot_mode == SERVO_BOOT_GOTO_START)
        A_Servo_Submit(g_config.startup_pwm, 0U);
    else if (g_config.boot_mode == SERVO_BOOT_RELEASE)
        A_Servo_Release(0U);
    else if (Servo_IsPosition())
        C_Traj_Hold(s_servo.angle); /* SERVO_BOOT_HOLD：原地保持 */
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
    uint16_t raw; /* 定圈模式建立累计基准时读到的原始角度 */

    if (pwm < SERVO_PWM_MIN) pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX) pwm = SERVO_PWM_MAX;

    if (s_servo.torque != SERVO_TORQUE_ON) A_Servo_RestoreTorque();
    s_servo.paused       = 0U;
    s_servo.resume_valid = 1U;

    /* ---- 位置模式：交给轨迹规划器，闭环跟随 ---- */
    if (Servo_IsPosition())
    {
        s_servo.target_angle   = Servo_PwmToAngle(pwm);
        C_Traj_Plan(s_servo.target_angle, value);
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
        || s_servo.paused || !s_servo.resume_valid) return;

    s_servo.paused = 1U;
    if (Servo_IsPosition())
    {
        (void)Servo_ReadPosition();
        Servo_ResyncLoops(0U);
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

    if (!s_servo.paused || !s_servo.resume_valid) return;
    s_servo.paused = 0U;
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
    s_servo.paused       = 0U;
    s_servo.resume_valid = 0U;
    s_servo.remaining    = 0U;
    s_servo.motor_pwm    = 0;

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
     * 得到一个上次遗留的(甚至是0的)行程。想进11只有 SMI/SMX 一条路。 */
    if (mode < SERVO_MODE_270_CW || mode > SERVO_MODE_TIMED_CCW) return 0U;

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
uint16_t A_Servo_GetPositionPwm(void)
{
    uint32_t angle; /* 行程坐标，反向模式下已翻转 */

    if (!Servo_IsPosition() || s_servo.span_cdeg == 0U) return SERVO_PWM_MID;

    (void)Servo_ReadPosition();
    angle = s_servo.angle;
    if (s_servo.reverse) angle = s_servo.span_cdeg - angle;

    return (uint16_t)(SERVO_PWM_MIN
         + (angle * (SERVO_PWM_MAX - SERVO_PWM_MIN) + s_servo.span_cdeg / 2U)
           / s_servo.span_cdeg);
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
    uint16_t raw = A_Encoder_Read(); /* 当前编码器原始角度   */

    if (!Servo_IsPosition() || raw == ENCODER_ANGLE_ERROR) return 0U;

    center = (uint16_t)(s_servo.span_cdeg / 2U);
    Servo_SetOffset(Servo_WrapAngle((uint32_t)raw + CDEG_RANGE - center));
    A_Config_MarkDirty();

    /* 坐标系整体平移了，所有环路都要按新坐标重来 */
    s_servo.raw_angle  = raw;
    s_servo.angle_circ = center; /* 偏移刚按center标定，两个坐标此刻相等 */
    s_servo.angle      = center;
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
    uint16_t raw;       /* 当前编码器原始角度               */
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
        offset = Servo_WrapAngle((uint32_t)Servo_Offset() + pos);
        span   = (uint16_t)(s_servo.span_cdeg - pos);
    }
    else
    {
        /* 高端压到当前位置：零点不动，行程就是当前位置的坐标 */
        offset = Servo_Offset();
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
    if (!Servo_IsPosition()) return 0U;

    g_config.startup_pwm = A_Servo_GetPositionPwm();
    A_Config_MarkDirty();
    return 1U;
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
    uint16_t raw;   /* 本拍编码器原始角度       */
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
            /* 连续失败到阈值才停机，单次I2C抖动不打断动作 */
            if (s_servo.enc_fail < SERVO_ENC_FAIL_MAX) s_servo.enc_fail++;
            if (s_servo.enc_fail == SERVO_ENC_FAIL_MAX)
            {
                s_servo.last_pwm = D_Motor_Set(0);
                return;
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

    /* PWM输入模式：每拍把最新脉宽当成一条新的运动指令 */
    if (s_servo.input_source == SERVO_INPUT_PWM)
    {
        pulse = D_PWM_Read();
        if (pulse != 0U) A_Servo_Submit(pulse, 0U);
    }

    if (s_servo.torque != SERVO_TORQUE_ON) return; /* 卸力中，输出已由Release设定 */

    if (s_servo.paused)
    {
        s_servo.last_pwm = D_Motor_Set(0);
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
            /* 编码器彻底失效：停机比带着错误反馈继续跑安全 */
            s_servo.last_pwm = D_Motor_Set(0);
            C_PosCtrl_Reset();
            return;
        }
    }

    /* 反馈必须用未截断的圆周坐标：轴被推到行程端点之外时，截断值会把误差
     * 抹成0，舵机反而在最该顶住的位置松手。详见 Servo_ClampToTravel。 */
    C_SpeedObs_Update(s_servo.angle_circ, s_servo.last_pwm, &s_servo.obs);

    /* 输出饱和且实际落后于参考时冻结轨迹时钟(参考调节器)，
     * 避免轨迹一路跑远、误差越积越大。判据要求误差和参考速度同号，
     * 也就是"确实是跟不上"，而不是刚换向的正常瞬态。 */
    C_Traj_Step(&s_servo.ref,
        (uint8_t)(s_servo.dbg.sat
               && (int64_t)s_servo.dbg.e_pos * s_servo.ref.vel > 0));

    pwm = C_PosCtrl_Update(&s_servo.ref, &s_servo.obs,
                           C_Traj_Is_Done(), &s_servo.dbg);
    s_servo.last_pwm = D_Motor_Set(pwm);

    if (s_servo.timed_position && s_servo.remaining != 0U) s_servo.remaining--;
}
