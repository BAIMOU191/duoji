/*
 * A_Protect.c —— 保护与功率限制
 *
 * 整个工程唯一会"越过用户指令去关电机"的地方，所以判据全部集中在这里，
 * 不散到 A_Servo 里面去。本模块只调用 A_Sensor / A_Servo 的公开API：
 * 传感器给业务单位(0.1摄氏度 / 毫安)，舵机给一个扭矩状态查询和几个动作接口，
 * 保护自己不碰任何寄存器，也不知道H桥长什么样。
 *
 * ============================ 一拍(10ms)干什么 ============================
 *
 *     每拍   读电流 -> 过流计数 -> 功率限制回路调PWM上限
 *     25拍   读温度 -> 过温判据(250ms)
 *     每拍   把当前故障状态落实到舵机上(该卸力的卸力，该恢复的恢复)
 *
 * 分频用一个10ms基拍去凑250ms，而不是再注册一个任务：调度器是协作式的，
 * 任务数越少、每个任务越短，1ms控制拍被挤掉的概率越低。两条链里重的是
 * 温度那条(一次I2C事务，约100us)，被放在250ms分频上。
 *
 * ============================ 三部分各自的取舍 ============================
 *
 * 过温：TMP112直接给绝对温度，判据就是比大小，唯一要设计的是回差。没有回差
 *       的话温度会贴着阈值反复穿越，舵机变成低频通断开关。读失败(-32768)时
 *       保持上一次结论，不把一次I2C抖动当成"温度正常"。
 *
 * 过流：要连续3拍(30ms)超阈值才跳闸。起转和换向瞬间的电流尖峰是正常物理现象，
 *       单拍跳闸会把正常动作打停；30ms又远短于电机热时间常数，也远早于AT8236
 *       自身5.5A的硬件斩波，所以既不误触发也不失职。中间只要有一拍回落就清零。
 *       恢复走固定冷却时间而不是回差：卸力之后绕组里根本没有电流，"电流回落"
 *       这个条件恒成立，用它判恢复会立刻弹回去。冷却期满后放开，如果故障还在
 *       就会在30ms内重新跳闸，形成 30ms通 / 500ms断 的打嗝式重试，占空比6%，
 *       热功率不会累积。
 *
 * 功率限制：扭矩百分比先换成限流值(扭矩% x 3A)，再由一条慢回路把PWM上限调到
 *       让实测电流落在限流值上。为什么不直接按比例砍PWM上限：电机扭矩正比于
 *       电流而不是占空比，同一个占空比在电池满电和亏电时出的扭矩差一大截，
 *       开环映射标出来的"50%扭矩"会随电压漂。闭环之后扭矩只跟限流值挂钩，
 *       回路要几拍才收敛；新扭矩设置从开环线性估计起步，之后由电流反馈
 *       调整上限。采样无效时保持当前上限，零扭矩设置始终保持最小限幅。
 */

#include "A_Protect.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "A_Servo.h"
#include <string.h>

#define PROT_TICK_MS          10U                   /* 任务基拍     */
#define PROT_TEMP_DIV         (250U / PROT_TICK_MS) /* 过温检测分频 */
#define PROT_COOLDOWN_TICKS   (PROT_CURRENT_COOLDOWN_MS / PROT_TICK_MS)

/* 限流回路的步长。本板满PWM堵转电流约5A，2400计数摊下来1个计数约2mA，
 * 所以"超出量/4"是半步收敛：够快，又不会一步压过头来回震荡。
 * 回升做成慢速定步长，形成快降慢升的不对称回路——这是限流折返的标准做法，
 * 保证瞬时过流被立刻按住，而恢复过程平滑到看不出来。 */
#define PROT_LIMIT_DOWN_DIV      4
#define PROT_LIMIT_DOWN_MAX    400
#define PROT_LIMIT_UP_STEP    (CFG_PWM_FULL / 50) /* 每10ms回升，满量程约500ms */
#define PROT_LIMIT_MIN           1                /* 控制器把0解释为"取消限幅" */

typedef struct {
    uint8_t  fault;      /* PROT_FAULT_* 位掩码                          */
    uint8_t  released;   /* 1=当前的卸力由本模块发出，只有它该由本模块解除 */
    uint8_t  torque_pct; /* 已生效的扭矩上限，与g_config不一致时重算      */
    uint8_t  cur_cnt;    /* 连续过流拍数                                  */
    uint8_t  temp_div;   /* 250ms分频计数                                 */
    uint16_t cooldown;   /* 过流冷却剩余拍数，0=不在冷却中                */
    uint16_t i_limit_ma; /* 扭矩百分比换算出的限流值                      */
    int16_t  pwm_limit;  /* 当前下发给舵机的PWM上限                       */
} Protect_t;

static Protect_t s_prot;

/* ---- 编译期护栏：宏被覆盖成不合理的值时直接编译失败，不留到运行时 ---- */
typedef char guard_prot_temp_hyst_invalid
    [((PROT_TEMP_HYST_C > 0) && (PROT_TEMP_HYST_C < PROT_TEMP_LIMIT_C)) ? 1 : -1];
typedef char guard_prot_current_exceeds_adc_range
    [(PROT_CURRENT_LIMIT_MA < (int32_t)CURRENT_FULL_SCALE_MA) ? 1 : -1];
typedef char guard_prot_torque_default_out_of_range
    [(PROT_TORQUE_DEFAULT <= PROT_TORQUE_MAX) ? 1 : -1];
typedef char guard_prot_trip_count_invalid
    [(PROT_CURRENT_TRIP >= 1) ? 1 : -1];
typedef char guard_prot_period_not_multiple_of_tick
    [((250U % PROT_TICK_MS) == 0U) ? 1 : -1];

/* ============================ 扭矩上限设置 ============================ */

/*
 * @fn      Protect_ApplyTorqueSetting
 * @brief   把配置里的扭矩百分比展开成限流值和开环估计，并重置限流回路
 * @param   无
 * @return  无
 *
 * 改设定后不保留旧的PWM上限：新设定对应的工作点可能离得很远，直接把回路
 * 放到开环线性估计上重新收敛，比让它一步步爬过去快得多。
 */
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

/*
 * @fn      A_Protect_SetTorque
 * @brief   设置扭矩上限百分比并落盘，立即生效
 * @param   percent 扭矩上限，0~100
 * @return  1=已设置，0=百分比越界
 */
uint8_t A_Protect_SetTorque(uint8_t percent)
{
    if (percent > PROT_TORQUE_MAX) return 0U;

    g_config.torque_limit = percent;
    A_Config_MarkDirty();
    Protect_ApplyTorqueSetting();
    return 1U;
}

uint8_t A_Protect_GetTorque(void)
{
    return s_prot.torque_pct;
}

uint8_t A_Protect_Fault(void)
{
    return s_prot.fault;
}

/* ============================== 功率限制 ============================== */

/*
 * @fn      Protect_PowerLimit
 * @brief   按实测电流调整PWM上限，把扭矩压在设定值上
 * @param   valid 本拍电流采样是否有效
 * @param   ma    实测电流(毫安)，valid=0时无意义
 * @return  无
 */
static void Protect_PowerLimit(uint8_t valid, uint16_t ma)
{
    int32_t limit = s_prot.pwm_limit; /* 本拍算出的新上限 */
    int32_t ceiling;                  /* 回升目标         */
    int32_t step;                     /* 降额步长         */

    if (s_prot.torque_pct == 0U)
    {
        /* 零扭矩设定不参与回升；0是控制器的取消限幅标记，保留最小限幅。 */
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
        /* 回升目标分两种。采样有效说明确实在驱动段而电流够低，限流值还有
         * 余量，可以一路回到满量程——闭环的意义就在这里：低电压或轻载时
         * 允许用比线性估计更高的占空比去凑够设定电流。
         *
         * 采样无效说明电流**不可测**(占空比太低，或处在刹车段——两路全高时
         * ISEN上电流相消，硬件上就看不见)。"不可测"不等于"该降额"，所以这里
         * **保持当前上限不动**，等下一个有效样本再说。
         *
         * 原来这里回落到开环估计 pwm_ff，实机代价很贵：停着的每一拍都在把
         * 上限往 pwm_ff 拉，于是**每条新指令都要重新爬 250ms**(每10ms回升
         * PROT_LIMIT_UP_STEP=48，1200->2400 要 25 步)。2026-09-09 实测每条
         * 指令头几拍的PWM恒为 1200,1344,1488,1632,...，而短位移 120ms 就跑
         * 完了——全程只拿到一半功率，可规划器是按满功率规划的，必然饱和。
         *
         * 起步过冲改由下降支路兜住：它每10ms最多降 PROT_LIMIT_DOWN_MAX=400
         * 计数，比机构的机械时间常数(53ms)快一个数量级，来得及。 */
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

/* ============================= 两条检测链 ============================= */

/*
 * @fn      Protect_OverCurrent
 * @brief   过流检测与冷却计时
 * @param   valid 本拍电流采样是否有效
 * @param   ma    实测电流(毫安)
 * @return  无
 */
static void Protect_OverCurrent(uint8_t valid, uint16_t ma)
{
    if (s_prot.cooldown != 0U)
    {
        /* 冷却期间不再判据：卸力状态下绕组无电流，判什么都是"正常" */
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

/*
 * @fn      Protect_OverTemp
 * @brief   过温检测，250ms一次
 * @param   无
 * @return  无
 */
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

/*
 * @fn      Protect_Enforce
 * @brief   把当前故障状态落实到舵机输出上
 * @param   无
 * @return  无
 *
 * 每拍复检扭矩状态而不是只在故障边沿动一次：故障期间上位机完全可能发来
 * ULR或一条运动指令。舵机入口会拒绝故障期间恢复扭矩，这里仍逐拍落实卸力。
 *
 * 恢复只解除"本模块发出的"那次卸力：用户自己发的ULK不该被保护顺手撤销。
 */
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

/*
 * @fn      A_Protect_Init
 * @brief   初始化保护状态并让保存的扭矩上限立即生效
 * @param   无
 * @return  无
 *
 * 必须排在 A_Servo_Init 之后：那里会把PWM上限重置成满量程，本函数再按
 * 配置压下去，顺序反了上电第一条指令就是满扭矩。
 */
void A_Protect_Init(void)
{
    memset(&s_prot, 0, sizeof(s_prot));
    Protect_ApplyTorqueSetting();
}

/*
 * @fn      A_Protect_Task
 * @brief   10ms任务入口，由调度器调用
 * @param   无
 * @return  无
 */
void A_Protect_Task(void)
{
    uint16_t ma = 0U; /* 实测电流，毫安       */
    uint8_t  valid;   /* 本拍电流采样是否有效 */

    /* CLE恢复出厂或其它路径改了配置，这里跟上，不用额外的通知机制 */
    if (g_config.torque_limit != s_prot.torque_pct) Protect_ApplyTorqueSetting();

    valid = A_Current_Read(&ma);

    Protect_OverCurrent(valid, ma);
    Protect_PowerLimit(valid, ma);

    if (++s_prot.temp_div >= PROT_TEMP_DIV)
    {
        s_prot.temp_div = 0U;
        Protect_OverTemp();
    }

    Protect_Enforce();
}
