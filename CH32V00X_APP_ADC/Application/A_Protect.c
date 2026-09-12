/*
 * A_Protect.c —— 保护与功率限制
 *
 * 整个工程唯一会"越过用户指令去关电机"的地方，所以判据全部集中在这里，不散到
 * A_Servo 里面去。本模块只调用 A_Sensor / A_Servo 的公开API，自己不碰任何寄存器。
 *
 * 一拍(10ms)干什么：
 *     每拍  读电流 -> 过流计数 -> 功率限制回路调PWM上限
 *      2拍  取运动采样 -> 堵转判据(20ms)
 *     25拍  读温度 -> 过温判据(250ms)
 *     每拍  把当前故障状态落实到舵机上(该卸力的卸力，该恢复的恢复)
 * 用一个10ms基拍分频去凑20ms和250ms，而不是再注册两个任务：调度器是协作式的，
 * 任务数越少、每个任务越短，1ms控制拍被挤掉的概率越低。三条链里重的是温度那条
 * (一次I2C事务约100us)，被放在最长的分频上。
 *
 * 四部分各自的取舍：
 *
 * 过温 —— 判据就是比大小，唯一要设计的是回差。没有回差温度会贴着阈值反复穿越，
 *   舵机变成低频通断开关。读失败时保持上一次结论，不把一次I2C抖动当成"温度正常"。
 *
 * 过流 —— 要连续 PROT_CURRENT_TRIP 拍超阈值才跳闸：起转和换向的电流尖峰是正常
 *   物理现象，单拍跳闸会把正常动作打停；30ms又远短于电机热时间常数，也远早于
 *   H桥自身的硬件斩波。恢复走固定冷却时间而不是回差——卸力之后绕组里根本没有
 *   电流，"电流回落"恒成立，用它判恢复会立刻弹回去。冷却期满后放开，故障还在就
 *   会重新跳闸，形成 30ms通/500ms断 的打嗝式重试，占空比6%，热功率不会累积。
 *
 * 堵转 —— "参考轨迹在往前走，实际位置不动"，20ms采一次位移，连续5次(100ms)才算
 *   数。100ms 是拿机械时间常数定的：真堵住时两个时间常数足够让速度衰到零，误判
 *   不了；而正常运动里最慢的那一段也没有连续100ms位移低于分辨率的情况。
 *   不用电流判堵转，是因为这类小电机的堵转电流离跳闸点还有很大余量，等不到——
 *   位置反馈才是最灵敏的堵转传感器。两者互补：过流管"电流大到会烧"，堵转管
 *   "动不了还在硬顶"，后者在小电机上先发生。
 *   **必须同时看轨迹冻结**：A_Servo_Control 的参考调节器在输出饱和且实际落后时
 *   会冻结轨迹时钟，顶死正是这条支路持续成立的工况，于是参考位移恒为0，光看位移
 *   会把最典型的堵转整个漏掉。所以采样带回冻结拍数，与位移判据取或。
 *   处理动作只有"把目标改成当前实际位置"，不卸力也不记故障位，理由见 A_Protect.h。
 *
 * 功率限制 —— 扭矩百分比先换成限流值，再由一条慢回路把PWM上限调到让实测电流落在
 *   限流值上。不直接按比例砍PWM上限，是因为电机扭矩正比于电流而不是占空比：同一
 *   占空比在电池满电和亏电时出的扭矩差一大截，开环映射标出来的"50%扭矩"会随电压
 *   漂。闭环之后扭矩只跟限流值挂钩。采样无效时保持当前上限，零扭矩始终保持最小限幅。
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

/* 限流回路的步长。满PWM堵转电流约5A，2400计数摊下来1个计数约2mA，所以"超出量/4"
 * 是半步收敛：够快，又不会一步压过头来回震荡。 */
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
    int16_t  pwm_limit;  /** 当前下发给舵机的PWM上限 */
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
    [(((250U % PROT_TICK_MS) == 0U)
   && ((PROT_STALL_PERIOD_MS % PROT_TICK_MS) == 0U)) ? 1 : -1];
/* 两个位移阈值必须拉开，否则存在"参考刚够算在走、实际刚够算没动"的含糊带 */
typedef char guard_prot_stall_thresholds_overlap
    [(PROT_STALL_REF_CDEG > PROT_STALL_ACT_CDEG) ? 1 : -1];
typedef char guard_prot_stall_trip_invalid
    [(PROT_STALL_TRIP >= 1) ? 1 : -1];
/* 冻结判据的窗口就是一个检测周期，要求的拍数不能超过周期里的控制拍数 */
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

/* 最近一次动作是不是以堵转收场，1=上次判过堵转且之后还没真正转起来。
 * 只是个诊断标志，不参与任何判据，所以不进 A_Protect_Fault 的掩码。清除条件是
 * "看到机构真的动了"而不是"过了多久"：堵转之后舵机就地保持，本来就不动，用时间
 * 清会在什么都没改善的情况下把标志抹掉。 */
uint8_t A_Protect_Stalled(void)
{
    return s_prot.stalled;
}

/* ============================== 功率限制 ============================== */

/* 限流回路：按实测电流调整PWM上限。快降慢升的不对称回路是限流折返的标准做法，
 * 保证瞬时过流被立刻按住，恢复过程平滑到看不出来。 */
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
        /* 回升目标分两种。采样有效说明确实在驱动段而电流够低，限流值还有余量，
         * 可以一路回到满量程——闭环的意义就在这里：低电压或轻载时允许用比线性
         * 估计更高的占空比去凑够设定电流。
         *
         * 采样无效说明电流**不可测**(占空比太低，或处在刹车段——两路全高时ISEN
         * 上电流相消，硬件上就看不见)。"不可测"不等于"该降额"，所以这里**保持
         * 当前上限不动**，等下一个有效样本再说。原来这里回落到开环线性估计，
         * 实机代价很贵：停着的每一拍都在把上限往下拉，于是每条新指令都要重新爬
         * 几百毫秒，而短位移一百多毫秒就跑完了——全程只拿到一半功率，可规划器
         * 是按满功率规划的，必然饱和。
         *
         * 起步过冲改由下降支路兜住：它每10ms最多降 PROT_LIMIT_DOWN_MAX 计数，
         * 比机构的机械时间常数快一个数量级，来得及。 */
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

/* 过流检测，每10ms一次：连续超阈值达到 PROT_CURRENT_TRIP 拍则卸力并进入冷却 */
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

/* 堵转检测，20ms一次。一次采样只回答一个问题：这20ms里参考走了多少、机构走了多少。
 * 采样必须每个周期都取(即使不在运动)，因为 A_Servo_SampleMotion 顺手把基准推到本拍
 * 并清冻结计数，漏一次下一次拿到的就是两个周期的累计量。
 * valid=0(卸力/暂停/电机模式/脱困/轨迹已走完)时连续计数清零——堵转的判据是"还在执行
 * 一条动不了的指令"，没有指令在跑就无从谈起。 */
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

    /* "轨迹在要求运动"有两种表现：参考自己在走，或者参考被饱和冻结按在
     * 原地。后者恰恰是顶死时的样子，漏了它就等于没做堵转保护。 */
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
    /* 只有确实转起来了才清诊断标志。"这一拍不算堵转"还不够——参考没在走的
     * 静止段也满足它，那会在机构一动没动的情况下把上次的堵转记录抹掉。 */
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

/* 把当前故障状态落实到舵机上。每拍复检扭矩状态而不是只在故障边沿动一次：故障期间
 * 上位机完全可能发来ULR或运动指令，舵机入口会拒绝故障期间恢复扭矩，这里仍逐拍落实
 * 卸力。恢复只解除"本模块发出的"那次卸力，用户自己发的ULK不该被保护顺手撤销。 */
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

    /* CLE恢复出厂或其它路径改了配置，这里跟上，不用额外的通知机制 */
    if (g_config.torque_limit != s_prot.torque_pct) Protect_ApplyTorqueSetting();

    valid = A_Current_Read(&ma);

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
