/* C_Friction.c 摩擦补偿，纯算法，无硬件依赖 */

#include "C_Friction.h"
#include "C_Fixed.h"

#define FRIC_ONE_Q15     32767                              /* 滑动程度满量程，存进int16不溢出 */
#define FRIC_SLEW_PWM    ((CTRL_FRIC_STATIC / 12 > 0) ? (CTRL_FRIC_STATIC / 12) : 1) /** 每拍变化上限(12ms爬满) */
#define FRIC_WIN_SHIFT   2                                  /** 实测位移窗口 = 2^2 = 4拍 */
#define FRIC_RELEASE_Q15 ((1L << 15) / 2)                   /** 动起来2ms放开到动摩擦 */
#define FRIC_REGRIP_Q15  ((1L << 15) / 20)                  /** 停下来20ms才顶回起转值，慢一点不与噪声打架 */
/* 方向在零速附近连续建立：从0到 PLANT_WCMD_MIN_CDPS 线性到满，Q15/(厘度/秒) */
#define FRIC_DIR_K       ((1 << 15) / ((PLANT_WCMD_MIN_CDPS > 0) ? PLANT_WCMD_MIN_CDPS : 1))
/* 个体差异自适应：起转标定偏小、或临时带重负载/低温润滑脂变稠时，"要动却不动"就逐拍把顶起值抬上去，
 * 最多抬 100%，50ms 抬满；挪动起来再逐拍退回。原来上限50%：摩擦翻倍(仿真)时顶不开，
 * 只能靠积分慢慢攒，顶开后积分多出的力又把轴推过头，一个30度动作拖到2.6秒。 */
#define FRIC_KICK_MAX    ((int32_t)CTRL_FRIC_STATIC * 100 / 100)
#define FRIC_KICK_STEP   ((FRIC_KICK_MAX / 50 > 0) ? (FRIC_KICK_MAX / 50) : 1)
#define FRIC_STUCK_MS    20                                 /** 连续这么多拍没挪够一个最小步长才认定粘住 */

static struct {
    int16_t  kick;       /* 顶起值在标定起转值之上的自适应抬升量，PWM计数 */
    int16_t  out;        /* 上一拍的输出，用于每拍变化限幅 */
    int16_t  slip_q15;   /* 滑动程度，0=粘住按起转值顶、32767=已滑动只给动摩擦 */
    uint8_t  ramp;       /* 1=输出还在变化限幅里爬，顶起值先别往上抬 */
    uint8_t  pos_ok;     /* 0=还没有上一拍位置，本拍只对齐不出速度 */
    uint16_t stop_cnt;   /* 要求动却没挪够一个最小可靠位移的拍数，用于判"真的粘住了" */
    int32_t  pos_prev;   /* 上一拍观测位置，厘度 */
    int32_t  disp_q5;    /* 窗口内累计位移，Q5厘度 */
    int32_t  kick_pos;   /* 上次挪动够一个最小可靠位移时的位置，厘度 */
} s_fric;

void C_Fric_Reset(void)
{
    s_fric.kick = 0; s_fric.out = 0; s_fric.slip_q15 = 0; s_fric.ramp = 0;
    s_fric.pos_ok = 0; s_fric.stop_cnt = 0; s_fric.disp_q5 = 0; s_fric.kick_pos = 0;
}

/* 实测位移速度：窗口 2^FRIC_WIN_SHIFT 拍(4拍)内的累计位移折算成速度。
 * 不能用观测器速度：顶着不动时它也会按已下发的PWM"看见"几百厘度/秒，而摩擦前馈要判的
 * 恰恰是"轴到底动没动"，只能看位置本身，否则顶起力会被自己的预测卸掉。 */
int32_t C_Fric_MeasVel(int32_t pos)
{
    if (!s_fric.pos_ok)
    {
        s_fric.pos_ok   = 1U;          /* 第一拍只对齐，不出速度 */
        s_fric.disp_q5  = 0;
        s_fric.kick_pos = pos;
    }
    else
    {
        int32_t dp = (pos - s_fric.pos_prev);
        /* dp 可能为负，负数左移是未定义行为，所以乘32；行程范围保证不溢出 */
        s_fric.disp_q5 += dp * 32 - (s_fric.disp_q5 >> FRIC_WIN_SHIFT);
    }
    s_fric.pos_prev = pos;
    return Fx_Abs32((s_fric.disp_q5 * 1000) >> (5 + FRIC_WIN_SHIFT));
}

/* 摩擦前馈，返回带符号PWM；详见 docs/ALGORITHM_NOTES.md。 */
int32_t C_Fric_Update(int32_t w_cmd, int32_t av_meas, int32_t pos, int32_t u_rest,
                      uint8_t ahead)
{
    int32_t dir_q15, slip_q15, end, u_dyn, want, out, step;

    /* 正反向共用一组摩擦值：两向差异在标定重复性以内 */
    dir_q15 = Fx_Clamp32(w_cmd * FRIC_DIR_K, 1 << 15);

    /* 滑动程度：0=粘住，1<<15=已完全滑动 */
    end = Fx_Abs32(w_cmd);
    if (end > CTRL_FRIC_MOVE_CDPS) end = CTRL_FRIC_MOVE_CDPS;
    if (end < CTRL_VEL_DB_CDPS + CTRL_VEL_NOISE_CDPS) end = CTRL_VEL_DB_CDPS + CTRL_VEL_NOISE_CDPS;

    if (av_meas <= CTRL_VEL_DB_CDPS)   slip_q15 = 0;
    else if (av_meas >= end)           slip_q15 = FRIC_ONE_Q15;
    else slip_q15 = Fx_RatioQ15((uint32_t)(av_meas - CTRL_VEL_DB_CDPS),
                                (uint32_t)(end - CTRL_VEL_DB_CDPS));

    /* 动起来就快放开(FRIC_RELEASE_Q15)，停下来才慢慢顶回去(FRIC_REGRIP_Q15)。
       对称地跟随会与速度噪声打架：噪声一往下抖就又顶一下，正是粘滑的来源；
       放开也不能一拍到位，否则一个噪声毛刺就把正要挣脱的顶起力卸掉、起步反而更晚。 */
    if (slip_q15 > s_fric.slip_q15 + FRIC_RELEASE_Q15)
        s_fric.slip_q15 = (int16_t)(s_fric.slip_q15 + FRIC_RELEASE_Q15);
    else if (slip_q15 > s_fric.slip_q15) s_fric.slip_q15 = (int16_t)slip_q15;
    else if (s_fric.slip_q15 > FRIC_REGRIP_Q15)
        s_fric.slip_q15 = (int16_t)(s_fric.slip_q15 - FRIC_REGRIP_Q15);
    else s_fric.slip_q15 = 0;
    slip_q15 = s_fric.slip_q15;

    /* 要求动却没动：逐拍抬高顶起值，直到轴真的挣脱，补偿起转标定偏小或带载变重。
       判据用"自上次挪动以来的位移"而不是速度：顶着不动时位移就是不动，速度却怎么滤都有噪声。
       还在变化限幅里往上爬时不算"顶足了也不动"，否则挣脱时白白多出一截力。 */
    if (Fx_Abs32(w_cmd) <= PLANT_WCMD_MIN_CDPS)
    {
        s_fric.kick     = 0;
        s_fric.kick_pos = pos;
        s_fric.stop_cnt = 0U;
    }
    else if (Fx_Abs32(pos - s_fric.kick_pos) >= CAL_MIN_STEP_CDEG)
    {
        s_fric.kick_pos = pos;                      /* 挪动了一个最小可靠位移：顶起值逐拍退回 */
        s_fric.stop_cnt = 0U;
        if (s_fric.kick > 0)
        {
            s_fric.kick = (int16_t)(s_fric.kick - FRIC_KICK_STEP);
            if (s_fric.kick < 0) s_fric.kick = 0;
        }
    }
    else if (!s_fric.ramp && !ahead)
    {
        if (s_fric.stop_cnt < FRIC_STUCK_MS) s_fric.stop_cnt++;   /* 先确认真的不动 */
        else if (s_fric.kick < FRIC_KICK_MAX)
            s_fric.kick = (int16_t)(s_fric.kick + FRIC_KICK_STEP);
    }

    /* 已滑动的那一份：动摩擦是持续阻力，直接加上 */
    u_dyn = (int32_t)(((int64_t)dir_q15 * CAL_FRICTION_PWM) >> 15);

    /* 粘住的那一份：把总输出顶到起转值，方向由 w_cmd 给，已经够大就不再补；轴已超前时不顶 */
    want = ahead ? 0 : (int32_t)(((int64_t)dir_q15 * (CTRL_FRIC_STATIC + s_fric.kick)) >> 15);
    out  = u_rest + u_dyn;
    if (want > 0 && out < want)      out = want - out;
    else if (want < 0 && out > want) out = want - out;
    else                             out = 0;

    /* 两份按滑动程度混合：粘住时总输出正好是起转值，滑开后只剩动摩擦 */
    out = u_dyn + (int32_t)(((int64_t)out * (FRIC_ONE_Q15 - slip_q15)) >> 15);

    /* 每拍变化限幅：挣脱/换向时前馈平滑接续，不产生冲击 */
    step = out - s_fric.out;
    s_fric.ramp = (uint8_t)(Fx_Abs32(step) > FRIC_SLEW_PWM);
    out = s_fric.out + Fx_Clamp32(step, FRIC_SLEW_PWM);
    s_fric.out = (int16_t)out;
    return out;
}
