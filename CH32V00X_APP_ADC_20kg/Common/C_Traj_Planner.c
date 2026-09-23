/* C_Traj_Planner.c 在线轨迹规划器(速度/加速度/加加速度限幅型)，无硬件依赖 */

#include "C_Traj_Planner.h"

#define TRAJ_VEL_SHIFT   16U  /* Q16：内部速度(厘度/拍)定点小数位数 */
#define TRAJ_U_SHIFT     15U  /* Q15：段内归一化时间u及其幂次的定点小数位数 */
#define TRAJ_INV_SHIFT   30U  /* Q30：1/段长 的定点小数位数 */
#define TRAJ_SOLVE_ITERS 14U  /* 二分求峰值速度的轮数；第3步会按整数拍精确反解巡航速度，这里够近即可 */
/* 第3步"加长过渡段->重解vc"的轮数。J受限段长随速度差非线性增长，带时间的换向在低加速度下
 * 每轮只收敛约一半(随机审计里最慢的一例8轮不够)，没收敛时由兜底夹紧留下到位残差 */
#define TRAJ_FIT_ITERS   12

/*
 * 硬件可用加速度推导：维持速度v要吃掉 v/Kv 的PWM，可用加速度 a(v) = a0*(1 - v/v_ss)。
 * 恒加速度参考取速度中点的可用加速度 a(vp/2) 最优(仿真扫描)。由 D = (1+lam/2)*tau*v_ss*H(f)、
 * H(f) = f^2/(1-f/2) 反查 f = vp/v_ss，无需迭代。两张表是纯数学函数，与具体舵机无关。
 */
#define TRAJ_TBL_N  12U
static const int32_t TRAJ_F_Q15[TRAJ_TBL_N] = {   /* f = vp/v_ss，Q15 */
     1638,  3277,  6554,  9830, 13107, 16384, 19661, 22938, 26214, 29491, 31130, 32112 };
static const int32_t TRAJ_H_Q12[TRAJ_TBL_N] = {   /* H(f) = f^2/(1-f/2)，Q12 */
       11,    43,   182,   434,   819,  1365,  2107,  3088,  4369,  6032,  7041,  7713 };
static const int32_t TRAJ_G_Q15[TRAJ_TBL_N] = {   /* g(f) = 1-f/2，Q15 */
    31949, 31130, 29491, 27853, 26214, 24576, 22938, 21299, 19661, 18022, 17203, 16712 };

/* 在单调表 xs[] 上反查 x 并插值 ys[]，超出范围钳在端点 */
static int32_t Traj_TblLerp(int32_t x, const int32_t *xs, const int32_t *ys)
{
    uint8_t i;

    if (x <= xs[0]) return ys[0];
    if (x >= xs[TRAJ_TBL_N - 1U]) return ys[TRAJ_TBL_N - 1U];
    for (i = 1U; i < TRAJ_TBL_N; i++)
    {
        if (x <= xs[i])
        {
            int32_t span = xs[i] - xs[i - 1U];

            if (span <= 0) return ys[i];
            return ys[i - 1U] + (int32_t)(((int64_t)(ys[i] - ys[i - 1U])
                                         * (x - xs[i - 1U])) / span);
        }
    }
    return ys[TRAJ_TBL_N - 1U];
}

/* 本段硬件能给出的加速度上限(厘度/秒^2)，只作天花板：用户加速度更小时不起作用；
 * 短行程爬不到高速区，自动得到更大的上限 */
static int32_t Traj_AutoAmax(int32_t D, int32_t vmax, int32_t v0_cdps)
{
    int64_t denom;
    int32_t h_q12, f_q15, g_q15, f_cap, f_now;

    if (PLANT_VSS_CDPS <= 0 || PLANT_TAU_MS == 0U
        || PLANT_A0_CDPSS <= 0)
        return TRAJ_AMAX_MAX_CDPSS;      /* 未标定：不另设硬件天花板，只受用户上限约束 */

    /* H = D*1000/(tau_ms*v_ss)，Q12 */
    denom = (int64_t)PLANT_TAU_MS * PLANT_VSS_CDPS;
    if (denom <= 0) return TRAJ_AMAX_MAX_CDPSS;
    h_q12 = (int32_t)(((int64_t)D * 1000 * 4096) / denom);

    f_q15 = Traj_TblLerp(h_q12, TRAJ_H_Q12, TRAJ_F_Q15);

    /* 峰值速度还受本段速度上限约束，取两者较小的那个来查 g */
    f_cap = (int32_t)(((int64_t)vmax << 15) / PLANT_VSS_CDPS);
    if (f_q15 > f_cap) f_q15 = f_cap;

    /* 起点速度也算进工作点：运动中改目标时反电势已吃掉大部分PWM，只按行程查会高估加速度 */
    f_now = (int32_t)(((int64_t)v0_cdps << 15) / PLANT_VSS_CDPS);
    if (f_now > 32767) f_now = 32767;
    if (f_q15 < f_now) f_q15 = f_now;

    g_q15 = Traj_TblLerp(f_q15, TRAJ_F_Q15, TRAJ_G_Q15);

    return (int32_t)(((int64_t)PLANT_A0_CDPSS * g_q15) >> 15);
}

/*
 * 一种过渡段(加速/减速/换向)的限幅换算结果，每次 Plan 按本段加速度上限 a 和加加速度上限 J 算一次。
 * 跨越速度差 dv 的最短过渡：dv >= a^2/J 时加速度为梯形，时长 dv/a + a/J；否则为三角形，
 * 时长 2*sqrt(dv/J)，峰值加速度 sqrt(dv*J) < a。两式在 dv = a^2/J 处相接。
 */
typedef struct {
    int32_t kl_q32;   /* 1/a：每单位速度差(Q16厘度/拍)的恒加速时长，Q32拍 */
    int32_t tj_q16;   /* a/J：jerk斜坡时长，Q16拍 */
    int32_t dvj_qt;   /* a^2/J：够得着 a 的最小速度差，Q16厘度/拍 */
    int32_t cj_q16;   /* 三角加速度段：时长^2(Q8拍^2) = dv*cj >> 16 */
} TrajLim_t;

/* 规划器内部状态，单实例 */
static struct {
    /* 指令状态：重规划时作为新段起点，保证位置/速度连续 */
    int32_t pos;         /* 当前参考位置，厘度 */
    int32_t vel_qt;      /* 当前参考速度，Q16 厘度/拍 */

    /* 用户轨迹上限，下一次 Plan 起生效 */
    int32_t vmax_cdps;   /* 最大速度，厘度/秒 */
    int32_t amax_cdpss;  /* 最大加速度，厘度/秒^2 */
    int32_t dmax_cdpss;  /* 最大减速度，厘度/秒^2 */
    int32_t jmax_cdpss3; /* 最大加加速度，厘度/秒^3 */
    uint16_t move_min_ms;/* 小位移最短运动时间，0=关闭 */

    /* 本段计划(全部在C_Traj_Plan中一次算好) */
    int32_t p0;          /* 段起点位置(未回绕)，厘度 */
    int32_t v0_qt;       /* 段起点速度，Q16 厘度/拍 */
    int32_t vc_qt;       /* 巡航速度，Q16 厘度/拍(情形B下为负) */
    int32_t p_a;         /* 加速段末位置(未回绕)，厘度 */
    int32_t p_c;         /* 匀速段末位置(未回绕)，厘度 */
    int32_t p_end;       /* 段终点位置(未回绕)，厘度 */
    int32_t inv_t1_q30;  /* 1/t1 的Q30定点 */
    int32_t inv_t3_q30;  /* 1/t3 的Q30定点 */
    int32_t posk1;       /* (dv1*t1)>>15，加速段位置整形系数 */
    int32_t posk3;       /* (dv3*t3)>>15，减速段位置整形系数 */
    int32_t acck1;       /* 加速段 dv1/t1 换算成厘度/秒^2，配归一化加速度用 */
    int32_t acck3;       /* 减速段同上 */
    int32_t dv1_qt;      /* 加速段速度增量 vc-v0 */
    int32_t dv3_qt;      /* 减速段速度增量 0-vc  */
    /* 过渡段形状：r 为jerk斜坡占段长比例(Q15，0.5=三角加速度)，peak=1/(1-r)，inv_r=1/r */
    int32_t seg1_r_q15;
    int32_t seg3_r_q15;
    int32_t seg1_peak_q15;
    int32_t seg3_peak_q15;
    int32_t seg1_inv_r_q15;
    int32_t seg3_inv_r_q15;

    int32_t t1;          /* 加速段时长(拍) */
    int32_t tc;          /* 匀速段时长(拍) */
    int32_t t3;          /* 减速段时长(拍) */
    int32_t T;           /* 本段总时长(拍) */
    int32_t t;           /* 段内已走拍数 */

    /* 可用行程(厘度)。范围以外即死区，参考位置永不进入，轨迹也永不穿过。 */
    int32_t lo;
    int32_t hi;
} s_tj;

/* 把位置夹进可用行程[lo,hi]；位移一律直线计算，不走最短路径，不穿越死区 */
static int32_t Traj_Clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 32位整数平方根向上取整(逐位试探，无乘除)；向上取整使加速度只会偏小，安全侧 */
static uint32_t Traj_Sqrt_Ceil(uint32_t x)
{
    uint32_t rem  = 0;
    uint32_t root = 0;
    uint32_t v    = x;
    uint32_t i    = 0;

    /* 高位成对的0不影响结果，先跳过，小参数时少走一半以上的轮次 */
    while (i < 16U && (v >> 30) == 0U) { v <<= 2; i++; }
    for (; i < 16U; i++)
    {
        root <<= 1;
        rem    = (rem << 2) | (v >> 30);
        v    <<= 2;
        if (root < rem)
        {
            root++;
            rem -= root;
            root++;
        }
    }
    /* 循环结束时 root>>1 = floor(sqrt(x))，rem 非零即非完全平方 */
    return (root >> 1) + ((rem != 0U) ? 1U : 0U);
}

/* radix-16 除法要求 16*den 不溢出32位 */
#define TRAJ_DIV_DEN_MAX  0x0FFFFFFFu

/*
 * 64位被除数/32位除数，要求商放得下32位，越界返回0xFFFFFFFF。
 * 目标核无DIV指令，__udivdi3 要800~1500条指令且耗时抖动；这里用 radix-16 移位相减，
 * 固定8轮约120条，结果已与编译器的 / 逐一比对。
 */
static uint32_t Traj_Div_U64(uint64_t num, uint32_t den)
{
    uint32_t r  = (uint32_t)(num >> 32);
    uint32_t lo = (uint32_t)num;
    uint32_t q  = 0;
    uint32_t d2, d4, d8;
    uint32_t i;

    /* 商放不下32位，或除数超出 radix-16 的适用范围(见上)，纯防御 */
    if (den == 0U || den > TRAJ_DIV_DEN_MAX || r >= den) return 0xFFFFFFFFu;

    d2 = den << 1;
    d4 = den << 2;
    d8 = den << 3;

    for (i = 0; i < 8U; i++)
    {
        r    = (r << 4) | (lo >> 28);   /* 一次吃进4位被除数 */
        lo <<= 4;
        q  <<= 4;
        /* 从高到低试减 8/4/2/1 倍除数，求这一位的4bit商 */
        if (r >= d8)  { r -= d8;  q |= 8U; }
        if (r >= d4)  { r -= d4;  q |= 4U; }
        if (r >= d2)  { r -= d2;  q |= 2U; }
        if (r >= den) { r -= den; q |= 1U; }
    }
    return q;
}

/* 带符号版本，向零取整 */
static int32_t Traj_Div_S64(int64_t num, int32_t den)
{
    int32_t s = 1;
    uint32_t q;

    if (num < 0) { num = -num; s = -s; }
    if (den < 0) { den = -den; s = -s; }
    q = Traj_Div_U64((uint64_t)num, (uint32_t)den);
    /* 行程放开到整圈后商可能超 int32，饱和后由调用方的 |vc|>vmax 分支顶开总时间 */
    if (q > 0x7FFFFFFFu) q = 0x7FFFFFFFu;
    return (s > 0) ? (int32_t)q : -(int32_t)q;
}

/* 无符号商饱和到 int32，除法越界(0xFFFFFFFF)同样饱和 */
static int32_t Traj_Sat32(uint32_t q)
{
    return (q > 0x7FFFFFFFu) ? 0x7FFFFFFF : (int32_t)q;
}

/* 按加速度上限 a 与加加速度上限 j 生成过渡段限幅；a、j 已由调用方保证为正 */
static void Traj_Lim_Make(TrajLim_t *l, int32_t a, int32_t j)
{
    int32_t tj;

    if (a < TRAJ_AMAX_MIN_CDPSS) a = TRAJ_AMAX_MIN_CDPSS;  /* 保证 1/a 的Q32放得下 */
    if (j < TRAJ_JMAX_MIN_CDPSS3) j = TRAJ_JMAX_MIN_CDPSS3;
    if (j > TRAJ_JMAX_MAX_CDPSS3) j = TRAJ_JMAX_MAX_CDPSS3;

    /* 恒加速度段 t=|dv|/a 的系数，Q32拍/(Q16厘度/拍)；用定长除法，__divdi3 太慢 */
    l->kl_q32 = Traj_Sat32(Traj_Div_U64((uint64_t)CFG_TICK_HZ * CFG_TICK_HZ * 65536, (uint32_t)a));
    /* a/J 秒 -> Q16拍 = a*1000*65536/J；限到 2^30 拍Q16(约16秒)，更长的斜坡没有意义 */
    tj = Traj_Sat32(Traj_Div_U64((uint64_t)a * (65536ULL * CFG_TICK_HZ), (uint32_t)j));
    if (tj > 0x40000000) tj = 0x40000000;
    l->tj_q16 = tj;
    /* a^2/J = (a/J)/(1/a)，两者都已是拍单位 */
    l->dvj_qt = Traj_Sat32(Traj_Div_U64((uint64_t)l->tj_q16 << 16, (uint32_t)l->kl_q32));
    /* 时长^2 = 4*dv/J：dv(Q16厘度/拍) 换成厘度/秒要 *1000/65536，秒^2换拍^2要 *1e6，再取Q8 */
    l->cj_q16 = Traj_Sat32(Traj_Div_U64(1024000000000ULL, (uint32_t)j));
}

/* 跨越速度差 dv 的最短过渡时长，Q16拍；dv<=0 为0 */
static int32_t Traj_Seg_Q16(const TrajLim_t *l, int32_t dv)
{
    int64_t t;

    if (dv <= 0) return 0;
    if (dv >= l->dvj_qt)
    {
        t = (((int64_t)l->kl_q32 * dv) >> 16) + l->tj_q16;          /* 梯形加速度 */
    }
    else
    {
        uint64_t x = ((uint64_t)(uint32_t)dv * (uint32_t)l->cj_q16) >> 16; /* 三角加速度，Q8拍^2 */
        if (x > 0xFFFFFFFFu) x = 0xFFFFFFFFu;
        t = (int64_t)Traj_Sqrt_Ceil((uint32_t)x) << 12;               /* Q4拍 -> Q16拍 */
    }
    return (t > 0x7FFFFFFF) ? 0x7FFFFFFF : (int32_t)t;
}

/* 同上，向上取整到整拍且至少1拍；取整只会拉长，峰值加速度和加加速度只低不高 */
static int32_t Traj_Seg_Ticks(const TrajLim_t *l, int32_t dv)
{
    int32_t t = (int32_t)(((int64_t)Traj_Seg_Q16(l, dv) + 0xFFFF) >> 16);
    return (t < 1) ? 1 : t;
}

/* 过渡段形状 r(Q15)：三角加速度 0.5，梯形 (a/J)/时长。段被拉长时沿用该形状，加速度与加加速度同比下降 */
static int32_t Traj_Seg_R(const TrajLim_t *l, int32_t dv)
{
    int32_t t, tj, r;

    if (dv < l->dvj_qt) return 16384;
    t  = Traj_Seg_Q16(l, dv);
    tj = l->tj_q16;
    while ((uint32_t)t > TRAJ_DIV_DEN_MAX) { t >>= 1; tj >>= 1; } /* 超长段降精度，比例不变 */
    if (t <= 0) return 16384;
    r = Traj_Sat32(Traj_Div_U64((uint64_t)(uint32_t)tj << 15, (uint32_t)t));
    if (r < 1) r = 1;
    if (r > 16384) r = 16384;
    return r;
}

/* 形状 r 的两个倒数：peak=1/(1-r)，inv_r=1/r，均Q15 */
static void Traj_Seg_Shape(int32_t r, int32_t *peak, int32_t *inv_r)
{
    *peak  = (int32_t)Traj_Div_U64((uint64_t)1 << 30, (uint32_t)(32768 - r));
    *inv_r = (int32_t)Traj_Div_U64((uint64_t)1 << 30, (uint32_t)r);
}

/* Traj_Seg_Ticks 的反函数：t 拍内能完成的最大速度差(Q16厘度/拍)，用于把巡航速度夹回J允许的范围 */
static int32_t Traj_Seg_DvMax(const TrajLim_t *l, int32_t t)
{
    if (t <= 0) return 0;
    if (((int64_t)t << 16) >= 2 * (int64_t)l->tj_q16)
    {
        /* 梯形：dv/a + a/J <= t  =>  dv <= (t - a/J)*a */
        return Traj_Sat32(Traj_Div_U64((uint64_t)(((int64_t)t << 16) - l->tj_q16) << 16,
                                       (uint32_t)l->kl_q32));
    }
    /* 三角形：2*sqrt(dv/J) <= t  =>  dv*cj>>16 <= (16t)^2 */
    return Traj_Sat32(Traj_Div_U64((uint64_t)t * (uint64_t)t << 24, (uint32_t)l->cj_q16));
}

/* 两段过渡 u0->vp->0、中间不留匀速段时按整数拍的位移x2，单位 厘度*2^16 */
static int64_t Traj_Ramp2(const TrajLim_t *l1, const TrajLim_t *l3, int32_t u0, int32_t vp)
{
    int32_t d1 = (vp > u0) ? (vp - u0) : (u0 - vp);
    int32_t d3 = (vp < 0) ? -vp : vp;

    /* 对称过渡段的平均速度是首末速度的中点；按取整后的拍数算，第3步反解时才不会差出一拍的位移 */
    return (int64_t)(u0 + vp) * Traj_Seg_Ticks(l1, d1)
         + (int64_t)vp * Traj_Seg_Ticks(l3, d3);
}

/*
 * 在[lo,hi]上求使两段过渡位移恰为 D 的峰值速度 vp。vp>=max(u0,0) 时位移随vp递增(正常情形)，
 * vp<=0 时同样随vp递增(情形B：冲过目标再倒回)，统一取位移不超过 D 的最大 vp。
 */
static int32_t Traj_Solve_Vp(const TrajLim_t *l1, const TrajLim_t *l3,
                             int32_t u0, int32_t D, int32_t lo, int32_t hi)
{
    int64_t target = (int64_t)D << 17;
    uint32_t i;

    if (hi <= lo) return hi;
    if (Traj_Ramp2(l1, l3, u0, hi) <= target) return hi;
    for (i = 0; i < TRAJ_SOLVE_ITERS && hi - lo > 1; i++)
    {
        int32_t mid = lo + ((hi - lo) >> 1);

        if (Traj_Ramp2(l1, l3, u0, mid) <= target) lo = mid;
        else                                       hi = mid;
    }
    return lo;
}

/*
 * 带时间指令中途换向的首段限幅(越小越柔)：按时间缩放同比放缓(a/lambda^2)，避免加速度前馈1拍跳变上千PWM；
 * 但参考反向滑出不超过 D/TUNE_REV_SLIDE_DIV，高速反向时自动回到硬件极限。情形B不受影响。
 */
static void Traj_ReverseLim(TrajLim_t *l, int32_t a_hw, uint32_t lam_q8, int32_t au0, int32_t D)
{
    int64_t a_soft, v;

    if (TUNE_REV_SLIDE_DIV <= 0 || au0 <= 0 || D <= 0) return;
    a_soft = ((int64_t)a_hw << 16) / ((int64_t)lam_q8 * lam_q8);
    v = ((int64_t)au0 * CFG_TICK_HZ) >> TRAJ_VEL_SHIFT;            /* 厘度/秒 */
    {
        /* 刹车距离 v^2/(2a) <= D/div  =>  a >= v^2*div/(2D) */
        int64_t a_min = (v * v * TUNE_REV_SLIDE_DIV) / (2 * (int64_t)D);
        if (a_soft < a_min) a_soft = a_min;
    }
    if (a_soft >= a_hw) return;
    Traj_Lim_Make(l, (int32_t)a_soft, s_tj.jmax_cdpss3);
}

/* 静止起步的最短运动时间(拍)，随起点参考速度线性淡出：已在运动中的改目标不被拉长 */

/* 设置轨迹上限(超出可设范围自动夹紧)，下一次 Plan 起生效；dmax<=0 表示减速跟随加速 */
void C_Traj_Set_Limits(int32_t vmax_cdps, int32_t amax_cdpss, int32_t dmax_cdpss, int32_t jmax_cdpss3)
{
    if (vmax_cdps < 1) vmax_cdps = 1;
    if (vmax_cdps > TRAJ_VMAX_HW_CDPS) vmax_cdps = TRAJ_VMAX_HW_CDPS;
    if (amax_cdpss < TRAJ_AMAX_MIN_CDPSS) amax_cdpss = TRAJ_AMAX_MIN_CDPSS;
    if (amax_cdpss > TRAJ_AMAX_MAX_CDPSS) amax_cdpss = TRAJ_AMAX_MAX_CDPSS;
    if (dmax_cdpss <= 0) dmax_cdpss = amax_cdpss;
    if (dmax_cdpss < TRAJ_AMAX_MIN_CDPSS) dmax_cdpss = TRAJ_AMAX_MIN_CDPSS;
    if (dmax_cdpss > TRAJ_AMAX_MAX_CDPSS) dmax_cdpss = TRAJ_AMAX_MAX_CDPSS;
    if (jmax_cdpss3 < TRAJ_JMAX_MIN_CDPSS3) jmax_cdpss3 = TRAJ_JMAX_MIN_CDPSS3;
    if (jmax_cdpss3 > TRAJ_JMAX_MAX_CDPSS3) jmax_cdpss3 = TRAJ_JMAX_MAX_CDPSS3;

    s_tj.vmax_cdps   = vmax_cdps;
    s_tj.amax_cdpss  = amax_cdpss;
    s_tj.dmax_cdpss  = dmax_cdpss;
    s_tj.jmax_cdpss3 = jmax_cdpss3;
}

void C_Traj_Set_MoveMinMs(uint16_t move_min_ms)
{
    s_tj.move_min_ms = (move_min_ms <= 200U) ? move_min_ms : 200U;
}

/* 初始化并停在 init_pos，轨迹上限取调参面板 */
void C_Traj_Init(int32_t init_pos)
{
    C_Traj_Set_Limits(TRAJ_VMAX_CDPS, TRAJ_AMAX_CDPSS, TRAJ_DMAX_CDPSS, TRAJ_JMAX_CDPSS3);
    C_Traj_Set_MoveMinMs(TRAJ_MOVE_MIN_MS);

    /* 默认放开整圈，A_Servo 随后按模式收紧 */
    s_tj.lo     = 0;
    s_tj.hi     = CDEG_RANGE;
    s_tj.pos    = Traj_Clamp(init_pos, s_tj.lo, s_tj.hi);
    s_tj.vel_qt = 0;

    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = 0;
    s_tj.vc_qt = 0;
    s_tj.p_a   = s_tj.pos;
    s_tj.p_c   = s_tj.pos;
    s_tj.p_end = s_tj.pos;
    s_tj.inv_t1_q30 = 0;
    s_tj.inv_t3_q30 = 0;
    s_tj.posk1  = 0;
    s_tj.posk3  = 0;
    s_tj.dv1_qt = 0;
    s_tj.dv3_qt = 0;
    s_tj.seg1_r_q15 = s_tj.seg3_r_q15 = 16384;
    s_tj.seg1_peak_q15 = s_tj.seg3_peak_q15 = 65536;
    s_tj.seg1_inv_r_q15 = s_tj.seg3_inv_r_q15 = 65536;

    s_tj.t1 = 0;
    s_tj.tc = 0;
    s_tj.t3 = 0;
    s_tj.T  = 0;
    s_tj.t  = 0;   /* t >= T 即视为已走完 */
}

/* 设置可用行程并停住参考(换模式时不能带着旧速度冲进死区)；hi<lo 时保持原设置 */
void C_Traj_Set_Range(int32_t lo, int32_t hi)
{
    if (hi < lo) return;

    s_tj.lo = lo;
    s_tj.hi = hi;

    s_tj.pos    = Traj_Clamp(s_tj.pos, lo, hi);
    s_tj.vel_qt = 0;
    s_tj.p0     = s_tj.pos;
    s_tj.p_a    = s_tj.pos;
    s_tj.p_c    = s_tj.pos;
    s_tj.p_end  = s_tj.pos;
    s_tj.vc_qt  = 0;
    s_tj.t1 = 1; s_tj.tc = 0; s_tj.t3 = 1;
    s_tj.T  = 0; s_tj.t  = 0;   /* T=0 => 立即视为走完，停在当前位置 */
}

/* 当前参考位置，即下一次Plan的段起点；刚Hold完还没Step时与调用方手里的快照不同 */
int32_t C_Traj_Get_Pos(void)
{
    return s_tj.pos;
}

void C_Traj_Set_Range_Open(int32_t lo, int32_t hi)
{
    if (hi < lo || s_tj.pos < lo || s_tj.pos > hi)
    {
        C_Traj_Set_Range(lo, hi);
        return;
    }
    s_tj.lo = lo;
    s_tj.hi = hi;
}

void C_Traj_Hold(int32_t pos)
{
    s_tj.pos = Traj_Clamp(pos, s_tj.lo, s_tj.hi);
    s_tj.vel_qt = 0;
    s_tj.p0 = s_tj.p_a = s_tj.p_c = s_tj.p_end = s_tj.pos;
    s_tj.v0_qt = s_tj.vc_qt = s_tj.dv1_qt = s_tj.dv3_qt = 0;
    s_tj.t1 = s_tj.t3 = 1;
    s_tj.tc = s_tj.T = s_tj.t = 0;
}

/* 把段的 dv/t_seg 换算成厘度/秒^2：再乘归一化加速度 A(u) 即为本拍加速度 */
static int32_t Traj_Acc_Coef(int32_t dv_qt, int32_t inv_q30)
{
    int64_t product = (int64_t)dv_qt * inv_q30;
    int64_t per_tick2;
    int64_t scaled;

    /* 显式向零截断，避免负数右移把减速度多算，正反向都只低不高 */
    per_tick2 = (product >= 0)
              ? (product >> TRAJ_INV_SHIFT)
              : -((-product) >> TRAJ_INV_SHIFT);
    scaled = per_tick2 * CFG_TICK_HZ * CFG_TICK_HZ;
    return (int32_t)((scaled >= 0)
                   ? (scaled >> TRAJ_VEL_SHIFT)
                   : -((-scaled) >> TRAJ_VEL_SHIFT));
}

/*
 * 以当前参考状态为起点规划到 target_pos，time_ms=0 为最快。
 *   1) 按速度/加速度/加加速度上限解最速轨迹得 t1/tc/t3 与 T_fast；
 *   2) 请求时间(或静止起步最短运动时间)更长时按 lambda=T_req/T_fast 拉伸；
 *   3) 由位移方程反解巡航速度 vc 保证准时到位，并复核速度/过渡段可达，最多4轮。
 * 全部除法与开方集中在这里，每拍 Step 无除法。
 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms)
{
    int32_t dp, sgn, D, u0, au0;
    int32_t t1, tc, t3, T, T_fast, T_req;
    int32_t vp, vc, vmax_qt;
    int32_t n1, n3, i;
    int32_t amax_r;
    int64_t numv;
    uint32_t lam_q8;
    uint8_t  case_b, stretched, first_brake;
    TrajLim_t lim_acc, lim_rev, lim3, lim1;

    /* 新段起点取规划器自身状态，位置/速度连续 */
    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = s_tj.vel_qt;

    /* 目标夹进可用行程，位移直接相减，不穿过死区 */
    target_pos = Traj_Clamp(target_pos, s_tj.lo, s_tj.hi);
    dp  = target_pos - s_tj.p0;
    sgn = (dp >= 0) ? 1 : -1;
    D   = dp * sgn;                 /* 运动方向上的行程，>=0 */
    u0  = s_tj.v0_qt * sgn;         /* 起点速度沿运动方向的分量，反向时为负 */
    au0 = (u0 < 0) ? -u0 : u0;

    if (D == 0 && u0 == 0)
    {
        C_Traj_Hold(s_tj.p0);
        return; /* 已静止且同目标，不再做限幅换算 */
    }

    /* 本段限幅：用户上限与硬件天花板取小；只在收到指令时算一次 */
    {
        int32_t vmax_use = s_tj.vmax_cdps;
        int32_t amax_a, amax_d, v0_cdps, a_ceil;

        v0_cdps = (int32_t)(((int64_t)au0 * CFG_TICK_HZ) >> TRAJ_VEL_SHIFT);

        /* 小位移整形，两级上限，大位移时被 TUNE_VMAX/TUNE_AMAX 夹住自动失效：
         *   1) 巡航速度 <= 行程/TRAJ_MOVE_MIN_MS：位移越小走得越慢，但曲线形状不变；
         *   2) 加速度 <= 巡航速度/TRAJ_ACCEL_MIN_MS：加速段至少持续这么久，起步不会顶一下。
         * 不能改成"把整条曲线按时间等比拉长"：那样加减速段一起被拉长成S形，同样时长下峰值加速度
         * 反而高一倍，而且参考会长时间待在轴爬不动的低速区(本机挣脱静摩擦必然窜到6~9度/秒)，
         * 于是每个小位移都变成"顶住-窜一下-再顶"，手感是明显的冲击。限速则是"快速起步-匀速-
         * 柔和停"，轴一次滑到位。 */
        if (s_tj.move_min_ms > 0U)
        {
            int32_t v_cap = (int32_t)(((int64_t)D * 1000) / s_tj.move_min_ms);
            if (v_cap < 1) v_cap = 1;
            if (vmax_use > v_cap) vmax_use = v_cap;
        }
        /* 运动中把目标改到眼前时仍须保留消除已有动量的能力，否则会生成极慢的错误轨迹 */
        if (vmax_use < v0_cdps) vmax_use = v0_cdps;
        if (vmax_use < 1) vmax_use = 1;
        vmax_qt = (int32_t)(((int64_t)vmax_use << TRAJ_VEL_SHIFT) / CFG_TICK_HZ);
        if (vmax_qt < au0) vmax_qt = au0;   /* 上面的厘度/秒截断不能把起点速度判成超速 */

        a_ceil = (int32_t)(((int64_t)vmax_use * 1000) / TRAJ_ACCEL_MIN_MS);
        if (a_ceil < 1) a_ceil = 1;

        amax_a = Traj_AutoAmax(D, vmax_use, v0_cdps);
        if (amax_a > s_tj.amax_cdpss) amax_a = s_tj.amax_cdpss;
        if (amax_a > a_ceil) amax_a = a_ceil;
        if (amax_a < 1) amax_a = 1;
        amax_d = s_tj.dmax_cdpss;
        if (amax_d > TRAJ_DEC_HW_CDPSS) amax_d = TRAJ_DEC_HW_CDPSS;   /* 制动侧硬件天花板 */
        /* 减速段同样套 a_ceil：略增过冲，但到位瞬间的PWM跳变小得多，治机械震动 */
        if (amax_d > a_ceil) amax_d = a_ceil;
        if (amax_d < 1) amax_d = 1;
        /* 首段需刹车或换向时取两侧较弱的加速度 */
        amax_r = (amax_a < amax_d) ? amax_a : amax_d;

        Traj_Lim_Make(&lim_acc, amax_a, s_tj.jmax_cdpss3);
        Traj_Lim_Make(&lim3,    amax_d, s_tj.jmax_cdpss3);
        lim_rev = (amax_r == amax_d) ? lim3 : lim_acc;   /* amax_r 就是两者中较小的那个 */
    }

    T_req = ((int32_t)time_ms * CFG_TICK_HZ) / 1000;

    /* 情形B：正向运动且直接刹车的位移已超过剩余行程 -> 必然过冲；位移x2与 D<<17 同单位比较 */
    case_b = 0;
    if (u0 > 0)
    {
        if (((int64_t)D << 17) < (int64_t)u0 * Traj_Seg_Ticks(&lim3, u0)) case_b = 1;
    }
    /* 首段用哪套限幅：正向加速用加速上限；情形B和换向要先把已有速度刹掉，取两侧较弱的；
     * 带时间指令降到更低的巡航速度时首段其实是在减速，解出 vc<u0 后在第3步改用减速上限 */
    lim1 = (case_b || u0 < 0) ? lim_rev : lim_acc;
    first_brake = 0U;

    /* 第1步：最速轨迹 */
    if (case_b)
    {
        /* 冲过目标再倒回来：vp<0，且 |vp|<=|u0|<=vmax，永远不需要匀速段 */
        vp = Traj_Solve_Vp(&lim1, &lim3, u0, D, -vmax_qt, 0);
        t1 = Traj_Seg_Ticks(&lim1, u0 - vp);
        t3 = Traj_Seg_Ticks(&lim3, -vp);
        tc = 0;
    }
    else
    {
        /* 先按 vp=vmax 试梯形：两段过渡位移放得下就是梯形，否则是三角形 */
        int32_t d_ramp;
        t1 = Traj_Seg_Ticks(&lim1, vmax_qt - u0);
        t3 = Traj_Seg_Ticks(&lim3, vmax_qt);
        d_ramp = (int32_t)((((int64_t)(u0 + vmax_qt) * t1)
                          + ((int64_t)vmax_qt * t3)) >> (TRAJ_VEL_SHIFT + 1));

        if (d_ramp <= D)
        {
            /* 梯形：匀速段补足剩余行程，分子落在uint32内 */
            uint32_t rest = ((uint32_t)(D - d_ramp) << TRAJ_VEL_SHIFT);
            tc = (int32_t)((rest + (uint32_t)vmax_qt - 1U) / (uint32_t)vmax_qt);
            vp = vmax_qt;
        }
        else
        {
            /* 三角形：过渡段时长不再与速度差成正比，没有闭式解，二分求 vp */
            vp = Traj_Solve_Vp(&lim1, &lim3, u0, D, (u0 > 0) ? u0 : 0, vmax_qt);
            t1 = Traj_Seg_Ticks(&lim1, (vp > u0) ? (vp - u0) : (u0 - vp));
            t3 = Traj_Seg_Ticks(&lim3, vp);
            tc = 0;
        }
    }

    T_fast = t1 + tc + t3;

    /* 第2步：请求时间更长 -> 三段同乘 lambda，速度/lambda、加速度/lambda^2、加加速度/lambda^3。
     * 只有带时间指令会走到这里；小位移由上面的两级上限整形，不拉长曲线。
     * 首段在抵消已有速度时(情形B或u0<0)不按lambda拉长(高速反向会冲出很远)，取抵消u0的时长为种子、
     * 换向时由 Traj_ReverseLim 适度放缓，再由第3步长到够用，余下时间给匀速段。 */
    T = T_fast;
    stretched = (uint8_t)(T_req > T_fast);
    if (stretched)
    {
        lam_q8 = ((uint32_t)T_req << 8) / (uint32_t)T_fast;
        if (!case_b && u0 >= 0) t1 = (int32_t)(((uint32_t)t1 * lam_q8) >> 8);
        else
        {
            Traj_ReverseLim(&lim1, amax_r, lam_q8, au0, D);
            t1 = Traj_Seg_Ticks(&lim1, au0);
        }
        t3 = (int32_t)(((uint32_t)t3 * lam_q8) >> 8);
        T  = T_req;
        if (t1 < 1) t1 = 1;
        if (t1 + t3 > T) t3 = T - t1;   /* 加速段不缩放时减速段可能挤不下 */
        if (t3 < 1) t3 = 1;
    }

    /* 第3步：由 D = (u0+vc)*t1/2 + vc*tc + vc*t3/2 反解 vc = (2D - u0*t1)/(2T - t1 - t3)，
     * 使轨迹精确在T拍到位；再复核 |vc|<=vmax 与过渡段可达，不满足就加长后重解。 */
    vc = 0;
    for (i = 0; i < TRAJ_FIT_ITERS; i++)
    {
        int32_t den;
        if (T < t1 + t3) T = t1 + t3;
        den  = 2 * T - t1 - t3;
        numv = ((int64_t)D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)u0 * t1;
        vc   = Traj_Div_S64(numv, den);

        if (vc > vmax_qt || vc < -vmax_qt)
        {
            /* T短到巡航速度顶破vmax：把T顶开到刚好不越限，宁可晚到 */
            int64_t an   = (numv < 0) ? -numv : numv;
            int32_t need = (int32_t)((Traj_Div_U64((uint64_t)(an + vmax_qt - 1),
                                                  (uint32_t)vmax_qt)
                                      + (uint32_t)(t1 + t3 + 1)) >> 1);
            if (need > T) { T = need; continue; }
            vc = (vc > 0) ? vmax_qt : -vmax_qt;
        }

        /* 最速且朝目标运动时，最优解的巡航速度不低于u0；反解出 vc<u0 只是整数拍取整造成的，
         * 这时若照常加长首段，J受限下极小的速度差也要几十拍，会越解越慢、变成两级刹车。
         * 改为"以u0滑行一段再刹车"：首段吃满刹车前的全部时间，速度微调只在J允许的范围内做，
         * 剩下不足1拍行程的误差由终点吸收(见 p_end)。 */
        if (!stretched && !case_b && u0 > 0 && vc < u0)
        {
            int32_t pre;
            t3  = Traj_Seg_Ticks(&lim3, u0);
            pre = Traj_Div_S64((((int64_t)D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)u0 * t3) + u0,
                               2 * u0);   /* 刹车前以u0走的拍数，四舍五入 */
            t1 = (pre < 1) ? 1 : pre;
            T  = t1 + t3;
            vc = Traj_Div_S64(((int64_t)D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)u0 * t1, t1 + t3);
            break;
        }
        /* 首段变成减速(降速到更低巡航速度)：改用减速上限重算，否则会越过 TUNE_DMAX */
        if (!case_b && u0 > 0 && vc < u0 && !first_brake)
        {
            first_brake = 1U;
            lim1 = lim3;
            t1 = Traj_Seg_Ticks(&lim1, u0 - vc);
            if (T < t1 + t3) T = t1 + t3;
            continue;
        }
        if (i == TRAJ_FIT_ITERS - 1) break;   /* 最后一轮只解vc，保证 vc 与 t1/t3/T 自洽 */

        n1 = Traj_Seg_Ticks(&lim1, (vc > u0) ? (vc - u0) : (u0 - vc));
        n3 = Traj_Seg_Ticks(&lim3, (vc < 0) ? -vc : vc);
        if (n1 <= t1 && n3 <= t3) break;
        if (n1 > t1) t1 = n1;
        if (n3 > t3) t3 = n3;
        /* 加速段变长挤掉匀速段时先压减速段保准时；压不下才由循环顶部顶开总时间 */
        if (t1 + t3 > T && (T - t1) >= n3) t3 = T - t1;
    }
    if (T < t1 + t3) T = t1 + t3;

    /* 兜底：没收敛时把 vc 夹回两段过渡在现有时长内能完成的范围，加速度与加加速度上限永远成立，
     * 代价是一点到位误差(由终点吸收)，只出现在整数拍分辨不开的边角情形 */
    {
        int64_t w1 = Traj_Seg_DvMax(&lim1, t1);
        int64_t w3 = Traj_Seg_DvMax(&lim3, t3);
        if ((int64_t)vc > u0 + w1) vc = (int32_t)(u0 + w1);
        if ((int64_t)vc < u0 - w1) vc = (int32_t)(u0 - w1);
        if ((int64_t)vc >  w3) vc = (int32_t)w3;
        if ((int64_t)vc < -w3) vc = (int32_t)-w3;
        if (vc >  vmax_qt) vc =  vmax_qt;
        if (vc < -vmax_qt) vc = -vmax_qt;
    }
    tc = T - t1 - t3;

    /* 换回全局符号并预计算每拍常量 */
    s_tj.vc_qt  = vc * sgn;
    s_tj.dv1_qt = s_tj.vc_qt - s_tj.v0_qt;
    s_tj.dv3_qt = -s_tj.vc_qt;
    s_tj.t1 = t1;
    s_tj.tc = tc;
    s_tj.t3 = t3;
    s_tj.T  = T;
    s_tj.t  = 0;

    /* 每段形状按本段速度差取J受限最短过渡的形状；段比最短时长长时沿用，加速度与加加速度只低不高 */
    s_tj.seg1_r_q15 = Traj_Seg_R(&lim1, (vc > u0) ? (vc - u0) : (u0 - vc));
    s_tj.seg3_r_q15 = Traj_Seg_R(&lim3, (vc < 0) ? -vc : vc);
    Traj_Seg_Shape(s_tj.seg1_r_q15, &s_tj.seg1_peak_q15, &s_tj.seg1_inv_r_q15);
    Traj_Seg_Shape(s_tj.seg3_r_q15, &s_tj.seg3_peak_q15, &s_tj.seg3_inv_r_q15);

    /* 过渡段位移 = v0*tt + (dv*t_seg)*P(u)，P为S的积分 */
    s_tj.posk1 = (int32_t)(((int64_t)s_tj.dv1_qt * t1) >> TRAJ_U_SHIFT);
    s_tj.posk3 = (int32_t)(((int64_t)s_tj.dv3_qt * t3) >> TRAJ_U_SHIFT);

    /* 段末位置与逐拍公式同式计算，接缝处无舍入差 */
    s_tj.p_a = s_tj.p0
             + (int32_t)((((int64_t)s_tj.v0_qt * t1)
                        + (int64_t)s_tj.posk1 * (1L << (TRAJ_U_SHIFT - 1))) >> TRAJ_VEL_SHIFT);
    s_tj.p_c = s_tj.p_a
             + (int32_t)(((int64_t)s_tj.vc_qt * tc) >> TRAJ_VEL_SHIFT);
    s_tj.p_end = s_tj.p0 + dp;   /* 终点用解析值，避免逐段舍入累积 */

    /* 这两个倒数商有23~27位，Traj_Div_U64(定长约130条)比 __udivsi3 快；10位左右的商保留除号 */
    s_tj.inv_t1_q30 = (int32_t)Traj_Div_U64((uint64_t)1 << TRAJ_INV_SHIFT, (uint32_t)t1);
    s_tj.inv_t3_q30 = (int32_t)Traj_Div_U64((uint64_t)1 << TRAJ_INV_SHIFT, (uint32_t)t3);

    /* 加速度系数，复用上面的Q30倒数 */
    s_tj.acck1 = Traj_Acc_Coef(s_tj.dv1_qt, s_tj.inv_t1_q30);
    s_tj.acck3 = Traj_Acc_Coef(s_tj.dv3_qt, s_tj.inv_t3_q30);
}

/* 参考夹进行程后输出；撞到边界时速度归零，不把前馈往墙里送 */
static void Traj_Emit(TrajRef_t *out, int32_t pos, int32_t acc)
{
    if (pos < s_tj.lo || pos > s_tj.hi)
    {
        pos = Traj_Clamp(pos, s_tj.lo, s_tj.hi);
        s_tj.vel_qt = 0;
        acc = 0;          /* 撞死区边界：速度和加速度前馈都不该再往墙里送 */
    }
    s_tj.pos = pos;
    out->pos = pos;
    out->vel = (int32_t)(((int64_t)s_tj.vel_qt * CFG_TICK_HZ) >> TRAJ_VEL_SHIFT);
    out->acc = acc;
}

/* jerk受限速度过渡的归一化型线：S速度比例，P其积分，A归一化加速度；r=0 退化为恒加速度 */
static void Traj_SCurve(int32_t u, int32_t r, int32_t peak, int32_t inv_r,
                        int32_t *s, int32_t *p, int32_t *a)
{
    int32_t w, ratio, pr, sr;

    if (r <= 0)
    {
        *s = u;
        *p = (int32_t)(((int64_t)u * u) >> 16);
        *a = 32768;
        return;
    }

    if (u > 32768 - r)
    {
        int32_t sw, pw, aw;

        w = 32768 - u;
        Traj_SCurve(w, r, peak, inv_r, &sw, &pw, &aw);
        *s = 32768 - sw;
        *p = 16384 - w + pw;
        *a = aw;
        return;
    }

    if (u < r)
    {
        ratio = (int32_t)(((int64_t)u * inv_r) >> 15); /* u/r，Q15 */
        *a = (int32_t)(((int64_t)peak * ratio) >> 15);
        *s = (int32_t)(((int64_t)peak * u * ratio) >> 31);
        /* S=peak*u^2/(2r)，P=S*u/3 */
        /* 21845≈2^16/3，因此 >>31 等价于 /(3*2^15)，避免RV32上的软除法。 */
        *p = (int32_t)(((int64_t)(*s) * u * 21845) >> 31);
        return;
    }

    /* 中段加速度恒为 peak，P 增量 peak*u*(u-r)/2 */
    sr = (int32_t)(((int64_t)peak * r) >> 16); /* S(r)=peak*r/2 */
    pr = (int32_t)(((int64_t)sr * r * 21845) >> 31);
    *a = peak;
    *s = (int32_t)(((int64_t)peak * (u - (r >> 1))) >> 15);
    *p = pr + (int32_t)(((int64_t)peak * u * (u - r)) >> 31);
}

/* 推进一拍输出参考；每拍无除法、无开方、无浮点 */
void C_Traj_Step(TrajRef_t *out, uint8_t hold)
{
    int32_t tt;
    int32_t u_q15;
    int32_t s_q15;      /* 归一化速度 */
    int32_t p_q15;      /* S(u)从0到u的积分 */
    int32_t seg_p0, seg_v0_qt, seg_dv_qt, seg_inv_q30, seg_posk, seg_acck;
    int32_t a_q15;      /* 归一化加速度，Q15，1.0=32768 */
    int32_t pos, acc;
    int32_t seg_r = 0, seg_peak = 32768, seg_inv_r = 0;

    if (out == 0) return;

    /* 饱和保持：输出已饱和且落后时冻结轨迹时钟，参考自动退化为"硬件能做到的最快"，
     * 不依赖任何标定精度。只在加速/匀速段冻结：减速段冻结会把 ref.vel 钉在高位、帮着冲过头。 */
    if (s_tj.t < s_tj.T &&
        (!hold || s_tj.t >= s_tj.t1 + s_tj.tc))
    {
        s_tj.t++;
    }

    if (s_tj.t >= s_tj.T)
    {
        /* 最后一拍起锁在解析终点，速度归零 */
        s_tj.pos    = Traj_Clamp(s_tj.p_end, s_tj.lo, s_tj.hi);
        s_tj.vel_qt = 0;
        out->pos    = s_tj.pos;
        out->vel    = 0;
        out->acc    = 0;
        return;
    }

    if (s_tj.t <= s_tj.t1)
    {
        tt          = s_tj.t;                       /* 加速段 */
        seg_p0      = s_tj.p0;
        seg_v0_qt   = s_tj.v0_qt;
        seg_dv_qt   = s_tj.dv1_qt;
        seg_inv_q30 = s_tj.inv_t1_q30;
        seg_posk    = s_tj.posk1;
        seg_acck    = s_tj.acck1;
        seg_r       = s_tj.seg1_r_q15;
        seg_peak    = s_tj.seg1_peak_q15;
        seg_inv_r   = s_tj.seg1_inv_r_q15;
    }
    else if (s_tj.t <= s_tj.t1 + s_tj.tc)
    {
        /* 匀速段：无过渡整形，1次乘法 */
        tt = s_tj.t - s_tj.t1;
        s_tj.vel_qt = s_tj.vc_qt;
        pos = s_tj.p_a + (int32_t)(((int64_t)s_tj.vc_qt * tt) >> TRAJ_VEL_SHIFT);
        Traj_Emit(out, pos, 0);          /* 匀速段加速度恒为0 */
        return;
    }
    else
    {
        tt          = s_tj.t - s_tj.t1 - s_tj.tc;   /* 减速段 */
        seg_p0      = s_tj.p_c;
        seg_v0_qt   = s_tj.vc_qt;
        seg_dv_qt   = s_tj.dv3_qt;
        seg_inv_q30 = s_tj.inv_t3_q30;
        seg_posk    = s_tj.posk3;
        seg_acck    = s_tj.acck3;
        seg_r       = s_tj.seg3_r_q15;
        seg_peak    = s_tj.seg3_peak_q15;
        seg_inv_r   = s_tj.seg3_inv_r_q15;
    }

    /* u = tt/段长，用预存的Q30倒数换成乘法。tt<=段长 => 乘积<=2^30，不溢出。 */
    u_q15 = (int32_t)(((uint32_t)tt * (uint32_t)seg_inv_q30) >> TRAJ_U_SHIFT);
    if (u_q15 > (1 << TRAJ_U_SHIFT)) u_q15 = (1 << TRAJ_U_SHIFT); /* 舍入兜底 */

    Traj_SCurve(u_q15, seg_r, seg_peak, seg_inv_r, &s_q15, &p_q15, &a_q15);
    acc = (int32_t)(((int64_t)seg_acck * a_q15) >> TRAJ_U_SHIFT);

    s_tj.vel_qt = seg_v0_qt + (int32_t)(((int64_t)seg_dv_qt * s_q15) >> TRAJ_U_SHIFT);

    pos = seg_p0 + (int32_t)((((int64_t)seg_v0_qt * tt)
                            + ((int64_t)seg_posk * p_q15)) >> TRAJ_VEL_SHIFT);

    /* 减速段速度单调收敛不换向，按vc方向单侧钳位，防舍入越过终点 */
    if (s_tj.t > s_tj.t1 + s_tj.tc)
    {
        if (s_tj.vc_qt >= 0) { if (pos > s_tj.p_end) pos = s_tj.p_end; }
        else                 { if (pos < s_tj.p_end) pos = s_tj.p_end; }
    }

    Traj_Emit(out, pos, acc);
}

/* 本段轨迹是否已走完 */
uint8_t C_Traj_Is_Done(void)
{
    return (uint8_t)(s_tj.t >= s_tj.T);
}

/* 本段实际总时长(拍)，请求时间不可达时大于请求值 */
uint16_t C_Traj_Get_Ticks(void)
{
    return (uint16_t)((s_tj.T > 0xFFFF) ? 0xFFFF : s_tj.T);
}
