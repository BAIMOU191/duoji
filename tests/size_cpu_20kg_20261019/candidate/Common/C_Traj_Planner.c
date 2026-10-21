/* C_Traj_Planner.c 在线 S 曲线规划器：限幅换算 -> 最速三段 -> 时间拉伸 -> 巡航速度反解 -> 每拍常量
 * 内部速度为 Q16 厘度/拍；除法走 Traj_DivSat，开方只在规划时做。 */
#include "C_Traj_Planner.h"
#include "C_Fixed.h"

#define TRAJ_TICK_HZ      1000
#define TRAJ_VEL_SHIFT    16U  /* Q16：内部速度(厘度/拍)定点小数位数 */
#define TRAJ_U_SHIFT      15U  /* Q15：段内归一化时间u及其幂次的定点小数位数 */
#define TRAJ_INV_SHIFT    30U  /* Q30：1/段长 的定点小数位数 */
#define TRAJ_SOLVE_ITERS  14U  /* 二分求峰值速度的轮数；第3步还会反解一次巡航速度，这里够近即可 */
#define TRAJ_SQRT_X_LIMIT 274876858369ULL /* (INT32_MAX >> 12)^2：开方后左移12位仍放得进 int32 */

/* 64位整数平方根向上取整：开方只会偏大，过渡段只长不短，加速度只低不高 */
static uint32_t Traj_Sqrt64_Ceil(uint64_t x)
{
    uint64_t rem = x, root = 0U, bit = 1ULL << 62;

    while (bit > rem) bit >>= 2;
    while (bit != 0U)
    {
        if (rem >= root + bit) { rem -= root + bit; root = (root >> 1) + bit; }
        else root >>= 1;
        bit >>= 2;
    }
    return (uint32_t)root + ((rem != 0U) ? 1U : 0U);
}

static int32_t Traj_BitLen32(uint32_t x) { return (x == 0U) ? 0 : 32 - __builtin_clz(x); }

/* 64位除以32位，要求 (n>>32) < d。只做商的有效位数那么多轮逐位试商，比库函数 __divdi3 快一倍多 */
static uint32_t Traj_DivU64(uint64_t n, uint32_t d)
{
    int32_t  k = ((n >> 32) ? 32 + Traj_BitLen32((uint32_t)(n >> 32)) : Traj_BitLen32((uint32_t)n))
               - Traj_BitLen32(d) + 1;
    uint32_t r, bits, carry, q = 0U;

    if (k <= 0) return 0U;
    if (k > 32) k = 32;
    r    = (uint32_t)(n >> k);
    bits = (k >= 32) ? (uint32_t)n : ((uint32_t)n << (32 - k));
    while (k-- > 0)
    {
        carry = r >> 31;
        r     = (r << 1) | (bits >> 31);
        bits <<= 1;
        q   <<= 1;
        if (carry || r >= d) { r -= d; q |= 1U; }
    }
    return q;
}

/* 除法并饱和到 int32(向零取整)，本文件所有除法都走它 */
static int32_t Traj_DivSat(int64_t num, int64_t den)
{
    int64_t  q;
    uint64_t an = (num < 0) ? 0U - (uint64_t)num : (uint64_t)num;
    uint64_t ad = (den < 0) ? 0U - (uint64_t)den : (uint64_t)den;

    if (den == 0) return 0x7FFFFFFF;
    if ((ad >> 32) != 0U) q = num / den;
    else
    {
        if ((an >> 32) >= ad) q = 0x80000000LL;
        else if ((an >> 32) == 0U) q = (int64_t)((uint32_t)an / (uint32_t)ad);
        else q = (int64_t)Traj_DivU64(an, (uint32_t)ad);
        if ((num < 0) != (den < 0)) q = -q;
    }
    if (q >  0x7FFFFFFF) return  0x7FFFFFFF;
    if (q < -0x7FFFFFFF) return -0x7FFFFFFF;
    return (int32_t)q;
}

/*
 * 一种过渡段(加速/减速/换向)的限幅换算结果，按本段加速度上限 a 和加加速度上限 J 算一次。
 * 跨越速度差 dv 的最短过渡：dv >= a^2/J 时加速度为梯形，时长 dv/a + a/J；否则为三角形，
 * 时长 2*sqrt(dv/J)。两式在 dv = a^2/J 处相接。
 */
typedef struct {
    int32_t  kl_q32;   /* 1/a：每单位速度差(Q16厘度/拍)的恒加速时长，Q32拍 */
    int32_t  tj_q16;   /* a/J：jerk斜坡时长，Q16拍 */
    int32_t  dvj_qt;   /* a^2/J：够得着 a 的最小速度差，Q16厘度/拍 */
    uint64_t cj_q16;   /* 三角加速度段：时长^2(Q8拍^2) = dv*cj >> 16 */
    uint64_t dv_sat;   /* dv 超过它时 dv*cj 会越过开方上限，直接取上限 */
} TrajLim_t;

/* 一个过渡段每拍要用的常量。r 为 jerk 斜坡占段长的比例(Q15，16384=三角加速度)，peak=1/(1-r)，inv_r=1/r */
typedef struct {
    int32_t p0, v0_qt, dv_qt;
    int32_t inv_q30;   /* 1/段长，Q30 */
    int32_t posk;      /* (dv*段长)>>15，位置整形系数 */
    int32_t acck;      /* dv/段长 换算成厘度/秒^2 */
    int32_t r_q15, peak_q15, inv_r_q15;
} TrajSeg_t;

#define TRAJ_PRE_MAX ((int32_t)TRAJ_SMOOTH_MS_MAX)   /* 加速度接管段最长拍数：接管时长 = 当前加速度/J ≤ A/J = 平滑时间 */

static struct {
    /* 当前参考 */
    int32_t pos, vel_qt;
    int32_t acc_now;     /* 上一拍输出的加速度，厘度/秒^2：新规划从它接管 */

    /* 加速度接管段：新规划前先把当前加速度按 J 线性降到0，再接新的 S 曲线(加速度连续) */
    uint16_t pre_t, pre_T;
    int64_t  pre_pos_q16;            /* Q16 厘度 */
    int32_t  pre_vel_q16;            /* Q16 厘度/拍 */
    int32_t  pre_acc_q24, pre_dacc_q24; /* Q24 厘度/拍^2 及每拍减量 */

    /* 用户上限 */
    int32_t vmax_cdps, amax_cdpss, dmax_cdpss, jmax_cdpss3;
    uint16_t move_min_ms;   /* 最短运动时间 */
    uint8_t poly;          /* 带时间在线接续：两段三次速度曲线，共用seg存储 */
    TrajLim_t lim_dec;   /* 纯用户减速上限，刹停距离用 */

    /* 本段计划 */
    int32_t p0, v0_qt, vc_qt, p_a, p_c, p_end;
    TrajSeg_t seg[2];    /* [0]=加速段 [1]=减速段 */
    int32_t t1, tc, t3, T, t;

    /* 行程 */
    int32_t lo, hi;
} s_tj;

static void Traj_Lim_Make(TrajLim_t *l, int32_t a, int32_t j)
{
    static int32_t  s_j;       /* 只随 J 变的两项按 J 缓存 */
    static uint64_t s_cj, s_dv_sat;
    int32_t tj;

    a = Fx_ClampRange(a, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    j = Fx_ClampRange(j, TRAJ_JMAX_MIN_CDPSS3, TRAJ_JMAX_MAX_CDPSS3);
    l->kl_q32 = Traj_DivSat((int64_t)TRAJ_TICK_HZ * TRAJ_TICK_HZ * 65536, a);
    tj = Traj_DivSat((int64_t)a * 65536 * TRAJ_TICK_HZ, j);
    l->tj_q16 = (tj > 0x40000000) ? 0x40000000 : tj;
    l->dvj_qt = Traj_DivSat((int64_t)l->tj_q16 << 16, l->kl_q32);
    if (j != s_j)
    {
        s_cj     = 1024000000000ULL / (uint32_t)j;
        s_dv_sat = (TRAJ_SQRT_X_LIMIT << 16) / s_cj;
        s_j      = j;
    }
    l->cj_q16 = s_cj;
    l->dv_sat = s_dv_sat;
}

/* 跨越速度差 dv 的最短过渡时长，Q16拍；dv<=0 为0 */
static int32_t Traj_Seg_Q16(const TrajLim_t *l, int32_t dv)
{
    int64_t t;

    if (dv <= 0) return 0;
    if (dv >= l->dvj_qt) t = (((int64_t)l->kl_q32 * dv) >> 16) + l->tj_q16;      /* 梯形加速度 */
    else
    {
        uint64_t x = ((uint32_t)dv > l->dv_sat) ? TRAJ_SQRT_X_LIMIT : (((uint64_t)(uint32_t)dv * l->cj_q16) >> 16);
        t = (int64_t)Traj_Sqrt64_Ceil(x) << 12;                                    /* Q4拍 -> Q16拍 */
    }
    return (t > 0x7FFFFFFF) ? 0x7FFFFFFF : (int32_t)t;
}

/* 同上，向上取整到整拍且至少1拍；取整只会拉长，峰值加速度和加加速度只低不高 */
static int32_t Traj_Seg_Ticks(const TrajLim_t *l, int32_t dv)
{
    int32_t t = (int32_t)(((int64_t)Traj_Seg_Q16(l, dv) + 0xFFFF) >> 16);
    return (t < 1) ? 1 : t;
}

/* 过渡段形状 r(Q15)：三角加速度 0.5，梯形 (a/J)/时长 */
static int32_t Traj_Seg_R(const TrajLim_t *l, int32_t dv)
{
    int32_t t;

    if (dv < l->dvj_qt) return 16384;
    t = Traj_Seg_Q16(l, dv);
    if (t <= 0) return 16384;
    return Fx_ClampRange(Traj_DivSat((int64_t)l->tj_q16 << 15, t), 1, 16384);
}

/* Traj_Seg_Ticks 的反函数：t 拍内能完成的最大速度差 */
static int32_t Traj_Seg_DvMax(const TrajLim_t *l, int32_t t)
{
    if (t <= 0) return 0;
    if (((int64_t)t << 16) >= 2 * (int64_t)l->tj_q16)
        return Traj_DivSat(((((int64_t)t << 16) - l->tj_q16) << 16), l->kl_q32);   /* 梯形 */
    return Traj_DivSat((int64_t)t * t << 24, (int64_t)l->cj_q16);                   /* 三角形 */
}

/* 两段过渡 u0->vp->0、中间不留匀速段时按整数拍的位移x2，单位 厘度*2^16 */
static int64_t Traj_Ramp2(const TrajLim_t *l1, const TrajLim_t *l3, int32_t u0, int32_t vp)
{
    return (int64_t)(u0 + vp) * Traj_Seg_Ticks(l1, Fx_Abs32(vp - u0)) + (int64_t)vp * Traj_Seg_Ticks(l3, Fx_Abs32(vp));
}

/* 在[lo,hi]上求两段过渡位移不超过 D 的最大峰值速度 vp(位移随 vp 单调递增) */
static int32_t Traj_Solve_Vp(const TrajLim_t *l1, const TrajLim_t *l3, int32_t u0, int32_t D, int32_t lo, int32_t hi)
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

/* 速度(Q16厘度/拍)以减速上限刹停的距离，厘度；多算一拍的行程作余量 */
static int32_t Traj_StopDist(int32_t v_qt)
{
    int32_t av = Fx_Abs32(v_qt);

    if (av == 0) return 0;
    return (int32_t)(((int64_t)av * (Traj_Seg_Ticks(&s_tj.lim_dec, av) + 2)) >> (TRAJ_VEL_SHIFT + 1));
}

void C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, int32_t jerk)
{
    s_tj.vmax_cdps   = Fx_ClampRange(vel, TRAJ_VMAX_MIN_CDPS, TRAJ_VMAX_MAX_CDPS);
    s_tj.amax_cdpss  = Fx_ClampRange(acc, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    s_tj.dmax_cdpss  = Fx_ClampRange(dec, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    s_tj.jmax_cdpss3 = Fx_ClampRange(jerk, TRAJ_JMAX_MIN_CDPSS3, TRAJ_JMAX_MAX_CDPSS3);
    Traj_Lim_Make(&s_tj.lim_dec, s_tj.dmax_cdpss, s_tj.jmax_cdpss3);
}

void C_Traj_SetMoveMinMs(uint16_t ms)
{
    s_tj.move_min_ms = (ms <= TRAJ_MOVE_MIN_MS_MAX) ? ms : TRAJ_MOVE_MIN_MS_MAX;
}

void C_Traj_Hold(int32_t pos)
{
    s_tj.poly = 0U;
    s_tj.pos    = Fx_ClampRange(pos, s_tj.lo, s_tj.hi);
    s_tj.vel_qt = 0;
    s_tj.p0 = s_tj.p_a = s_tj.p_c = s_tj.p_end = s_tj.pos;
    s_tj.v0_qt = s_tj.vc_qt = 0;
    s_tj.seg[0].dv_qt = s_tj.seg[1].dv_qt = 0;
    s_tj.t1 = s_tj.t3 = 1;
    s_tj.tc = s_tj.T = s_tj.t = 0;   /* t >= T 即视为已走完 */
    s_tj.acc_now = 0;
    s_tj.pre_t = s_tj.pre_T = 0U;
}

void C_Traj_Init(int32_t pos)
{
    s_tj.lo = 0;
    s_tj.hi = 36000;
    C_Traj_SetLimits(10000, 100000, 100000, 10000000);
    C_Traj_SetMoveMinMs(0U);
    C_Traj_Hold(pos);
}

void C_Traj_SetRange(int32_t lo, int32_t hi)
{
    if (hi < lo) return;
    s_tj.lo = lo;
    s_tj.hi = hi;
    C_Traj_Hold(s_tj.pos);
}

/* 把段的 dv/段长 换算成厘度/秒^2(向零截断，正反向都只低不高) */
static int32_t Traj_Acc_Coef(int32_t dv_qt, int32_t inv_q30)
{
    int64_t product = (int64_t)dv_qt * inv_q30;
    int64_t per_tick2 = (product >= 0) ? (product >> TRAJ_INV_SHIFT) : -((-product) >> TRAJ_INV_SHIFT);
    int64_t scaled = per_tick2 * TRAJ_TICK_HZ * TRAJ_TICK_HZ;

    return (int32_t)((scaled >= 0) ? (scaled >> TRAJ_VEL_SHIFT) : -((-scaled) >> TRAJ_VEL_SHIFT));
}

/* Plan 的工作区：运动方向为正的归一化坐标，最后由 Traj_Commit 换回全局符号。速度 Q16 厘度/拍，时长整数拍 */
typedef struct {
    int32_t   D;          /* 本段行程，>=0 */
    int32_t   u0;         /* 起点速度沿运动方向的分量，反向时为负 */
    int32_t   vmax;
    TrajLim_t lim1;       /* 首段限幅(加速/刹车/换向) */
    TrajLim_t lim3;       /* 减速段限幅 */
    int32_t   t1, tc, t3, T, vc;
    uint8_t   case_b;     /* 1=刹不住，只能冲过目标再倒回 */
    uint8_t   stretched;  /* 1=按请求时间拉长过 */
} TrajPlan_t;

/* 本段速度/加速度上限：用户设定；已有速度高于速度上限时以它为准(已有的动量仍要能消掉) */
static void Traj_Limits(TrajPlan_t *p)
{
    TrajLim_t lim_acc;
    int32_t au0 = Fx_Abs32(p->u0);

    p->vmax = Traj_DivSat((int64_t)s_tj.vmax_cdps << TRAJ_VEL_SHIFT, TRAJ_TICK_HZ);
    if (p->vmax < au0) p->vmax = au0;

    Traj_Lim_Make(&lim_acc, s_tj.amax_cdpss, s_tj.jmax_cdpss3);
    Traj_Lim_Make(&p->lim3, s_tj.dmax_cdpss, s_tj.jmax_cdpss3);

    /* 正向运动而直接刹车的位移已超过剩余行程 -> 必然过冲，只能冲过去再倒回来 */
    p->case_b = (uint8_t)(p->u0 > 0 && ((int64_t)p->D << 17) < (int64_t)p->u0 * Traj_Seg_Ticks(&p->lim3, p->u0));
    /* 情形B和换向要先把已有速度刹掉，取两侧较弱的 */
    if ((p->case_b || p->u0 < 0) && s_tj.dmax_cdpss <= s_tj.amax_cdpss) p->lim1 = p->lim3;
    else p->lim1 = lim_acc;
}

/* 第1步：不管请求时间，先求最速轨迹的三段时长 */
static void Traj_Fastest(TrajPlan_t *p)
{
    int32_t vp, d_ramp;

    if (p->case_b) vp = Traj_Solve_Vp(&p->lim1, &p->lim3, p->u0, p->D, -p->vmax, 0);
    else
    {
        p->t1 = Traj_Seg_Ticks(&p->lim1, p->vmax - p->u0);
        p->t3 = Traj_Seg_Ticks(&p->lim3, p->vmax);
        d_ramp = (int32_t)(((int64_t)(p->u0 + p->vmax) * p->t1 + (int64_t)p->vmax * p->t3) >> (TRAJ_VEL_SHIFT + 1));
        if (d_ramp <= p->D)
        {
            uint32_t rest = (uint32_t)(p->D - d_ramp) << TRAJ_VEL_SHIFT;
            p->tc = (int32_t)((rest + (uint32_t)p->vmax - 1U) / (uint32_t)p->vmax);
            return;
        }
        vp = Traj_Solve_Vp(&p->lim1, &p->lim3, p->u0, p->D, p->u0 > 0 ? p->u0 : 0, p->vmax);
    }
    p->t1 = Traj_Seg_Ticks(&p->lim1, Fx_Abs32(vp - p->u0));
    p->t3 = Traj_Seg_Ticks(&p->lim3, Fx_Abs32(vp));
    p->tc = 0;
}

/* 第2步：请求时间比最速还长 -> 三段同乘 lambda */
static void Traj_Stretch(TrajPlan_t *p, int32_t T_req)
{
    int32_t T_fast = p->t1 + p->tc + p->t3;
    uint32_t lam_q8;

    p->T = T_fast;
    p->stretched = (uint8_t)(T_req > T_fast);
    if (!p->stretched) return;

    lam_q8 = ((uint32_t)T_req << 8) / (uint32_t)T_fast;
    if (!p->case_b && p->u0 >= 0) p->t1 = (int32_t)(((uint32_t)p->t1 * lam_q8) >> 8);
    else                          p->t1 = Traj_Seg_Ticks(&p->lim1, Fx_Abs32(p->u0));
    p->t3 = (int32_t)(((uint32_t)p->t3 * lam_q8) >> 8);
    p->T  = T_req;
    if (p->t1 < 1) p->t1 = 1;
    if (p->t1 + p->t3 > p->T) p->t3 = p->T - p->t1;
    if (p->t3 < 1) p->t3 = 1;
}

/* 由位移方程反解巡航速度：D = (u0+vc)*t1/2 + vc*tc + vc*t3/2 => vc = (2D - u0*t1)/(2T - t1 - t3) */
static int32_t Traj_SolveVc(const TrajPlan_t *p)
{
    return Traj_DivSat(((int64_t)p->D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)p->u0 * p->t1, 2 * p->T - p->t1 - p->t3);
}

/* 把 vc 夹回速度上限与两段过渡在现有时长内做得到的范围 */
static void Traj_ClampVc(TrajPlan_t *p)
{
    int64_t w1 = Traj_Seg_DvMax(&p->lim1, p->t1);
    int64_t w3 = Traj_Seg_DvMax(&p->lim3, p->t3);

    if ((int64_t)p->vc > p->u0 + w1) p->vc = (int32_t)(p->u0 + w1);
    if ((int64_t)p->vc < p->u0 - w1) p->vc = (int32_t)(p->u0 - w1);
    if ((int64_t)p->vc >  w3) p->vc = (int32_t)w3;
    if ((int64_t)p->vc < -w3) p->vc = (int32_t)-w3;
    p->vc = Fx_Clamp32(p->vc, p->vmax);
}

/* 第3步：整数拍取整后用位移方程反解巡航速度补回，再由匀速段长度补齐位移：终点精确，总时长可差几十毫秒 */
static void Traj_FitCruise(TrajPlan_t *p)
{
    int32_t n1, n3;

    if (p->T < p->t1 + p->t3) p->T = p->t1 + p->t3;
    p->vc = Traj_SolveVc(p);

    if (!p->case_b && p->u0 > 0 && p->vc < p->u0)
    {
        if (!p->stretched)
        {   /* 最速轨迹被取整压低：以 u0 滑行到底再按减速上限刹车，就此定稿 */
            int32_t pre;

            p->t3 = Traj_Seg_Ticks(&p->lim3, p->u0);
            pre   = Traj_DivSat((((int64_t)p->D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)p->u0 * p->t3) + p->u0, 2 * p->u0);
            p->t1 = (pre < 1) ? 1 : pre;
            p->T  = p->t1 + p->t3;
            p->vc = Traj_SolveVc(p);
            Traj_ClampVc(p);
            p->tc = 0;
            p->T  = p->t1 + p->t3;
            return;
        }
        /* 带时间指令要降到更低的巡航速度：首段其实在减速，改用减速上限 */
        p->lim1 = p->lim3;
        p->t1   = Traj_Seg_Ticks(&p->lim1, p->u0 - p->vc);
        if (p->T < p->t1 + p->t3) p->T = p->t1 + p->t3;
        p->vc = Traj_SolveVc(p);
    }

    /* 过渡段必须放得下这个速度差，不够就加长并重解一次 */
    n1 = Traj_Seg_Ticks(&p->lim1, Fx_Abs32(p->vc - p->u0));
    n3 = Traj_Seg_Ticks(&p->lim3, Fx_Abs32(p->vc));
    if (n1 > p->t1 || n3 > p->t3)
    {
        if (n1 > p->t1) p->t1 = n1;
        if (n3 > p->t3) p->t3 = n3;
        if (p->T < p->t1 + p->t3) p->T = p->t1 + p->t3;
        p->vc = Traj_SolveVc(p);
    }
    Traj_ClampVc(p);

    /* 取整边界上夹完的 vc 可能仍需 t+1 拍：段长只增不减，位移差由匀速段吸收 */
    n1 = Traj_Seg_Ticks(&p->lim1, Fx_Abs32(p->vc - p->u0));
    n3 = Traj_Seg_Ticks(&p->lim3, Fx_Abs32(p->vc));
    if (n1 > p->t1) p->t1 = n1;
    if (n3 > p->t3) p->t3 = n3;

    if (p->vc == 0) p->tc = 0;
    else
    {
        p->tc = (Traj_DivSat(((int64_t)p->D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)p->u0 * p->t1, p->vc) - p->t1 - p->t3) / 2;
        if (p->tc < 0) p->tc = 0;
    }
    p->T = p->t1 + p->tc + p->t3;
}

static void Traj_Seg_Shape(int32_t r, int32_t *peak, int32_t *inv_r)
{
    *peak  = Traj_DivSat(1LL << 30, 32768 - r);
    *inv_r = Traj_DivSat(1LL << 30, r);
}

static void Traj_Seg_Make(TrajSeg_t *g, int32_t p0, int32_t v0_qt, int32_t dv_qt,
                          int32_t ticks, const TrajLim_t *lim, int32_t dv_shape)
{
    g->p0      = p0;
    g->v0_qt   = v0_qt;
    g->dv_qt   = dv_qt;
    g->inv_q30 = Traj_DivSat(1LL << TRAJ_INV_SHIFT, ticks);
    g->posk    = (int32_t)(((int64_t)dv_qt * ticks) >> TRAJ_U_SHIFT);
    g->acck    = Traj_Acc_Coef(dv_qt, g->inv_q30);
    g->r_q15   = Traj_Seg_R(lim, dv_shape);
    Traj_Seg_Shape(g->r_q15, &g->peak_q15, &g->inv_r_q15);
}

/* 换回全局符号，把每拍 Step 要用的常量一次算好 */
static void Traj_Commit(const TrajPlan_t *p, int32_t sgn, int32_t dp)
{
    int32_t vc_g = p->vc * sgn;

    s_tj.vc_qt = vc_g;
    s_tj.t1 = p->t1; s_tj.tc = p->tc; s_tj.t3 = p->t3; s_tj.T = p->T; s_tj.t = 0;
    Traj_Seg_Make(&s_tj.seg[0], s_tj.p0, s_tj.v0_qt, vc_g - s_tj.v0_qt, p->t1, &p->lim1, Fx_Abs32(p->vc - p->u0));
    s_tj.p_a = s_tj.p0 + (int32_t)((((int64_t)s_tj.v0_qt * p->t1)
                                    + (int64_t)s_tj.seg[0].posk * (1L << (TRAJ_U_SHIFT - 1))) >> TRAJ_VEL_SHIFT);
    s_tj.p_c = s_tj.p_a + (int32_t)(((int64_t)vc_g * p->tc) >> TRAJ_VEL_SHIFT);
    s_tj.p_end = s_tj.p0 + dp;   /* 终点用解析值，避免逐段舍入累积 */
    Traj_Seg_Make(&s_tj.seg[1], s_tj.p_c, vc_g, -vc_g, p->t3, &p->lim3, Fx_Abs32(p->vc));
}

/* 加速度接管段一拍：加速度每拍减一个固定量，速度、位置按欧拉积分 */
static void Traj_PreTick(int64_t *pos_q16, int32_t *vel_q16, int32_t *acc_q24, uint16_t k, uint16_t n)
{
    *pos_q16 += *vel_q16;
    *acc_q24  = (k >= n) ? 0 : *acc_q24 - s_tj.pre_dacc_q24;
    *vel_q16 += *acc_q24 >> 8;
}

/* 新规划前的加速度接管：当前加速度 a0 按 J 用 a0/J 拍线性降到0，新规划从接管段末的位置、速度开始。
 * 预测与逐拍推进走同一套整数递推，接缝处位置、速度、加速度都连续 */
static void Traj_Prefix(void)
{
    int64_t pos;
    int32_t vel, acc, n;
    uint16_t k;

    s_tj.pre_t = s_tj.pre_T = 0U;
    if (s_tj.acc_now == 0) return;
    n = Traj_DivSat((int64_t)Fx_Abs32(s_tj.acc_now) * 1000 + s_tj.jmax_cdpss3 - 1, s_tj.jmax_cdpss3);
    if (n < 2) return;
    if (n > TRAJ_PRE_MAX) n = TRAJ_PRE_MAX;
    s_tj.pre_acc_q24  = (int32_t)(((int64_t)s_tj.acc_now << 24) / 1000000);
    s_tj.pre_dacc_q24 = s_tj.pre_acc_q24 / n;
    s_tj.pre_pos_q16  = (int64_t)s_tj.pos << 16;
    s_tj.pre_vel_q16  = s_tj.vel_qt;
    s_tj.pre_T        = (uint16_t)n;
    pos = s_tj.pre_pos_q16; vel = s_tj.pre_vel_q16; acc = s_tj.pre_acc_q24;
    for (k = 1U; k <= s_tj.pre_T; k++) Traj_PreTick(&pos, &vel, &acc, k, s_tj.pre_T);
    s_tj.pos    = (int32_t)((pos + 32768) >> 16);
    s_tj.vel_qt = vel;
}

/* 从当前参考状态(接管段之后)规划到 target，time_ms 比最速时间长时整体拉伸 */
static void Traj_PlanFrom(int32_t target, uint16_t time_ms)
{
    TrajPlan_t p;
    int32_t dp, sgn;

    s_tj.poly = 0U;
    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = s_tj.vel_qt;
    dp   = target - s_tj.p0;
    sgn  = (dp >= 0) ? 1 : -1;
    p.D  = dp * sgn;
    p.u0 = s_tj.v0_qt * sgn;
    if (p.D == 0 && p.u0 == 0)
    {
        s_tj.p_a = s_tj.p_c = s_tj.p_end = s_tj.p0;
        s_tj.vc_qt = 0; s_tj.t1 = s_tj.t3 = 1; s_tj.tc = s_tj.T = s_tj.t = 0;
        return;
    }
    Traj_Limits(&p);
    Traj_Fastest(&p);
    Traj_Stretch(&p, ((int32_t)time_ms * TRAJ_TICK_HZ) / 1000);
    Traj_FitCruise(&p);
    Traj_Commit(&p, sgn, dp);
}

/* 同向接续的三次速度段。调用方vmax<=6553600(Q16)，an限于27位；
 * 先检查端点范围后，系数及Bernstein控制点均可用int32精确计算。 */
static uint8_t Traj_PolyMake(TrajSeg_t *g, int32_t v0, int32_t an, int32_t v1,
                             int32_t n, int32_t vmax)
{
    int32_t dv, b, c, d, av[3], bv[4], j0, j1;
    int64_t den;
    uint8_t i;

    /* 量化后的末速度与v1相差至多6；提前剔除必不满足原包络检查的端点。 */
    if (n < 8 || v0 < -4 || v0 > vmax + 4 || v1 < -10 || v1 > vmax + 10
        || an < -0x07FFFFFF || an > 0x07FFFFFF) return 0U;
    dv = v1 - v0;
    b = an / 2; c = (3 * dv - 2 * an) / 3; d = (-2 * dv + an) / 4;
    if (c < -0x07FFFFFF || c > 0x07FFFFFF || d < -0x07FFFFFF || d > 0x07FFFFFF) return 0U;
    bv[0] = v0; bv[1] = v0 + 2 * b / 3;
    bv[2] = v0 + 4 * b / 3 + c; bv[3] = v0 + 2 * b + 3 * c + 4 * d;
    av[0] = 2 * b; av[1] = 2 * b + 3 * c; av[2] = 2 * b + 6 * c + 12 * d;
    for (i = 0U; i < 4U; i++) if (bv[i] < -4 || bv[i] > vmax + 4) return 0U;
    for (i = 0U; i < 3U; i++)
        if ((int64_t)av[i] * 1000000 > (int64_t)s_tj.amax_cdpss * n * 65536
            || (int64_t)av[i] * 1000000 < -(int64_t)s_tj.dmax_cdpss * n * 65536) return 0U;
    /* jerk线性：只比较两端最大值。1e9/65536约成1953125/128，商与原式完全相同。 */
    j0 = Fx_Abs32(6 * c); j1 = Fx_Abs32(6 * c + 24 * d);
    if (j1 > j0) j0 = j1;
    den = (int64_t)n * n * 128;
    if (Traj_DivSat((int64_t)j0 * 1953125 + den - 1, den) > s_tj.jmax_cdpss3) return 0U;
    g->v0_qt = v0; g->r_q15 = b; g->peak_q15 = c; g->inv_r_q15 = d;
    g->posk = n;
    return 1U;
}

/* 私有求值只接收0<=tick<=段长，tick*floor(2^30/段长)<=2^30，不需要64位乘法。 */
static int32_t Traj_PolyU(const TrajSeg_t *g, int32_t tick)
{
    return (int32_t)(((uint32_t)tick * (uint32_t)g->inv_q30) >> 14);
}

static int32_t Traj_PolyPos(const TrajSeg_t *g, int32_t u)
{
    int32_t p = g->peak_q15 + (int32_t)(((int64_t)g->inv_r_q15 * u) >> 16);
    p = g->r_q15 + (int32_t)(((int64_t)p * u) >> 16);
    p = g->v0_qt + (int32_t)(((int64_t)p * u) >> 16);
    p = (int32_t)(((int64_t)p * u) >> 16);
    return g->p0 + (int32_t)(((int64_t)p * g->posk) >> 16);
}

static void Traj_PolyEval(const TrajSeg_t *g, int32_t tick,
                          int32_t *pos, int32_t *vel, int32_t *acc)
{
    int32_t u = Traj_PolyU(g, tick);
    int32_t b = g->r_q15, c = g->peak_q15, d = g->inv_r_q15, v;
    int64_t a;

    v = 3 * c + (int32_t)(((int64_t)(4 * d) * u) >> 16);
    v = 2 * b + (int32_t)(((int64_t)v * u) >> 16);
    v = g->v0_qt + (int32_t)(((int64_t)v * u) >> 16);
    a = 6LL * c + (((int64_t)(12 * d) * u) >> 16);
    a = 2LL * b + (a * u >> 16);
    *pos = Traj_PolyPos(g, u);
    *vel = v;
    *acc = (int32_t)((a * g->acck) >> 30);
}

static uint8_t Traj_TimedJoin(int32_t target, uint16_t ms)
{
    TrajSeg_t g[2];
    int32_t dp = target - s_tj.pos, dir = (dp >= 0) ? 1 : -1;
    int32_t v0 = s_tj.vel_qt * dir, a0 = s_tj.acc_now * dir;
    int32_t T = (ms > s_tj.move_min_ms) ? ms : s_tj.move_min_ms;
    int32_t vmax = Traj_DivSat((int64_t)s_tj.vmax_cdps << 16, 1000);
    int32_t percent;

    if (s_tj.t >= s_tj.T || v0 <= 0 || v0 > vmax || dp == 0) return 0U;
    for (percent = 35; percent >= 5; percent -= 5) {
        int32_t n = (T * percent + 99) / 100, vc, pa, pc, end;
        int32_t an = Traj_DivSat((int64_t)a0 * n * 1024, 15625);
        int64_t distance = (int64_t)Fx_Abs32(dp) << 16;
        int64_t numerator = distance - (int64_t)v0 * n / 2 - (int64_t)an * n / 12;

        if (n < 8 || T <= 2 * n || numerator <= 0 || Fx_Abs32(an) > 0x07FFFFFF) continue;
        vc = Traj_DivSat(numerator, T - n);
        if (!Traj_PolyMake(&g[0], v0, an, vc, n, vmax)
            || !Traj_PolyMake(&g[1], vc, 0, 0, n, vmax)) continue;
        /* 两段同长，倒数各算一次；端点只求位置，不做无用的速度/加速度计算。 */
        g[0].inv_q30 = g[1].inv_q30 = Traj_DivSat(1LL << 30, n);
        g[0].acck = g[1].acck = Traj_DivSat(1000000LL << 14, n);
        g[0].p0 = 0;
        pa = Traj_PolyPos(&g[0], Traj_PolyU(&g[0], n));
        pc = pa + (int32_t)(((int64_t)vc * (T - 2 * n)) >> 16);
        g[1].p0 = pc;
        end = Traj_PolyPos(&g[1], Traj_PolyU(&g[1], n));
        /* 不用回摆补位移；终点量化差最多2厘度，最后一拍落到解析目标。 */
        if (end < 0 || end > Fx_Abs32(dp) || Fx_Abs32(dp) - end > 2) continue;
        s_tj.p0 = s_tj.pos; s_tj.v0_qt = s_tj.vel_qt; s_tj.vc_qt = vc * dir;
        s_tj.p_a = s_tj.pos + pa * dir; s_tj.p_c = s_tj.pos + pc * dir; s_tj.p_end = target;
        g[0].p0 = s_tj.p0; g[1].p0 = s_tj.p_c;
        if (dir < 0) {
            uint8_t i;
            for (i = 0U; i < 2U; i++) {
                g[i].v0_qt = -g[i].v0_qt; g[i].r_q15 = -g[i].r_q15;
                g[i].peak_q15 = -g[i].peak_q15; g[i].inv_r_q15 = -g[i].inv_r_q15;
            }
        }
        s_tj.seg[0] = g[0]; s_tj.seg[1] = g[1];
        s_tj.t1 = s_tj.t3 = n; s_tj.tc = T - 2 * n; s_tj.T = T; s_tj.t = 0;
        s_tj.pre_t = s_tj.pre_T = 0U; s_tj.poly = 1U;
        return 1U;
    }
    return 0U;
}

/* 普通规划保留原路径，含T=0连续目标的内部展望时间。 */
void C_Traj_Plan(int32_t target, uint16_t time_ms)
{
    target = Fx_ClampRange(target, s_tj.lo, s_tj.hi);
    Traj_Prefix();
    Traj_PlanFrom(target, (time_ms > s_tj.move_min_ms) ? time_ms : s_tj.move_min_ms);
}

/* 显式带时间的新指令：同向且约束可行时连续接续，否则走原规划。 */
void C_Traj_PlanTimed(int32_t target, uint16_t time_ms)
{
    target = Fx_ClampRange(target, s_tj.lo, s_tj.hi);
    if (time_ms != 0U && Traj_TimedJoin(target, time_ms)) return;
    C_Traj_Plan(target, time_ms);
}

/* 刹停不套最短运动时间 */
void C_Traj_Stop(void)
{
    int32_t v;

    Traj_Prefix();                       /* 刹停点按接管段之后的速度算 */
    v = s_tj.vel_qt;
    Traj_PlanFrom(Fx_ClampRange(s_tj.pos + ((v >= 0) ? Traj_StopDist(v) : -Traj_StopDist(v)), s_tj.lo, s_tj.hi), 0U);
}

/* jerk受限速度过渡的归一化型线：S速度比例，P其积分，A归一化加速度 */
static void Traj_SCurve(int32_t u, int32_t r, int32_t peak, int32_t inv_r, int32_t *s, int32_t *p, int32_t *a)
{
    int32_t ratio, pr, sr;

    if (r <= 0) { *s = u; *p = (int32_t)(((int64_t)u * u) >> 16); *a = 32768; return; }
    if (u > 32768 - r)
    {
        int32_t sw, pw, aw, w = 32768 - u;
        Traj_SCurve(w, r, peak, inv_r, &sw, &pw, &aw);
        *s = 32768 - sw;
        *p = 16384 - w + pw;
        *a = aw;
        return;
    }
    if (u < r)
    {
        ratio = (int32_t)(((int64_t)u * inv_r) >> 15);
        *a = (int32_t)(((int64_t)peak * ratio) >> 15);
        *s = (int32_t)(((int64_t)peak * u * ratio) >> 31);
        *p = (int32_t)(((int64_t)(*s) * u * 21845) >> 31);   /* S*u/3，21845≈2^16/3 */
        return;
    }
    sr = (int32_t)(((int64_t)peak * r) >> 16);
    pr = (int32_t)(((int64_t)sr * r * 21845) >> 31);
    *a = peak;
    *s = (int32_t)(((int64_t)peak * (u - (r >> 1))) >> 15);
    *p = pr + (int32_t)(((int64_t)peak * u * (u - r)) >> 31);
}

/* 参考推进一拍，输出位置与加速度(速度在 s_tj.vel_qt) */
static void Traj_CoreStep(uint8_t hold, int32_t *pos_out, int32_t *acc_out)
{
    const TrajSeg_t *g;
    int32_t tt, u_q15, s_q15, p_q15, a_q15, pos;
    uint8_t decel;

    if (s_tj.pre_t < s_tj.pre_T)
    {   /* 加速度接管段：不冻结(它本身就是在把加速度收掉) */
        Traj_PreTick(&s_tj.pre_pos_q16, &s_tj.pre_vel_q16, &s_tj.pre_acc_q24, ++s_tj.pre_t, s_tj.pre_T);
        s_tj.vel_qt  = s_tj.pre_vel_q16;
        *pos_out     = (int32_t)((s_tj.pre_pos_q16 + 32768) >> 16);
        s_tj.pos     = *pos_out;   /* 接管段中再来新规划时从当前状态出发 */
        *acc_out     = (int32_t)(((int64_t)s_tj.pre_acc_q24 * 1000000) >> 24);
        s_tj.acc_now = *acc_out;
        if (s_tj.pre_t >= s_tj.pre_T) s_tj.vel_qt = s_tj.v0_qt;   /* 与规划起点逐位相同 */
        return;
    }
    /* 饱和保持只在加速/匀速段冻结：减速段冻结会把参考速度钉在高位、帮着冲过头 */
    if (s_tj.t < s_tj.T && (!hold || s_tj.t >= s_tj.t1 + s_tj.tc
        || (s_tj.poly && s_tj.t < s_tj.t1
            && (int64_t)s_tj.v0_qt * (s_tj.vc_qt - s_tj.v0_qt) < 0))) s_tj.t++;
    s_tj.acc_now = 0;
    if (s_tj.t >= s_tj.T)
    {
        s_tj.pos = s_tj.p_end;
        s_tj.vel_qt = 0;
        *pos_out = s_tj.pos; *acc_out = 0;
        return;
    }
    if (s_tj.t > s_tj.t1 && s_tj.t <= s_tj.t1 + s_tj.tc)
    {
        s_tj.vel_qt = s_tj.vc_qt;
        s_tj.pos = s_tj.p_a + (int32_t)(((int64_t)s_tj.vc_qt * (s_tj.t - s_tj.t1)) >> TRAJ_VEL_SHIFT);
        *pos_out = s_tj.pos; *acc_out = 0;
        return;
    }
    decel = (uint8_t)(s_tj.t > s_tj.t1 + s_tj.tc);
    g     = &s_tj.seg[decel];
    tt    = decel ? (s_tj.t - s_tj.t1 - s_tj.tc) : s_tj.t;
    if (s_tj.poly) {
        Traj_PolyEval(g, tt, &s_tj.pos, &s_tj.vel_qt, acc_out);
        s_tj.acc_now = *acc_out;
        *pos_out = s_tj.pos;
        return;
    }
    u_q15 = (int32_t)(((uint32_t)tt * (uint32_t)g->inv_q30) >> TRAJ_U_SHIFT);
    if (u_q15 > (1 << TRAJ_U_SHIFT)) u_q15 = 1 << TRAJ_U_SHIFT;
    Traj_SCurve(u_q15, g->r_q15, g->peak_q15, g->inv_r_q15, &s_q15, &p_q15, &a_q15);
    *acc_out = (int32_t)(((int64_t)g->acck * a_q15) >> TRAJ_U_SHIFT);
    s_tj.acc_now = *acc_out;
    s_tj.vel_qt = g->v0_qt + (int32_t)(((int64_t)g->dv_qt * s_q15) >> TRAJ_U_SHIFT);
    pos = g->p0 + (int32_t)((((int64_t)g->v0_qt * tt) + ((int64_t)g->posk * p_q15)) >> TRAJ_VEL_SHIFT);
    if (decel)
    {   /* 减速段速度单调收敛不换向，单侧钳位防舍入越过终点 */
        if (s_tj.vc_qt >= 0) { if (pos > s_tj.p_end) pos = s_tj.p_end; }
        else                 { if (pos < s_tj.p_end) pos = s_tj.p_end; }
    }
    s_tj.pos = pos;
    *pos_out = pos;
}

void C_Traj_Step(TrajRef_t *out, uint8_t hold)
{
    int32_t pos, acc, vel_qt;

    Traj_CoreStep(hold, &pos, &acc);
    vel_qt = s_tj.vel_qt;
    if (pos < s_tj.lo || pos > s_tj.hi)
    {   /* 撞到行程边界：位置夹住，速度、加速度前馈不再往墙里送 */
        pos = Fx_ClampRange(pos, s_tj.lo, s_tj.hi);
        vel_qt = 0;
        acc = 0;
    }
    out->pos  = pos;
    out->vel  = (int32_t)(((int64_t)vel_qt * TRAJ_TICK_HZ) >> TRAJ_VEL_SHIFT);
    out->acc  = acc;
    out->done = C_Traj_Is_Done();
}

uint8_t C_Traj_Is_Done(void) { return (uint8_t)(s_tj.t >= s_tj.T); }

void C_Traj_SetRangeOpen(int32_t lo, int32_t hi)
{
    if (lo > hi) { int32_t swap = lo; lo = hi; hi = swap; }
    s_tj.lo = lo;
    s_tj.hi = hi;
}

int32_t C_Traj_GetPos(void) { return s_tj.pos; }
