/* C_Traj_Planner.c 在线轨迹规划器(速度/加速度/加加速度限幅型)，无硬件依赖 */

#include "C_Traj_Planner.h"
#include "A_Config.h"
#include "C_Fixed.h"

/* 硬件制动上限：末端(反电势帮不上忙)满PWM扣掉反馈余量、再按制动侧增益折算的减速度，本机约6030度/秒^2。
 * 再往上要吃掉给反馈的余量，减速跟不上就会过冲。 */
#define TRAJ_DEC_HW_RAW ((int32_t)(((int64_t)(CFG_PWM_FULL - PLANT_FF_MARGIN_PWM) * PLANT_KA_Q8) / MODEL_BRAKE_GAIN_Q8))
#define TRAJ_DEC_HW_CDPSS ((TRAJ_DEC_HW_RAW > 0) ? TRAJ_DEC_HW_RAW : 1)

#define TRAJ_VEL_SHIFT   16U  /* Q16：内部速度(厘度/拍)定点小数位数 */
#define TRAJ_U_SHIFT     15U  /* Q15：段内归一化时间u及其幂次的定点小数位数 */
#define TRAJ_INV_SHIFT   30U  /* Q30：1/段长 的定点小数位数 */
#define TRAJ_SOLVE_ITERS 14U  /* 二分求峰值速度的轮数；第3步还会反解一次巡航速度，这里够近即可 */
#define TRAJ_SQRT_X_LIMIT 274876858369ULL /* (INT32_MAX >> 12)^2：开方后左移12位仍放得进 int32 */

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

/* 64位整数平方根向上取整；只在收到运动指令后规划一次，用于支持低至1度/秒^3的Jerk。 */
static uint32_t Traj_Sqrt64_Ceil(uint64_t x)
{
    uint64_t rem = x;
    uint64_t root = 0U;
    uint64_t bit = 1ULL << 62;

    while (bit > rem) bit >>= 2;
    while (bit != 0U)
    {
        if (rem >= root + bit)
        {
            rem -= root + bit;
            root = (root >> 1) + bit;
        }
        else root >>= 1;
        bit >>= 2;
    }
    return (uint32_t)root + ((rem != 0U) ? 1U : 0U);
}

static int32_t Traj_DivSat(int64_t num, int64_t den);

/* 随负载惯量缩放的硬件能力(标定值 / 加速度增益比)，见 C_Traj_SetPlant */
static int32_t s_a0_cdpss     = PLANT_A0_CDPSS;    /* 零速起步加速度 */
static int32_t s_dec_hw_cdpss = TRAJ_DEC_HW_CDPSS; /* 制动侧硬件减速度上限 */

/* 本段硬件能给出的加速度上限(厘度/秒^2)，只作天花板：用户设的加速度更小时它不起作用；详见 docs/ALGORITHM_NOTES.md。 */
static int32_t Traj_AutoAmax(int32_t D, int32_t vmax, int32_t v0_cdps)
{
    int64_t denom;
    int32_t h_q12, f_q15, f_cap, f_now;

    if (PLANT_VSS_CDPS <= 0 || s_a0_cdpss <= 0)
        return TRAJ_AMAX_MAX_CDPSS;      /* 未标定：不另设硬件天花板，只受用户上限约束 */

    denom = (int64_t)MODEL_TAU_MS * PLANT_VSS_CDPS;
    if (denom <= 0) return TRAJ_AMAX_MAX_CDPSS;

    /* H = D*1000/(tau_ms*v_ss)，Q12。H=2 时 f 就已经到 1，再大没有意义，
     * 先夹住，顺便保证下面的平方在 uint32 里放得下 */
    h_q12 = Traj_DivSat((int64_t)D * 1000 * 4096, denom);
    if (h_q12 > 8192) h_q12 = 8192;
    /* 开方向上取整 => f 偏大 => 加速度只会偏小，安全侧 */
    f_q15 = ((int32_t)Traj_Sqrt_Ceil((uint32_t)h_q12 * (uint32_t)h_q12
                                     + 65536U * (uint32_t)h_q12) - h_q12) * 2;

    /* 峰值速度还受本段速度上限约束 */
    f_cap = Traj_DivSat((int64_t)vmax << 15, PLANT_VSS_CDPS);
    if (f_q15 > f_cap) f_q15 = f_cap;

    /* 起点速度也算进工作点：运动中改目标时反电势已吃掉大部分PWM，只按行程算会高估加速度 */
    f_now = Traj_DivSat((int64_t)v0_cdps << 15, PLANT_VSS_CDPS);
    if (f_now > 32767) f_now = 32767;
    if (f_q15 < f_now) f_q15 = f_now;
    if (f_q15 > 32767) f_q15 = 32767;

    return (int32_t)(((int64_t)s_a0_cdpss * (32768 - f_q15 / 2)) >> 15);
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
    uint64_t cj_q16;  /* 三角加速度段：时长^2(Q8拍^2) = dv*cj >> 16 */
    uint64_t dv_sat;  /* dv 超过它时 dv*cj 会越过开方上限，直接取上限 */
} TrajLim_t;

/*
 * 一个过渡段每拍要用的全部常量。加速段和减速段各一份，Step 里按当前拍取其一，
 * 于是三段的逐拍公式完全共用。r 为 jerk 斜坡占段长的比例(Q15，16384=三角加速度)，
 * peak=1/(1-r) 是归一化加速度峰值，inv_r=1/r。
 */
typedef struct {
    int32_t p0;        /* 段起点位置，厘度 */
    int32_t v0_qt;     /* 段起点速度 */
    int32_t dv_qt;     /* 段内速度增量 */
    int32_t inv_q30;   /* 1/段长，Q30，把 tt 换成归一化时间 u */
    int32_t posk;      /* (dv*段长)>>15，位置整形系数 */
    int32_t acck;      /* dv/段长 换算成厘度/秒^2 */
    int32_t r_q15;
    int32_t peak_q15;
    int32_t inv_r_q15;
} TrajSeg_t;

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

    /* 本段计划(全部在 C_Traj_Plan 中一次算好) */
    int32_t p0;          /* 段起点位置(未回绕)，厘度 */
    int32_t v0_qt;       /* 段起点速度，Q16 厘度/拍 */
    int32_t vc_qt;       /* 巡航速度，Q16 厘度/拍(情形B下为负) */
    int32_t p_a;         /* 加速段末位置(未回绕)，厘度 */
    int32_t p_c;         /* 匀速段末位置(未回绕)，厘度 */
    int32_t p_end;       /* 段终点位置(未回绕)，厘度 */
    TrajSeg_t seg[2];    /* [0]=加速段 [1]=减速段，Step 按当前拍直接取一个 */

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
#define Traj_Clamp Fx_ClampRange

/* 除法并饱和到 int32(向零取整)，本文件所有除法都走它；详见 docs/ALGORITHM_NOTES.md。 */
/* x 的有效位数，x=0 为0 */
static int32_t Traj_BitLen32(uint32_t x)
{
    return (x == 0U) ? 0 : 32 - __builtin_clz(x);
}

/*
 * 64位除以32位，要求 (n>>32) < d(商放得进32位)。只做商的有效位数那么多轮逐位试商：
 * 规划器里这类除法的商多是十几位，约两百条指令，库函数 __divdi3 则要五百条左右。
 */
static uint32_t Traj_DivU64(uint64_t n, uint32_t d)
{
    int32_t  k = ((n >> 32) ? 32 + Traj_BitLen32((uint32_t)(n >> 32)) : Traj_BitLen32((uint32_t)n))
               - Traj_BitLen32(d) + 1;              /* 商的位数上限 */
    uint32_t r, bits, carry, q = 0U;

    if (k <= 0) return 0U;
    if (k > 32) k = 32;                              /* (n>>32) < d 已保证商不超过32位 */
    r    = (uint32_t)(n >> k);                       /* 高位部分，必小于 d */
    bits = (k >= 32) ? (uint32_t)n : ((uint32_t)n << (32 - k)); /* 低 k 位左对齐，逐位移入 */
    while (k-- > 0)
    {
        carry = r >> 31;                             /* d >= 2^31 时左移会溢出，借这一位判大小 */
        r     = (r << 1) | (bits >> 31);
        bits <<= 1;
        q   <<= 1;
        if (carry || r >= d) { r -= d; q |= 1U; }
    }
    return q;
}

static int32_t Traj_DivSat(int64_t num, int64_t den)
{
    int64_t  q;
    uint64_t an = (num < 0) ? 0U - (uint64_t)num : (uint64_t)num;
    uint64_t ad = (den < 0) ? 0U - (uint64_t)den : (uint64_t)den;

    if (den == 0) return 0x7FFFFFFF;
    if ((ad >> 32) != 0U) q = num / den;             /* 除数超过32位：规划器里不出现，保留原路 */
    else
    {
        if ((an >> 32) >= ad) q = 0x80000000LL;      /* 商 >= 2^32，下面按符号饱和 */
        else if ((an >> 32) == 0U) q = (int64_t)((uint32_t)an / (uint32_t)ad);
        else q = (int64_t)Traj_DivU64(an, (uint32_t)ad);
        if ((num < 0) != (den < 0)) q = -q;
    }
    if (q >  0x7FFFFFFF) return  0x7FFFFFFF;
    if (q < -0x7FFFFFFF) return -0x7FFFFFFF;
    return (int32_t)q;
}

/* 按加速度上限 a 与加加速度上限 j 生成过渡段限幅。夹紧只此一处：调用方(C_Traj_Set_Limits
 * 和 C_Traj_Plan)算出来的 a/j 都直接送进来，合法区间在这里统一兜住 */
static void Traj_Lim_Make(TrajLim_t *l, int32_t a, int32_t j)
{
    int32_t tj;

    if (a < TRAJ_AMAX_MIN_CDPSS) a = TRAJ_AMAX_MIN_CDPSS;  /* 保证 1/a 的Q32放得下 */
    if (j < TRAJ_JMAX_MIN_CDPSS3) j = TRAJ_JMAX_MIN_CDPSS3;
    if (j > TRAJ_JMAX_MAX_CDPSS3) j = TRAJ_JMAX_MAX_CDPSS3;

    /* 恒加速度段 t=|dv|/a 的系数，Q32拍/(Q16厘度/拍)；用定长除法，__divdi3 太慢 */
    l->kl_q32 = Traj_DivSat((int64_t)CFG_TICK_HZ * CFG_TICK_HZ * 65536, a);
    /* a/J 秒 -> Q16拍 = a*1000*65536/J；限到 2^30 拍Q16(约16秒)，更长的斜坡没有意义 */
    tj = Traj_DivSat((int64_t)a * 65536 * CFG_TICK_HZ, j);
    if (tj > 0x40000000) tj = 0x40000000;
    l->tj_q16 = tj;
    /* a^2/J = (a/J)/(1/a)，两者都已是拍单位 */
    l->dvj_qt = Traj_DivSat((int64_t)l->tj_q16 << 16, l->kl_q32);
    /* 时长^2 = 4*dv/J：dv(Q16厘度/拍) 换成厘度/秒要 *1000/65536，秒^2换拍^2要 *1e6，再取Q8。
     * 这两项只随 J 变，而 J 只在改上限时才变：按 J 缓存，每次规划省两到四次64位除法 */
    {
        static int32_t  s_j;       /* 缓存对应的 J，0=还没算过 */
        static uint64_t s_cj, s_dv_sat;

        if (j != s_j)
        {
            s_cj     = 1024000000000ULL / (uint32_t)j;
            s_dv_sat = (TRAJ_SQRT_X_LIMIT << 16) / s_cj;
            s_j      = j;
        }
        l->cj_q16 = s_cj;
        l->dv_sat = s_dv_sat;
    }
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
        uint64_t x;

        if ((uint32_t)dv > l->dv_sat) x = TRAJ_SQRT_X_LIMIT;
        else x = ((uint64_t)(uint32_t)dv * l->cj_q16) >> 16;
        t = (int64_t)Traj_Sqrt64_Ceil(x) << 12;                       /* Q4拍 -> Q16拍 */
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
    if (t <= 0) return 16384;
    r = Traj_DivSat((int64_t)tj << 15, t);
    if (r < 1) r = 1;
    if (r > 16384) r = 16384;
    return r;
}

/* 形状 r 的两个倒数：peak=1/(1-r)，inv_r=1/r，均Q15 */
static void Traj_Seg_Shape(int32_t r, int32_t *peak, int32_t *inv_r)
{
    *peak  = Traj_DivSat(1LL << 30, 32768 - r);
    *inv_r = Traj_DivSat(1LL << 30, r);
}

/* Traj_Seg_Ticks 的反函数：t 拍内能完成的最大速度差(Q16厘度/拍)，用于把巡航速度夹回J允许的范围 */
static int32_t Traj_Seg_DvMax(const TrajLim_t *l, int32_t t)
{
    if (t <= 0) return 0;
    if (((int64_t)t << 16) >= 2 * (int64_t)l->tj_q16)
    {
        /* 梯形：dv/a + a/J <= t  =>  dv <= (t - a/J)*a */
        return Traj_DivSat(((((int64_t)t << 16) - l->tj_q16) << 16), l->kl_q32);
    }
    /* 三角形：2*sqrt(dv/J) <= t  =>  dv*cj>>16 <= (16t)^2 */
    return Traj_DivSat((int64_t)t * t << 24, (int64_t)l->cj_q16);
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

/* 设置轨迹上限(超出可设范围自动夹紧)，下一次 Plan 起生效；V/A/D/J为0时取本机硬件最大值。 */
void C_Traj_Set_Limits(int32_t vmax_cdps, int32_t amax_cdpss, int32_t dmax_cdpss, int32_t jmax_cdpss3)
{
    s_tj.vmax_cdps = Fx_ClampRange(vmax_cdps == 0 ? PLANT_VMAX_HW_CDPS : vmax_cdps,
                                   1, PLANT_VMAX_HW_CDPS);
#define LIMIT(value, lo, hi) Fx_ClampRange((value) <= 0 ? (hi) : (value), (lo), (hi))
    s_tj.amax_cdpss = LIMIT(amax_cdpss, TRAJ_AMAX_MIN_CDPSS, (int32_t)PLANT_AMAX_HW_DPS2 * 100);
    s_tj.dmax_cdpss = LIMIT(dmax_cdpss, TRAJ_AMAX_MIN_CDPSS, (int32_t)PLANT_DMAX_HW_DPS2 * 100);
    s_tj.jmax_cdpss3 = LIMIT(jmax_cdpss3, TRAJ_JMAX_MIN_CDPSS3, (int32_t)PLANT_JMAX_HW_DPS3 * 100);
#undef LIMIT
}

void C_Traj_SetPlant(int32_t ka_q12)
{
    if (ka_q12 <= 0) ka_q12 = 4096;
    s_a0_cdpss     = (int32_t)(((int64_t)PLANT_A0_CDPSS << 12) / ka_q12);
    s_dec_hw_cdpss = (int32_t)(((int64_t)TRAJ_DEC_HW_CDPSS << 12) / ka_q12);
    if (s_dec_hw_cdpss < 1) s_dec_hw_cdpss = 1;
}

void C_Traj_Set_MoveMinMs(uint16_t move_min_ms)
{
    s_tj.move_min_ms = (move_min_ms <= 200U) ? move_min_ms : 200U;
}

/* 初始化并停在 init_pos，轨迹上限取调参面板 */
void C_Traj_Init(int32_t init_pos)
{
    C_Traj_Set_Limits((int32_t)CFG_DEF_VMAX_DPS * 100, (int32_t)CFG_DEF_AMAX_DPS2 * 100,
                      (int32_t)CFG_DEF_DMAX_DPS2 * 100, (int32_t)CFG_DEF_JMAX_DPS3 * 100);
    C_Traj_Set_MoveMinMs(CFG_DEF_MOVE_MIN_MS);
    C_Traj_SetPlant(4096);

    /* 先放开到满量程，A_Servo 随后按模式收紧 */
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
    s_tj.seg[0].r_q15 = s_tj.seg[1].r_q15 = 16384;
    s_tj.seg[0].peak_q15 = s_tj.seg[1].peak_q15 = 65536;
    s_tj.seg[0].inv_r_q15 = s_tj.seg[1].inv_r_q15 = 65536;

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

int32_t C_Traj_Get_Pos(void) { return s_tj.pos; }

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
    s_tj.v0_qt = s_tj.vc_qt = 0;
    s_tj.seg[0].dv_qt = s_tj.seg[1].dv_qt = 0;
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
 * C_Traj_Plan 的工作区。一律用"运动方向为正"的归一化坐标，最后由 Traj_Commit 换回全局符号。
 * 位置厘度，速度 Q16 厘度/拍，时长为整数拍。
 */
typedef struct {
    int32_t   D;          /* 本段行程，>=0 */
    int32_t   u0;         /* 起点速度沿运动方向的分量，反向时为负 */
    int32_t   vmax;       /* 本段速度上限 */
    TrajLim_t lim1;       /* 首段限幅(加速/刹车/换向) */
    TrajLim_t lim3;       /* 减速段限幅 */
    int32_t   t1, tc, t3; /* 加速段 / 匀速段 / 减速段时长 */
    int32_t   T;          /* 总时长 */
    int32_t   vc;         /* 巡航速度 */
    uint8_t   case_b;     /* 1=刹不住了，只能冲过目标再倒回来 */
    uint8_t   stretched;  /* 1=按请求时间拉长过 */
} TrajPlan_t;

/* 本段的速度/加速度上限：用户设定、硬件能力、小位移整形三者取最小；详见 docs/ALGORITHM_NOTES.md。 */
static void Traj_Limits(TrajPlan_t *p)
{
    TrajLim_t lim_acc;
    int32_t vmax_use = s_tj.vmax_cdps;
    int32_t au0 = (p->u0 < 0) ? -p->u0 : p->u0;
    int32_t v0_cdps = (int32_t)(((int64_t)au0 * CFG_TICK_HZ) >> TRAJ_VEL_SHIFT);
    int32_t amax_a, amax_d, amax_r, a_ceil;

    if (s_tj.move_min_ms > 0U)
    {
        int32_t v_cap = Traj_DivSat((int64_t)p->D * 1000, s_tj.move_min_ms);

        if (v_cap < 1) v_cap = 1;
        if (vmax_use > v_cap) vmax_use = v_cap;
    }
    /* 运动中把目标改到眼前时仍须保留消除已有动量的能力，否则会生成极慢的错误轨迹 */
    if (vmax_use < v0_cdps) vmax_use = v0_cdps;
    if (vmax_use < 1) vmax_use = 1;
    p->vmax = Traj_DivSat((int64_t)vmax_use << TRAJ_VEL_SHIFT, CFG_TICK_HZ);
    if (p->vmax < au0) p->vmax = au0;   /* 上面的厘度/秒截断不能把起点速度判成超速 */

    a_ceil = Traj_DivSat((int64_t)vmax_use * 1000, TUNE_ACCEL_MIN_MS);
    if (a_ceil < 1) a_ceil = 1;

    amax_a = Traj_AutoAmax(p->D, vmax_use, v0_cdps);
    if (amax_a > s_tj.amax_cdpss) amax_a = s_tj.amax_cdpss;
    if (amax_a > a_ceil) amax_a = a_ceil;
    amax_d = s_tj.dmax_cdpss;
    if (amax_d > s_dec_hw_cdpss) amax_d = s_dec_hw_cdpss;   /* 制动侧硬件天花板 */
    /* 减速段同样套 a_ceil：略增过冲，但到位瞬间的PWM跳变小得多，治机械震动 */
    if (amax_d > a_ceil) amax_d = a_ceil;

    Traj_Lim_Make(&lim_acc, amax_a, s_tj.jmax_cdpss3);
    Traj_Lim_Make(&p->lim3, amax_d, s_tj.jmax_cdpss3);

    /* 正向运动而直接刹车的位移已超过剩余行程 -> 必然过冲，只能冲过去再倒回来 */
    p->case_b = (uint8_t)(p->u0 > 0
                          && ((int64_t)p->D << 17) < (int64_t)p->u0
                                                   * Traj_Seg_Ticks(&p->lim3, p->u0));

    /* 首段用哪套限幅：正向加速用加速上限；情形B和换向要先把已有速度刹掉，取两侧较弱的。
     * 带时间指令降到更低的巡航速度时首段其实也是在减速，那种情况由 Traj_FitCruise 改用减速上限 */
    amax_r = (amax_a < amax_d) ? amax_a : amax_d;
    if (p->case_b || p->u0 < 0) p->lim1 = (amax_r == amax_d) ? p->lim3 : lim_acc;
    else                        p->lim1 = lim_acc;
}

/* 第1步：不管请求时间，先求最速轨迹的三段时长 */
static void Traj_Fastest(TrajPlan_t *p)
{
    int32_t vp, d_ramp;

    if (p->case_b)
    {
        /* 冲过目标再倒回来：vp<0，且 |vp|<=|u0|<=vmax，永远不需要匀速段 */
        vp = Traj_Solve_Vp(&p->lim1, &p->lim3, p->u0, p->D, -p->vmax, 0);
    }
    else
    {
        /* 先试 vmax：过渡位移放得下就加匀速段，否则求三角轨迹峰值。 */
        p->t1 = Traj_Seg_Ticks(&p->lim1, p->vmax - p->u0);
        p->t3 = Traj_Seg_Ticks(&p->lim3, p->vmax);
        d_ramp = (int32_t)(((int64_t)(p->u0 + p->vmax) * p->t1
                           + (int64_t)p->vmax * p->t3) >> (TRAJ_VEL_SHIFT + 1));
        if (d_ramp <= p->D)
        {
            uint32_t rest = (uint32_t)(p->D - d_ramp) << TRAJ_VEL_SHIFT;
            p->tc = (int32_t)((rest + (uint32_t)p->vmax - 1U) / (uint32_t)p->vmax);
            return;
        }
        vp = Traj_Solve_Vp(&p->lim1, &p->lim3, p->u0, p->D,
                           p->u0 > 0 ? p->u0 : 0, p->vmax);
    }
    p->t1 = Traj_Seg_Ticks(&p->lim1, Fx_Abs32(vp - p->u0));
    p->t3 = Traj_Seg_Ticks(&p->lim3, Fx_Abs32(vp));
    p->tc = 0;
}

/* 第2步：请求时间比最速还长 -> 三段同乘 lambda(速度/lambda、加速度/lambda^2、加加速度/lambda^3)；详见 docs/ALGORITHM_NOTES.md。 */
static void Traj_Stretch(TrajPlan_t *p, int32_t T_req)
{
    int32_t T_fast = p->t1 + p->tc + p->t3;
    uint32_t lam_q8;

    p->T = T_fast;
    p->stretched = (uint8_t)(T_req > T_fast);
    if (!p->stretched) return;

    lam_q8 = ((uint32_t)T_req << 8) / (uint32_t)T_fast;
    if (!p->case_b && p->u0 >= 0)
        p->t1 = (int32_t)(((uint32_t)p->t1 * lam_q8) >> 8);
    else
        p->t1 = Traj_Seg_Ticks(&p->lim1, (p->u0 < 0) ? -p->u0 : p->u0);
    p->t3 = (int32_t)(((uint32_t)p->t3 * lam_q8) >> 8);
    p->T  = T_req;
    if (p->t1 < 1) p->t1 = 1;
    if (p->t1 + p->t3 > p->T) p->t3 = p->T - p->t1;  /* 加速段不缩放时减速段可能挤不下 */
    if (p->t3 < 1) p->t3 = 1;
}

/* 由位移方程反解巡航速度：D = (u0+vc)*t1/2 + vc*tc + vc*t3/2 => vc = (2D - u0*t1)/(2T - t1 - t3) */
static int32_t Traj_SolveVc(const TrajPlan_t *p)
{
    return Traj_DivSat(((int64_t)p->D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)p->u0 * p->t1,
                       2 * p->T - p->t1 - p->t3);
}

/* 把 vc 夹回"速度上限"与"两段过渡在现有时长内真做得到"的范围，A/J 上限由此永远成立 */
static void Traj_ClampVc(TrajPlan_t *p)
{
    int64_t w1 = Traj_Seg_DvMax(&p->lim1, p->t1);
    int64_t w3 = Traj_Seg_DvMax(&p->lim3, p->t3);

    if ((int64_t)p->vc > p->u0 + w1) p->vc = (int32_t)(p->u0 + w1);
    if ((int64_t)p->vc < p->u0 - w1) p->vc = (int32_t)(p->u0 - w1);
    if ((int64_t)p->vc >  w3) p->vc = (int32_t)w3;
    if ((int64_t)p->vc < -w3) p->vc = (int32_t)-w3;
    if (p->vc >  p->vmax) p->vc =  p->vmax;
    if (p->vc < -p->vmax) p->vc = -p->vmax;
}

/*
 * 第3步：整数拍取整后位移会差一点，用位移方程反解巡航速度补回来。一次解到位，不迭代。
 * 解完保证两段过渡放得下这个速度差(不够就加长并重解一次)，再把 vc 夹回真做得到的范围，
 * 最后由匀速段长度把位移补齐——位置永远精确落在 D 上，只有总时长会与请求值差几十毫秒。
 */
static void Traj_FitCruise(TrajPlan_t *p)
{
    int32_t n1, n3;

    if (p->T < p->t1 + p->t3) p->T = p->t1 + p->t3;
    p->vc = Traj_SolveVc(p);

    /* 朝目标运动却解出 vc < u0，两种成因分别处理 */
    if (!p->case_b && p->u0 > 0 && p->vc < p->u0)
    {
        if (!p->stretched)
        {
            /* 最速轨迹只是被整数拍取整压低了。这时加长首段会在J受限下越解越慢、变成两级刹车，
             * 改为"以u0滑行一段再刹车"：首段吃满刹车前的全部时间，不足1拍的行程由终点吸收。 */
            int32_t pre;

            p->t3 = Traj_Seg_Ticks(&p->lim3, p->u0);
            pre   = Traj_DivSat((((int64_t)p->D << (TRAJ_VEL_SHIFT + 1))
                                 - (int64_t)p->u0 * p->t3) + p->u0,
                                2 * p->u0);   /* 刹车前以u0走的拍数，四舍五入 */
            p->t1 = (pre < 1) ? 1 : pre;
            p->T  = p->t1 + p->t3;
            p->vc = Traj_SolveVc(p);
            /* 这一支已经自洽(以u0滑行到底再按减速上限刹车)，就此定稿。
             * 若继续往下走复解，vc 会被拉离 u0，首段就变成"用加速上限去刹车"——
             * 加速上限远高于减速上限时，减速段的峰值加速度会数倍于用户设的 DMAX。 */
            Traj_ClampVc(p);
            p->tc = 0;
            p->T  = p->t1 + p->t3;
            return;
        }
        else
        {
            /* 带时间指令要降到更低的巡航速度：首段其实是在减速，改用减速上限，否则会越过 DMAX */
            p->lim1 = p->lim3;
            p->t1   = Traj_Seg_Ticks(&p->lim1, p->u0 - p->vc);
            if (p->T < p->t1 + p->t3) p->T = p->t1 + p->t3;
            p->vc = Traj_SolveVc(p);
        }
    }

    /* 过渡段必须放得下这个速度差，不够就加长：A 和 J 的上限任何情况下都不许破。
     * 段变长了 vc 就不再自洽，重解一次(只此一次，剩下的偏差交给匀速段) */
    n1 = Traj_Seg_Ticks(&p->lim1, (p->vc > p->u0) ? (p->vc - p->u0) : (p->u0 - p->vc));
    n3 = Traj_Seg_Ticks(&p->lim3, (p->vc < 0) ? -p->vc : p->vc);
    if (n1 > p->t1 || n3 > p->t3)
    {
        if (n1 > p->t1) p->t1 = n1;
        if (n3 > p->t3) p->t3 = n3;
        if (p->T < p->t1 + p->t3) p->T = p->t1 + p->t3;
        p->vc = Traj_SolveVc(p);
    }

    Traj_ClampVc(p);

    /* Traj_Seg_DvMax 是 Traj_Seg_Ticks 的反函数，但后者向上取整到整拍，两者在边界上会差一拍：
     * 夹完的 vc 有可能仍需要 t+1 拍。段长只增不减是安全的(A 与 J 只会更小)，位移差由匀速段吸收，
     * 所以这里按最终 vc 再兜一次底。不补这一下，高 A 高 J 下 1~2 拍的过渡段峰值加速度会超限。 */
    {
        int32_t m1 = Traj_Seg_Ticks(&p->lim1, (p->vc > p->u0) ? (p->vc - p->u0) : (p->u0 - p->vc));
        int32_t m3 = Traj_Seg_Ticks(&p->lim3, (p->vc < 0) ? -p->vc : p->vc);

        if (m1 > p->t1) p->t1 = m1;
        if (m3 > p->t3) p->t3 = m3;
    }

    /* 匀速段吸收剩下的位移：由 vc*(2T-t1-t3) = 2D-u0*t1 反推总时长，再减掉两段过渡。
     * vc 被上面夹小了就自动多走一会儿匀速，位移仍然精确。 */
    if (p->vc == 0) p->tc = 0;
    else
    {
        p->tc = (Traj_DivSat(((int64_t)p->D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)p->u0 * p->t1,
                             p->vc) - p->t1 - p->t3) / 2;
        if (p->tc < 0) p->tc = 0;
    }
    p->T = p->t1 + p->tc + p->t3;
}

/* 把一个过渡段的每拍常量一次算好 */
static void Traj_Seg_Make(TrajSeg_t *g, int32_t p0, int32_t v0_qt, int32_t dv_qt,
                          int32_t ticks, const TrajLim_t *lim, int32_t dv_shape)
{
    g->p0      = p0;
    g->v0_qt   = v0_qt;
    g->dv_qt   = dv_qt;
    g->inv_q30 = Traj_DivSat(1LL << TRAJ_INV_SHIFT, ticks);
    g->posk    = (int32_t)(((int64_t)dv_qt * ticks) >> TRAJ_U_SHIFT);
    g->acck    = Traj_Acc_Coef(dv_qt, g->inv_q30);
    /* 形状按本段速度差取 J 受限最短过渡的形状；段比最短时长长时沿用，加速度与加加速度只低不高 */
    g->r_q15   = Traj_Seg_R(lim, dv_shape);
    Traj_Seg_Shape(g->r_q15, &g->peak_q15, &g->inv_r_q15);
}

/* 换回全局符号，把每拍 Step 要用的常量一次算好(Step 里只剩乘加移位) */
static void Traj_Commit(const TrajPlan_t *p, int32_t sgn, int32_t dp)
{
    int32_t vc_g = p->vc * sgn;          /* 全局符号下的巡航速度 */
    int32_t dv1  = vc_g - s_tj.v0_qt;
    int32_t dv3  = -vc_g;

    s_tj.vc_qt = vc_g;
    s_tj.t1 = p->t1;
    s_tj.tc = p->tc;
    s_tj.t3 = p->t3;
    s_tj.T  = p->T;
    s_tj.t  = 0;

    Traj_Seg_Make(&s_tj.seg[0], s_tj.p0, s_tj.v0_qt, dv1, p->t1, &p->lim1,
                  (p->vc > p->u0) ? (p->vc - p->u0) : (p->u0 - p->vc));

    /* 段末位置与逐拍公式同式计算，接缝处无舍入差 */
    s_tj.p_a = s_tj.p0
             + (int32_t)((((int64_t)s_tj.v0_qt * p->t1)
                        + (int64_t)s_tj.seg[0].posk * (1L << (TRAJ_U_SHIFT - 1)))
                         >> TRAJ_VEL_SHIFT);
    s_tj.p_c = s_tj.p_a + (int32_t)(((int64_t)vc_g * p->tc) >> TRAJ_VEL_SHIFT);
    s_tj.p_end = s_tj.p0 + dp;           /* 终点用解析值，避免逐段舍入累积 */

    Traj_Seg_Make(&s_tj.seg[1], s_tj.p_c, vc_g, dv3, p->t3, &p->lim3,
                  (p->vc < 0) ? -p->vc : p->vc);
}

/* 以当前参考状态为起点规划到 target_pos，time_ms=0 为最快。全部除法与开方只在这里，每拍 Step 无除法；详见 docs/ALGORITHM_NOTES.md。 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms)
{
    TrajPlan_t p;
    int32_t dp, sgn;

    /* 新段起点取规划器自身状态，位置/速度连续 */
    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = s_tj.vel_qt;

    /* 目标夹进可用行程，位移直接相减，不穿过死区 */
    target_pos = Traj_Clamp(target_pos, s_tj.lo, s_tj.hi);
    dp   = target_pos - s_tj.p0;
    sgn  = (dp >= 0) ? 1 : -1;
    p.D  = dp * sgn;                 /* 运动方向上的行程，>=0 */
    p.u0 = s_tj.v0_qt * sgn;         /* 起点速度沿运动方向的分量，反向时为负 */

    if (p.D == 0 && p.u0 == 0)
    {
        C_Traj_Hold(s_tj.p0);
        return;                      /* 已静止且同目标，不必做限幅换算 */
    }

    Traj_Limits(&p);
    Traj_Fastest(&p);
    Traj_Stretch(&p, ((int32_t)time_ms * CFG_TICK_HZ) / 1000);
    Traj_FitCruise(&p);
    Traj_Commit(&p, sgn, dp);
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
    const TrajSeg_t *g;
    int32_t tt, u_q15, s_q15, p_q15, a_q15, pos, acc;
    uint8_t decel;

    /* 饱和保持：输出已饱和且落后时冻结轨迹时钟，参考自动退化为"硬件能做到的最快"，
     * 不依赖任何标定精度。只在加速/匀速段冻结：减速段冻结会把 ref.vel 钉在高位、帮着冲过头。 */
    if (s_tj.t < s_tj.T && (!hold || s_tj.t >= s_tj.t1 + s_tj.tc)) s_tj.t++;

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

    if (s_tj.t > s_tj.t1 && s_tj.t <= s_tj.t1 + s_tj.tc)
    {
        /* 匀速段：无过渡整形，1次乘法 */
        tt = s_tj.t - s_tj.t1;
        s_tj.vel_qt = s_tj.vc_qt;
        Traj_Emit(out, s_tj.p_a + (int32_t)(((int64_t)s_tj.vc_qt * tt) >> TRAJ_VEL_SHIFT), 0);
        return;
    }

    decel = (uint8_t)(s_tj.t > s_tj.t1 + s_tj.tc);
    g     = &s_tj.seg[decel];
    tt    = decel ? (s_tj.t - s_tj.t1 - s_tj.tc) : s_tj.t;

    /* u = tt/段长，用预存的Q30倒数换成乘法。tt<=段长 => 乘积<=2^30，不溢出。 */
    u_q15 = (int32_t)(((uint32_t)tt * (uint32_t)g->inv_q30) >> TRAJ_U_SHIFT);
    if (u_q15 > (1 << TRAJ_U_SHIFT)) u_q15 = (1 << TRAJ_U_SHIFT); /* 舍入兜底 */

    Traj_SCurve(u_q15, g->r_q15, g->peak_q15, g->inv_r_q15, &s_q15, &p_q15, &a_q15);
    acc = (int32_t)(((int64_t)g->acck * a_q15) >> TRAJ_U_SHIFT);
    s_tj.vel_qt = g->v0_qt + (int32_t)(((int64_t)g->dv_qt * s_q15) >> TRAJ_U_SHIFT);
    pos = g->p0 + (int32_t)((((int64_t)g->v0_qt * tt)
                           + ((int64_t)g->posk * p_q15)) >> TRAJ_VEL_SHIFT);

    /* 减速段速度单调收敛不换向，按vc方向单侧钳位，防舍入越过终点 */
    if (decel)
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
