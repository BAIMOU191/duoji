/* C_Traj_Planner.c 在线轨迹规划器(运动学限幅型)，无硬件依赖 */

#include "C_Traj_Planner.h"

#define TRAJ_VEL_SHIFT   16U  /* Q16：内部速度(厘度/拍)定点小数位数 */
#define TRAJ_U_SHIFT     15U  /* Q15：段内归一化时间u及其幂次的定点小数位数 */
#define TRAJ_INV_SHIFT   30U  /* Q30：1/段长 的定点小数位数 */
#define TRAJ_W_SHIFT      8U  /* 解二次方程时把Q16速度降到Q8，防止平方溢出 */

/* 恒加速度段：t=|dv|/amax。 */
static int32_t Traj_K_Linear_Q32(int32_t amax_cdpss)
{
    return (int32_t)(((int64_t)CFG_TICK_HZ * CFG_TICK_HZ * 65536) /
                     amax_cdpss);
}

/*
 * 加速度上限自动推导：维持速度v要吃掉 v/Kv 的PWM，可用加速度 a(v) = a0*(1 - v/v_ss)。
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

/* 本段硬件能给出的加速度上限(厘度/秒^2)；短行程爬不到高速区，自动得到更大的加速度 */
static int32_t Traj_AutoAmax(int32_t D, int32_t shape_q15, int32_t vmax, int32_t v0_cdps)
{
    int64_t denom;
    int32_t h_q12, f_q15, g_q15, f_cap, f_now;

    if (PLANT_VSS_CDPS <= 0 || PLANT_TAU_MS == 0U
        || PLANT_A0_CDPSS <= 0)
        return TRAJ_AMAX_ACC_CDPSS;      /* 未标定：退回固定值 */

    /* H = D*1000/(shape*tau_ms*v_ss)，Q12；shape 为S曲线段时长倍率 1..2 */
    denom = ((int64_t)shape_q15 * PLANT_TAU_MS
             * PLANT_VSS_CDPS) >> 15;
    if (denom <= 0) return TRAJ_AMAX_ACC_CDPSS;
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

    /* 这里只给硬件极限，"想要多柔"由平滑度的段时长倍率负责，不在两处重复压低 */
    return (int32_t)(((int64_t)PLANT_A0_CDPSS * g_q15) >> 15);
}

/* 位移相关的速度上限 v = D/T_move：小位移等时移动、不甩出高峰速；只限速不限加速度，大位移时自动失效 */
static int32_t Traj_VelCap(int32_t D)
{
    int32_t t = TRAJ_MOVE_MIN_MS;
    int32_t v;

    if (t <= 0) return TRAJ_VMAX_CDPS;          /* 0 = 关闭本机制 */
    if (D <= 0) return TRAJ_VMAX_CDPS;

    v = (int32_t)(((int64_t)D * 1000) / t);     /* 厘度/毫秒 -> 厘度/秒 */
    if (v > TRAJ_VMAX_CDPS) v = TRAJ_VMAX_CDPS;
    return (v > 0) ? v : 1;
}

/* 由速度上限推出的加速度上限：加速段至少持续 TRAJ_ACCEL_MIN_MS(比纯延迟短的加速跟不上)；大位移时自动失效 */
static int32_t Traj_AccelCeil(int32_t v_cap)
{
    int32_t a = (int32_t)(((int64_t)v_cap * 1000) / TRAJ_ACCEL_MIN_MS);

    return (a > 0) ? a : 1;
}

/* 规划器内部状态，单实例 */
static struct {
    /* 指令状态：重规划时作为新段起点，保证位置/速度连续 */
    int32_t pos;         /* 当前参考位置，厘度 */
    int32_t vel_qt;      /* 当前参考速度，Q16 厘度/拍 */

    /* 平滑度换算结果，Init 时算一次；r 为过渡段jerk斜坡占比，peak=1/(1-r) */
    int32_t k_acc_q32;   /* 加速段：过渡时长/速度差 */
    int32_t k_dec_q32;   /* 减速段：过渡时长/速度差 */
    int32_t k_rev_q32;   /* 首段需先刹车/换向时用，取较小加速度同时满足两侧上限 */
    int32_t r_acc_q15;
    int32_t r_dec_q15;
    int32_t peak_acc_q15;    /* 1/(1-r)，也是段时长倍率，Q15 */
    int32_t peak_dec_q15;
    int32_t inv_r_acc_q15;   /* 1/r，Q15；r=0时为0 */
    int32_t inv_r_dec_q15;

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
    int32_t acck1;       /* 加速段 dv1/t1 换算成厘度/秒^2，配 6u(1-u) 用 */
    int32_t acck3;       /* 减速段同上 */
    int32_t dv1_qt;      /* 加速段速度增量 vc-v0 */
    int32_t dv3_qt;      /* 减速段速度增量 0-vc  */
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
    uint32_t i;

    for (i = 0; i < 16U; i++)
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

/* 跨越速度差 dv_qt 的过渡段时长(拍)，向上取整且至少1拍；只与速度差成正比 */
static int32_t Traj_Seg_Ticks(int32_t k_q32, int32_t dv_qt)
{
    int32_t t = (int32_t)((((int64_t)k_q32 * dv_qt) + 0xFFFFFFFFLL) >> 32);
    return (t < 1) ? 1 : t;
}

/*
 * 带时间指令中途换向的刹车k(越大越柔)：按时间缩放同比放缓(a/lambda^2)，避免加速度前馈1拍跳变上千PWM；
 * 但参考反向滑出不超过 D/TUNE_REV_SLIDE_DIV，高速反向时自动回到硬件极限。情形B不受影响。
 */
static int32_t Traj_ReverseK(int32_t k_hw, uint32_t lam_q8, int32_t W0, int32_t D)
{
    int64_t k_soft, k_cap;

    if (TUNE_REV_SLIDE_DIV <= 0 || W0 <= 0) return k_hw;
    k_soft = ((int64_t)k_hw * lam_q8 * lam_q8) >> 16;
    k_cap  = (((int64_t)D / TUNE_REV_SLIDE_DIV) << 33) / ((int64_t)W0 * W0); /* 刹车距离=k*W0^2/2^33 */
    if (k_soft > k_cap) k_soft = k_cap;
    if (k_soft > 0x7FFFFFFFLL) k_soft = 0x7FFFFFFFLL;
    return (k_soft > k_hw) ? (int32_t)k_soft : k_hw;
}

/* 初始化并停在 init_pos，设置加/减速平滑度(超上限自动夹紧) */
void C_Traj_Init(int32_t init_pos, uint8_t accel_smooth, uint8_t decel_smooth)
{
    int32_t max = TRAJ_SMOOTH_MAX;
    int32_t sa = (accel_smooth > max) ? max : (int32_t)accel_smooth;
    int32_t sd = (decel_smooth > max) ? max : (int32_t)decel_smooth;

    /* 平滑度 -> jerk受限S曲线：r=0.5*s/max，s=0 恒加速度最快，s=max 三角加速度最柔；段时长倍率 peak=1/(1-r) */
    if (max <= 0) max = 1;
    s_tj.r_acc_q15 = (int32_t)(((int64_t)sa << 14) / max);
    s_tj.r_dec_q15 = (int32_t)(((int64_t)sd << 14) / max);
    /* 换向段形状只跟随 r_acc，不取 max：停车平滑度不应拖慢在线换向 */

    s_tj.peak_acc_q15 = (int32_t)(((int64_t)32768 * 32768)
                                   / (32768 - s_tj.r_acc_q15));
    s_tj.peak_dec_q15 = (int32_t)(((int64_t)32768 * 32768)
                                   / (32768 - s_tj.r_dec_q15));
    s_tj.inv_r_acc_q15 = (s_tj.r_acc_q15 > 0)
                       ? (int32_t)(((int64_t)32768 * 32768) / s_tj.r_acc_q15) : 0;
    s_tj.inv_r_dec_q15 = (s_tj.r_dec_q15 > 0)
                       ? (int32_t)(((int64_t)32768 * 32768) / s_tj.r_dec_q15) : 0;

    /* k 仅由 Plan 计算和读取。T=0 时 Step 直接输出静止参考，无需预算。 */

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
    s_tj.seg1_r_q15 = s_tj.r_acc_q15;
    s_tj.seg3_r_q15 = s_tj.r_dec_q15;
    s_tj.seg1_peak_q15 = s_tj.peak_acc_q15;
    s_tj.seg3_peak_q15 = s_tj.peak_dec_q15;
    s_tj.seg1_inv_r_q15 = s_tj.inv_r_acc_q15;
    s_tj.seg3_inv_r_q15 = s_tj.inv_r_dec_q15;

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

/* 把段的 dv/t_seg 换算成厘度/秒^2：恒加速度段直接输出，jerk受限段再乘 A(u) */
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
 *   1) 解最速轨迹得 t1/tc/t3 与 T_fast；
 *   2) 请求时间更长时按 lambda=T_req/T_fast 拉伸；
 *   3) 由位移方程反解巡航速度 vc 保证准时到位，并复核速度/加速度上限，最多4轮。
 * 全部除法集中在这里，每拍 Step 无除法。
 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms)
{
    int32_t dp, sgn, D, u0, au0, W0;
    int32_t k1, k3;
    int32_t t1, tc, t3, T, T_fast, T_req;
    int32_t vp, vc, vmax_qt;
    int32_t n1, n3, i;
    int64_t num, numv;
    uint32_t lam_q8;
    uint8_t  case_b;

    /* 新段起点取规划器自身状态，位置/速度连续 */
    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = s_tj.vel_qt;

    /* 目标夹进可用行程，位移直接相减，不穿过死区 */
    target_pos = Traj_Clamp(target_pos, s_tj.lo, s_tj.hi);
    dp  = target_pos - s_tj.p0;
    sgn = (dp >= 0) ? 1 : -1;
    D   = dp * sgn;                 /* 运动方向上的行程，>=0 */
    u0  = s_tj.v0_qt * sgn;         /* 起点速度沿运动方向的分量，反向时为负 */

    if (D == 0 && u0 == 0)
    {
        C_Traj_Hold(s_tj.p0);
        return; /* 已静止且同目标，不再做速度/加速度上限的除法计算 */
    }

    /* 本段动力学上限：小位移限速(见 Traj_VelCap)，加速度上限按峰值速度推导；只在收到指令时算一次 */
    {
        int32_t vmax_use = Traj_VelCap(D);
        int32_t a_ceil;
        int32_t amax_a, amax_d, kl_a, kl_d, v0_cdps;

        /* 起点速度换算成厘度/秒，供加速度上限推导用(见 Traj_AutoAmax) */
        v0_cdps = (int32_t)((((int64_t)((u0 < 0) ? -u0 : u0)) * CFG_TICK_HZ)
                            >> TRAJ_VEL_SHIFT);
        /* 运动中把目标改到眼前时仍须保留消除已有动量的能力，否则会生成极慢的错误轨迹 */
        if (vmax_use < v0_cdps) vmax_use = v0_cdps;
        if (vmax_use < 1) vmax_use = 1;
        vmax_qt = (int32_t)(((int64_t)vmax_use << TRAJ_VEL_SHIFT) / CFG_TICK_HZ);
        a_ceil = Traj_AccelCeil(vmax_use);

        amax_a = Traj_AutoAmax(D, s_tj.peak_acc_q15, vmax_use, v0_cdps);
        if (amax_a > a_ceil) amax_a = a_ceil;
        if (amax_a < 1) amax_a = 1;

        /* 减速侧基准取配置的减速上限 */
        amax_d = TRAJ_AMAX_DEC_CDPSS;

        /* 减速段同样套 a_ceil(至少持续 TRAJ_ACCEL_MIN_MS)：略增过冲，但到位瞬间PWM跳变小约7倍，治机械震动 */
        if (amax_d > a_ceil) amax_d = a_ceil;
        if (amax_d < 1) amax_d = 1;

        /* 标准 jerk 受限过渡的段时长：k = peak/a，peak=1/(1-r)。 */
        kl_a = Traj_K_Linear_Q32(amax_a);
        kl_d = Traj_K_Linear_Q32(amax_d);
        s_tj.k_acc_q32 = (int32_t)(((int64_t)kl_a * s_tj.peak_acc_q15) >> 15);
        s_tj.k_dec_q32 = (int32_t)(((int64_t)kl_d * s_tj.peak_dec_q15) >> 15);
        /* 换向段取两侧较弱的物理加速度，拉长倍率用 peak_acc(不跟停车平滑度走) */
        {
            int32_t kl_r = (kl_a > kl_d) ? kl_a : kl_d;
            s_tj.k_rev_q32 = (int32_t)(((int64_t)kl_r * s_tj.peak_acc_q15) >> 15);
        }
    }

    T_req = ((int32_t)time_ms * CFG_TICK_HZ) / 1000;

    au0 = (u0 < 0) ? -u0 : u0;
    W0  = au0 >> TRAJ_W_SHIFT;      /* Q8，用于平方时不溢出 */
    k3  = s_tj.k_dec_q32;           /* 末段恒为"减速到0"，永远用减速上限 */

    /* 情形B：正向运动且刹车距离 k3*u0^2/2 已超过剩余行程 -> 必然过冲；同乘2^33做整数比较 */
    case_b = 0;
    if (u0 > 0)
    {
        if (((int64_t)D << 33) < ((int64_t)k3 * W0 * W0)) case_b = 1;
    }
    /* 首段需刹车或换向时取更保守的k */
    k1 = (case_b || u0 < 0) ? s_tj.k_rev_q32 : s_tj.k_acc_q32;
    s_tj.seg1_r_q15 = s_tj.r_acc_q15;
    s_tj.seg1_peak_q15 = s_tj.peak_acc_q15;
    s_tj.seg1_inv_r_q15 = s_tj.inv_r_acc_q15;
    s_tj.seg3_r_q15 = s_tj.r_dec_q15;
    s_tj.seg3_peak_q15 = s_tj.peak_dec_q15;
    s_tj.seg3_inv_r_q15 = s_tj.inv_r_dec_q15;

    /* 第1步：最速轨迹 */
    if (case_b)
    {
        /* 冲过目标再倒回来：vp<0，且 |vp|<=|u0|<=vmax，永远不需要匀速段 */
        num = (int64_t)k1 * W0 * W0 - ((int64_t)D << 33);
        if (num < 0) num = 0;                       /* 纯防御，判据已保证>0 */
        vp = -(int32_t)(Traj_Sqrt_Ceil(Traj_Div_U64((uint64_t)num,
                                       (uint32_t)(k1 + k3))) << TRAJ_W_SHIFT);
        if (vp < -vmax_qt) vp = -vmax_qt;
        t1 = Traj_Seg_Ticks(k1, u0 - vp);
        t3 = Traj_Seg_Ticks(k3, -vp);
        tc = 0;
    }
    else
    {
        /* 先按 vp=vmax 试梯形：两段过渡位移放得下就是梯形，否则是三角形 */
        int32_t d_ramp;
        t1 = Traj_Seg_Ticks(k1, vmax_qt - u0);
        t3 = Traj_Seg_Ticks(k3, vmax_qt);
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
            /* 三角形：vp = sqrt((2D + k1*u0^2)/(k1+k3))，闭式解，一次开方 */
            num = ((int64_t)D << 33) + (int64_t)k1 * W0 * W0;
            vp  = (int32_t)(Traj_Sqrt_Ceil(Traj_Div_U64((uint64_t)num,
                                           (uint32_t)(k1 + k3))) << TRAJ_W_SHIFT);
            if (vp > vmax_qt) vp = vmax_qt;
            t1 = Traj_Seg_Ticks(k1, (vp > u0) ? (vp - u0) : (u0 - vp));
            t3 = Traj_Seg_Ticks(k3, vp);
            tc = 0;
        }
    }

    T_fast = t1 + tc + t3;

    /* 第2步：请求时间更长 -> 三段同乘 lambda，速度/lambda、加速度/lambda^2。
     * 首段在抵消已有速度时(情形B或u0<0)不按lambda拉长(高速反向会冲出很远)，取抵消u0的时长为种子、
     * 换向时由 Traj_ReverseK 适度放缓，再由第3步长到够用，余下时间给匀速段。 */
    T = T_fast;
    if (T_req > T_fast)
    {
        lam_q8 = ((uint32_t)T_req << 8) / (uint32_t)T_fast;
        if (!case_b && u0 >= 0) t1 = (int32_t)(((uint32_t)t1 * lam_q8) >> 8);
        else
        {
            k1 = Traj_ReverseK(k1, lam_q8, W0, D);
            t1 = Traj_Seg_Ticks(k1, au0);
        }
        t3 = (int32_t)(((uint32_t)t3 * lam_q8) >> 8);
        T  = T_req;
        if (t1 < 1) t1 = 1;
        if (t1 + t3 > T) t3 = T - t1;   /* 加速段不缩放时减速段可能挤不下 */
        if (t3 < 1) t3 = 1;
    }

    /* 第3步：由 D = (u0+vc)*t1/2 + vc*tc + vc*t3/2 反解 vc = (2D - u0*t1)/(2T - t1 - t3)，
     * 使轨迹精确在T拍到位；再复核 |vc|<=vmax 与过渡段可达，不满足就加长后重解。
     * 最后一轮只解vc，保证 vc 与 t1/t3/T 自洽。 */
    vc = 0;
    for (i = 0; i < 4; i++)
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
        if (i == 3) break;

        n1 = Traj_Seg_Ticks(k1, (vc > u0) ? (vc - u0) : (u0 - vc));
        n3 = Traj_Seg_Ticks(k3, (vc < 0) ? -vc : vc);
        if (n1 <= t1 && n3 <= t3) break;
        if (n1 > t1) t1 = n1;
        if (n3 > t3) t3 = n3;
        /* 加速段变长挤掉匀速段时先压减速段保准时；压不下才由循环顶部顶开总时间 */
        if (t1 + t3 > T && (T - t1) >= n3) t3 = T - t1;
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
