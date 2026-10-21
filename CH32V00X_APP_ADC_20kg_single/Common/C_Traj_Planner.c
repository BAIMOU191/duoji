/* C_Traj_Planner.c 在线轨迹规划器：跟踪器(速度/加速度/减速度受限，制动曲线停在终点) + 速度滑动平均
 * 内部单位：位置 Q8 厘度；速度 Q20 厘度/拍；加速度 Q20 厘度/拍^2。1拍 = 1ms。 */

#include "C_Traj_Planner.h"
#include "C_Fixed.h"
#include <string.h>

#define Q_POS   8    /* 位置小数位 */
#define Q_VEL   20   /* 速度/加速度小数位 */
#define POS_TO_VEL (Q_VEL - Q_POS)

static struct {
    /* 用户上限 */
    int32_t vmax_cdps, amax_cdpss, dmax_cdpss;
    uint16_t move_min_ms, smooth_ms;
    int32_t lo, hi;              /* 行程，厘度 */

    /* 第1级：跟踪器 */
    int32_t p;                   /* 位置，Q8 */
    int32_t end;                 /* 终点，Q8 */
    int32_t v;                   /* 速度，Q20/拍 */
    int32_t carry;               /* 速度换成位置增量时的余数，Q20 */
    int32_t vseg, aseg, dseg;    /* 本段速度上限、加速度、减速度，Q20 */
    uint8_t braking;             /* 本拍在减速 */

    /* 第2级：速度滑动平均 */
    int16_t buf[TRAJ_SMOOTH_MS_MAX];   /* 最近 n 拍的位置增量，Q8 */
    uint16_t n, idx, idle;       /* 窗口长度；写指针；第1级静止后已过的拍数 */
    int32_t sum, sum_prev;       /* 窗口内增量之和，Q8 */
    int32_t out, rem;            /* 输出位置 Q8；n·输出 的余数 */
    int32_t inv_n_q24;           /* 1/n，Q24 */
    uint8_t done;
    TrajRef_t last;              /* 冻结时钟时原样输出 */
} s_tj;

static int32_t Traj_VelQ(int32_t cdps)  { return (int32_t)(((int64_t)cdps << Q_VEL) / 1000); }
static int32_t Traj_AccQ(int32_t cdpss) { int32_t a = (int32_t)(((int64_t)cdpss << Q_VEL) / 1000000); return (a > 0) ? a : 1; }

static void Traj_SetWindow(uint16_t n)
{
    s_tj.n = (n < 1U) ? 1U : ((n > TRAJ_SMOOTH_MS_MAX) ? TRAJ_SMOOTH_MS_MAX : n);
    s_tj.inv_n_q24 = (int32_t)((1L << 24) / s_tj.n);
}

void C_Traj_Hold(int32_t pos)
{
    pos = Fx_ClampRange(pos, s_tj.lo, s_tj.hi);
    s_tj.p = s_tj.end = s_tj.out = pos << Q_POS;
    s_tj.v = s_tj.carry = 0;
    s_tj.vseg = s_tj.aseg = s_tj.dseg = 0;
    s_tj.braking = 0U;
    memset(s_tj.buf, 0, sizeof(s_tj.buf));
    Traj_SetWindow(s_tj.smooth_ms);
    s_tj.idx = 0U; s_tj.idle = s_tj.n;
    s_tj.sum = s_tj.sum_prev = s_tj.rem = 0;
    s_tj.done = 1U;
    s_tj.last.pos = pos; s_tj.last.vel = s_tj.last.acc = 0; s_tj.last.done = 1U;
}

void C_Traj_Init(int32_t pos)
{
    memset(&s_tj, 0, sizeof(s_tj));
    s_tj.vmax_cdps = TRAJ_VMAX_MAX_CDPS;
    s_tj.amax_cdpss = s_tj.dmax_cdpss = TRAJ_AMAX_MAX_CDPSS;
    s_tj.smooth_ms = 1U;
    s_tj.lo = INT32_MIN >> Q_POS; s_tj.hi = INT32_MAX >> Q_POS;
    C_Traj_Hold(pos);
}

void C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, uint16_t smooth_ms)
{
    s_tj.vmax_cdps  = Fx_ClampRange(vel, TRAJ_VMAX_MIN_CDPS, TRAJ_VMAX_MAX_CDPS);
    s_tj.amax_cdpss = Fx_ClampRange(acc, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    s_tj.dmax_cdpss = Fx_ClampRange(dec, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    s_tj.smooth_ms  = (smooth_ms < 1U) ? 1U : ((smooth_ms > TRAJ_SMOOTH_MS_MAX) ? TRAJ_SMOOTH_MS_MAX : smooth_ms);
    if (s_tj.done) Traj_SetWindow(s_tj.smooth_ms);   /* 窗口里还有增量时不改长度，等静止后生效 */
}

void C_Traj_SetMoveMinMs(uint16_t ms)
{
    s_tj.move_min_ms = (ms > TRAJ_MOVE_MIN_MS_MAX) ? TRAJ_MOVE_MIN_MS_MAX : ms;
}

void C_Traj_SetRange(int32_t lo, int32_t hi)
{
    int32_t pos = Fx_MulShift(s_tj.out, 1, Q_POS);

    s_tj.lo = lo; s_tj.hi = hi;
    if (pos < lo || pos > hi || (s_tj.end >> Q_POS) < lo || (s_tj.end >> Q_POS) > hi) C_Traj_Hold(pos);
}

void C_Traj_SetRangeOpen(int32_t lo, int32_t hi)
{
    s_tj.lo = lo; s_tj.hi = hi;
    s_tj.end = Fx_ClampRange(s_tj.end >> Q_POS, lo, hi) << Q_POS;
}

int32_t C_Traj_GetPos(void) { return Fx_MulShift(s_tj.p, 1, Q_POS); }
uint8_t C_Traj_Is_Done(void) { return s_tj.done; }

/* 设定新终点与本段约束。正朝终点走时，保证减速度至少够在终点前停下(不超过用户减速度) */
static void Traj_Begin(int32_t target, int32_t vseg, int32_t aseg, int32_t dseg)
{
    int32_t d;

    s_tj.end  = Fx_ClampRange(target, s_tj.lo, s_tj.hi) << Q_POS;
    s_tj.vseg = vseg; s_tj.aseg = aseg; s_tj.dseg = dseg;
    d = s_tj.end - s_tj.p;
    if (d != 0 && (int64_t)d * s_tj.v > 0)
    {
        int64_t need = ((int64_t)s_tj.v * s_tj.v) / (2 * ((int64_t)Fx_Abs32(d) << POS_TO_VEL));
        int32_t dfull = Traj_AccQ(s_tj.dmax_cdpss);
        if (need > s_tj.dseg) s_tj.dseg = (int32_t)((need < dfull) ? need : dfull);
    }
    s_tj.done = 0U;
    s_tj.idle = 0U;
}

/* 静止起步、按 V/A/D 走完 d 的最短时间，ms */
static uint32_t Traj_FastMs(int32_t d)
{
    int64_t v = s_tj.vmax_cdps, a = s_tj.amax_cdpss, b = s_tj.dmax_cdpss;
    int64_t cross = v * v / (2 * a) + v * v / (2 * b);      /* 能加到 V 的最短距离 */

    if (d >= cross) return (uint32_t)(d * 1000 / v + v * 500 / a + v * 500 / b);
    return Fx_Isqrt64((uint64_t)(2000000LL * d / a + 2000000LL * d / b));
}

void C_Traj_Plan(int32_t target, uint16_t time_ms)
{
    int32_t d = Fx_Abs32(Fx_ClampRange(target, s_tj.lo, s_tj.hi) - C_Traj_GetPos());
    int64_t v = s_tj.vmax_cdps, a = s_tj.amax_cdpss, b = s_tj.dmax_cdpss;
    uint32_t t = (time_ms > s_tj.move_min_ms) ? time_ms : s_tj.move_min_ms;

    /* 平滑窗口另占 n 拍，第1级只用剩下的时间；比最快走法长时 V/λ、A/λ²、D/λ² 整体放慢 */
    t = (t > s_tj.smooth_ms) ? t - s_tj.smooth_ms : 0U;
    if (d > 0 && t > 0U)
    {
        uint32_t tf = Traj_FastMs(d);
        if (t > tf && tf > 0U)
        {
            v = v * tf / t;
            a = a * tf / t * tf / t;
            b = b * tf / t * tf / t;
        }
    }
    Traj_Begin(target, Traj_VelQ((int32_t)((v > 0) ? v : 1)), Traj_AccQ((int32_t)a), Traj_AccQ((int32_t)b));
}

/* 速度 = 平均指令速度 + 滞后偏差/(2·展望)：滞后正好 = 速度·展望 时就按平均速度走，多了少了在两个展望内慢慢追平，
 * 单条指令早到晚到只改一点点速度 */
void C_Traj_Follow(int32_t target, int32_t vel_cdps, uint16_t horizon_ms)
{
    int32_t d = Fx_Abs32((Fx_ClampRange(target, s_tj.lo, s_tj.hi) << Q_POS) - s_tj.p);
    int32_t vmax = Traj_VelQ(s_tj.vmax_cdps), vff = Traj_VelQ(Fx_ClampRange(vel_cdps, 0, s_tj.vmax_cdps));
    uint32_t h = (horizon_ms > 0U) ? horizon_ms : 1U;
    int64_t lag = (int64_t)vff * h;                                  /* 期望滞后，Q20 厘度 */
    int64_t vseg = vff + (((int64_t)d << POS_TO_VEL) - lag) / (2 * (int64_t)h);

    if (vseg < 0) vseg = 0;

    Traj_Begin(target, (int32_t)((vseg < vmax) ? vseg : vmax), Traj_AccQ(s_tj.amax_cdpss), Traj_AccQ(s_tj.dmax_cdpss));
}

void C_Traj_Stop(void)
{
    int32_t dfull = Traj_AccQ(s_tj.dmax_cdpss), stop;

    if (s_tj.done) return;
    /* 按全减速度刹停的距离：v²/(2D) + v/2(离散每拍先走后减) */
    stop = (int32_t)((((int64_t)s_tj.v * s_tj.v) / (2 * (int64_t)dfull) + Fx_Abs32(s_tj.v) / 2) >> POS_TO_VEL);
    stop = Fx_MulShift(s_tj.p + Fx_Sign32(s_tj.v) * stop, 1, Q_POS);   /* 终点取整到厘度 */
    s_tj.end  = Fx_ClampRange(stop, s_tj.lo, s_tj.hi) << Q_POS;
    s_tj.vseg = Fx_Abs32(s_tj.v);
    s_tj.dseg = dfull;
    if (s_tj.aseg == 0) s_tj.aseg = Traj_AccQ(s_tj.amax_cdpss);
}

/* 第1级一拍：速度目标 = 朝终点的 min(本段上限, 制动曲线, 剩余距离)，按加速度/减速度逼近；返回位置增量 Q8 */
static int32_t Traj_Track(void)
{
    int32_t d = s_tj.end - s_tj.p, ad = Fx_Abs32(d), cap, vdes, dv, lim, inc;

    if (d == 0 && s_tj.v == 0) { s_tj.braking = 0U; return 0; }
    cap = s_tj.vseg;
    {   /* 制动曲线(离散)：v + (v−D) + … ≤ 剩余距离  ⇒  v ≤ √(D²/4 + 2·D·d) − D/2；只在可能起作用时开方 */
        int64_t two_dd = (int64_t)2 * s_tj.dseg * ((int64_t)ad << POS_TO_VEL);
        if ((int64_t)cap * cap + (int64_t)cap * s_tj.dseg >= two_dd)
        {
            int32_t b = (int32_t)Fx_Isqrt64((uint64_t)(two_dd + (int64_t)s_tj.dseg * s_tj.dseg / 4)) - s_tj.dseg / 2;
            if (b < cap) cap = (b > 0) ? b : 0;
        }
        if ((int64_t)cap > ((int64_t)ad << POS_TO_VEL)) cap = ad << POS_TO_VEL;   /* 一拍不越过终点 */
    }
    vdes = (d >= 0) ? cap : -cap;
    dv = vdes - s_tj.v;
    s_tj.braking = (uint8_t)(s_tj.v != 0 && (int64_t)dv * s_tj.v < 0);   /* 速度幅值在减小(含反向前) */
    lim = s_tj.braking ? s_tj.dseg : s_tj.aseg;
    s_tj.v += Fx_Clamp32(dv, lim);

    inc = (s_tj.v + s_tj.carry) >> POS_TO_VEL;
    s_tj.carry = s_tj.v + s_tj.carry - (inc << POS_TO_VEL);
    /* 到达或越过终点(含取整余量)：落在终点、速度归零 */
    if ((d > 0 && inc >= d) || (d < 0 && inc <= d) || (d == 0))
    {
        if (d == 0 || Fx_Abs32(s_tj.v) <= s_tj.dseg + (1 << POS_TO_VEL))
        {
            inc = d; s_tj.v = 0; s_tj.carry = 0;
        }
    }
    s_tj.p += inc;
    return inc;
}

void C_Traj_Step(TrajRef_t *out, uint8_t hold)
{
    int32_t inc, q, r;

    if (hold && !s_tj.braking && !s_tj.done) { *out = s_tj.last; return; }
    if (s_tj.done) { *out = s_tj.last; return; }

    inc = Traj_Track();
    if (inc > 32767) inc = 32767;
    if (inc < -32768) inc = -32768;

    /* 第2级：窗口和 += 新增量 − 移出的增量；输出位置每拍加 窗口和/n(余数留到下一拍，位置严格守恒) */
    s_tj.sum += inc - s_tj.buf[s_tj.idx];
    s_tj.buf[s_tj.idx] = (int16_t)inc;
    if (++s_tj.idx >= s_tj.n) s_tj.idx = 0U;
    r = s_tj.rem + s_tj.sum;
    q = (int32_t)(((int64_t)r * s_tj.inv_n_q24) >> 24);
    r -= q * (int32_t)s_tj.n;
    while (r >= (int32_t)s_tj.n) { q++; r -= s_tj.n; }
    while (r < 0) { q--; r += s_tj.n; }
    s_tj.out += q;
    s_tj.rem = r;

    out->vel = (int32_t)(((int64_t)s_tj.sum * 1000 * s_tj.inv_n_q24) >> 32);
    out->acc = (int32_t)(((int64_t)(s_tj.sum - s_tj.sum_prev) * 1000000 * s_tj.inv_n_q24) >> 32);
    s_tj.sum_prev = s_tj.sum;

    if (inc == 0 && s_tj.v == 0 && s_tj.p == s_tj.end) { if (s_tj.idle < s_tj.n) s_tj.idle++; }
    else s_tj.idle = 0U;
    if (s_tj.idle >= s_tj.n)
    {   /* 窗口已排空：静止在终点，平滑时间的新设定此时生效 */
        C_Traj_Hold(Fx_MulShift(s_tj.p, 1, Q_POS));
        *out = s_tj.last;
        return;
    }
    out->pos  = Fx_MulShift(s_tj.out, 1, Q_POS);
    out->done = 0U;
    s_tj.last = *out;
}
