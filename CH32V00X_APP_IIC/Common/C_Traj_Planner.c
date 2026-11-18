/* C_Traj_Planner.c 在线轨迹规划器：期望路径 → 跟踪器(V/A/D 受限) → 速度滑动平均。
 * 内部单位：位置 Q8 厘度；速度 Q20 厘度/拍；加速度 Q20 厘度/拍²。1拍=1ms。 */

#include "C_Traj_Planner.h"
#include "C_Fixed.h"
#include <string.h>

#define Q_POS   8    /* 位置小数位 */
#define Q_VEL   20   /* 速度/加速度小数位 */
#define POS_TO_VEL (Q_VEL - Q_POS)
#define TRACK_TAU_MS 25U  /* 路径还在走时跟踪器追路径误差的时间常数 ms：有限增益，速度变化不冲过头 */
#define TRACK_GAIN_Q16 ((int32_t)((1UL << (POS_TO_VEL + 16)) / TRACK_TAU_MS))   /* Q8 误差→Q20 速度/τ，Q16 */

static struct {
    /* 用户上限 */
    int32_t vmax_cdps, amax_cdpss, dmax_cdpss;
    uint16_t move_min_ms, smooth_ms;
    int32_t lo, hi;              /* 行程，厘度 */
    int32_t vcap, acc, dec;      /* 上面三个换成 Q20 */

    /* 第1级：期望路径。最新指令：从 r0 匀速走到终点，用 n 拍 */
    int32_t end;                 /* 终点(最新目标)，Q8 */
    int32_t r0;                  /* 路径起点，Q8 */
    int32_t slope;               /* 路径速度，Q20/拍 */
    uint16_t k, n;               /* 路径已走/总拍数；k≥n 即到终点 */

    /* 第1级：跟踪器 */
    int32_t p;                   /* 位置，Q8 */
    int32_t v;                   /* 速度，Q20/拍 */
    int32_t carry;               /* 速度换位置增量的余数，Q20 */
    uint8_t braking;             /* 本拍在减速 */

    /* 第2级：速度滑动平均 */
    int16_t buf[TRAJ_SMOOTH_MS_MAX];   /* 最近 n 拍的位置增量，Q8 */
    uint16_t win, idx, idle;     /* 窗口长度；写指针；第1级静止后已过拍数 */
    int32_t sum, sum_prev;       /* 窗口内增量之和，Q8 */
    int32_t out, rem;            /* 输出位置 Q8；win·输出 的余数 */
    int32_t inv_n_q24;           /* 1/win，Q24 */
    uint8_t done;
    TrajRef_t last;              /* 冻结时钟时原样输出 */
} s_tj;

static int32_t Traj_VelQ(int32_t cdps)  { return (int32_t)(((int64_t)cdps << Q_VEL) / 1000); }
static int32_t Traj_AccQ(int32_t cdpss) { int32_t a = (int32_t)(((int64_t)cdpss << Q_VEL) / 1000000); return (a > 0) ? a : 1; }

static void Traj_SetWindow(uint16_t n)
{
    s_tj.win = (n < 1U) ? 1U : ((n > TRAJ_SMOOTH_MS_MAX) ? TRAJ_SMOOTH_MS_MAX : n);
    s_tj.inv_n_q24 = (int32_t)((1L << 24) / s_tj.win);
}

void C_Traj_Hold(int32_t pos)
{
    pos = Fx_ClampRange(pos, s_tj.lo, s_tj.hi);
    s_tj.p = s_tj.end = s_tj.r0 = s_tj.out = pos << Q_POS;
    s_tj.v = s_tj.carry = s_tj.slope = 0;
    s_tj.k = s_tj.n = 0U;
    s_tj.braking = 0U;
    memset(s_tj.buf, 0, sizeof(s_tj.buf));
    Traj_SetWindow(s_tj.smooth_ms);
    s_tj.idx = 0U; s_tj.idle = s_tj.win;
    s_tj.sum = s_tj.sum_prev = s_tj.rem = 0;
    s_tj.done = 1U;
    s_tj.last.pos = pos; s_tj.last.vel = s_tj.last.acc = 0; s_tj.last.done = 1U;
}

void C_Traj_Init(int32_t pos)
{
    memset(&s_tj, 0, sizeof(s_tj));
    s_tj.smooth_ms = 1U;
    s_tj.lo = INT32_MIN >> Q_POS; s_tj.hi = INT32_MAX >> Q_POS;
    C_Traj_SetLimits(TRAJ_VMAX_MAX_CDPS, TRAJ_AMAX_MAX_CDPSS, TRAJ_AMAX_MAX_CDPSS, 1U);
    C_Traj_Hold(pos);
}

void C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, uint16_t smooth_ms)
{
    s_tj.vmax_cdps  = Fx_ClampRange(vel, TRAJ_VMAX_MIN_CDPS, TRAJ_VMAX_MAX_CDPS);
    s_tj.amax_cdpss = Fx_ClampRange(acc, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    s_tj.dmax_cdpss = Fx_ClampRange(dec, TRAJ_AMAX_MIN_CDPSS, TRAJ_AMAX_MAX_CDPSS);
    s_tj.smooth_ms  = (smooth_ms < 1U) ? 1U : ((smooth_ms > TRAJ_SMOOTH_MS_MAX) ? TRAJ_SMOOTH_MS_MAX : smooth_ms);
    s_tj.vcap = Traj_VelQ(s_tj.vmax_cdps);
    s_tj.acc  = Traj_AccQ(s_tj.amax_cdpss);
    s_tj.dec  = Traj_AccQ(s_tj.dmax_cdpss);
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

/* 唯一运动入口：从参考此刻位置用 max(T,最短运动时间) 走到 target(T=0 即最短时间)；路径匀速直线。
 * 平滑窗口半窗延迟不扣：扣掉则 T=指令间隔的连续指令每段提前走完、之间停一下。
 * 例外：从静止起步且 T≥4倍(窗口+跟踪滞后)时扣掉这份滞后，否则单条带时间指令到达偏晚；
 * T 短的不扣(流式第一条也是静止起步，扣了第一段接第二条时速度回落顿一下)。
 * 新指令随时覆盖旧的，速度由跟踪器连续接过去，不先停 */
void C_Traj_Command(int32_t target, uint16_t time_ms)
{
    uint32_t t = (time_ms > s_tj.move_min_ms) ? time_ms : s_tj.move_min_ms;
    uint32_t lag = (uint32_t)s_tj.win + TRACK_TAU_MS;

    if (s_tj.done && time_ms >= 4U * lag && time_ms - lag > s_tj.move_min_ms) t = time_ms - lag;
    int32_t  r = (s_tj.k < s_tj.n) ? s_tj.r0 + (int32_t)(((int64_t)s_tj.slope * s_tj.k) >> POS_TO_VEL) : s_tj.end;   /* 旧路径当前点 */

    s_tj.end   = Fx_ClampRange(target, s_tj.lo, s_tj.hi) << Q_POS;
    /* 旧路径当前点顺路(在参考与新目标之间)就从它接着走，误差连续速度不跳；
     * 否则(反向或更近)从参考位置起，由跟踪器按全减速度掉头 */
    s_tj.r0 = ((int64_t)(r - s_tj.p) * (s_tj.end - r) >= 0 && (int64_t)(r - s_tj.p) * (s_tj.end - s_tj.p) > 0) ? r : s_tj.p;
    if (s_tj.end == s_tj.r0) t = 0U;   /* 零长路径直接到头：不让参考白白"在动"T ms */
    s_tj.k     = 0U;
    s_tj.n     = (uint16_t)((t > 0xFFFFU) ? 0xFFFFU : t);
    if (s_tj.n)
    {   /* 路径速度超过 V 没有意义(跟踪器也走不了)，限住；T 只有几 ms 时不限会溢出 */
        int64_t slope = (((int64_t)(s_tj.end - s_tj.r0)) << POS_TO_VEL) / s_tj.n;
        s_tj.slope = (int32_t)((slope > s_tj.vcap) ? s_tj.vcap : ((slope < -s_tj.vcap) ? -s_tj.vcap : slope));
    }
    else s_tj.slope = 0;
    s_tj.done  = 0U;
    s_tj.idle  = 0U;
}

/* 按全减速度刹停，不回拉：终点改到刹车距离处，路径立即到头 */
void C_Traj_Stop(void)
{
    int32_t stop;

    if (s_tj.done) return;
    /* 刹停距离：v²/(2D)+v/2(离散每拍先走后减) */
    stop = (int32_t)((((int64_t)s_tj.v * s_tj.v) / (2 * (int64_t)s_tj.dec) + Fx_Abs32(s_tj.v) / 2) >> POS_TO_VEL);
    stop = Fx_MulShift(s_tj.p + Fx_Sign32(s_tj.v) * stop, 1, Q_POS);   /* 终点取整到厘度 */
    s_tj.end = s_tj.r0 = Fx_ClampRange(stop, s_tj.lo, s_tj.hi) << Q_POS;
    s_tj.slope = 0;
    s_tj.k = s_tj.n = 0U;
}

/* 离散制动曲线：v+(v−D)+…≤d ⇒ v≤√(D²/4+2·D·d)−D/2，且一拍不越过 d；d 为 Q8 距离，返回 Q20 速度。
 * 开方只在可能起作用(cap²+cap·D≥2·D·d)时做 */
static int32_t Traj_BrakeCap(int32_t d, int32_t cap)
{
    int64_t dq = (int64_t)d << POS_TO_VEL, two_dd = 2 * (int64_t)s_tj.dec * dq;

    if ((int64_t)cap * cap + (int64_t)cap * s_tj.dec >= two_dd)
    {
        int32_t b = (int32_t)Fx_Isqrt64((uint64_t)(two_dd + (int64_t)s_tj.dec * s_tj.dec / 4)) - s_tj.dec / 2;
        if (b < cap) cap = (b > 0) ? b : 0;
    }
    return ((int64_t)cap > dq) ? (int32_t)dq : cap;
}

/* 第1级一拍：期望速度 = 路径速度 + 追平路径误差的制动曲线速度，再受"终点前刹得住"与 V 限制；
 * 按 A(变快)或 D(变慢/掉头)逼近；返回位置增量 Q8 */
static int32_t Traj_Track(void)
{
    int32_t r, rdot, e, d, vdes, dv, lim, inc;

    if (s_tj.k < s_tj.n)
    {
        s_tj.k++;
        r = (s_tj.k < s_tj.n) ? s_tj.r0 + (int32_t)(((int64_t)s_tj.slope * s_tj.k) >> POS_TO_VEL) : s_tj.end;
        rdot = (s_tj.k < s_tj.n) ? s_tj.slope : 0;
    }
    else { r = s_tj.end; rdot = 0; }
    d = s_tj.end - s_tj.p;
    if (d == 0 && s_tj.v == 0 && rdot == 0) { s_tj.braking = 0U; return 0; }

    e = r - s_tj.p;
    {
        int32_t cap = s_tj.vcap;
        /* 路径还在走：误差按 τ 追(有限增益)；路径到头后按制动曲线最快收尾 */
        if (s_tj.k < s_tj.n)
        {
            int32_t lin = Fx_MulShift(Fx_Abs32(e), TRACK_GAIN_Q16, 16);   /* |e|/τ，Q20/拍 */
            if (lin < cap) cap = (int32_t)lin;
        }
        vdes = rdot + Fx_Sign32(e) * Traj_BrakeCap(Fx_Abs32(e), cap);
    }
    if ((int64_t)vdes * d > 0)
    {   /* 朝终点走：速度不超过在终点前刹得住的值 */
        int32_t cap = Traj_BrakeCap(Fx_Abs32(d), Fx_Abs32(vdes));
        vdes = (vdes > 0) ? cap : -cap;
    }
    else if (d == 0) vdes = 0;
    vdes = Fx_Clamp32(vdes, s_tj.vcap);

    dv = vdes - s_tj.v;
    s_tj.braking = (uint8_t)(s_tj.v != 0 && (int64_t)dv * s_tj.v < 0);   /* 速度幅值在减小(含掉头前) */
    lim = s_tj.braking ? s_tj.dec : s_tj.acc;
    s_tj.v += Fx_Clamp32(dv, lim);

    inc = (s_tj.v + s_tj.carry) >> POS_TO_VEL;
    s_tj.carry = s_tj.v + s_tj.carry - (inc << POS_TO_VEL);
    /* 路径已到头且到达/越过终点(含取整余量)：落在终点、速度归零 */
    if (s_tj.k >= s_tj.n && ((d > 0 && inc >= d) || (d < 0 && inc <= d) || d == 0)
        && (d == 0 || Fx_Abs32(s_tj.v) <= s_tj.dec + (1 << POS_TO_VEL)))
    {
        inc = d; s_tj.v = 0; s_tj.carry = 0;
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

    /* 第2级：窗口和 += 新增量 − 移出的增量；输出位置每拍加 窗口和/win(余数留下一拍，位置严格守恒) */
    s_tj.sum += inc - s_tj.buf[s_tj.idx];
    s_tj.buf[s_tj.idx] = (int16_t)inc;
    if (++s_tj.idx >= s_tj.win) s_tj.idx = 0U;
    r = s_tj.rem + s_tj.sum;
    q = (int32_t)(((int64_t)r * s_tj.inv_n_q24) >> 24);
    r -= q * (int32_t)s_tj.win;
    while (r >= (int32_t)s_tj.win) { q++; r -= s_tj.win; }
    while (r < 0) { q--; r += s_tj.win; }
    s_tj.out += q;
    s_tj.rem = r;

    out->vel = (int32_t)(((int64_t)s_tj.sum * 1000 * s_tj.inv_n_q24) >> 32);
    out->acc = (int32_t)(((int64_t)(s_tj.sum - s_tj.sum_prev) * 1000000 * s_tj.inv_n_q24) >> 32);
    s_tj.sum_prev = s_tj.sum;

    if (inc == 0 && s_tj.v == 0 && s_tj.p == s_tj.end && s_tj.k >= s_tj.n) { if (s_tj.idle < s_tj.win) s_tj.idle++; }
    else s_tj.idle = 0U;
    if (s_tj.idle >= s_tj.win)
    {   /* 窗口已排空：静止在终点，平滑时间的新设定此时生效 */
        C_Traj_Hold(Fx_MulShift(s_tj.p, 1, Q_POS));
        *out = s_tj.last;
        return;
    }
    out->pos  = Fx_MulShift(s_tj.out, 1, Q_POS);
    out->done = 0U;
    s_tj.last = *out;
}
