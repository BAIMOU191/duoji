/*
 * A_Calib_Min.c —— 标定固件：额定电压下开环激励电机，辨识电压域模型并打印可整节替换 App_Model.h 对应型号电机标定区的标定块。
 *
 * 方法(约50秒)：全程"运动中扫描"，不再逐条预定位、静止起步。
 *   1. sign  测极性；noise 测静止读数噪声。
 *   2. LIN   恒占空比往返扫4趟(兼作预热)：真实转速恒定 → 读数穿过各分箱的时长 ∝ 该段真实角宽 → 读数线性化表。
 *            辨识全用线性化位置，表保持全程平均斜率不变：所得 kv/ka/fc 即"读数单位下全行程平均"的值，
 *            正是正式固件(直接用原始读数)平均看到的对象；同批次各台由端点标定把平均斜率校到1，参数可通用。
 *   3. RAMP  匀速扫2趟，途中各4处同向停车缓升：起转电压、最小位移(齿隙保持贴合)。
 *   4. SMALL 运动中 1080↔1800 每220/250/280ms轮换切换：小幅电压阶跃的升速过渡 → 小信号 τ/延迟(KA、DELAY)。
 *            同向运动中切换，库仑摩擦前后相同而抵消，无静摩擦；中速使位置相关摩擦纹波被惯性滤掉；
 *            只用升速过渡：降速时电机反拖，齿隙翻面、输出轴自由滑行，形状不可用(仅诊断)。
 *            各趟首段错开¼段使切换点散布；全部过渡共用截距(去极值均值)后合拟 τ。
 *   5. BIG   720↔2400 大幅切换：大信号 τ(额定加/减速度)与满速。
 *   额定速度/加速度再乘读数最小局部斜率：正式固件在读数偏慢处同样指令需要更高真实速度，按最坏处给。
 * 主机联合仿真(scratchpad hostcal/：本文件编译成 DLL 驱动仿真舵机，同批次随机电位器非线性×摩擦纹波×限流×齿隙)。
 * 串口 @SIGN/@SEG 进度、@FAIL 错误码、@DIAG 诊断、@DONE 结束。
 * 内存：大数组全静态、有编译期上限；栈初始化时涂色，结束打印实际用量。
 */

#include "A_Calib_Min.h"
#include "A_Config.h"
#include "A_Sensor.h"
#include "C_Fixed.h"
#include "C_Task_Scheduler.h"
#include "D_motor.h"
#include "D_iwdg.h"
#include "D_uart.h"
#include "iap.h"
#include <stdio.h>
#include <string.h>

/* ==================== 行程与动作参数 ==================== */

/* 工作扇区 CAL_SECTOR_LO/HI 随编码器定义在 A_Sensor.h：电位器取可读全程，磁编码取270度(不跨零点) */
#define CAL_GUARD_CDEG   500       /* 距扇区两端的保护余量，厘度 */
#define CAL_LO           (CAL_SECTOR_LO + CAL_GUARD_CDEG)
#define CAL_HI           (CAL_SECTOR_HI - CAL_GUARD_CDEG)
#define SW_MARGIN        2000      /* 扫描端点再内缩：满速反向制动约0.5°～1°，留足 */
#define SW_A             (CAL_LO + SW_MARGIN)
#define SW_B             (CAL_HI - SW_MARGIN)
#define CAL_CENTER       ((CAL_LO + CAL_HI) / 2)

#define GOTO_TOL_CDEG    300       /* 预定位容差(只用于到起点，精度无关紧要) */
#define GOTO_FAST_DUTY   900
#define GOTO_SLOW_DUTY   600
#define GOTO_SLOW_CDEG   2500
#define GOTO_CHECK_MS    150U      /* 极性未知时误差不减小则掉头，ms */
#define GOTO_TIMEOUT_MS  8000U
#define REST_MS          300U

#define SIGN_DUTY        900
#define SIGN_MS          200U
#define SIGN_MIN_CDEG    50

#define NOISE_SETTLE_MS  300U
#define NOISE_MS         1100U

#define LIN_DUTY         1440      /* 线性化扫描占空比 */
#define LIN_SKIP_MS      200U      /* 起步加速段不计 */
#define N_BIN            48U       /* 线性化分箱数 */
#define BIN_W            ((CAL_HI - CAL_LO) / (int32_t)N_BIN)

#define SEG_MS           250U      /* 切换周期 */
#define WIN_MS           100U      /* 每段末稳速窗口 */
#define RAMP_TRAVEL      900       /* 起转扫描的行进占空比 */
#define RAMP_STEP_DUTY   5         /* 起转缓升：每10ms加 */
#define RAMP_MAX_DUTY    900
#define BRAKE_MS         200U      /* 扫描端反向制动最长时间 */
#define BRAKE_REST_MS    250U
#define SWEEP_TIMEOUT_MS 15000U

static const int16_t  k_small[2] = { 1080, 1800 };   /* 首段(低)/交替段(高)：先升速；3.5V 阶跃，摩擦纹波占比小(仿真 KA 误差中位1.9%) */   /* 首段(低), 交替段(高)：先升速，升速过渡占多数 */
static const int16_t  k_big[2]   = { 720, 2400 };
static const uint16_t k_shape_ms[] = { 5, 10, 20, 30, 40, 60, 80, 100 }; /* 形状点(各取前后共3拍均值) */
static const uint8_t  k_ramp_pct[] = { 20, 40, 60, 80 };                 /* 起转位置，扫描行程百分比 */
#define N_POINT   (sizeof(k_shape_ms) / sizeof(k_shape_ms[0]))
#define N_RAMP_AT (sizeof(k_ramp_pct) / sizeof(k_ramp_pct[0]))

/* 动作序列 */
enum { ACT_NOISE = 0, ACT_GOTO_A, ACT_LIN, ACT_BUILD, ACT_RAMP, ACT_SMALL, ACT_BIG, ACT_REPORT };
static const uint8_t k_plan[] = {
    ACT_NOISE, ACT_GOTO_A,
    ACT_LIN, ACT_LIN, ACT_LIN, ACT_LIN, ACT_BUILD,
    ACT_RAMP, ACT_RAMP,
    ACT_SMALL, ACT_SMALL, ACT_SMALL, ACT_SMALL, ACT_SMALL, ACT_SMALL, ACT_SMALL, ACT_SMALL,
    ACT_BIG, ACT_BIG, ACT_BIG, ACT_BIG, ACT_BIG, ACT_BIG,
    ACT_REPORT };
#define N_PLAN (sizeof(k_plan) / sizeof(k_plan[0]))

#define MAX_STEADY_R 16U  /* 稳速段记录上限，按扫描类分开封顶：起转扫描 */
#define MAX_STEADY_S 40U  /* 小信号扫描 */
#define MAX_STEADY_B 24U  /* 大信号扫描(含满速，额定速度靠它) */
#define MAX_STEADY  (MAX_STEADY_R + MAX_STEADY_S + MAX_STEADY_B)
#define MAX_TRANS_S 32U   /* 小信号过渡上限(实际约24) */
#define MAX_TRANS_B 16U   /* 大信号过渡上限(实际约12)：分开封顶，小信号多了也不挤掉大信号 */
#define MAX_TRANS   (MAX_TRANS_S + MAX_TRANS_B)
#define MAX_BREAK   (2U * N_RAMP_AT)

/* ---- 编译期上限 ---- */
_Static_assert(N_PLAN <= 255U && MAX_STEADY <= 255U && MAX_TRANS <= 255U, "counters are uint8_t");
_Static_assert(SW_B - SW_A > 10000, "sweep span too short");
_Static_assert(BIN_W > 100, "linearization bins too narrow");
_Static_assert(SEG_MS > WIN_MS + 100U && 2U * SEG_MS < 65535U, "segment timer");
_Static_assert(SWEEP_TIMEOUT_MS < 65535U && GOTO_TIMEOUT_MS < 65535U && NOISE_MS < 65535U, "timers are uint16_t");
_Static_assert(N_POINT <= 8U, "shape mask is uint8_t");
_Static_assert(CAL_HI + 4000 < 32767 && CAL_LO - 4000 > -32767, "linearization table stored in int16");
_Static_assert(CFG_PWM_FULL / 12 <= 255 && CFG_PWM_FULL % 12 == 0, "steady level stored as duty/12");

/* ==================== 状态 ==================== */

enum { PH_BOOT = 0, PH_GOTO, PH_REST, PH_SIGN, PH_NEXT, PH_NOISE, PH_SWEEP, PH_RAMP, PH_BRAKE, PH_DONE };
enum { SW_LIN = 0, SW_RAMP, SW_SMALL, SW_BIG };

/* 最小二乘累加器：t 为段内毫秒，x 为同向位移；乘积项用64位防溢出 */
typedef struct { int32_t n, st, sx; int64_t stt, stx; } CalLs_t;

static struct {
    uint8_t  phase, after, fail, enc_bad, flip, sign_known, plan;
    int8_t   sign, dir;
    uint16_t timer;
    uint32_t t_total;
    int32_t  raw, pos;                 /* 原始读数 / 线性化位置，厘度 */
    int32_t  mark, peak, target, goto_err;
    int32_t  avg_buf[8], avg_sum;      /* 线性化位置8拍滑动和 */
    uint8_t  avg_idx;
    int32_t  r4[4];                    /* 原始读数最近4拍(线性化扫描找穿越) */
    uint8_t  r4_idx;
    uint8_t  sub;
    /* 当前扫描 */
    uint8_t  kind, n_kind[4];
    int16_t  lv[2];                    /* 首段/交替段占空比 */
    int16_t  level;
    uint16_t seg_t, seg_len, sweep_t;
    uint8_t  seg_j, start_seg, prev_ok, ramp_i;
    int16_t  ramp_level, ramp_quiet;
} s_cal;

/* 当前段临时量；与结果阶段的排序暂存共用内存(扫描结束后才辨识) */
static union {
  struct {
    CalLs_t  ls;
    uint32_t cv;
    int32_t  pred0_q8, slope_q8;       /* 上一段稳速直线外推到本段：x̂(t)=pred0+slope·t，Q8 厘度/ms */
    int32_t  pt_acc;
    int16_t  pts[N_POINT];
    uint8_t  pt_mask;
    /* 线性化扫描：本趟各分箱边界穿越时刻(Q8 ms)，-1=未穿越 */
    int32_t  cross[N_BIN + 1U];
    int32_t  r_prev;
    int16_t  nb;
    uint8_t  have_prev;
  } seg;
  int32_t med[MAX_STEADY > MAX_TRANS ? MAX_STEADY : MAX_TRANS];   /* 中位数/排序暂存，只在 Cal_Report 用 */
} s_u;
#define s_seg s_u.seg
#define s_med s_u.med

/* 测量结果 */
typedef struct { int32_t speed; uint16_t cv; uint8_t level_div; int8_t dir; } Steady_t;   /* level_div = 占空比/12 */
typedef struct { int16_t dv2; uint16_t s_us; int16_t pts[N_POINT]; uint8_t big, up; } Trans_t;  /* dv2: 2厘度/s；s_us: µs；pts: Q2 厘度 */

static struct {
    uint16_t rest_cv;
    int32_t  noise_lo, noise_hi, noise, noise_ref;
    CalLs_t  noise_ls;
    int64_t  noise_sxx;
    uint16_t binw[N_BIN];              /* 分箱归一化穿越时长累加，Q8(每趟约256，最多4趟) */
    uint8_t  binc[N_BIN];
    int16_t  lin_tab[N_BIN + 1U];      /* 分箱边界的线性化位置，厘度；lin_ok=0 时不用 */
    uint8_t  lin_ok, lin_bins;
    int32_t  lin_dev;                  /* 线性化表与直线最大偏差，厘度 */
    int32_t  g_min_pm;                 /* 读数最小局部斜率(读数/真实)，千分比 */
    int32_t  breakaway[MAX_BREAK], rstep[MAX_BREAK];
    uint8_t  n_break, n_rstep;
    Steady_t steady[MAX_STEADY];
    uint8_t  n_steady, n_steady_k[4];
    Trans_t  trans[MAX_TRANS];
    uint8_t  n_trans, n_trans_s, n_trans_b;
} s_meas;

static uint8_t s_iap_stopped;

/* ==================== 栈用量 ==================== */

#define STACK_PAINT 0xA5A5A5A5UL
extern uint32_t _heap_end[];   /* 栈底(低地址)，Link.ld */
extern uint32_t _eusrstack[];  /* 栈顶 */

/* 从栈底涂到当前栈指针下方64字节，之后被覆盖的就是用过的 */
static void Cal_StackPaint(void)
{
#ifndef CAL_HOST               /* 主机联合仿真时不涂(没有这段栈) */
    volatile uint32_t here = 0U;
    uint32_t *p = _heap_end;
    uint32_t *lim = (uint32_t *)&here - 16;

    while (p < lim) *p++ = STACK_PAINT;
#endif
}

static uint32_t Cal_StackUsed(void)
{
    const uint32_t *p = _heap_end;

    while (p < _eusrstack && *p == STACK_PAINT) p++;
    return (uint32_t)((const uint8_t *)_eusrstack - (const uint8_t *)p);
}

/* ==================== 小工具 ==================== */

static int32_t Cal_Round(double x) { return (int32_t)((x >= 0.0) ? x + 0.5 : x - 0.5); }

static void Ls_Add(CalLs_t *a, int32_t t, int32_t x)
{
    a->n++; a->st += t; a->sx += x;
    a->stt += (int64_t)t * t; a->stx += (int64_t)t * x;
}

/* 解直线 x = mx + slope·(t − mt)；返回0=点数不足或退化 */
static uint8_t Ls_Solve(const CalLs_t *a, double *slope, double *mt, double *mx)
{
    double n = (double)a->n;
    double det = n * (double)a->stt - (double)a->st * (double)a->st;

    if (a->n < 3 || det <= 0.0) return 0U;
    *slope = (n * (double)a->stx - (double)a->st * (double)a->sx) / det;
    *mt = (double)a->st / n;
    *mx = (double)a->sx / n;
    return 1U;
}

/* 原地排序取中位数(偶数取中间两数均值)；可选输出最小/最大 */
static int32_t Cal_Median(int32_t *v, uint8_t n, int32_t *lo, int32_t *hi)
{
    uint8_t i, j;

    for (i = 1U; i < n; i++)
        for (j = i; j > 0U && v[j - 1U] > v[j]; j--) { int32_t t = v[j]; v[j] = v[j - 1U]; v[j - 1U] = t; }
    if (lo) *lo = n ? v[0] : 0;
    if (hi) *hi = n ? v[n - 1U] : 0;
    if (n == 0U) return 0;
    return (n & 1U) ? v[n / 2U] : (v[n / 2U - 1U] + v[n / 2U]) / 2;
}

static int32_t Cal_AvgPos(void) { return (s_cal.avg_sum >= 0) ? (s_cal.avg_sum + 4) / 8 : (s_cal.avg_sum - 4) / 8; }

/* 原始读数 → 线性化位置(建表前原样返回) */
static int32_t Cal_Lin(int32_t raw)
{
    int32_t k, b;

    if (!s_meas.lin_ok) return raw;
    k = (raw < CAL_LO) ? 0 : (raw - CAL_LO) / BIN_W;
    if (k > (int32_t)N_BIN - 1) k = (int32_t)N_BIN - 1;
    b = CAL_LO + k * BIN_W;
    return (int32_t)s_meas.lin_tab[k] + ((int32_t)s_meas.lin_tab[k + 1] - s_meas.lin_tab[k]) * (raw - b) / BIN_W;
}

static void Cal_Drive(int16_t duty) { D_Motor_Set(duty); }

/* 方向 dir(角度增大=+1) 的幅值换成物理占空比 */
static int16_t Cal_Out(int8_t dir, int16_t mag) { return (int16_t)(mag * dir * s_cal.sign); }

static void Cal_Fail(uint8_t code)
{
    Cal_Drive(0);
    if (s_cal.fail == 0U) printf("@FAIL,%u,%ld\r\n", (unsigned)code, (long)s_cal.raw);
    s_cal.fail = code;
    s_cal.phase = PH_DONE;
}

static void Cal_Enter(uint8_t phase) { s_cal.phase = phase; s_cal.timer = 0U; s_cal.sub = 0U; }

/* ==================== 预定位 ==================== */

static void Cal_Goto(int32_t target, uint8_t after)
{
    s_cal.target   = target;
    s_cal.after    = after;
    s_cal.goto_err = target - s_cal.raw;
    s_cal.flip     = 0U;
    Cal_Enter(PH_GOTO);
}

/* bang-bang 靠近，接近换小幅值；极性未知时误差不减小则掉头 */
static void Cal_GotoTick(void)
{
    int32_t err = s_cal.target - s_cal.raw;
    int16_t mag = (Fx_Abs32(err) > GOTO_SLOW_CDEG) ? GOTO_FAST_DUTY : GOTO_SLOW_DUTY;

    if (Fx_Abs32(err) <= GOTO_TOL_CDEG) { Cal_Drive(0); Cal_Enter(PH_REST); return; }
    if (s_cal.timer >= GOTO_TIMEOUT_MS) { Cal_Fail(5U); return; }
    if (!s_cal.sign_known && s_cal.timer > 0U && s_cal.timer % GOTO_CHECK_MS == 0U)
    {
        if (Fx_Abs32(err) >= Fx_Abs32(s_cal.goto_err)) s_cal.flip ^= 1U;
        s_cal.goto_err = err;
    }
    Cal_Drive(Cal_Out((int8_t)(((err > 0) != (s_cal.flip != 0U)) ? 1 : -1), mag));
}

/* ==================== 噪声 ==================== */

static uint8_t Cal_NoiseTick(void)
{
    uint16_t t = s_cal.timer;

    Cal_Drive(0);
    if (t >= NOISE_SETTLE_MS)
    {
        int32_t d;
        if (t == NOISE_SETTLE_MS) { s_meas.noise_lo = s_meas.noise_hi = s_meas.noise_ref = s_cal.raw; }
        if (s_cal.raw < s_meas.noise_lo) s_meas.noise_lo = s_cal.raw;
        if (s_cal.raw > s_meas.noise_hi) s_meas.noise_hi = s_cal.raw;
        d = s_cal.raw - s_meas.noise_ref;
        Ls_Add(&s_meas.noise_ls, (int32_t)(t - NOISE_SETTLE_MS), d);
        s_meas.noise_sxx += (int64_t)d * d;
    }
    if (t < NOISE_MS) return 0U;
    {   /* 3σ，扣掉线性趋势(齿轮回弹/蠕动不算噪声) */
        const CalLs_t *a = &s_meas.noise_ls;
        double n = (double)a->n;
        double stt = (double)a->stt - (double)a->st * a->st / n;
        double stx = (double)a->stx - (double)a->st * a->sx / n;
        double sxx = (double)s_meas.noise_sxx - (double)a->sx * a->sx / n;
        double var = (stt > 0.0) ? (sxx - stx * stx / stt) / n : sxx / n;
        if (var > 2.0e7) var = 2.0e7;   /* 碰轴等异常：钳住防 ×100 后越 int32 */
        s_meas.noise = (var > 0.0) ? Cal_Round(3.0 * (double)Fx_Isqrt((uint32_t)Cal_Round(var * 100.0)) / 10.0) : 0;
        if (s_meas.noise < 1) s_meas.noise = 1;
    }
    return 1U;
}

/* ==================== 扫描 ==================== */

static void Cal_SegReset(void)
{
    memset(&s_seg.ls, 0, sizeof(s_seg.ls));
    s_seg.cv = 0U; s_seg.pt_acc = 0; s_seg.pt_mask = 0U;
    s_cal.seg_t = 0U;
}

static void Cal_StartSweep(uint8_t kind)
{
    uint8_t i = s_cal.n_kind[kind];

    s_cal.kind = kind;
    s_cal.n_kind[kind]++;
    if (kind == SW_SMALL)     { s_cal.lv[0] = k_small[0]; s_cal.lv[1] = k_small[1]; }
    else if (kind == SW_BIG)  { s_cal.lv[0] = k_big[0];   s_cal.lv[1] = k_big[1]; }
    else if (kind == SW_RAMP) { s_cal.lv[0] = s_cal.lv[1] = RAMP_TRAVEL; }
    else                      { s_cal.lv[0] = s_cal.lv[1] = LIN_DUTY; }
    s_cal.seg_j = 0U; s_cal.start_seg = 1U; s_cal.prev_ok = 0U; s_cal.ramp_i = 0U; s_cal.sweep_t = 0U;
    /* 首段依次加长 0/¼/½/¾ 段，各趟切换点落在不同位置 */
    s_cal.seg_len = (uint16_t)(SEG_MS + ((kind == SW_SMALL || kind == SW_BIG) ? (SEG_MS * (i % 4U)) / 4U : 0U));
    Cal_SegReset();
    if (kind == SW_LIN)
    {
        uint8_t k;
        for (k = 0U; k <= N_BIN; k++) s_seg.cross[k] = -1;
        s_seg.have_prev = 0U; s_seg.nb = -1;
    }
    printf("@SEG,%u/%u\r\n", (unsigned)s_cal.plan, (unsigned)N_PLAN);
    Cal_Enter(PH_SWEEP);
}

/* 线性化扫描：4拍均值穿越分箱边界的插值时刻 */
static void Cal_LinTrack(uint16_t t)
{
    int32_t r = s_cal.r4[0] + s_cal.r4[1] + s_cal.r4[2] + s_cal.r4[3];   /* ×4 */
    int8_t dir = s_cal.dir;

    if (t < LIN_SKIP_MS) return;
    if (s_seg.have_prev)
    {
        if (s_seg.nb < 0)
        {
            int32_t k = (s_seg.r_prev / 4 - CAL_LO) / BIN_W;
            s_seg.nb = (int16_t)((dir > 0) ? k + 1 : k);
        }
        while (s_seg.nb >= 0 && s_seg.nb <= (int16_t)N_BIN)
        {
            int32_t b4 = 4 * (CAL_LO + s_seg.nb * BIN_W);
            if (dir * (r - b4) < 0) break;
            s_seg.cross[s_seg.nb] = ((int32_t)(t - 1U) << 8)
                + ((r != s_seg.r_prev) ? ((b4 - s_seg.r_prev) * 256) / (r - s_seg.r_prev) : 256);
            s_seg.nb = (int16_t)(s_seg.nb + dir);
        }
    }
    s_seg.r_prev = r; s_seg.have_prev = 1U;
}

/* 一趟线性化扫描结束：各箱时长按本趟均值归一化后累加 */
static void Cal_LinFinish(void)
{
    uint32_t sum = 0U, n = 0U;
    uint8_t k;

    for (k = 0U; k < N_BIN; k++)
        if (s_seg.cross[k] >= 0 && s_seg.cross[k + 1U] >= 0) { sum += (uint32_t)Fx_Abs32(s_seg.cross[k + 1U] - s_seg.cross[k]); n++; }
    if (n < 4U || sum == 0U) return;
    for (k = 0U; k < N_BIN; k++)
        if (s_seg.cross[k] >= 0 && s_seg.cross[k + 1U] >= 0 && s_meas.binc[k] < 255U)
        {
            uint32_t q8 = (uint32_t)(((uint64_t)Fx_Abs32(s_seg.cross[k + 1U] - s_seg.cross[k]) * n << 8) / sum);
            s_meas.binw[k] = (uint16_t)((s_meas.binw[k] + q8 > 65535U) ? 65535U : s_meas.binw[k] + q8);
            s_meas.binc[k]++;
        }
}

/* 建线性化表：覆盖箱按实测角宽，未覆盖箱取平均；整体平均斜率保持(=原读数单位)，中点读数不动。
 * 同时求读数最小局部斜率 g_min = 箱宽/真实角宽，额定能力按它折算 */
static void Cal_LinBuild(void)
{
    uint32_t w[N_BIN], mean = 0U, wmax = 0U;
    int32_t tab[N_BIN + 1U];
    uint8_t k, n = 0U;
    int32_t off, dev = 0;

    for (k = 0U; k < N_BIN; k++)
    {
        w[k] = s_meas.binc[k] ? s_meas.binw[k] / s_meas.binc[k] : 0U;
        if (w[k]) { mean += w[k]; n++; }
    }
    s_meas.lin_bins = n;
    s_meas.g_min_pm = 1000;
    if (n < N_BIN / 2U) { s_meas.lin_ok = 0U; return; }   /* 覆盖太少：不线性化(自检报出) */
    mean /= n;
    for (k = 0U; k < N_BIN; k++) if (w[k] > wmax) wmax = w[k];
    s_meas.g_min_pm = (int32_t)((uint64_t)mean * 1000U / wmax);
    tab[0] = CAL_LO * 64;                                  /* 先用 Q6 累加防截断 */
    for (k = 0U; k < N_BIN; k++)
        tab[k + 1U] = tab[k] + (int32_t)(((int64_t)BIN_W * 64 * (w[k] ? w[k] : mean)) / mean);
    off = (CAL_LO + (int32_t)(N_BIN / 2U) * BIN_W) * 64 - tab[N_BIN / 2U];
    for (k = 0U; k <= N_BIN; k++)
    {
        int32_t d, v = (tab[k] + off + 32) >> 6;
        s_meas.lin_tab[k] = (int16_t)((v > 32767) ? 32767 : (v < -32767) ? -32767 : v);
        d = Fx_Abs32(s_meas.lin_tab[k] - (CAL_LO + (int32_t)k * BIN_W));
        if (d > dev) dev = d;
    }
    s_meas.lin_dev = dev;
    s_meas.lin_ok = 1U;
}

/* 段末：稳速记录 + (若是切换段)过渡记录；为下一段准备外推直线 */
static void Cal_SegEnd(void)
{
    double v, tm, xm;
    uint16_t len = s_cal.seg_len;

    if (Ls_Solve(&s_seg.ls, &v, &tm, &xm))
    {
        static const uint8_t k_cap[4] = { 0U, MAX_STEADY_R, MAX_STEADY_S, MAX_STEADY_B };
        if (s_meas.n_steady_k[s_cal.kind] < k_cap[s_cal.kind] && s_meas.n_steady < MAX_STEADY && v > 0.0)
        {
            s_meas.n_steady_k[s_cal.kind]++;
            Steady_t *r = &s_meas.steady[s_meas.n_steady++];
            r->speed = Cal_Round(v * 1000.0);
            r->cv    = (uint16_t)((s_seg.cv + (uint32_t)s_seg.ls.n / 2U) / (uint32_t)s_seg.ls.n);
            r->level_div = (uint8_t)(s_cal.level / 12);
            r->dir   = s_cal.dir;
        }
        if (s_cal.lv[0] != s_cal.lv[1] && s_cal.prev_ok && !s_cal.start_seg
            && s_seg.pt_mask == (uint8_t)((1U << N_POINT) - 1U)
            && ((s_cal.kind == SW_BIG) ? s_meas.n_trans_b < MAX_TRANS_B : s_meas.n_trans_s < MAX_TRANS_S))
        {   /* 过渡直线 x̃(t) = xm − x̂(tm) + (v − vA)(t − tm)，零点即截距 S = d + τ */
            double vA = s_seg.slope_q8 / 256.0;
            double dv = v - vA;
            double rel = xm - (s_seg.pred0_q8 + s_seg.slope_q8 * tm) / 256.0;
            if (dv > 0.5 || dv < -0.5)
            {
                Trans_t *r = &s_meas.trans[s_meas.n_trans++];
                uint8_t k;
                int32_t s_us = Cal_Round((tm - rel / dv) * 1000.0), dv2 = Cal_Round(dv * 500.0);
                r->dv2  = (int16_t)((dv2 > 32767) ? 32767 : (dv2 < -32767) ? -32767 : dv2);
                r->s_us = (uint16_t)((s_us < 0) ? 0 : (s_us > 65535) ? 65535 : s_us);
                r->big  = (uint8_t)(s_cal.kind == SW_BIG);
                if (r->big) s_meas.n_trans_b++; else s_meas.n_trans_s++;
                r->up   = (uint8_t)(s_cal.level > s_cal.lv[(s_cal.lv[0] == s_cal.level) ? 1 : 0]);
                for (k = 0U; k < N_POINT; k++) r->pts[k] = s_seg.pts[k];
            }
        }
        /* 下一段的外推基线：本段直线平移到下一段起点 */
        s_seg.slope_q8 = Cal_Round(v * 256.0);
        s_seg.pred0_q8 = Cal_Round((xm + v * ((double)len - tm)) * 256.0);
        s_cal.prev_ok = 1U;
    }
    else s_cal.prev_ok = 0U;
    s_cal.start_seg = 0U;
    s_cal.seg_j++;
    s_cal.seg_len = (uint16_t)(SEG_MS - 30U + 30U * (s_cal.seg_j % 3U));   /* 220/250/280 轮换 */
    Cal_SegReset();
}

static int32_t Cal_SweepEnd(void) { return (s_cal.dir > 0) ? SW_B : SW_A; }

static void Cal_ToBrake(void)
{
    s_cal.peak = s_cal.dir * Cal_AvgPos();
    Cal_Enter(PH_BRAKE);
}

static void Cal_SweepTick(void)
{
    int8_t dir = s_cal.dir;
    uint16_t st = s_cal.seg_t;
    int32_t x = dir * s_cal.pos;

    if (++s_cal.sweep_t >= SWEEP_TIMEOUT_MS) { Cal_Fail(8U); return; }
    if (dir * (s_cal.raw - Cal_SweepEnd()) >= 0)
    {   /* 到端：未完成的段丢弃 */
        if (s_cal.kind == SW_LIN) Cal_LinFinish();
        Cal_ToBrake();
        return;
    }
    s_cal.level = s_cal.lv[s_cal.seg_j & 1U];

    if (s_cal.kind == SW_LIN)
    {
        Cal_LinTrack(s_cal.sweep_t);
        Cal_Drive(Cal_Out(dir, s_cal.level));
        return;
    }

    /* 形状点：本拍读数在本段指令之前，读数时刻 = 切换后 st ms */
    if (s_cal.prev_ok && !s_cal.start_seg && s_cal.lv[0] != s_cal.lv[1])
    {
        uint8_t k;
        for (k = 0U; k < N_POINT; k++)
        {
            if (st + 1U < k_shape_ms[k] || st > k_shape_ms[k] + 1U) continue;
            s_seg.pt_acc += (x << 8) - (s_seg.pred0_q8 + s_seg.slope_q8 * (int32_t)st);
            if (st == k_shape_ms[k] + 1U)
            {   /* Q8 三拍和 → Q2 均值 */
                int32_t q2 = s_seg.pt_acc / 192;
                s_seg.pts[k] = (int16_t)((q2 > 32767) ? 32767 : (q2 < -32767) ? -32767 : q2);
                s_seg.pt_mask |= (uint8_t)(1U << k);
                s_seg.pt_acc = 0;
            }
        }
    }
    if (st + WIN_MS >= s_cal.seg_len) { Ls_Add(&s_seg.ls, (int32_t)st, x); s_seg.cv += A_Voltage_Read(); }

    Cal_Drive(Cal_Out(dir, s_cal.level));
    s_cal.seg_t++;
    if (s_cal.seg_t < s_cal.seg_len) return;
    Cal_SegEnd();

    /* 起转扫描：段边界处到达下一个停车位置就停下缓升 */
    if (s_cal.kind == SW_RAMP && s_cal.ramp_i < N_RAMP_AT)
    {
        uint8_t idx = (dir > 0) ? s_cal.ramp_i : (uint8_t)(N_RAMP_AT - 1U - s_cal.ramp_i);
        int32_t at = SW_A + (int32_t)(SW_B - SW_A) * k_ramp_pct[idx] / 100;
        if (dir * (s_cal.raw - at) >= 0) { s_cal.ramp_i++; s_cal.phase = PH_RAMP; s_cal.sub = 0U; s_cal.timer = 0U; }
    }
}

/* 起转：短路刹停静置 → 同向缓升到8拍均值移动超阈 → 撤力静置 → 净位移；然后从静止续扫 */
static void Cal_RampTick(void)
{
    uint16_t t = s_cal.timer;
    int8_t dir = s_cal.dir;

    if (s_cal.sub == 0U)
    {
        Cal_Drive(0);
        if (t < REST_MS) return;
        s_cal.sub = 1U; s_cal.mark = s_cal.avg_sum; s_cal.ramp_level = 0; s_cal.ramp_quiet = 0; s_cal.timer = 0U;
        return;
    }
    if (s_cal.sub == 1U)
    {   /* 阈值=噪声×3/4(8拍均值)，取最后仍在半阈内的幅值为起转值 */
        int32_t thr = (s_meas.noise * 3 / 4 > 4) ? s_meas.noise * 3 / 4 : 4;
        int32_t moved = Fx_Abs32(s_cal.avg_sum - s_cal.mark) / 8;
        if (moved * 2 <= thr) s_cal.ramp_quiet = s_cal.ramp_level;
        if (moved >= thr || s_cal.ramp_level >= RAMP_MAX_DUTY)
        {
            if (s_cal.ramp_level < RAMP_MAX_DUTY && s_meas.n_break < MAX_BREAK)
                s_meas.breakaway[s_meas.n_break++] = (int32_t)s_cal.ramp_quiet * A_Voltage_Read() * 10 / CFG_PWM_FULL;
            s_cal.sub = (s_cal.ramp_level < RAMP_MAX_DUTY) ? 2U : 3U;
            s_cal.timer = 0U; Cal_Drive(0);
            return;
        }
        if (t % 10U == 0U) s_cal.ramp_level += RAMP_STEP_DUTY;
        Cal_Drive(Cal_Out(dir, s_cal.ramp_level));
        return;
    }
    Cal_Drive(0);
    if (t < REST_MS) return;
    if (s_cal.sub == 2U && s_meas.n_rstep < MAX_BREAK)
        s_meas.rstep[s_meas.n_rstep++] = (Fx_Abs32(s_cal.avg_sum - s_cal.mark) + 4) / 8;
    /* 续扫：从静止起步的新首段 */
    s_cal.seg_j = 0U; s_cal.start_seg = 1U; s_cal.prev_ok = 0U; s_cal.seg_len = SEG_MS;
    Cal_SegReset();
    s_cal.phase = PH_SWEEP;
}

/* 扫描端：反向制动到8拍均值回退或超时，再短路刹车静置 */
static uint8_t Cal_BrakeTick(void)
{
    int8_t dir = s_cal.dir;
    int32_t xa = dir * Cal_AvgPos();

    if (s_cal.sub == 0U)
    {
        if (xa > s_cal.peak) s_cal.peak = xa;
        if (xa < s_cal.peak - s_meas.noise - 10 || s_cal.timer >= BRAKE_MS) { s_cal.sub = 1U; s_cal.timer = 0U; Cal_Drive(0); return 0U; }
        Cal_Drive(Cal_Out((int8_t)-dir, (s_cal.lv[0] > s_cal.lv[1]) ? s_cal.lv[0] : s_cal.lv[1]));
        return 0U;
    }
    Cal_Drive(0);
    return (uint8_t)(s_cal.timer >= BRAKE_REST_MS);
}

/* ==================== 辨识(全部数据采完后一次，用 double) ==================== */

#define ID_TAU_MIN_MS   3.0     /* τ 搜索下限，ms */
#define ID_TAU_MAX_MS 150.0
#define ID_DELAY_MAX_MS 8.0     /* 延迟搜索上限，ms；达上限说明对象不符一阶+延迟 */
#define ID_MIN_TRANS      5U    /* 每类过渡至少记录数 */
#define ID_GOLDEN_IT     30U
#define ID_OUTLIER_K      4     /* kv/fc 拟合：残差超过 K×中位残差的点剔除 */
#define ID_OUTLIER_MIN_MV 30    /* 剔除阈值下限，mV */
#define ID_MARGIN       0.9     /* 额定留10%余量 */
#define ID_CHECK_BW     120.0   /* 自检速度环带宽，rad/s */

static double Cal_Exp(double x)   /* x≤0：缩参后泰勒再平方回，免 libm */
{
    double s = 1.0, t = 1.0;
    uint8_t n = 0U, i;

    if (x < -40.0) return 0.0;
    while (x < -0.5) { x *= 0.5; n++; }
    for (i = 1U; i <= 8U; i++) { t *= x / i; s += t; }
    while (n--) s *= s;
    return s;
}

/* 一阶+纯延迟阶跃位移(单位速度差)：shape(t)=s−τ(1−e^(−s/τ))，s=t−d */
static double Cal_StepShape(double t, double d, double tau)
{
    double s = t - d;
    return (s <= 0.0) ? 0.0 : s - tau * (1.0 - Cal_Exp(-s / tau));
}

static double Cal_Volt(int16_t level, uint16_t cv) { return (double)level * cv * 10.0 / CFG_PWM_FULL; }

typedef struct {
    double kv, fc, kv_dir[2], fc_dir[2];  /* mV/(厘度/s)、mV */
    double tau, delay, S;                 /* 小信号，ms(→KA、DELAY) */
    double tau_big, S_big;                /* 大信号，ms(→额定加/减速度) */
    double tau_down;                      /* 小信号降速过渡 τ，只诊断(齿隙翻面) */
    double cal_mv, vel_full;
    int32_t fs, n_small, n_big, n_down;
    int32_t n_speed, n_drop, res_mv;
    int32_t s_lo, s_hi;                   /* 小信号各过渡截距最小/最大，µs */
    int32_t fs_lo, fs_hi;
} CalModel_t;

/* 一个方向的 U=kv·v+fc 直线；thr>0 时剔除 |残差|>thr 的点。返回点数 */
static uint8_t Cal_FitLine(int8_t dir, double thr, double *kv, double *fc)
{
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, det;
    uint8_t i;

    for (i = 0U; i < s_meas.n_steady; i++)
    {
        const Steady_t *r = &s_meas.steady[i];
        double v = r->speed, u = Cal_Volt((int16_t)(r->level_div * 12), r->cv);

        if (r->dir != dir || v < 500) continue;
        if (thr > 0.0) { double e = u - (*kv * v + *fc); if (e > thr || e < -thr) continue; }
        n += 1; sx += v; sy += u; sxx += v * v; sxy += v * u;
    }
    det = n * sxx - sx * sx;
    if (n < 3 || det <= 0) return 0U;
    *kv = (n * sxy - sx * sy) / det;
    *fc = (sy - *kv * sx) / n;
    return (uint8_t)n;
}

static uint8_t Cal_FitSpeed(CalModel_t *m)
{
    uint8_t d, i, ok = 1U, nres = 0U;
    double cv_sum = 0.0, vf[2] = { 0.0, 0.0 };
    uint16_t cv_n = 0U, vn[2] = { 0U, 0U };

    for (d = 0U; d < 2U; d++)
    {
        int8_t dir = (int8_t)(d ? -1 : 1);
        double kv = 0, fc = 0, thr;
        uint8_t n0 = Cal_FitLine(dir, 0.0, &kv, &fc), n1, k = 0U;

        if (n0 == 0U) { ok = 0U; continue; }
        for (i = 0U; i < s_meas.n_steady; i++)
        {
            const Steady_t *r = &s_meas.steady[i];
            if (r->dir != dir || r->speed < 500) continue;
            s_med[k++] = Fx_Abs32(Cal_Round((Cal_Volt((int16_t)(r->level_div * 12), r->cv) - (kv * r->speed + fc)) * 10.0)); /* 0.1mV */
        }
        thr = Cal_Median(s_med, k, 0, 0) * ID_OUTLIER_K / 10.0;
        if (thr < ID_OUTLIER_MIN_MV) thr = ID_OUTLIER_MIN_MV;
        n1 = Cal_FitLine(dir, thr, &kv, &fc);
        if (n1 == 0U) n1 = Cal_FitLine(dir, 0.0, &kv, &fc);
        m->kv_dir[d] = kv; m->fc_dir[d] = fc;
        m->n_speed += n1; m->n_drop += n0 - n1;
    }
    m->kv = (m->kv_dir[0] + m->kv_dir[1]) / 2;
    m->fc = (m->fc_dir[0] + m->fc_dir[1]) / 2;
    for (i = 0U; i < s_meas.n_steady; i++)
    {
        const Steady_t *r = &s_meas.steady[i];
        uint8_t di = (uint8_t)(r->dir < 0);
        if (r->speed < 500) continue;
        cv_sum += r->cv; cv_n++;
        if (r->level_div * 12 == CFG_PWM_FULL) { vf[di] += r->speed; vn[di]++; }
        if (ok) s_med[nres++] = Fx_Abs32(Cal_Round(Cal_Volt((int16_t)(r->level_div * 12), r->cv) - (m->kv_dir[di] * r->speed + m->fc_dir[di])));
    }
    m->res_mv = Cal_Median(s_med, nres, 0, 0);
    m->cal_mv = cv_n ? cv_sum / cv_n * 10.0 : s_meas.rest_cv * 10.0;
    if (vn[0] == 0U || vn[1] == 0U) return 0U;
    vf[0] /= vn[0]; vf[1] /= vn[1];
    m->vel_full = (vf[0] < vf[1]) ? vf[0] : vf[1];
    return (uint8_t)(ok && m->kv > 0);
}

static uint8_t s_fit_big, s_fit_up;

static uint8_t Cal_TransUse(const Trans_t *r) { return (uint8_t)(r->big == s_fit_big && r->up == s_fit_up); }

/* 所选过渡共用 d、τ；形状点按各自速度差归一化 */
static double Cal_PooledSse(double tau, double S)
{
    double sse = 0.0, d = S - tau;
    uint8_t i, k;

    D_IWDG_Feed();
    for (i = 0U; i < s_meas.n_trans; i++)
    {
        const Trans_t *r = &s_meas.trans[i];
        double inv;
        if (!Cal_TransUse(r)) continue;
        inv = 500.0 / ((double)r->dv2 * 4.0);          /* Q2 厘度 → 单位速度差(ms) */
        for (k = 0U; k < N_POINT; k++)
        {
            double e = r->pts[k] * inv - Cal_StepShape(k_shape_ms[k], d, tau);
            sse += e * e;
        }
    }
    return sse;
}

/* 截距取各过渡去掉两端各1/5后的均值(纹波误差正负相消)，再黄金分割求 τ。返回参与记录数，0=不足 */
static uint8_t Cal_FitTau(uint8_t big, uint8_t up, double *tau, double *S, int32_t *s_lo, int32_t *s_hi)
{
    uint8_t i, it, n = 0U, k0;
    double a, b, c1, c2, f1, f2, sum = 0.0;

    s_fit_big = big; s_fit_up = up;
    for (i = 0U; i < s_meas.n_trans; i++) if (Cal_TransUse(&s_meas.trans[i])) s_med[n++] = s_meas.trans[i].s_us;
    if (n < ID_MIN_TRANS) return 0U;
    (void)Cal_Median(s_med, n, s_lo, s_hi);
    k0 = (uint8_t)(n / 5U);
    for (i = k0; i < n - k0; i++) sum += s_med[i];
    *S = sum / (double)(n - 2U * k0) / 1000.0;
    a = (*S - ID_DELAY_MAX_MS > ID_TAU_MIN_MS) ? *S - ID_DELAY_MAX_MS : ID_TAU_MIN_MS;
    b = (*S < ID_TAU_MAX_MS) ? *S : ID_TAU_MAX_MS;
    if (b <= a) return 0U;
    c1 = b - 0.618 * (b - a); c2 = a + 0.618 * (b - a);
    f1 = Cal_PooledSse(c1, *S); f2 = Cal_PooledSse(c2, *S);
    for (it = 0U; it < ID_GOLDEN_IT; it++)
    {
        if (f1 < f2) { b = c2; c2 = c1; f2 = f1; c1 = b - 0.618 * (b - a); f1 = Cal_PooledSse(c1, *S); }
        else         { a = c1; c1 = c2; f1 = f2; c2 = a + 0.618 * (b - a); f2 = Cal_PooledSse(c2, *S); }
    }
    *tau = (a + b) / 2;
    return n;
}

static uint8_t Cal_FitDynamics(CalModel_t *m)
{
    int32_t lo, hi;
    double S;

    m->n_small = Cal_FitTau(0U, 1U, &m->tau, &m->S, &m->s_lo, &m->s_hi);
    if (m->n_small == 0) return 0U;
    m->delay = m->S - m->tau;
    if (m->delay < 0.0) m->delay = 0.0;
    m->n_down = Cal_FitTau(0U, 0U, &m->tau_down, &S, &lo, &hi);
    m->n_big = Cal_FitTau(1U, 1U, &m->tau_big, &m->S_big, &lo, &hi);
    return (uint8_t)(m->n_big != 0);
}

/* 打印可整节替换的标定块；自检不过时块内带 #error */
static void Cal_Report(void)
{
    CalModel_t m;
    int32_t step_lo = 0, step_hi = 0, min_step, kv_uv, ka_nv, fc_mv, delay_us;
    double ka, ka_big, asym, g, vel, acc, dec;
    const char *err = 0;
    uint32_t stack;
    uint8_t i;

    memset(&m, 0, sizeof(m));
    for (i = 0U; i < s_meas.n_rstep; i++) s_med[i] = s_meas.rstep[i];
    min_step = Cal_Median(s_med, s_meas.n_rstep, &step_lo, &step_hi);
    if (s_meas.n_break < 4U || s_meas.n_rstep < 2U || min_step <= 0) { Cal_Fail(6U); return; }
    if (!Cal_FitSpeed(&m) || !Cal_FitDynamics(&m)) { Cal_Fail(7U); return; }
    m.fs = Cal_Median(s_meas.breakaway, s_meas.n_break, &m.fs_lo, &m.fs_hi);

    ka     = m.kv * m.tau / 1000.0;                                   /* 小信号，mV/(厘度/s^2) */
    ka_big = m.kv * m.tau_big / 1000.0;                               /* 大信号，只算额定能力 */
    asym = (m.kv_dir[0] - m.kv_dir[1]) / m.kv;
    g    = s_meas.g_min_pm / 1000.0;                                  /* 读数最慢处：同样读数速度需要的真实速度最高 */
    vel  = ID_MARGIN * m.vel_full * g;                                /* 厘度/s */
    acc  = ID_MARGIN * (m.cal_mv - m.fc) / ka_big * g;                /* 厘度/s^2 */
    dec  = ID_MARGIN * (m.cal_mv + m.fc) / ka_big * g;                /* 反压与摩擦同向 */
    kv_uv = Cal_Round(m.kv * 1000.0);
    ka_nv = Cal_Round(ka * 1e6);
    fc_mv = Cal_Round(m.fc);
    delay_us = Cal_Round(m.delay * 1000.0);

    if (m.tau < 10.0 || m.tau > 80.0)                 err = "tau 不在 10~80ms：对象不像一阶电机，检查机构是否卡滞/打滑";
    else if (asym > 0.1 || asym < -0.1)               err = "两方向 kv 相差超过10%：检查机构是否单边受力";
    else if (m.fs <= fc_mv)                           err = "起转电压不大于动摩擦：起转或斜坡测量异常";
    else if (m.delay >= ID_DELAY_MAX_MS - 0.1)        err = "延迟落在搜索上限：位置采样或驱动有额外滞后";
    else if (ka * ID_CHECK_BW < 2.0 * m.kv)           err = "ka·120rad/s < 2kv：加速系数偏小，速度环带宽会被削弱";
    else if (min_step > 80)                           err = "最小可靠位移超过0.8度：保持窗口会被撑大，检查齿隙/起转测量";
    else if (!s_meas.lin_ok)                          err = "线性化扫描覆盖不足：检查行程是否被挡";

    printf("\r\n/* ---- 电机标定（标定固件打印，整节替换 App_Model.h 对应型号块） ----\r\n");
    printf(" * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，\r\n");
    printf(" * 运行时电压按实测母线换算成占空比。额定能力为标定电压、读数最慢处的值(已留10%%余量)，只用于配置校验与出厂默认值。\r\n");
    printf(" * 本节由 %ld.%02ldV 实测标定生成：tau 小信号%ld.%ldms/大信号%ld.%ldms，两向kv %ld/%ld µV，过渡%ld/%ld条。\r\n",
           (long)(Cal_Round(m.cal_mv) / 1000), (long)(Cal_Round(m.cal_mv) % 1000 / 10),
           (long)Cal_Round(m.tau * 10) / 10, (long)Cal_Round(m.tau * 10) % 10,
           (long)Cal_Round(m.tau_big * 10) / 10, (long)Cal_Round(m.tau_big * 10) % 10,
           (long)Cal_Round(m.kv_dir[0] * 1000), (long)Cal_Round(m.kv_dir[1] * 1000), (long)m.n_small, (long)m.n_big);
    printf(" * 诊断：最小步长%ld(%u次)，读数峰峰%ld，编码器非线性±%ld厘度、最慢处斜率%ld‰。 */\r\n",
           (long)min_step, (unsigned)s_meas.n_rstep, (long)(s_meas.noise_hi - s_meas.noise_lo),
           (long)s_meas.lin_dev, (long)s_meas.g_min_pm);
    if (err) printf("#error \"标定自检未通过：%s\"\r\n", err);
    printf("#define MOTOR_CAL_MV          %8ld   /* 标定电压，mV */\r\n", (long)Cal_Round(m.cal_mv));
    printf("#define MOTOR_KV_UV           %8ld   /* 反电势斜率，µV/(厘度/s) */\r\n", (long)kv_uv);
    printf("#define MOTOR_KA_NV           %8ld   /* 加速系数，nV/(厘度/s²) */\r\n", (long)ka_nv);
    printf("#define MOTOR_FC_MV           %8ld   /* 动摩擦，mV */\r\n", (long)fc_mv);
    printf("#define MOTOR_FS_MV           %8ld   /* 起转电压，mV */\r\n", (long)m.fs);
    printf("#define MOTOR_DELAY_US        %8ld   /* 输入到响应的纯延迟，µs */\r\n", (long)delay_us);
    printf("#define MOTOR_NOISE_CDEG      %8ld   /* 静止读数噪声幅值(±)，厘度 */\r\n", (long)s_meas.noise);
    printf("#define MOTOR_MIN_STEP_CDEG   %8ld   /* 最小可靠位移，厘度 */\r\n", (long)min_step);
    printf("#define MOTOR_SIGN            %8d   /* 正电压使输出轴角度增大 = 1 */\r\n", (int)s_cal.sign);
    printf("#define MOTOR_RATED_VEL_DPS   %8ld   /* 额定最高速度，度/秒 */\r\n", (long)Cal_Round(vel / 100.0));
    printf("#define MOTOR_RATED_ACC_DPS2  %8ld   /* 额定零速加速度，度/秒² */\r\n", (long)Cal_Round(acc / 100.0));
    printf("#define MOTOR_RATED_DEC_DPS2  %8ld   /* 额定低速减速度，度/秒² */\r\n", (long)Cal_Round(dec / 100.0));
    printf("/* ---- 电机标定结束 ---- */\r\n");

    printf("@DIAG,kv_uv,%ld,%ld,fc_mv,%ld,%ld,pts,%ld,drop,%ld,res_mv,%ld\r\n",
           (long)Cal_Round(m.kv_dir[0] * 1000), (long)Cal_Round(m.kv_dir[1] * 1000),
           (long)Cal_Round(m.fc_dir[0]), (long)Cal_Round(m.fc_dir[1]),
           (long)m.n_speed, (long)m.n_drop, (long)m.res_mv);
    printf("@DIAG,small,S_us,%ld,%ld,%ld,tau_us,%ld,delay_us,%ld,n,%ld,down_tau_us,%ld,n,%ld\r\n", (long)m.s_lo,
           (long)Cal_Round(m.S * 1000.0), (long)m.s_hi, (long)Cal_Round(m.tau * 1000.0), (long)delay_us, (long)m.n_small,
           (long)Cal_Round(m.tau_down * 1000.0), (long)m.n_down);
    printf("@DIAG,big,S_us,%ld,tau_us,%ld,n,%ld,vel_full,%ld\r\n", (long)Cal_Round(m.S_big * 1000.0),
           (long)Cal_Round(m.tau_big * 1000.0), (long)m.n_big, (long)Cal_Round(m.vel_full));
    printf("@DIAG,fs_mv,%ld,%ld,%ld,n,%u\r\n", (long)m.fs_lo, (long)m.fs, (long)m.fs_hi, (unsigned)s_meas.n_break);
    printf("@DIAG,step_cdeg,%ld,%ld,%ld,noise,%ld\r\n", (long)step_lo, (long)min_step, (long)step_hi, (long)s_meas.noise);
    printf("@DIAG,lin,bins,%u,dev_cdeg,%ld,g_min_pm,%ld\r\n", (unsigned)s_meas.lin_bins, (long)s_meas.lin_dev, (long)s_meas.g_min_pm);
#ifndef CAL_HOST
    stack = Cal_StackUsed();
#else
    stack = 0U; (void)Cal_StackUsed;
#endif
    printf("@DIAG,stack,%lu,%lu,time_s,%lu\r\n", (unsigned long)stack,
           (unsigned long)((const uint8_t *)_eusrstack - (const uint8_t *)_heap_end), (unsigned long)(s_cal.t_total / 1000U));
    if (!err && stack + 256U > (uint32_t)((const uint8_t *)_eusrstack - (const uint8_t *)_heap_end))
        err = "栈余量不足256字节";
    if (err) printf("@WARN,%s\r\n", err);
    printf("@DONE\r\n");
}

/* ==================== 动作序列 ==================== */

static void Cal_NextAction(void)
{
    uint8_t act;

    if (s_cal.plan >= N_PLAN) { Cal_Enter(PH_DONE); return; }
    act = k_plan[s_cal.plan++];
    switch (act)
    {
    case ACT_NOISE:  printf("@SEG,%u/%u\r\n", (unsigned)s_cal.plan, (unsigned)N_PLAN); Cal_Enter(PH_NOISE); break;
    case ACT_GOTO_A: s_cal.dir = 1; Cal_Goto(SW_A, PH_NEXT); break;
    case ACT_LIN:    Cal_StartSweep(SW_LIN); break;
    case ACT_BUILD:  Cal_LinBuild(); Cal_Enter(PH_NEXT); break;
    case ACT_RAMP:   Cal_StartSweep(SW_RAMP); break;
    case ACT_SMALL:  Cal_StartSweep(SW_SMALL); break;
    case ACT_BIG:    Cal_StartSweep(SW_BIG); break;
    default:         Cal_Drive(0); Cal_Report(); if (s_cal.phase != PH_DONE) Cal_Enter(PH_DONE); break;
    }
}

/* ==================== 任务入口 ==================== */

/* 1ms 标定拍：先读位置，再出本拍指令 */
static void A_Calib_Min_Tick(void)
{
    int32_t raw;

    if (s_iap_stopped || s_cal.phase == PH_DONE) return;
    if (s_cal.phase == PH_BOOT && !D_UART_Tx_Idle()) return;   /* 先让 IAP 应答发完 */

    raw = A_Encoder_Read();
    if (raw == ENCODER_ANGLE_ERROR) { if (s_cal.phase == PH_SWEEP || ++s_cal.enc_bad >= 10U) { Cal_Fail(3U); return; } }
    else { s_cal.enc_bad = 0U; s_cal.raw = raw; }
    s_cal.pos = Cal_Lin(s_cal.raw);
    s_cal.avg_sum += s_cal.pos - s_cal.avg_buf[s_cal.avg_idx];
    s_cal.avg_buf[s_cal.avg_idx] = s_cal.pos;
    s_cal.avg_idx = (uint8_t)((s_cal.avg_idx + 1U) & 7U);
    s_cal.r4[s_cal.r4_idx] = s_cal.raw;
    s_cal.r4_idx = (uint8_t)((s_cal.r4_idx + 1U) & 3U);
    if (s_cal.phase != PH_BOOT) s_cal.t_total++;
    if (s_cal.phase >= PH_REST && (s_cal.raw < CAL_LO || s_cal.raw > CAL_HI)) { Cal_Fail(4U); return; }

    switch (s_cal.phase)
    {
    case PH_BOOT:
        Cal_Drive(0);
        if (++s_cal.timer < 1000U) return;
        if (s_cal.enc_bad) { Cal_Fail(3U); return; }
        s_meas.rest_cv = A_Voltage_Read();
        printf("\r\nCALIB start %u.%02uV，约50秒，中途请勿触碰输出轴；结束后复制打印出的标定块。\r\n",
               (unsigned)(s_meas.rest_cv / 100U), (unsigned)(s_meas.rest_cv % 100U));
        { uint8_t i; for (i = 0U; i < 8U; i++) s_cal.avg_buf[i] = s_cal.pos; s_cal.avg_sum = 8 * s_cal.pos; }
        Cal_Goto(CAL_CENTER, PH_SIGN);      /* 先到中点测极性 */
        return;

    case PH_GOTO:
        Cal_GotoTick();
        break;

    case PH_REST:
        Cal_Drive(0);
        if (s_cal.timer < REST_MS) break;
        s_cal.mark = s_cal.raw;
        Cal_Enter(s_cal.after);
        return;

    case PH_SIGN:
        Cal_Drive(SIGN_DUTY);               /* 极性未知，先按正占空比试推 */
        if (s_cal.timer + 1U < SIGN_MS) break;
        Cal_Drive(0);
        if (Fx_Abs32(s_cal.raw - s_cal.mark) < SIGN_MIN_CDEG) { Cal_Fail(2U); return; }
        s_cal.sign = (int8_t)((s_cal.raw > s_cal.mark) ? 1 : -1);
        s_cal.sign_known = 1U;
        printf("@SIGN,%d\r\n", (int)s_cal.sign);
        Cal_Enter(PH_NEXT);
        return;

    case PH_NEXT:
        Cal_NextAction();
        return;

    case PH_NOISE:
        if (Cal_NoiseTick()) { Cal_Enter(PH_NEXT); return; }
        break;

    case PH_SWEEP:
        Cal_SweepTick();
        break;

    case PH_RAMP:
        Cal_RampTick();
        break;

    case PH_BRAKE:
        if (Cal_BrakeTick()) { s_cal.dir = (int8_t)-s_cal.dir; Cal_Enter(PH_NEXT); return; }
        break;

    default:
        Cal_Drive(0);
        return;
    }
    s_cal.timer++;
}

/* IAP 身份校验通过后停标定、低阻卸力；应答发送期间也不得施力 */
static void Cal_StopForIAP(void)
{
    s_iap_stopped = 1U;
    D_Motor_Release(0U);
}

void A_Calib_Min_Init(void)
{
    Cal_StackPaint();               /* 最先涂栈，结束时 @DIAG,stack 给出实际峰值 */
    setvbuf(stdout, 0, _IONBF, 0);  /* 省stdio缓存RAM */
    memset(&s_cal, 0, sizeof(s_cal));
    memset(&s_u, 0, sizeof(s_u));
    memset(&s_meas, 0, sizeof(s_meas));
    s_cal.sign = 1;                 /* 测极性前按正接线 */
    s_cal.dir = 1;
    s_iap_stopped = 0U;
    IAP_SetStopHandler(Cal_StopForIAP);
    Cal_Drive(0);
}

/* 只处理 IAP，普通舵机指令不参与标定 */
static void Task_CalibIAP(void)
{
    uint8_t byte;

    while (D_UART1_Rx_Get(&byte)) IAP_Rx_Deal(byte);
    D_UART_Service();
    IAP_Service();
}

static void Task_CalibWatchdog(void) { D_IWDG_Feed(); }

static Task_t s_tasks[] = {
    TASK_DEF(A_Calib_Min_Tick,   1,  0),
    TASK_DEF(Task_CalibIAP,      1,  1),
    TASK_DEF(Task_CalibWatchdog, 50, 2),
};

void A_Calib_Min_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
}
