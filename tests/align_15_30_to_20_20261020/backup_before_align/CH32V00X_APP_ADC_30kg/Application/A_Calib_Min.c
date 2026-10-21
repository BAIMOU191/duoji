/*
 * A_Calib_Min.c —— 标定固件：在额定电压下按固定序列开环激励电机，在 MCU 上辨识电压域模型，
 * 最后打印一段可直接整节替换 A_Config.h 第2节的标定块(无 Python 步骤)。
 *
 * 激励序列(每段之间先预定位到起点)：
 *   noise    静止，测读数噪声
 *   ramp ±   3个位置×两方向的慢速占空比斜坡，测起转电压
 *   step ±   3个起点×6档占空比×双方向：预载/静置 -> 驱动 -> 反向制动 -> 静置
 *   pulse ±  短脉冲，测最小可靠位移
 *
 * 辨识只用积分量，电位器随位置的读数波纹影响很小：
 *   kv/fc  每个方向多档稳速做 U = kv·v + fc 直线拟合
 *   τ/延迟 阶跃位移：后段渐近线截距给 d+τ，前段形状再定 τ(ka = kv·τ)
 *   制动比 反向同幅电压的刹停距离与模型对比
 *   fs、噪声、最小可靠位移直接测；额定 V/A/D 由模型在标定电压下算出并留10%余量
 * 自检不过时标定块里带 #error，粘贴后编译即报错，不会悄悄用上可疑参数。
 *
 * 串口(当前配置的波特率)：@SIGN、@SEG 为进度；标定块之后 @DONE；
 *   @FAIL,<码>,<位置>：2=方向测不出 3=编码器读不到 4=冲出行程 5=预定位超时 6=起转/脉冲测量不足 7=阶跃数据不足以辨识
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

#define CAL_GUARD_CDEG   500       /* 距可读两端还剩这么多就判越界 */
#define CAL_LO           (ENCODER_ANGLE_LO + CAL_GUARD_CDEG)
#define CAL_HI           (ENCODER_ANGLE_HI - CAL_GUARD_CDEG)
#define CAL_START_LO     2000      /* 正向阶跃起点 */
#define CAL_START_HI     ((int32_t)ENCODER_POT_SPAN_CDEG - 2000) /* 反向阶跃起点 */
#define CAL_CENTER       ((int32_t)ENCODER_POT_SPAN_CDEG / 2)

#define GOTO_TOL_CDEG    300       /* 预定位到位判据，标定不需要精确 */
#define GOTO_FAST_DUTY   900
#define GOTO_SLOW_DUTY   600
#define GOTO_SLOW_CDEG   2500
#define GOTO_CHECK_MS    150U      /* 每隔这么久检查一次误差是否在减小，否则掉头(极性未知时) */
#define GOTO_TIMEOUT_MS  8000U
#define REST_MS          300U

#define SIGN_DUTY        900
#define SIGN_MS          200U
#define SIGN_MIN_CDEG    50

#define NOISE_MS         1000U
#define RAMP_PRE_DUTY    1000      /* 同向预载：把齿隙顶到测试方向 */
#define RAMP_PRE_MS      150U
#define RAMP_STEP_MS     10U       /* 每10ms抬高 RAMP_STEP_DUTY */
#define RAMP_STEP_DUTY   3
#define RAMP_MAX_DUTY    900
#define PRE_MS           50U       /* 阶跃/方波/脉冲前的静止 */
#define STEP_RUN_CDEG   18000      /* 驱动段最长行程，给较快电机留出300~400ms稳速窗口 */
#define STEP_RUN_MS      600U
#define STEP_BRAKE_MS    150U
#define SPEED_FROM_MS    300U      /* 满占空比稳速测量窗口：驱动后 300~400ms */
#define SPEED_TO_MS      400U
#define START_MS         40U       /* 满占空比起步位移的测量时长 */
#define PULSE_DUTY       1400
#define STEP_PRELOAD_MS  100U
#define STEP_HISTORY    20U
static const uint16_t k_step_ms[] = { 5, 10, 20, 30, 40, 60, 80, 120, 200 };
#define N_POINT (sizeof(k_step_ms) / sizeof(k_step_ms[0]))
#define STEP_SPEED_VALID (1U << N_POINT)
#define STEP_BRAKE_VALID (1U << (N_POINT + 1U))

enum { K_NOISE = 0, K_RAMP, K_STEP, K_PULSE };
typedef struct { uint8_t kind; int8_t dir; int16_t arg; int32_t start; } CalSeg_t;

static const int16_t k_step_duty[] = { 360, 720, 1080, 1440, 1920, 2400 };
static const int16_t k_step_offset[] = { 0, 373, 937 }; /* 分散起点，避免单一位置波纹主导拟合 */
static const int16_t k_pulse_ms[]  = { 2, 3, 4, 6, 8, 12, 20, 32 };
static const int32_t k_ramp_pos[]  = { ENCODER_POT_SPAN_CDEG / 4, ENCODER_POT_SPAN_CDEG / 2,
                                       ENCODER_POT_SPAN_CDEG * 3 / 4 };
#define N_STEP   (sizeof(k_step_duty) / sizeof(k_step_duty[0]))
#define N_START  (sizeof(k_step_offset) / sizeof(k_step_offset[0]))
#define N_RECORD (2U * N_STEP * N_START)
#define N_PULSE  (sizeof(k_pulse_ms) / sizeof(k_pulse_ms[0]))
#define N_RAMP   (sizeof(k_ramp_pos) / sizeof(k_ramp_pos[0]))
#define N_SEG    (1U + 2U * N_RAMP + N_RECORD + 2U * N_PULSE)

enum { PH_BOOT = 0, PH_GOTO, PH_REST, PH_SIGN, PH_PRELOAD, PH_RUN, PH_DONE };

static struct {
    uint8_t  phase, after, seg, fail, enc_bad, flip;
    int8_t   sign;
    uint16_t timer, t_mark;
    int32_t  pos, mark, peak, target, goto_err;
    int32_t  avg_buf[8], avg_sum;  /* 最近8拍位置之和：缓升判动与步长测量用均值，噪声降到约1/3 */
    uint8_t  avg_idx;
    int16_t  duty, level, quiet_level;
    uint8_t  sub;                 /* 段内子阶段 */
    CalSeg_t cur;
} s_cal;

/* 直接测量项 */
static struct {
    uint16_t rest_cv;             /* 开始时的静止母线电压 */
    int32_t  noise_lo, noise_hi;
    int32_t  noise;               /* 读数噪声幅值(±)，取3倍标准差：单个毛刺不会把它抬高 */
    int32_t  noise_ref, noise_sum, noise_sum2;
    uint16_t noise_n;
    int32_t  ramp_step[2 * N_RAMP];   /* 缓升起转、动了立即撤力后的净位移：末端修正的真实最小步长 */
    uint8_t  n_rstep;
    int32_t  breakaway[2 * N_RAMP];
    uint8_t  n_break;
    int32_t  pulse[N_PULSE][2];   /* 每个脉宽两方向的位移 */
    int32_t  p_speed;
} s_meas;

/* 每个占空比/方向一条曲线。mask 标记实际测到的点，行程提前结束的点不补零当数据。 */
typedef struct {
    int32_t speed, brake_speed;
    uint32_t cv_sum;
    uint16_t cv_n, mask;
    int16_t pos[N_POINT], brake_dist; /* 全部在受保护的电位器行程内，压缩RAM */
} StepRecord_t;
typedef char guard_step_position_range[(CAL_HI - CAL_LO < 32767) ? 1 : -1];
static StepRecord_t s_step[N_RECORD];
static int32_t s_step_hist[STEP_HISTORY], s_step_zero_sum, s_brake_start;
static uint8_t s_step_hist_idx;

static StepRecord_t *Cal_StepRecord(void)
{
    return &s_step[s_cal.seg - 1U - 2U * N_RAMP];
}

static uint8_t s_iap_stopped;

/* ==================== 激励序列 ==================== */

/* 第 n 段的定义：噪声 -> 斜坡 -> 阶跃 -> 脉冲，方向正反交替 */
static CalSeg_t Cal_Segment(uint8_t n)
{
    CalSeg_t g = { K_NOISE, 1, 0, CAL_CENTER };

    if (n == 0U) return g;
    n--;
    g.dir = (int8_t)((n & 1U) ? -1 : 1);
    if (n < 2U * N_RAMP) { g.kind = K_RAMP; g.start = k_ramp_pos[n / 2U]; return g; }
    n -= (uint8_t)(2U * N_RAMP);
    if (n < N_RECORD)
    {
        int32_t offset = k_step_offset[n / (2U * N_STEP)];
        g.kind = K_STEP; g.arg = k_step_duty[(n / 2U) % N_STEP];
        g.start = (g.dir > 0) ? CAL_START_LO + offset : CAL_START_HI - offset;
        return g;
    }
    n -= (uint8_t)N_RECORD;
    g.kind = K_PULSE; g.arg = k_pulse_ms[n / 2U]; g.start = CAL_CENTER;
    return g;
}

static void Cal_Drive(int16_t duty)
{
    s_cal.duty = duty;
    D_Motor_Set(duty);
}

/* 方向 dir(角度增大=+1) 的幅值换成物理占空比 */
static int16_t Cal_Out(int8_t dir, int16_t mag) { return (int16_t)(mag * dir * s_cal.sign); }

static void Cal_Fail(uint8_t code)
{
    Cal_Drive(0);
    if (s_cal.fail == 0U) printf("@FAIL,%u,%ld\r\n", (unsigned)code, (long)s_cal.pos);
    s_cal.fail = code;
    s_cal.phase = PH_DONE;
}

static void Cal_Enter(uint8_t phase) { s_cal.phase = phase; s_cal.timer = 0U; s_cal.sub = 0U; }

/* 预定位到 target，到位静置后进入 after 阶段 */
static void Cal_Goto(int32_t target, uint8_t after)
{
    s_cal.target   = target;
    s_cal.after    = after;
    s_cal.goto_err = target - s_cal.pos;
    s_cal.flip     = 0U;
    Cal_Enter(PH_GOTO);
}

/* 预定位一拍：bang-bang 靠近目标，接近时换小幅值；误差不减小就掉头(兜住极性未知或判错) */
static void Cal_GotoTick(void)
{
    int32_t err = s_cal.target - s_cal.pos;
    int16_t mag = (Fx_Abs32(err) > GOTO_SLOW_CDEG) ? GOTO_FAST_DUTY : GOTO_SLOW_DUTY;

    if (Fx_Abs32(err) <= GOTO_TOL_CDEG) { Cal_Drive(0); Cal_Enter(PH_REST); return; }
    if (s_cal.timer >= GOTO_TIMEOUT_MS) { Cal_Fail(5U); return; }
    if (s_cal.timer > 0U && s_cal.timer % GOTO_CHECK_MS == 0U)
    {
        if (Fx_Abs32(err) >= Fx_Abs32(s_cal.goto_err)) s_cal.flip ^= 1U;
        s_cal.goto_err = err;
    }
    Cal_Drive(Cal_Out((int8_t)(((err > 0) != (s_cal.flip != 0U)) ? 1 : -1), mag));
}

/* ==================== 段内一拍 ==================== */

/* 返回1=本段结束 */
static uint8_t Cal_RunTick(void)
{
    uint16_t t = s_cal.timer;
    int8_t dir = s_cal.cur.dir;

    switch (s_cal.cur.kind)
    {
    case K_NOISE:
        Cal_Drive(0);
        if (t >= 50U)
        {
            int32_t d;
            if (t == 50U) { s_meas.noise_lo = s_meas.noise_hi = s_meas.noise_ref = s_cal.pos; }
            if (s_cal.pos < s_meas.noise_lo) s_meas.noise_lo = s_cal.pos;
            if (s_cal.pos > s_meas.noise_hi) s_meas.noise_hi = s_cal.pos;
            d = s_cal.pos - s_meas.noise_ref;
            s_meas.noise_sum += d; s_meas.noise_sum2 += d * d; s_meas.noise_n++;
        }
        if (t < NOISE_MS) return 0U;
        {   /* 3σ：σ×10 = √(100·方差) */
            int32_t n = s_meas.noise_n;
            int32_t var100 = (int32_t)((((int64_t)s_meas.noise_sum2 * n - (int64_t)s_meas.noise_sum * s_meas.noise_sum) * 100)
                                       / ((int64_t)n * n));
            s_meas.noise = ((int32_t)Fx_Isqrt((uint32_t)var100) * 3 + 5) / 10;
            if (s_meas.noise < 1) s_meas.noise = 1;
        }
        return 1U;

    case K_RAMP:
        if (s_cal.sub == 0U)
        {   /* 预载 -> 静置 -> 斜坡 */
            if (t < RAMP_PRE_MS) { Cal_Drive(Cal_Out(dir, RAMP_PRE_DUTY)); return 0U; }
            Cal_Drive(0);
            if (t < RAMP_PRE_MS + REST_MS) return 0U;
            s_cal.sub = 1U; s_cal.t_mark = t; s_cal.mark = s_cal.avg_sum; s_cal.level = 0; s_cal.quiet_level = 0;
        }
        if (s_cal.sub == 1U)
        {   /* 8拍均值越过阈值判动了(噪声幅值×3/4，约为两个8拍均值之差的5σ，斜坡几百拍里不误判)；
             * 起转值取最后一次仍在一半阈值内时的幅值。阈值就是测得步长的下限，所以不用单拍读数：
             * 噪声14时单拍3倍噪声阈值是42，会把步长测成1度以上 */
            int32_t thr = (s_meas.noise * 3 / 4 > 4) ? s_meas.noise * 3 / 4 : 4;
            int32_t moved = Fx_Abs32(s_cal.avg_sum - s_cal.mark) / 8;
            if (moved * 2 <= thr) s_cal.quiet_level = s_cal.level;
            if (moved >= thr || s_cal.level >= RAMP_MAX_DUTY)
            {
                if (s_cal.level < RAMP_MAX_DUTY && s_meas.n_break < 2U * N_RAMP)
                    s_meas.breakaway[s_meas.n_break++] = (int32_t)s_cal.quiet_level * A_Voltage_Read() * 10 / CFG_PWM_FULL;
                s_cal.sub = (s_cal.level < RAMP_MAX_DUTY) ? 2U : 3U;   /* 3=爬满没动，不量步长 */
                s_cal.t_mark = t; Cal_Drive(0);
                return 0U;
            }
            if ((uint16_t)(t - s_cal.t_mark) % RAMP_STEP_MS == 0U) s_cal.level += RAMP_STEP_DUTY;
            Cal_Drive(Cal_Out(dir, s_cal.level));
            return 0U;
        }
        Cal_Drive(0);
        if ((uint16_t)(t - s_cal.t_mark) < REST_MS) return 0U;
        if (s_cal.sub == 2U && s_meas.n_rstep < 2U * N_RAMP)   /* 撤力停稳后这一下到底走了多远(8拍均值) */
            s_meas.ramp_step[s_meas.n_rstep++] = (Fx_Abs32(s_cal.avg_sum - s_cal.mark) + 4) / 8;
        return 1U;

    case K_STEP:
        if (t < PRE_MS) { Cal_Drive(0); s_cal.mark = s_cal.pos;
            if (t >= PRE_MS - 10U) s_step_zero_sum += s_cal.pos;
            s_step_hist[s_step_hist_idx] = s_cal.pos;
            s_step_hist_idx = (uint8_t)((s_step_hist_idx + 1U) % STEP_HISTORY); return 0U; }
        if (s_cal.sub == 0U)
        {
            StepRecord_t *r = Cal_StepRecord();
            uint16_t elapsed = (uint16_t)(t - PRE_MS);
            uint8_t k;
            if (elapsed == 0U) s_cal.mark = s_step_zero_sum / 10;
            for (k = 0U; k < N_POINT; k++) if (elapsed == k_step_ms[k]) {
                r->pos[k] = dir * (s_cal.pos - s_cal.mark);
                r->mask |= (uint16_t)(1U << k);
            }
            if (elapsed <= 200U) { r->cv_sum += A_Voltage_Read(); r->cv_n++; }
            if (elapsed == SPEED_FROM_MS) s_meas.p_speed = s_cal.pos;
            if (elapsed == SPEED_TO_MS) {
                r->speed = dir * (s_cal.pos - s_meas.p_speed) * (1000 / (SPEED_TO_MS - SPEED_FROM_MS));
                r->mask |= STEP_SPEED_VALID;
            }
        }
        if (s_cal.sub == 0U)
        {   /* 驱动：到行程或时间上限 */
            Cal_Drive(Cal_Out(dir, s_cal.cur.arg));
            if (dir * (s_cal.pos - s_cal.mark) >= STEP_RUN_CDEG || t >= PRE_MS + STEP_RUN_MS)
            { s_cal.sub = 1U; s_cal.t_mark = (uint16_t)(t + 1U); s_cal.peak = dir * s_cal.pos; }
            s_step_hist[s_step_hist_idx] = s_cal.pos;
            s_step_hist_idx = (uint8_t)((s_step_hist_idx + 1U) % STEP_HISTORY);
            return 0U;
        }
        if (s_cal.sub == 1U)
        {   /* 反向制动：位置开始回退或时间上限 */
            StepRecord_t *r = Cal_StepRecord();
            if (t == s_cal.t_mark) {
                s_brake_start = dir * s_cal.pos;
                r->brake_speed = dir * (s_cal.pos - s_step_hist[s_step_hist_idx]) * (1000 / STEP_HISTORY);
                s_cal.peak = s_brake_start;
            }
            Cal_Drive(Cal_Out((int8_t)-dir, s_cal.cur.arg));
            if (dir * s_cal.pos > s_cal.peak) s_cal.peak = dir * s_cal.pos;
            if (dir * s_cal.pos < s_cal.peak - 2 * s_meas.noise - 10
                || (uint16_t)(t - s_cal.t_mark) >= STEP_BRAKE_MS)
            {
                if ((uint16_t)(t - s_cal.t_mark) < STEP_BRAKE_MS && r->brake_speed > 0) {
                    r->brake_dist = s_cal.peak - s_brake_start;
                    r->mask |= STEP_BRAKE_VALID;
                }
                s_cal.sub = 2U; s_cal.t_mark = t;
            }
            return 0U;
        }
        Cal_Drive(0);
        return (uint8_t)((uint16_t)(t - s_cal.t_mark) >= REST_MS);

    default: /* K_PULSE */
        if (t == PRE_MS) s_cal.mark = s_cal.pos;
        Cal_Drive((t >= PRE_MS && t < PRE_MS + (uint16_t)s_cal.cur.arg) ? Cal_Out(dir, PULSE_DUTY) : 0);
        if (t < PRE_MS + (uint16_t)s_cal.cur.arg + REST_MS) return 0U;
        s_meas.pulse[(s_cal.seg - 1U - 2U * N_RAMP - N_RECORD) / 2U][dir > 0 ? 0 : 1]
            = Fx_Abs32(s_cal.pos - s_cal.mark);
        return 1U;
    }
}

/* ==================== 结果 ==================== */

static int32_t Cal_Median(int32_t *v, uint8_t n)
{
    uint8_t i, j;

    for (i = 1U; i < n; i++)
        for (j = i; j > 0U && v[j - 1U] > v[j]; j--) { int32_t t = v[j]; v[j] = v[j - 1U]; v[j - 1U] = t; }
    return n ? v[n / 2U] : 0;
}

/* 脉冲法：从最短脉宽起，第一个两方向都超过3倍噪声的脉宽，取两方向较小位移。
 * 固定幅值的短脉冲受电气滞后影响，推得动时往往已冲出一大截，只作诊断和后备 */
static int32_t Cal_PulseStep(void)
{
    int32_t thr = (3 * s_meas.noise > 10) ? 3 * s_meas.noise : 10;
    uint8_t i;

    for (i = 0U; i < N_PULSE; i++)
    {
        int32_t m = (s_meas.pulse[i][0] < s_meas.pulse[i][1]) ? s_meas.pulse[i][0] : s_meas.pulse[i][1];
        if (m >= thr) return m;
    }
    return 0; /* 不能把没有测出的最小位移伪造为噪声阈值 */
}

/* 最小可靠位移：缓升起转、一动就撤力后的净位移中位数(末端修正的真实工况，同 30kg1)；不足2次时退回脉冲法 */
static int32_t Cal_MinStep(void)
{
    static int32_t v[2 * N_RAMP];
    uint8_t i;

    if (s_meas.n_rstep < 2U) return Cal_PulseStep();
    for (i = 0U; i < s_meas.n_rstep; i++) v[i] = s_meas.ramp_step[i];
    return Cal_Median(v, s_meas.n_rstep);
}

/* ==================== 辨识(全部在 MCU 上，停稳后做一次，用 double) ==================== */

#define ID_TAU_MIN_MS   3.0     /* τ 搜索范围 */
#define ID_TAU_MAX_MS 150.0
#define ID_DELAY_MAX_MS 8.0     /* 延迟搜索上限：落在上限说明对象不符合一阶+延迟 */
#define ID_FIT_DUTY    1080     /* 只用这档以上的阶跃拟合 τ/延迟与制动：低档受起转过程影响 */
#define ID_ASYM_MS      120U    /* 这个时刻以后的点用来求渐近线截距(d+τ) */
#define ID_MARGIN       0.9     /* 额定能力留10%余量 */
#define ID_CHECK_BW     120.0   /* 自检用的速度环带宽，与出厂默认值一致 */

static double Cal_Exp(double x)   /* x ≤ 0：缩小参数后泰勒展开再平方回去，不依赖 libm */
{
    double s = 1.0, t = 1.0;
    uint8_t n = 0U, i;

    if (x < -40.0) return 0.0;
    while (x < -0.5) { x *= 0.5; n++; }
    for (i = 1U; i <= 8U; i++) { t *= x / i; s += t; }
    while (n--) s *= s;
    return s;
}

static double Cal_Ln(double y)    /* y > 0：按2的幂归一化后用 atanh 级数 */
{
    double z, z2, s = 0.0, t;
    int8_t k = 0;
    uint8_t i;

    while (y > 2.0) { y *= 0.5; k++; }
    while (y < 1.0) { y *= 2.0; k--; }
    z = (y - 1.0) / (y + 1.0); z2 = z * z; t = z;
    for (i = 1U; i <= 15U; i += 2U) { s += t / i; t *= z2; }
    return 2.0 * s + k * 0.69314718056;
}

static int32_t Cal_Round(double x) { return (int32_t)((x >= 0.0) ? x + 0.5 : x - 0.5); }

/* 一阶+纯延迟的阶跃位移：x(t) = v∞·(s − τ(1 − e^(−s/τ)))，s = t − d */
static double Cal_StepShape(double t, double d, double tau)
{
    double s = t - d;
    return (s <= 0.0) ? 0.0 : s - tau * (1.0 - Cal_Exp(-s / tau));
}

static double Cal_Volt(const StepRecord_t *r, uint8_t i)
{
    return (double)k_step_duty[(i / 2U) % N_STEP] * (r->cv_n ? (double)r->cv_sum / r->cv_n : s_meas.rest_cv) * 10.0 / CFG_PWM_FULL;
}

typedef struct {
    double kv, fc, kv_dir[2], fc_dir[2];  /* mV/(厘度/s)、mV */
    double tau, delay;                    /* ms */
    double brake;                         /* 反向制动实测出力 / 模型出力：只用于算额定减速度，不输出参数 */
    double cal_mv, vel_full;              /* 标定电压、满占空比两向较小的稳速 */
    int32_t fs, n_fit;
} CalModel_t;

/* kv/fc：每个方向对多档稳速做 U = kv·v + fc 的直线拟合 */
static uint8_t Cal_FitSpeed(CalModel_t *m)
{
    uint8_t d, i, ok = 1U;
    double cv_sum = 0.0;
    uint16_t cv_n = 0U;

    m->vel_full = 1e9;
    for (d = 0U; d < 2U; d++)
    {
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, det;

        for (i = d; i < N_RECORD; i += 2U)
        {
            const StepRecord_t *r = &s_step[i];
            double v = r->speed, u = Cal_Volt(r, i);

            if (!(r->mask & STEP_SPEED_VALID) || v < 500) continue;
            n += 1; sx += v; sy += u; sxx += v * v; sxy += v * u;
            if (r->cv_n) { cv_sum += (double)r->cv_sum / r->cv_n; cv_n++; }
            if (k_step_duty[(i / 2U) % N_STEP] == CFG_PWM_FULL && v < m->vel_full) m->vel_full = v;
        }
        det = n * sxx - sx * sx;
        if (n < 3 || det <= 0) { ok = 0U; continue; }
        m->kv_dir[d] = (n * sxy - sx * sy) / det;
        m->fc_dir[d] = (sy - m->kv_dir[d] * sx) / n;
    }
    m->kv = (m->kv_dir[0] + m->kv_dir[1]) / 2;
    m->fc = (m->fc_dir[0] + m->fc_dir[1]) / 2;
    m->cal_mv = cv_n ? cv_sum / cv_n * 10.0 : s_meas.rest_cv * 10.0;
    return (uint8_t)(ok && m->kv > 0 && m->vel_full < 1e9);
}

/* 渐近速度用 120→200ms 的局部斜率：与渐近线截距取自同一段行程，不受 300~400ms 段的电位器刻度差、
 * 电压变化影响(用远处测的稳速会把 d+τ 估偏，实测出现过延迟被估成0)。两点不全时退回稳速 */
static double Cal_LocalSpeed(const StepRecord_t *r)
{
    const uint16_t m = (uint16_t)((1U << (N_POINT - 2U)) | (1U << (N_POINT - 1U)));

    if ((r->mask & m) == m && r->pos[N_POINT - 1U] > r->pos[N_POINT - 2U])
        return (r->pos[N_POINT - 1U] - r->pos[N_POINT - 2U]) * 1000.0 / (k_step_ms[N_POINT - 1U] - k_step_ms[N_POINT - 2U]);
    return r->speed;
}

/* 阶跃位移(积分量，对电位器随位置的波纹不敏感)反解 τ 与延迟：
 * 后段渐近线 x ≈ v∞·(t − d − τ) 给出 S = d + τ；再在 τ∈(0,S) 上用前段形状做一维黄金分割 */
static double Cal_ShapeSse(double tau, double S)
{
    double sse = 0.0, d = S - tau;
    uint8_t i, k;

    for (i = 0U; i < N_RECORD; i++)
    {
        const StepRecord_t *r = &s_step[i];
        if (k_step_duty[(i / 2U) % N_STEP] < ID_FIT_DUTY || !(r->mask & STEP_SPEED_VALID)) continue;
        for (k = 0U; k < N_POINT && k_step_ms[k] < ID_ASYM_MS; k++)
        {
            double e;
            if (!(r->mask & (1U << k))) continue;
            e = r->pos[k] - Cal_LocalSpeed(r) / 1000.0 * Cal_StepShape(k_step_ms[k], d, tau);
            sse += e * e;
        }
    }
    return sse;
}

static uint8_t Cal_FitDynamics(CalModel_t *m)
{
    double S = 0.0, a, b, c1, c2, f1, f2;
    uint8_t i, k, it;
    int32_t n = 0;

    for (i = 0U; i < N_RECORD; i++)
    {
        const StepRecord_t *r = &s_step[i];
        if (k_step_duty[(i / 2U) % N_STEP] < ID_FIT_DUTY || !(r->mask & STEP_SPEED_VALID)) continue;
        for (k = 0U; k < N_POINT; k++)
            if (k_step_ms[k] >= ID_ASYM_MS && (r->mask & (1U << k))) { S += k_step_ms[k] - r->pos[k] * 1000.0 / Cal_LocalSpeed(r); n++; }
    }
    if (n < 4) return 0U;
    S /= n;
    m->n_fit = n;
    a = (S - ID_DELAY_MAX_MS > ID_TAU_MIN_MS) ? S - ID_DELAY_MAX_MS : ID_TAU_MIN_MS;
    b = (S < ID_TAU_MAX_MS) ? S : ID_TAU_MAX_MS;
    if (b <= a) return 0U;
    c1 = b - 0.618 * (b - a); c2 = a + 0.618 * (b - a);
    f1 = Cal_ShapeSse(c1, S); f2 = Cal_ShapeSse(c2, S);
    for (it = 0U; it < 40U; it++)
    {
        D_IWDG_Feed();
        if (f1 < f2) { b = c2; c2 = c1; f2 = f1; c1 = b - 0.618 * (b - a); f1 = Cal_ShapeSse(c1, S); }
        else         { a = c1; c1 = c2; f1 = f2; c2 = a + 0.618 * (b - a); f2 = Cal_ShapeSse(c2, S); }
    }
    m->tau = (a + b) / 2;
    m->delay = S - m->tau;
    if (m->delay < 0.0) m->delay = 0.0;
    return 1U;
}

/* 制动侧出力比 g：反向同幅电压时刹停距离 = v0·d + τ·(v0 − c·ln(1 + v0/c))，c = (g·U + fc)/kv，按距离二分求 g */
static double Cal_BrakeDist(double g, double u, double v0, const CalModel_t *m)
{
    double c = (g * u + m->fc) / m->kv;
    return v0 * m->delay / 1000.0 + m->tau / 1000.0 * (v0 - c * Cal_Ln(1.0 + v0 / c));
}

static void Cal_FitBrake(CalModel_t *m)
{
    static int16_t g[N_RECORD];   /* 千分比，省 RAM */
    uint8_t i, n = 0U, j, it;

    for (i = 0U; i < N_RECORD; i++)
    {
        const StepRecord_t *r = &s_step[i];
        double u = Cal_Volt(r, i), lo = 0.1, hi = 3.0;

        if (k_step_duty[(i / 2U) % N_STEP] < ID_FIT_DUTY || !(r->mask & STEP_BRAKE_VALID) || r->brake_speed < 1000) continue;
        for (it = 0U; it < 30U; it++)
        {
            double mid = (lo + hi) / 2;
            if (Cal_BrakeDist(mid, u, r->brake_speed, m) > r->brake_dist) lo = mid; else hi = mid;
        }
        g[n++] = (int16_t)Cal_Round((lo + hi) * 500.0);
    }
    for (i = 1U; i < n; i++)   /* 中位数 */
        for (j = i; j > 0U && g[j - 1U] > g[j]; j--) { int16_t t = g[j]; g[j] = g[j - 1U]; g[j - 1U] = t; }
    m->brake = n ? g[n / 2U] / 1000.0 : 1.0;
}

/* 打印可直接整节替换 A_Config.h 第2节的标定块；自检不过时块里带 #error，粘贴后编译即报错 */
static void Cal_Report(void)
{
    CalModel_t m;
    int32_t min_step = Cal_MinStep(), kv_uv, ka_nv, fc_mv, delay_us;
    double ka, asym, vel, acc, dec;
    const char *err = 0;

    memset(&m, 0, sizeof(m));
    if (s_meas.n_break < 4U || min_step == 0) { Cal_Fail(6U); return; }
    if (!Cal_FitSpeed(&m) || !Cal_FitDynamics(&m)) { Cal_Fail(7U); return; }
    Cal_FitBrake(&m);
    m.fs = Cal_Median(s_meas.breakaway, s_meas.n_break);

    ka   = m.kv * m.tau / 1000.0;                                     /* mV/(厘度/s^2) */
    asym = (m.kv_dir[0] - m.kv_dir[1]) / m.kv;
    vel  = ID_MARGIN * m.vel_full;                                     /* 厘度/s */
    acc  = (ID_MARGIN * m.cal_mv - m.fc) / ka;                         /* 厘度/s^2 */
    dec  = (ID_MARGIN * m.brake * m.cal_mv + m.fc) / ka;
    kv_uv = Cal_Round(m.kv * 1000.0);
    ka_nv = Cal_Round(ka * 1e6);
    fc_mv = Cal_Round(m.fc);
    delay_us = Cal_Round(m.delay * 1000.0);

    if (m.tau < 10.0 || m.tau > 80.0)                 err = "tau 不在 10~80ms：对象不像一阶电机，检查机构是否卡滞/打滑";
    else if (asym > 0.1 || asym < -0.1)               err = "两方向 kv 相差超过10%：检查机构是否单边受力";
    else if (m.fs <= fc_mv)                           err = "起转电压不大于动摩擦：起转或斜坡测量异常";
    else if (m.delay >= ID_DELAY_MAX_MS - 0.1)        err = "延迟落在搜索上限：位置采样或驱动有额外滞后";
    else if (ka * ID_CHECK_BW < 2.0 * m.kv)           err = "ka·120rad/s < 2kv：加速系数偏小，速度环带宽会被削弱";
    else if (m.brake < 0.3 || m.brake > 2.5)          err = "制动出力比超出 30%~250%：制动测量异常，额定减速度不可信";
    else if (min_step > 80)                           err = "最小可靠位移超过0.8度：保持窗口会被撑大，检查齿隙/起转测量";

    printf("\r\n/* ==================== 2. 电机标定（标定固件打印，整节替换） ====================\r\n");
    printf(" * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，\r\n");
    printf(" * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%%余量)，只用于配置校验与出厂默认值。\r\n");
    printf(" * 本节由 %ld.%02ldV 实测标定生成：tau=%ld.%ldms，两向kv %ld/%ld µV，拟合点%ld。\r\n",
           (long)(Cal_Round(m.cal_mv) / 1000), (long)(Cal_Round(m.cal_mv) % 1000 / 10),
           (long)Cal_Round(m.tau * 10) / 10, (long)Cal_Round(m.tau * 10) % 10,
           (long)Cal_Round(m.kv_dir[0] * 1000), (long)Cal_Round(m.kv_dir[1] * 1000), (long)m.n_fit);
    printf(" * 诊断：最小步长 缓升法%ld(%u次)/脉冲法%ld，读数峰峰%ld，制动出力比%ld%%(已计入额定减速度)。 */\r\n",
           (long)min_step, (unsigned)s_meas.n_rstep, (long)Cal_PulseStep(), (long)(s_meas.noise_hi - s_meas.noise_lo),
           (long)Cal_Round(m.brake * 100.0));
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
    printf("/* ==================== 电机标定结束 ==================== */\r\n");
    if (err) printf("@WARN,%s\r\n", err);
    printf("@DONE\r\n");
}

static void Cal_StartSegment(void)
{
    static const char *const k_name[] = { "noise", "ramp", "step", "pulse" };

    printf("@SEG,%u/%u,%s\r\n", (unsigned)(s_cal.seg + 1U), (unsigned)N_SEG, k_name[s_cal.cur.kind]);
    s_step_zero_sum = 0;
    s_step_hist_idx = 0U;
    memset(s_step_hist, 0, sizeof(s_step_hist));
    Cal_Enter(s_cal.cur.kind == K_STEP ? PH_PRELOAD : PH_RUN);
}

static void Cal_NextSegment(void)
{
    if (++s_cal.seg >= N_SEG) { Cal_Drive(0); Cal_Report(); Cal_Enter(PH_DONE); return; }
    s_cal.cur = Cal_Segment(s_cal.seg);
    Cal_Goto(s_cal.cur.start, PH_RUN);
}

/* ==================== 任务入口 ==================== */

/* 1ms 标定拍：读编码器 -> 行程保护 -> 状态机 */
static void A_Calib_Min_Tick(void)
{
    int32_t raw;

    if (s_iap_stopped || s_cal.phase == PH_DONE) return;
    if (s_cal.phase == PH_BOOT && !D_UART_Tx_Idle()) return;   /* 先让 IAP 应答发完 */

    raw = A_Encoder_Read();
    if (raw == ENCODER_ANGLE_ERROR) { if (s_cal.phase == PH_RUN || ++s_cal.enc_bad >= 10U) { Cal_Fail(3U); return; } }
    else { s_cal.enc_bad = 0U; s_cal.pos = raw; }
    s_cal.avg_sum += s_cal.pos - s_cal.avg_buf[s_cal.avg_idx];
    s_cal.avg_buf[s_cal.avg_idx] = s_cal.pos;
    s_cal.avg_idx = (uint8_t)((s_cal.avg_idx + 1U) & 7U);
    if (s_cal.phase >= PH_REST && (s_cal.pos < CAL_LO || s_cal.pos > CAL_HI)) { Cal_Fail(4U); return; }

    switch (s_cal.phase)
    {
    case PH_BOOT:
        Cal_Drive(0);
        if (++s_cal.timer < 1000U) return;
        if (s_cal.enc_bad) { Cal_Fail(3U); return; }
        s_meas.rest_cv = A_Voltage_Read();
        printf("\r\nCALIB start %u.%02uV，约3~4分钟，中途请勿触碰输出轴；结束后复制打印出的标定块。\r\n",
               (unsigned)(s_meas.rest_cv / 100U), (unsigned)(s_meas.rest_cv % 100U));
        Cal_Goto(CAL_CENTER, PH_SIGN);      /* 先到中点测极性 */
        return;

    case PH_GOTO:
        Cal_GotoTick();
        break;

    case PH_REST:
        Cal_Drive(0);
        if (s_cal.timer < REST_MS) break;
        if (s_cal.after == PH_RUN) { Cal_StartSegment(); return; }
        s_cal.mark = s_cal.pos;
        Cal_Enter(s_cal.after);
        return;

    case PH_SIGN:
        Cal_Drive(SIGN_DUTY);               /* 极性未知，按正占空比试推 */
        if (s_cal.timer + 1U < SIGN_MS) break;
        Cal_Drive(0);
        if (Fx_Abs32(s_cal.pos - s_cal.mark) < SIGN_MIN_CDEG) { Cal_Fail(2U); return; }
        s_cal.sign = (int8_t)((s_cal.pos > s_cal.mark) ? 1 : -1);
        printf("@SIGN,%d\r\n", (int)s_cal.sign);
        s_cal.seg = 0U;
        s_cal.cur = Cal_Segment(0U);
        Cal_Goto(s_cal.cur.start, PH_RUN);
        return;

    case PH_PRELOAD:
        Cal_Drive(s_cal.timer < STEP_PRELOAD_MS ? Cal_Out(s_cal.cur.dir, RAMP_PRE_DUTY) : 0);
        if (s_cal.timer >= STEP_PRELOAD_MS + REST_MS) { Cal_Enter(PH_RUN); return; }
        break;

    case PH_RUN:
        if (Cal_RunTick()) { Cal_Drive(0); Cal_NextSegment(); return; }
        break;

    default:
        Cal_Drive(0);
        return;
    }
    s_cal.timer++;
}

/* 通过身份校验后先停标定、低阻卸力，等待应答发送结束期间也不能重新施力 */
static void Cal_StopForIAP(void)
{
    s_iap_stopped = 1U;
    D_Motor_Release(0U);
}

void A_Calib_Min_Init(void)
{
    setvbuf(stdout, 0, _IONBF, 0); /* 单片机RAM有限，避免stdio分配1KB发送缓存 */
    memset(&s_cal, 0, sizeof(s_cal));
    memset(&s_meas, 0, sizeof(s_meas));
    memset(s_step, 0, sizeof(s_step));
    s_cal.sign = 1;                 /* 测出极性前先按正接线 */
    s_iap_stopped = 0U;
    IAP_SetStopHandler(Cal_StopForIAP);
    Cal_Drive(0);
}

/* 只处理 IAP，普通舵机指令不能干扰开环标定 */
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
