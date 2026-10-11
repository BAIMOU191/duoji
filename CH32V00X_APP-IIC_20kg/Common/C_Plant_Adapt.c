/* C_Plant_Adapt.c 对象参数自学习，纯算法，无硬件依赖 */

#include "C_Plant_Adapt.h"
#include "C_Fixed.h"
#include "A_Config.h"

/* 回归量滤波极点与观测器相同(三重极点，C_SpeedObs_AlphaQ15)，把 z、w 延迟成和 d 一样再回归。
 * 样本只取"驱动段"(净输入与速度同号，制动侧本来就另有制动增益)且轴确实在动(速度超过4倍噪声底)。 */
#define AD_Q          12
#define AD_ONE        (1L << AD_Q)
#define AD_VEL_GATE   (PLANT_VEL_NOISE_CDPS * 4)                  /** 收样本的最低速度(4倍速度噪声底)，厘度/秒 */
#define AD_N_MIN      30                                         /** 一段运动至少这么多样本才结算 */
#define AD_N_MAX      16384                                      /** 超过就不再收：和不会溢出int64 */
#define AD_MINVAR_Q8  ((int64_t)5000000 * 256)                   /** 激励门限 N*var(z)，PWM^2(Q8)；小位移不学 */
/* 每次更新只走一半(取平方根)、单步比例有上下限：一次异常运动不会把模型带歪 */
#define AD_RHO_STEP_MIN 2731   /** 0.667 */
#define AD_RHO_STEP_MAX 6144   /** 1.5 */
#define AD_K_STEP_MIN   3277   /** 0.8 */
#define AD_K_STEP_MAX   5120   /** 1.25 */
#define AD_RHO_MIN      2048   /** 0.5 */
#define AD_RHO_MAX      24576  /** 6 */
#define AD_K_MIN        2458   /** 0.6 */
#define AD_K_MAX        6554   /** 1.6 */
#define AD_KA_MIN       2458   /** 加速度增益比 rho/k 的范围 0.6~4，控制器增益随它缩放 */
#define AD_KA_MAX       16384

static struct {
    int32_t rho_q12;     /* 负载惯量比(实际采用的) */
    int32_t k_q12;       /* 速度增益比(实际采用的) */
    int32_t rho_dir_q12[2]; /* 两个运动方向各自的估计，[0]=角度增大 */
    int32_t k_dir_q12[2];
    int32_t slope_q16;   /* 当前模型 1/Kv：w = v*slope>>16，PWM */
    int32_t zf_q8[3];    /* z 的三级低通，Q8 PWM */
    int32_t wf_q8[3];    /* w 的三级低通 */
    int64_t szz, szw, sww, sz, sw, sd, sdz, sdw; /* 回归和，Q4 */
    int32_t n;
} s_ad = { .rho_q12 = AD_ONE, .k_q12 = AD_ONE, .rho_dir_q12 = { AD_ONE, AD_ONE }, .k_dir_q12 = { AD_ONE, AD_ONE },
            .slope_q16 = CAL_SPEED_SLOPE_Q16 };

/* 每拍在观测器之后调用：z、w 过三级低通，moving 且满足条件就收一个样本 */
void C_PlantAdapt_Sample(const SpeedObsOut_t *obs, uint8_t moving)
{
    int32_t vel = obs->vel, u_net = obs->effort, alpha = C_SpeedObs_AlphaQ15();
    int32_t w   = (int32_t)(((int64_t)vel * s_ad.slope_q16) >> 16);
    int32_t in_z = (u_net - w) * (1 << 8);
    int32_t in_w = w * (1 << 8);
    uint8_t i;

    for (i = 0U; i < 3U; i++)
    {
        s_ad.zf_q8[i] += (int32_t)(((int64_t)(in_z - s_ad.zf_q8[i]) * (32768 - alpha)) >> 15);
        s_ad.wf_q8[i] += (int32_t)(((int64_t)(in_w - s_ad.wf_q8[i]) * (32768 - alpha)) >> 15);
        in_z = s_ad.zf_q8[i];
        in_w = s_ad.wf_q8[i];
    }
    if (moving && s_ad.n < AD_N_MAX
        && (vel > AD_VEL_GATE || vel < -AD_VEL_GATE)
        && (int64_t)u_net * vel > 0)
    {
        int64_t z = s_ad.zf_q8[2] >> 4, wq = s_ad.wf_q8[2] >> 4, d = obs->load_q8 >> 4; /* Q4 */
        s_ad.szz += z * z;  s_ad.szw += z * wq; s_ad.sww += wq * wq;
        s_ad.sz  += z;      s_ad.sw  += wq;     s_ad.sd  += d;
        s_ad.sdz += d * z;  s_ad.sdw += d * wq;
        s_ad.n++;
    }
}

void C_PlantAdapt_Abort(void)
{
    s_ad.szz = s_ad.szw = s_ad.sww = s_ad.sz = s_ad.sw = s_ad.sd = s_ad.sdz = s_ad.sdw = 0;
    s_ad.n = 0;
}

void C_PlantAdapt_Reset(void)
{
    s_ad.rho_q12 = AD_ONE;
    s_ad.k_q12   = AD_ONE;
    s_ad.rho_dir_q12[0] = s_ad.rho_dir_q12[1] = AD_ONE;
    s_ad.k_dir_q12[0]   = s_ad.k_dir_q12[1]   = AD_ONE;
    s_ad.slope_q16 = CAL_SPEED_SLOPE_Q16;
    C_PlantAdapt_Abort();
}

int32_t C_PlantAdapt_Rho(void) { return s_ad.rho_q12; }
int32_t C_PlantAdapt_Kv(void)  { return s_ad.k_q12; }

/* 两个方向的估计都偏向同一侧才采用，取偏得少的那个；一个偏大一个偏小说明是两个方向本身不对称
 * (仿真30kg两向时间常数差一倍)，不是负载，保持标定值。只用一个方向的估计时，单一模型会按上一段的方向
 * 来回翻(30kg往返时 rho 在0.77与0.98之间跳)，每段都用着对面方向的参数。 */
static int32_t Ad_Common(int32_t a, int32_t b)
{
    if (a > AD_ONE && b > AD_ONE) return (a < b) ? a : b;
    if (a < AD_ONE && b < AD_ONE) return (a > b) ? a : b;
    return AD_ONE;
}

/*
 * 一段运动结束：解 d = s*z + g*w + c 的最小二乘(先对均值中心化消去 c，再解2x2)。
 * 只在运动之间算一次，有几次64位除法无妨。两个回归量几乎共线(没有匀速段的三角形轨迹常见)时，
 * 速度增益分不出来，退化成只修正加速度增益比 rho/k。
 */
uint8_t C_PlantAdapt_MoveEnd(void)
{
    int64_t n = s_ad.n, czz, cww, czw, cdz, cdw, mx, a, b, c, p, q, det, s, g;
    int32_t rho_step, k_step, rho, k, ka, den;
    uint8_t sh = 0U, dir = (uint8_t)(s_ad.sw < 0);   /* 样本只收"出力与运动同向"的，w 之和的符号就是运动方向 */

    if (n < AD_N_MIN) { C_PlantAdapt_Abort(); return 0U; }
    czz = s_ad.szz - s_ad.sz * s_ad.sz / n;
    cww = s_ad.sww - s_ad.sw * s_ad.sw / n;
    czw = s_ad.szw - s_ad.sz * s_ad.sw / n;
    cdz = s_ad.sdz - s_ad.sd * s_ad.sz / n;
    cdw = s_ad.sdw - s_ad.sd * s_ad.sw / n;
    C_PlantAdapt_Abort();
    if (czz < AD_MINVAR_Q8) return 0U;                     /* 激励不够(小位移)：不学 */

    /* 规格化到 2^30 以内，后面的乘积才放得进 int64 */
    mx = czz;
    if (cww > mx) mx = cww;
    if ((czw < 0 ? -czw : czw) > mx) mx = (czw < 0 ? -czw : czw);
    if ((cdz < 0 ? -cdz : cdz) > mx) mx = (cdz < 0 ? -cdz : cdz);
    if ((cdw < 0 ? -cdw : cdw) > mx) mx = (cdw < 0 ? -cdw : cdw);
    while ((mx >> sh) >= (1LL << 30)) sh++;
    a = czz >> sh; b = cww >> sh; c = czw >> sh; p = cdz >> sh; q = cdw >> sh;
    det = a * b - c * c;

    if (det > ((a * b) >> 4) && (det >> AD_Q) > 0)       /* 两个回归量相关系数 < 0.97 */
    {
        s = (p * b - q * c) / (det >> AD_Q);              /* Q12 */
        g = (a * q - c * p) / (det >> AD_Q);
        s = (s > 3277) ? 3277 : ((s < -8192) ? -8192 : s); /* -2 ~ 0.8 */
        g = (g > 8192) ? 8192 : ((g < -8192) ? -8192 : g);
        den = (int32_t)(g + AD_ONE - s);
        if (den < 819) den = 819;                          /* 0.2 */
        rho_step = (int32_t)((1LL << (2 * AD_Q)) / den);
        k_step   = (int32_t)(((int64_t)(AD_ONE - s) * rho_step) >> AD_Q);
    }
    else
    {
        /* 共线：单回归量 s1，只修正加速度增益比，速度增益不动 */
        s = ((a >> 2) > 0) ? (p << AD_Q) / a : 0;
        s = (s > 3277) ? 3277 : ((s < -8192) ? -8192 : s);
        rho_step = (int32_t)((1LL << (2 * AD_Q)) / (AD_ONE - s));
        k_step   = AD_ONE;
    }

    /* 每次只走一半(几何平均)，单步比例限幅 */
    rho_step = Fx_ClampRange((int32_t)Fx_Isqrt((uint32_t)rho_step << AD_Q), AD_RHO_STEP_MIN, AD_RHO_STEP_MAX);
    k_step   = Fx_ClampRange((int32_t)Fx_Isqrt((uint32_t)k_step << AD_Q),   AD_K_STEP_MIN,   AD_K_STEP_MAX);

    /* 回归给的是相对当前所用模型的修正；本方向的估计 = sqrt(本方向旧估计 x 当前模型) x 半步 */
    rho = (int32_t)Fx_Isqrt((uint32_t)s_ad.rho_dir_q12[dir] * (uint32_t)s_ad.rho_q12);
    k   = (int32_t)Fx_Isqrt((uint32_t)s_ad.k_dir_q12[dir] * (uint32_t)s_ad.k_q12);
    s_ad.rho_dir_q12[dir] = Fx_ClampRange((int32_t)(((int64_t)rho * rho_step) >> AD_Q), AD_RHO_MIN, AD_RHO_MAX);
    s_ad.k_dir_q12[dir]   = Fx_ClampRange((int32_t)(((int64_t)k * k_step) >> AD_Q),     AD_K_MIN,   AD_K_MAX);
    rho = Ad_Common(s_ad.rho_dir_q12[0], s_ad.rho_dir_q12[1]);
    k   = Ad_Common(s_ad.k_dir_q12[0], s_ad.k_dir_q12[1]);
    ka  = (int32_t)(((int64_t)rho << AD_Q) / k);
    if (ka > AD_KA_MAX) rho = (int32_t)(((int64_t)AD_KA_MAX * k) >> AD_Q);
    if (ka < AD_KA_MIN) rho = (int32_t)(((int64_t)AD_KA_MIN * k) >> AD_Q);

    if (rho == s_ad.rho_q12 && k == s_ad.k_q12) return 0U;
    s_ad.rho_q12 = rho;
    s_ad.k_q12   = k;
    s_ad.slope_q16 = (int32_t)(((int64_t)CAL_SPEED_SLOPE_Q16 << AD_Q) / k);
    return 1U;
}
