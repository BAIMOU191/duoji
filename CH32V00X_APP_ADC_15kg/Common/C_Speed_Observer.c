/* C_Speed_Observer.c 三状态模型观测器：输入对齐 -> 摩擦 -> 预测 -> 残差校正
 * 增益只在 Configure 里算一次(含64位除法)；每拍只有乘加移位。 */
#include "C_Speed_Observer.h"
#include "C_Fixed.h"
#include <string.h>

#define Q24 16777216LL

static struct {
    int32_t l1_q16, l2_q16, l3_q16; /* 位置、速度(1/s)、扰动(mV/厘度)的残差增益 */
    int32_t alpha_q16;              /* T/tau */
    int32_t gain_q16;               /* 1/kv，(厘度/s)/mV */
    int32_t fric_inv_q16;           /* 静摩擦过渡到动摩擦的速度(2个噪声底)的倒数，1/(厘度/s) */
    int32_t vel_max_q8, dist_max_q8;
    int16_t fc_mv, fs_mv;
    uint8_t delay;
} s_cfg;

static int32_t MulQ24(int64_t a, int64_t b) { return (int32_t)((a * b) >> 24); }

/* λ = e^(−2π·f·T)，Padé(2,2)：(12 − 6x + x²)/(12 + 6x + x²)，x < 0.3 时误差 < 3e-6；Q24 */
static int64_t Obs_Lambda(uint16_t hz)
{
    int64_t x = 105414357LL * hz / 1000;          /* 2π·f·T，Q24 */
    int64_t x2 = MulQ24(x, x);

    return (12 * Q24 - 6 * x + x2) * Q24 / (12 * Q24 + 6 * x + x2);
}

/*
 * 误差动态 (I − L·C)·A 的特征多项式配成 (z − λ1)²(z − λ2) = z³ − c1·z² + c2·z − c3，解得：
 *   l1 = (1 − α − c3) / (1 − α)
 *   l2 = (α² + α·c1 − 3α − c1 + c3 + 2) / (T·(1 − α))
 *   l3 = (λ1 − 1)²(λ2 − 1)·kv / α        (kv 单位 mV/(厘度/s))
 * λ1 取 bw_hz(位置/速度两重极点)，λ2 取 dist_hz(扰动极点)：扰动极点放低，速度估计里电机模型的份量更重，
 * 电位器噪声进速度的就少，而起步/刹车这类由电压引起的速度变化仍由模型直接给出，不靠残差慢慢追。
 */
void C_SpeedObs_Configure(const MotorCal_t *cal, uint16_t bw_hz, uint16_t dist_hz, int32_t vel_noise)
{
    int64_t lam1 = Obs_Lambda(bw_hz), lam2 = Obs_Lambda(dist_hz);
    int64_t c1 = 2 * lam1 + lam2;
    int64_t c3 = MulQ24(MulQ24(lam1, lam1), lam2);
    int64_t alpha = ((int64_t)cal->kv_uv * Q24) / cal->ka_nv;
    int64_t one_a = Q24 - alpha;
    int64_t num2 = MulQ24(alpha, alpha) + MulQ24(alpha, c1) - 3 * alpha - c1 + c3 + 2 * Q24;
    int64_t prod = MulQ24(MulQ24(lam1 - Q24, lam1 - Q24), lam2 - Q24);
    uint32_t delay = cal->delay_us / 1000U;                 /* 向下取整：另有1拍零阶保持 */

    s_cfg.l1_q16    = (int32_t)(((one_a - c3) << 16) / one_a);
    s_cfg.l2_q16    = (int32_t)((num2 * 1000 * 65536) / one_a);
    s_cfg.l3_q16    = (int32_t)((prod * cal->kv_uv * 65536) / alpha);
    s_cfg.alpha_q16 = (int32_t)(alpha >> 8);
    s_cfg.gain_q16  = (int32_t)((1000LL << 16) / cal->kv_uv);
    s_cfg.fric_inv_q16 = (int32_t)(65536 / (2 * vel_noise));
    s_cfg.vel_max_q8   = (int32_t)(((int64_t)cal->cal_mv * 1250 / cal->kv_uv) << 8);   /* 标定电压空载速度的1.25倍 */
    s_cfg.dist_max_q8  = ((int32_t)cal->cal_mv / 2) << 8;
    s_cfg.fc_mv = cal->fc_mv;
    s_cfg.fs_mv = cal->fs_mv;
    s_cfg.delay = (uint8_t)((delay < SPEED_OBS_HISTORY) ? delay : SPEED_OBS_HISTORY - 1U);
}

void C_SpeedObs_Init(SpeedObserver_t *obs, int32_t pos)
{
    memset(obs, 0, sizeof(*obs));
    obs->pos_q8 = pos * 256;
}

/* 速度预测，摩擦随速度连续过渡：静止时摩擦抵消净电压直到起转电压 fs(不足就不加速)，过两个噪声底后是动摩擦 fc。
 * 不做"粘住/没粘住"的硬切换：同型号另一只舵机 fs 不一样时只差一点预测，不会整段速度估计在 0 和真实速度之间跳 */
static int32_t Obs_PredictVel(const SpeedObserver_t *obs, int32_t net, uint8_t *stuck)
{
    int32_t w = Fx_Clamp32(Fx_MulShift(Fx_Abs32(obs->vel_q8), s_cfg.fric_inv_q16, 8), 65536);   /* 0=静止 1=在转，Q16 */
    int32_t f_static = Fx_Clamp32(net, s_cfg.fs_mv);
    int32_t f_move = Fx_Sign32(obs->vel_q8) * s_cfg.fc_mv;
    int32_t target;

    *stuck = (uint8_t)(w < 65536 && Fx_Abs32(net) < s_cfg.fs_mv);
    net -= f_static + Fx_MulShift(f_move - f_static, w, 16);
    target = Fx_MulShift(net, s_cfg.gain_q16, 8);           /* 稳态速度，Q8 */
    return obs->vel_q8 + Fx_MulShift(target - obs->vel_q8, s_cfg.alpha_q16, 16);
}

void C_SpeedObs_Update(SpeedObserver_t *obs, int32_t pos, uint8_t valid, int32_t u_mv, SpeedObsOut_t *out)
{
    int32_t u, vel_pred, pos_pred, residual;
    uint8_t stuck;

    /* 1. 对齐纯延迟：预测用当时真正作用在电机上的电压 */
    obs->history[obs->index] = (int16_t)Fx_Clamp32(u_mv, 32767);
    obs->index = (uint8_t)((obs->index + 1U) % SPEED_OBS_HISTORY);
    u = obs->history[(obs->index + SPEED_OBS_HISTORY - 1U - s_cfg.delay) % SPEED_OBS_HISTORY];

    /* 2. 预测，T=1ms：位置按上一拍速度积分 */
    vel_pred = Obs_PredictVel(obs, u - Fx_MulShift(obs->dist_q8, 1, 8), &stuck);
    pos_pred = obs->pos_q8 + Fx_MulShift(obs->vel_q8, 1073742, 30);

    /* 3. 位置残差校正三个状态。实测是刚过去这1ms的块均值，代表半拍前的位置，所以和"预测位置 − 半拍速度"比；
     *    不扣的话位置估计按速度×0.5ms系统性滞后，积分会把它当成跟踪误差去攒。无效采样只预测 */
    residual = valid ? pos * 256 - pos_pred + Fx_MulShift(vel_pred, 1073742, 31) : 0;
    obs->pos_q8  = pos_pred + Fx_MulShift(s_cfg.l1_q16, residual, 16);
    obs->vel_q8  = Fx_Clamp32(vel_pred + Fx_MulShift(s_cfg.l2_q16, residual, 16), s_cfg.vel_max_q8);
    obs->dist_q8 = Fx_Clamp32(obs->dist_q8 + Fx_MulShift(s_cfg.l3_q16, residual, 16), s_cfg.dist_max_q8);

    out->pos   = Fx_MulShift(obs->pos_q8, 1, 8);
    out->vel   = Fx_MulShift(obs->vel_q8, 1, 8);
    out->dist  = Fx_MulShift(obs->dist_q8, 1, 8);
    out->stuck = stuck;
}
