/* C_Speed_Observer.c 三状态模型观测器：输入对齐 -> 摩擦/粘住 -> 预测 -> 残差校正
 * 增益只在 Configure 里算一次(含64位除法)；每拍只有乘加移位。 */
#include "C_Speed_Observer.h"
#include "C_Fixed.h"
#include <string.h>

#define Q24 16777216LL

static struct {
    int32_t l1_q16, l2_q16, l3_q16; /* 位置、速度(1/s)、扰动(mV/厘度)的残差增益 */
    int32_t alpha_q16;              /* T/tau */
    int32_t gain_q16;               /* 1/kv，(厘度/s)/mV */
    int32_t fric_inv_q16;           /* 摩擦过渡速度的倒数，1/(厘度/s) */
    int32_t vel_noise, vel_max_q8, dist_max_q8;
    int16_t fc_mv, fs_mv;
    uint8_t delay;
} s_cfg;

static int32_t MulQ24(int64_t a, int64_t b) { return (int32_t)((a * b) >> 24); }

/*
 * 误差动态 (I − L·C)·A 的特征多项式配成 (z − λ)^3，λ = e^(−2πf·T)，解得：
 *   l1 = (1 − α − λ³) / (1 − α)
 *   l2 = (α² + 3αλ − 3α + λ³ − 3λ + 2) / (T·(1 − α))
 *   l3 = (λ − 1)³ / (T·α·g) = (λ − 1)³·kv / α        (g = 1/kv，kv 单位 mV/(厘度/s))
 * λ 用 Padé(2,2) 近似 e^(−x) = (12 − 6x + x²)/(12 + 6x + x²)，x < 0.3 时误差 < 3e-6。
 */
void C_SpeedObs_Configure(const MotorCal_t *cal, uint16_t bandwidth_hz, int32_t vel_noise)
{
    int64_t x = 105414357LL * bandwidth_hz / 1000;          /* 2π·f·T，Q24 */
    int64_t x2 = MulQ24(x, x);
    int64_t lam = (12 * Q24 - 6 * x + x2) * Q24 / (12 * Q24 + 6 * x + x2);
    int64_t lam3 = MulQ24(MulQ24(lam, lam), lam);
    int64_t alpha = ((int64_t)cal->kv_uv * Q24) / cal->ka_nv;
    int64_t one_a = Q24 - alpha;
    int64_t num2 = MulQ24(alpha, alpha) + 3 * MulQ24(alpha, lam) - 3 * alpha + lam3 - 3 * lam + 2 * Q24;
    int64_t c = lam - Q24;
    int64_t c3 = MulQ24(MulQ24(c, c), c);
    uint32_t delay = cal->delay_us / 1000U;                 /* 向下取整：另有1拍零阶保持 */

    s_cfg.l1_q16    = (int32_t)(((one_a - lam3) << 16) / one_a);
    s_cfg.l2_q16    = (int32_t)((num2 * 1000 * 65536) / one_a);
    s_cfg.l3_q16    = (int32_t)((c3 * cal->kv_uv * 65536) / alpha);
    s_cfg.alpha_q16 = (int32_t)(alpha >> 8);
    s_cfg.gain_q16  = (int32_t)((1000LL << 16) / cal->kv_uv);
    s_cfg.vel_noise = vel_noise;
    s_cfg.fric_inv_q16 = (int32_t)(65536 / (3 * vel_noise));
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

/* 速度预测：粘住时为0；运动中扣除平滑库仑摩擦后按一阶模型逼近稳态速度 */
static int32_t Obs_PredictVel(const SpeedObserver_t *obs, int32_t net, uint8_t *stuck)
{
    int32_t frac, target;

    *stuck = (uint8_t)(Fx_Abs32(obs->vel_q8 >> 8) < s_cfg.vel_noise && Fx_Abs32(net) < s_cfg.fs_mv);
    if (*stuck) return 0;
    frac = Fx_Clamp32(Fx_MulShift(obs->vel_q8, s_cfg.fric_inv_q16, 8), 65536);
    net -= Fx_MulShift(s_cfg.fc_mv, frac, 16);
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

    /* 3. 位置残差校正三个状态；无效采样只预测 */
    residual = valid ? pos * 256 - pos_pred : 0;
    obs->pos_q8  = pos_pred + Fx_MulShift(s_cfg.l1_q16, residual, 16);
    obs->vel_q8  = Fx_Clamp32(vel_pred + Fx_MulShift(s_cfg.l2_q16, residual, 16), s_cfg.vel_max_q8);
    obs->dist_q8 = Fx_Clamp32(obs->dist_q8 + Fx_MulShift(s_cfg.l3_q16, residual, 16), s_cfg.dist_max_q8);

    out->pos   = Fx_MulShift(obs->pos_q8, 1, 8);
    out->vel   = Fx_MulShift(obs->vel_q8, 1, 8);
    out->dist  = Fx_MulShift(obs->dist_q8, 1, 8);
    out->stuck = stuck;
}
