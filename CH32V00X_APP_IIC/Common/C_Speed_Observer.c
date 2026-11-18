/* 三状态模型观测器：对齐延迟→摩擦→预测→残差校正。
 * 增益只在 Configure 算一次；每拍只有乘加移位。 */
#include "C_Speed_Observer.h"
#include "C_Fixed.h"
#include <string.h>

#define Q24 16777216LL

static struct {
    int32_t l1_q16, l2_q16, l3_q16; /* 位置/速度/扰动残差增益 */
    int32_t alpha_q16;              /* T/tau，Q16 */
    int32_t gain_q16;               /* 1/kv，(厘度/s)/mV，Q16 */
    int32_t fric_inv_q16;           /* 静→动摩擦过渡速度(2噪声底)的倒数 1/(厘度/s) Q16 */
    int32_t vel_max_q8, dist_max_q8;
    int16_t fc_mv, fs_mv;
    uint8_t delay;
} s_cfg;

static int32_t MulQ24(int64_t a, int64_t b) { return (int32_t)((a * b) >> 24); }

/* λ = e^(−2π·f·T)，Padé(2,2) 近似，Q24；x<0.3 时误差<3e-6 */
static int64_t Obs_Lambda(uint16_t hz)
{
    int64_t x = 105414357LL * hz / 1000;          /* 2π·f·T Q24 */
    int64_t x2 = MulQ24(x, x);

    return (12 * Q24 - 6 * x + x2) * Q24 / (12 * Q24 + 6 * x + x2);
}

/* 极点配置：λ1=bw_hz(位置/速度两重)，λ2=dist_hz(扰动)。
 * 扰动极点放低→速度更多来自电机模型、少受编码器读数噪声影响。 */
void C_SpeedObs_Configure(const MotorCal_t *cal, uint16_t bw_hz, uint16_t dist_hz, int32_t vel_noise)
{
    int64_t lam1 = Obs_Lambda(bw_hz), lam2 = Obs_Lambda(dist_hz);
    int64_t c1 = 2 * lam1 + lam2;
    int64_t c3 = MulQ24(MulQ24(lam1, lam1), lam2);
    int64_t alpha = ((int64_t)cal->kv_uv * Q24) / cal->ka_nv;
    int64_t one_a = Q24 - alpha;
    int64_t num2 = MulQ24(alpha, alpha) + MulQ24(alpha, c1) - 3 * alpha - c1 + c3 + 2 * Q24;
    int64_t prod = MulQ24(MulQ24(lam1 - Q24, lam1 - Q24), lam2 - Q24);
    uint32_t delay = cal->delay_us / 1000U;                 /* ms，向下取整(另有1拍零阶保持) */

    s_cfg.l1_q16    = (int32_t)(((one_a - c3) << 16) / one_a);
    s_cfg.l2_q16    = (int32_t)((num2 * 1000 * 65536) / one_a);
    s_cfg.l3_q16    = (int32_t)((prod * cal->kv_uv * 65536) / alpha);
    s_cfg.alpha_q16 = (int32_t)(alpha >> 8);
    s_cfg.gain_q16  = (int32_t)((1000LL << 16) / cal->kv_uv);
    s_cfg.fric_inv_q16 = (int32_t)(65536 / (2 * vel_noise));
    s_cfg.vel_max_q8   = (int32_t)(((int64_t)cal->cal_mv * 1250 / cal->kv_uv) << 8);   /* 标定空载速度×1.25 */
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

/* 速度预测：摩擦随速度连续过渡，不做粘住硬切换，避免估计在0与真实速度间跳变 */
static int32_t Obs_PredictVel(const SpeedObserver_t *obs, int32_t net, uint8_t *stuck)
{
    int32_t w = Fx_Clamp32(Fx_MulShift(Fx_Abs32(obs->vel_q8), s_cfg.fric_inv_q16, 8), 65536);   /* 0=静止 1=在转 Q16 */
    int32_t f_static = Fx_Clamp32(net, s_cfg.fs_mv);
    int32_t f_move = Fx_Sign32(obs->vel_q8) * s_cfg.fc_mv;
    int32_t target;

    *stuck = (uint8_t)(w < 65536 && Fx_Abs32(net) < s_cfg.fs_mv);
    net -= f_static + Fx_MulShift(f_move - f_static, w, 16);
    target = Fx_MulShift(net, s_cfg.gain_q16, 8);           /* 稳态速度 Q8 */
    return obs->vel_q8 + Fx_MulShift(target - obs->vel_q8, s_cfg.alpha_q16, 16);
}

void C_SpeedObs_Update(SpeedObserver_t *obs, int32_t pos, uint8_t valid, int32_t u_mv, SpeedObsOut_t *out)
{
    int32_t u, vel_pred, pos_pred, residual;
    uint8_t stuck;

    /* 1. 对齐纯延迟：用当时真正作用在电机上的电压 */
    obs->history[obs->index] = (int16_t)Fx_Clamp32(u_mv, 32767);
    obs->index = (uint8_t)((obs->index + 1U) % SPEED_OBS_HISTORY);
    u = obs->history[(obs->index + SPEED_OBS_HISTORY - 1U - s_cfg.delay) % SPEED_OBS_HISTORY];

    /* 2. 预测 T=1ms：位置按上一拍速度积分 */
    vel_pred = Obs_PredictVel(obs, u - Fx_MulShift(obs->dist_q8, 1, 8), &stuck);
    pos_pred = obs->pos_q8 + Fx_MulShift(obs->vel_q8, 1073742, 30);

    /* 3. 残差校正：实测为块中点(半拍前)，与预测位置−半拍速度比较，避免系统性滞后；
     *    无效采样只预测 */
    residual = valid ? pos * 256 - pos_pred + Fx_MulShift(vel_pred, 1073742, 31) : 0;
    obs->pos_q8  = pos_pred + Fx_MulShift(s_cfg.l1_q16, residual, 16);
    obs->vel_q8  = Fx_Clamp32(vel_pred + Fx_MulShift(s_cfg.l2_q16, residual, 16), s_cfg.vel_max_q8);
    obs->dist_q8 = Fx_Clamp32(obs->dist_q8 + Fx_MulShift(s_cfg.l3_q16, residual, 16), s_cfg.dist_max_q8);

    out->pos   = Fx_MulShift(obs->pos_q8, 1, 8);
    out->vel   = Fx_MulShift(obs->vel_q8, 1, 8);
    out->dist  = Fx_MulShift(obs->dist_q8, 1, 8);
    out->stuck = stuck;
}
