/* C_Speed_Observer.c 3状态模型观测器，无硬件依赖，全整数运算 */

#include "C_Speed_Observer.h"
#include "A_Parameter.h"

#define OBS_Q8      8U    /* 位置/速度/负载状态的定点小数位 */
#define OBS_Q15     15U   /* 校正增益的定点小数位 */
#define OBS_DEAD_MAX 8U   /* 纯延迟环形缓冲深度上限(拍) */

static struct {
    int32_t pos_q8;      /* 位置估计，Q8厘度 */
    int32_t vel_q8;      /* 速度估计，Q8厘度/秒 */
    int32_t load_q8;     /* 负载扰动估计，Q8 PWM计数 */

    int16_t u_hist[OBS_DEAD_MAX];  /* PWM历史，用于纯延迟对齐 */
    uint8_t u_idx;

    /* 预先算好的模型常数，避免每拍除法 */
    int32_t kv_q8;       /* Kv，Q8 厘度/秒 每PWM计数 */
    int32_t inv_tau_q16; /* T/tau，Q16 */
    uint8_t dead;        /* 纯延迟拍数 */
    uint8_t ready;
} s_obs;

/* 差值折算到最短路径(-range/2, range/2]，跨0点时避免一整圈的假残差 */
static int32_t Obs_WrapFold(int32_t diff, int32_t range)
{
    if (range == 0) return diff;
    if (diff >  (range >> 1)) diff -= range;
    if (diff < -(range >> 1)) diff += range;
    return diff;
}

static int32_t Obs_Clamp(int32_t v, int32_t lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* 初始化状态，并把标定常数换算成定点模型系数 */
void C_SpeedObs_Init(int32_t init_pos)
{
    int32_t slope_q16;
    uint8_t i;

    for (i = 0U; i < OBS_DEAD_MAX; i++) s_obs.u_hist[i] = 0;
    s_obs.u_idx = 0U;

    s_obs.pos_q8  = init_pos * (1 << OBS_Q8);
    s_obs.vel_q8  = 0;
    s_obs.load_q8 = 0;

    slope_q16 = PLANT_SPEED_SLOPE_Q16;
    if (slope_q16 <= 0) slope_q16 = 7400;              /* 防御：配置未填时给个合理值 */
    /* Kv = 65536/slope_q16 (厘度/秒 每计数)，再左移8位取Q8 */
    s_obs.kv_q8 = (int32_t)(((int64_t)65536 << OBS_Q8) / slope_q16);

    /* T/tau，T=1ms，tau单位ms，故 inv = 65536/tau_ms */
    s_obs.inv_tau_q16 = (PLANT_TAU_MS > 0U)
                      ? (int32_t)(65536 / PLANT_TAU_MS) : 1820;

    s_obs.dead  = (PLANT_DEAD_TICKS < OBS_DEAD_MAX)
                ? PLANT_DEAD_TICKS : (uint8_t)(OBS_DEAD_MAX - 1U);
    s_obs.ready = 1U;
}

/* 推进一拍：模型预测 + 编码器残差校正，每拍无除法 */
void C_SpeedObs_Update(int32_t measured_pos, int16_t pwm_applied, SpeedObsOut_t *out)
{
    int32_t u_delayed, u_net, drive_q8;
    int32_t pos_pred_q8, vel_pred_q8, resid_q8;
    int32_t range_q8;

    if (!s_obs.ready) C_SpeedObs_Init(measured_pos);

    /* 1. 纯延迟对齐：预测必须用当时真正作用在电机上的PWM */
    /* 控制器给的是H桥物理方向，换回"编码器增大为正"的算法坐标 */
    s_obs.u_hist[s_obs.u_idx] = (int16_t)(pwm_applied * CTRL_MOTOR_SIGN);
    s_obs.u_idx = (uint8_t)((s_obs.u_idx + 1U) % OBS_DEAD_MAX);
    u_delayed = s_obs.u_hist[(s_obs.u_idx + OBS_DEAD_MAX - 1U - s_obs.dead) % OBS_DEAD_MAX];

    /* 2. 连续扣除摩擦，零速附近不做硬切换(硬切换会反复扰动观测器)：
     *     f = clamp(v/Vs, -1, 1),  u_net = u - F*f - (1-|f|)*clamp(u, ±F)
     * |f|=1 为库仑摩擦；f=0 时 |u|<F 净驱动为0(静摩擦锁住)。 */
    {
        int32_t f_q15, af, fric, stick;
        int32_t fmax = PLANT_FRICTION_PWM;

        f_q15 = (int32_t)(((int64_t)s_obs.vel_q8
                           * OBS_FRIC_INV_Q15) >> 16);
        if (f_q15 >  32768) f_q15 =  32768;
        if (f_q15 < -32768) f_q15 = -32768;
        af = (f_q15 < 0) ? -f_q15 : f_q15;

        fric  = (int32_t)(((int64_t)fmax * f_q15) >> 15);

        stick = u_delayed;
        if (stick >  fmax) stick =  fmax;
        if (stick < -fmax) stick = -fmax;
        stick = (int32_t)(((int64_t)stick * (32768 - af)) >> 15);

        u_net = u_delayed - fric - stick;
    }

    /* 3. 模型预测：v += (Kv*(u_net - d) - v) * T/tau */
    drive_q8    = s_obs.kv_q8 * u_net
                - (int32_t)(((int64_t)s_obs.kv_q8 * s_obs.load_q8) >> OBS_Q8);
    vel_pred_q8 = s_obs.vel_q8
                + (int32_t)(((int64_t)(drive_q8 - s_obs.vel_q8) * s_obs.inv_tau_q16) >> 16);

    /* 位置积分：p += v*T，T=1ms。除以1000用Q22倒数(4194)代替，避免除法。 */
    pos_pred_q8 = s_obs.pos_q8 + (int32_t)(((int64_t)s_obs.vel_q8 * 4194) >> 22);

    range_q8 = (int32_t)CFG_WRAP_RANGE_CDEG * (1 << OBS_Q8);
    if (range_q8 > 0)
    {
        if (pos_pred_q8 >= range_q8) pos_pred_q8 -= range_q8;
        else if (pos_pred_q8 < 0)    pos_pred_q8 += range_q8;
    }

    /* ---- 4. 残差校正 ---- */
    resid_q8 = Obs_WrapFold(measured_pos * (1 << OBS_Q8) - pos_pred_q8, range_q8);

    s_obs.pos_q8 = pos_pred_q8
                 + (int32_t)(((int64_t)OBS_L1_Q15 * resid_q8) >> OBS_Q15);
    if (range_q8 > 0)
    {
        if (s_obs.pos_q8 >= range_q8) s_obs.pos_q8 -= range_q8;
        else if (s_obs.pos_q8 < 0)    s_obs.pos_q8 += range_q8;
    }

    s_obs.vel_q8 = vel_pred_q8
                 + (int32_t)(((int64_t)OBS_L2_Q15 * resid_q8) >> OBS_Q15);
    s_obs.vel_q8 = Obs_Clamp(s_obs.vel_q8, OBS_VMAX_CDPS << OBS_Q8);

    /* d += L3*e；L3 设计值为负，写成减法会变成正反馈 */
    s_obs.load_q8 += (int32_t)(((int64_t)OBS_L3_Q15 * resid_q8) >> OBS_Q15);
    s_obs.load_q8 = Obs_Clamp(s_obs.load_q8, OBS_LOAD_MAX << OBS_Q8);

    if (out == 0) return;
    out->pos  = s_obs.pos_q8  >> OBS_Q8;
    out->vel  = s_obs.vel_q8  >> OBS_Q8;
    out->load = s_obs.load_q8 >> OBS_Q8;
}
