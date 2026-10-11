/* C_Speed_Observer.c 3状态模型观测器，无硬件依赖，全整数运算 */

#include "C_Speed_Observer.h"
#include "A_Config.h"
#include "C_Fixed.h"

#define OBS_Q8      8U    /* 位置/速度/负载状态的定点小数位 */
#define OBS_Q15     15U   /* 校正增益的定点小数位 */
#define OBS_DEAD_MAX 8U   /* 纯延迟环形缓冲深度上限(拍) */

/* 观测器增益(编译期算好)；详见 docs/ALGORITHM_NOTES.md。 */
#define G_ONE       16777216L                                                /* Q24 的 1.0 */
#define G_MUL(a, b) ((int32_t)(((int64_t)(a) * (int64_t)(b)) >> 24))
#define G_X    ((int32_t)((105414357LL * PLANT_OBS_BW_HZ) / CFG_TICK_HZ))    /* 2*pi*f*T，Q24 */
#define G_E2   (G_MUL (G_X, G_X) / 2)                                        /* exp(-x) 的6阶泰勒展开 */
#define G_E3   (G_MUL (G_E2, G_X) / 3)
#define G_E4   (G_MUL (G_E3, G_X) / 4)
#define G_E5   (G_MUL (G_E4, G_X) / 5)
#define G_E6   (G_MUL (G_E5, G_X) / 6)
#define G_ALPHA (G_ONE - G_X + G_E2 - G_E3 + G_E4 - G_E5 + G_E6)             /* 离散极点，Q24 */
#define G_BETA  (G_ONE - G_ALPHA)
#define G_A3    G_MUL (G_MUL (G_ALPHA, G_ALPHA), G_ALPHA)
#define G_B2    G_MUL (G_BETA, G_BETA)
#define G_B3    G_MUL (G_B2, G_BETA)
#define G_A     (G_ONE / MODEL_TAU_MS)                                       /* T/tau，Q24 */
#define G_L1_N  (((G_ONE - G_A3 - G_A) > 0) ? (G_ONE - G_A3 - G_A) : 0)
#define G_L1_D  (((G_ONE - G_A) > 1) ? (G_ONE - G_A) : 1)
#define G_L1_RAW ((int32_t)(((int64_t)G_L1_N << 24) / G_L1_D))
#define G_L1    ((G_L1_RAW > G_ONE) ? (int32_t)G_ONE : G_L1_RAW)             /* Q24，限幅到1 */
#define G_L2    (((3 * G_B2 - G_B3 - G_MUL (G_L1, G_A)) > 0) ? (3 * G_B2 - G_B3 - G_MUL (G_L1, G_A)) : 0)

#define OBS_L1_Q15 (G_L1 >> 9)                                                    /** 位置校正增益，Q15 */
#define OBS_L2_Q15 ((int32_t)(((int64_t)G_L2 * CFG_TICK_HZ) >> 9))                /** 速度校正增益，Q15 */
#define OBS_L3_Q15 (-(int32_t)(((((((int64_t)G_B3 * CFG_TICK_HZ) * CFG_TICK_HZ) << 15) \
                                 / PLANT_KA_Q8) * 256) >> 24))                    /** 负载校正增益，Q15，设计值为负 */

/* ===================== 状态限幅与摩擦过渡 ===================== */
#define OBS_LOAD_MAX     (CFG_PWM_FULL / 3)                                  /** 负载估计限幅，PWM计数 */
#define OBS_VMAX_CDPS    (PLANT_VSS_CDPS + (PLANT_VSS_CDPS >> 3))            /** 速度估计限幅，厘度/秒 */
/* 库仑摩擦在零速附近连续建立的速度刻度，取3倍最小速度指令 */
#define OBS_FRIC_INV_Q15 ((int32_t)((1L << 23) / (PLANT_WCMD_MIN_CDPS * 3))) /** 摩擦过渡斜率倒数，Q15 */

static struct {
    int32_t pos_q8;      /* 位置估计，Q8厘度 */
    int32_t vel_q8;      /* 速度估计，Q8厘度/秒 */
    int32_t load_q8;     /* 负载扰动估计，Q8 PWM计数 */

    int16_t u_hist[OBS_DEAD_MAX];  /* PWM历史，用于纯延迟对齐 */
    uint8_t u_idx;

    /* 预先算好的模型常数，避免每拍除法 */
    int32_t kv_q8;       /* Kv，Q8 厘度/秒 每PWM计数(已乘对象参数比 k) */
    int32_t inv_tau_q16; /* T/tau，Q16(已除对象参数比 rho) */
    int32_t kv0_q8;      /* 标定值 */
    int32_t inv_tau0_q16;
    uint8_t dead;        /* 纯延迟拍数 */
    uint8_t ready;
} s_obs;

/* 对象参数比(Q12)，由 C_SpeedObs_SetPlant 下发；Init 不清 */
static int32_t s_rho_q12 = OBS_PLANT_ONE, s_k_q12 = OBS_PLANT_ONE;

/* 把 rho、k 乘进观测器模型 */
static void Obs_ApplyPlant(void)
{
    s_obs.kv_q8       = (int32_t)(((int64_t)s_obs.kv0_q8 * s_k_q12) >> 12);
    s_obs.inv_tau_q16 = (int32_t)(((int64_t)s_obs.inv_tau0_q16 << 12) / s_rho_q12);
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

    slope_q16 = CAL_SPEED_SLOPE_Q16;
    if (slope_q16 <= 0) slope_q16 = 7400;              /* 防御：配置未填时给个合理值 */
    /* Kv = 65536/slope_q16 (厘度/秒 每计数)，再左移8位取Q8 */
    s_obs.kv0_q8 = (int32_t)(((int64_t)65536 << OBS_Q8) / slope_q16);

    /* T/tau，T=1ms，tau单位ms，故 inv = 65536/tau_ms */
    s_obs.inv_tau0_q16 = (MODEL_TAU_MS > 0U)
                       ? (int32_t)(65536 / MODEL_TAU_MS) : 1820;
    Obs_ApplyPlant();

    s_obs.dead  = (MODEL_DELAY_TICKS < OBS_DEAD_MAX)
                ? MODEL_DELAY_TICKS : (uint8_t)(OBS_DEAD_MAX - 1U);
    s_obs.ready = 1U;
}

/* 推进一拍：模型预测 + 编码器残差校正，每拍无除法 */
void C_SpeedObs_Update(int32_t measured_pos, int16_t pwm_applied, SpeedObsOut_t *out)
{
    int32_t u_delayed, u_net, drive_q8;
    int32_t pos_pred_q8, vel_pred_q8, resid_q8;

    if (!s_obs.ready) C_SpeedObs_Init(measured_pos);

    /* 1. 纯延迟对齐：预测必须用当时真正作用在电机上的PWM */
    /* 控制器给的是H桥物理方向，换回"编码器增大为正"的算法坐标 */
    s_obs.u_hist[s_obs.u_idx] = (int16_t)(pwm_applied * CAL_MOTOR_SIGN);
    s_obs.u_idx = (uint8_t)((s_obs.u_idx + 1U) % OBS_DEAD_MAX);
    u_delayed = s_obs.u_hist[(s_obs.u_idx + OBS_DEAD_MAX - 1U - s_obs.dead) % OBS_DEAD_MAX];

    /* 2. 连续扣除摩擦，零速附近不做硬切换(硬切换会反复扰动观测器)：
     *     f = clamp(v/Vs, -1, 1),  u_net = u - F*f - (1-|f|)*clamp(u, ±F)
     * |f|=1 为库仑摩擦；f=0 时 |u|<F 净驱动为0(静摩擦锁住)。 */
    {
        int32_t f_q15, af, fric, stick;
        int32_t fmax = CAL_FRICTION_PWM;

        f_q15 = Fx_Clamp32((int32_t)(((int64_t)s_obs.vel_q8 * OBS_FRIC_INV_Q15) >> 16), 32768);
        af = Fx_Abs32(f_q15);

        fric  = (int32_t)(((int64_t)fmax * f_q15) >> 15);

        stick = Fx_Clamp32(u_delayed, fmax);
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

    /* ---- 4. 圆周预测与残差，跨零点不产生整圈的假速度 ---- */
    const int32_t range_q8 = CFG_WRAP_RANGE_CDEG * (1 << OBS_Q8);
    if (pos_pred_q8 >= range_q8) pos_pred_q8 -= range_q8;
    else if (pos_pred_q8 < 0) pos_pred_q8 += range_q8;
    resid_q8 = Fx_WrapDiff(measured_pos * (1 << OBS_Q8) - pos_pred_q8, range_q8);

    s_obs.pos_q8 = pos_pred_q8
                 + (int32_t)(((int64_t)OBS_L1_Q15 * resid_q8) >> OBS_Q15);

    if (s_obs.pos_q8 >= range_q8) s_obs.pos_q8 -= range_q8;
    else if (s_obs.pos_q8 < 0) s_obs.pos_q8 += range_q8;

    s_obs.vel_q8 = vel_pred_q8
                 + (int32_t)(((int64_t)OBS_L2_Q15 * resid_q8) >> OBS_Q15);
    s_obs.vel_q8 = Fx_Clamp32(s_obs.vel_q8, OBS_VMAX_CDPS << OBS_Q8);

    /* d += L3*e；L3 设计值为负，写成减法会变成正反馈 */
    s_obs.load_q8 += (int32_t)(((int64_t)OBS_L3_Q15 * resid_q8) >> OBS_Q15);
    s_obs.load_q8 = Fx_Clamp32(s_obs.load_q8, OBS_LOAD_MAX << OBS_Q8);

    if (out == 0) return;
    out->pos    = s_obs.pos_q8  >> OBS_Q8;
    out->vel    = s_obs.vel_q8  >> OBS_Q8;
    out->load   = s_obs.load_q8 >> OBS_Q8;
    out->load_q8 = s_obs.load_q8;
    out->effort = u_net;
}

void C_SpeedObs_SetPlant(int32_t rho_q12, int32_t k_q12)
{
    s_rho_q12 = rho_q12;
    s_k_q12   = k_q12;
    Obs_ApplyPlant();
}

int32_t C_SpeedObs_AlphaQ15(void) { return (int32_t)(G_ALPHA >> 9); }
