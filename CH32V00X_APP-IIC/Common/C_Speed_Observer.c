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

/*
 * @fn      Obs_WrapFold
 * @brief   把差值折算到最短路径(-range/2, range/2]
 *
 * 电机连续转动跨0点(360->0)时，测量位置会突然跳一整圈。所有位置差都必须
 * 经过本函数，否则那一拍的残差会被当成一个量程大小的假阶跃灌进观测器。
 * 前提是每拍真实位移远小于半圈，1kHz采样下恒满足。
 */
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

/*
 * @fn      C_SpeedObs_Init
 * @brief   初始化状态并把配置里的对象常数换算成定点模型系数
 * @param   init_pos 初始位置(厘度)
 *
 * Kv 由速度前馈斜率取倒数得到：前馈是 PWM/(厘度/秒)，倒数即
 * (厘度/秒)/PWM。取双向平均——两个方向差异只有1.3%，用平均值带来的
 * 误差远小于 d 状态能吸收的范围。
 */
void C_SpeedObs_Init(int32_t init_pos)
{
    int32_t slope_q16;
    uint8_t i;

    for (i = 0U; i < OBS_DEAD_MAX; i++) s_obs.u_hist[i] = 0;
    s_obs.u_idx = 0U;

    s_obs.pos_q8  = init_pos << OBS_Q8;
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

/*
 * @fn      C_SpeedObs_Update
 * @brief   推进一拍：模型预测 + 编码器残差校正
 *
 * 计算量：约8次乘法(其中4次在int64上)，无除法、无开方、无查表。
 */
void C_SpeedObs_Update(int32_t measured_pos, int16_t pwm_applied, SpeedObsOut_t *out)
{
    int32_t u_delayed, u_net, drive_q8;
    int32_t pos_pred_q8, vel_pred_q8, resid_q8;
    int32_t range_q8;

    if (!s_obs.ready) C_SpeedObs_Init(measured_pos);

    /* ---- 1. 纯延迟对齐 ----
     * 标定实测指令到编码器响应有 plant_dead_ticks 拍纯延迟。预测步必须用
     * 当时真正作用在电机上的那个PWM，否则模型会比实际提前一个延迟量，
     * 在加减速段产生系统性偏差(实测梯形RMS 37 -> 80)。 */
    /* 控制器返回的是 H 桥物理方向的 PWM；观测器状态则始终使用“编码器角度
     * 增大为正”的统一坐标。反向装配时必须在进入延迟线前把方向换回来，
     * 否则控制器虽然是负反馈，模型预测却会反向，残差和扰动状态都会失真。 */
    s_obs.u_hist[s_obs.u_idx] = (int16_t)(pwm_applied * CTRL_MOTOR_SIGN);
    s_obs.u_idx = (uint8_t)((s_obs.u_idx + 1U) % OBS_DEAD_MAX);
    u_delayed = s_obs.u_hist[(s_obs.u_idx + OBS_DEAD_MAX - 1U - s_obs.dead) % OBS_DEAD_MAX];

    /* ---- 2. 扣除摩擦(连续模型，无阈值无分支跳变) ----
     * 摩擦随运动方向翻符号。不扣掉的话每次换向 d 状态都要重新收敛一遍，
     * 换向工况误差会从62劣化到200以上。
     *
     * 旧版在 |v|=0.5°/s 处硬切换符号，模型输入会出现一个 ±coulomb(约88计数)
     * 的阶跃；零速附近来回穿越时(到位保持、爬行、换向)观测器被反复踢一下，
     * 这是一个**纯人为**的扰动源。这里换成连续过渡：
     *     f     = clamp(v/Vs, -1, +1)                    运动程度
     *     u_net = u - F*f - (1-|f|)*clamp(u, ±F)
     * 两端精确退化为正确的物理，中间连续：
     *     |f|=1 (已在动)  u_net = u - F*sign(v)          库仑摩擦
     *      f =0 (静止)    u_net = u - clamp(u,±F)        |u|<F 时净驱动为0
     *                                                    (静摩擦锁住，不会
     *                                                     预测出并不存在的加速)
     * 对 u 也连续，所以起转/停转的瞬间模型不再跳变。代价是 3 次乘法。 */
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

    /* ---- 3. 模型预测 ----
     * drive = Kv*(u_net - d) 是该净PWM对应的稳态速度；
     * 速度按一阶惯性向它收敛：v += (drive - v) * T/tau。 */
    drive_q8    = s_obs.kv_q8 * u_net
                - (int32_t)(((int64_t)s_obs.kv_q8 * s_obs.load_q8) >> OBS_Q8);
    vel_pred_q8 = s_obs.vel_q8
                + (int32_t)(((int64_t)(drive_q8 - s_obs.vel_q8) * s_obs.inv_tau_q16) >> 16);

    /* 位置积分：p += v*T，T=1ms。除以1000用Q22倒数(4194)代替，避免除法。 */
    pos_pred_q8 = s_obs.pos_q8 + (int32_t)(((int64_t)s_obs.vel_q8 * 4194) >> 22);

    range_q8 = (int32_t)CDEG_RANGE << OBS_Q8;
    if (pos_pred_q8 >= range_q8) pos_pred_q8 -= range_q8;
    else if (pos_pred_q8 < 0)    pos_pred_q8 += range_q8;

    /* ---- 4. 残差校正 ---- */
    resid_q8 = Obs_WrapFold((measured_pos << OBS_Q8) - pos_pred_q8, range_q8);

    s_obs.pos_q8 = pos_pred_q8
                 + (int32_t)(((int64_t)OBS_L1_Q15 * resid_q8) >> OBS_Q15);
    if (s_obs.pos_q8 >= range_q8) s_obs.pos_q8 -= range_q8;
    else if (s_obs.pos_q8 < 0)    s_obs.pos_q8 += range_q8;

    s_obs.vel_q8 = vel_pred_q8
                 + (int32_t)(((int64_t)OBS_L2_Q15 * resid_q8) >> OBS_Q15);
    s_obs.vel_q8 = Obs_Clamp(s_obs.vel_q8, OBS_VMAX_CDPS << OBS_Q8);

    /* d 与位置/速度用同一形式的校正：d += L3*e。obs_l3_q15 的设计值本身为负，
     * 所以实际效果是"测量位置落后于预测 -> 判定负载变大 -> d 增大"。
     * 写成减法会把负反馈变成正反馈，观测器直接发散。 */
    s_obs.load_q8 += (int32_t)(((int64_t)OBS_L3_Q15 * resid_q8) >> OBS_Q15);
    s_obs.load_q8 = Obs_Clamp(s_obs.load_q8, OBS_LOAD_MAX << OBS_Q8);

    if (out == 0) return;
    out->pos  = s_obs.pos_q8  >> OBS_Q8;
    out->vel  = s_obs.vel_q8  >> OBS_Q8;
    out->load = s_obs.load_q8 >> OBS_Q8;
}
