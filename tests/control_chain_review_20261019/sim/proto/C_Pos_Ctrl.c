/* 方案原型(浮点，仅仿真验证)：与生产版 C_Pos_Ctrl 同接口。
 * 一条连续控制律贯穿 MOVE/SETTLE/HOLD：
 *   u = 前馈(kv·v_r + ka_eff·a_r + fc·sat(v_r/v0)) + Kv·(v_cmd − v̂) + kv·ωp·e(修正量也走模型)
 *     + I(只积位置误差) + d̂(观测器扰动，低通，负载/惯量失配补偿) + 起转补偿(按参考判定)
 * HOLD 保持用户语义：窗内 u = 托举力(捕获时的 I + d̂ 冻结)，窗外 + K_h·ex，另加按标定换算的连续阻尼(ζ 统一)。
 * 各项由 PX_* 开关控制，便于逐项对比。 */
#include "C_Pos_Ctrl.h"
#include "C_Fixed.h"
#include <string.h>
#include <math.h>

double PX_J = 0;                 /* 模型惯量比(负载/电机) */
int    PX_DOB = 1;               /* 观测器扰动补偿 */
double PX_DOB_W = 40;            /* 扰动补偿低通，rad/s */
int    PX_CORR = 1;              /* 位置修正量同时走模型 kv */
int    PX_FRIC = 1;              /* 低速摩擦前馈按参考速度 */
int    PX_POSINT = 1;            /* 积分只积位置误差 */
int    PX_HOLDD = 1;             /* 保持阻尼按 ζ 统一 */
double PX_HOLD_ZETA = 0.7;
double PX_V0 = 50;               /* 摩擦前馈过渡速度，厘度/s */
double PX_dist_in;               /* 本拍观测器扰动估计(mV)，由编排层写入 */
int    PX_stuck_in;              /* 本拍观测器粘住 */
double PX_static_load;           /* 供规划用：当前托举力估计 mV */

#define CAPTURE_MS 20
static struct {
    double ka, kv, fc, fs, delay, cal;
    double Kv, wp, wi, wv_eff, Ki;
    double Kh, Kih, Kd;
    double w_in, vel_noise, noise, min_step;
    uint16_t precision, hold_kp, hold_ki;
    uint16_t vel_bw_max;
    PosCtrlGains_t g;
} G;
static struct {
    double I, dl, hold_force;
    int state, cap;
    double anchor; int still; double boost;
    double hold_p;
} S;

static double dz(double v, double d) { return v > d ? v - d : (v < -d ? v + d : 0); }
static double sat1(double x) { return x > 1 ? 1 : (x < -1 ? -1 : x); }

static void Gains(void)
{
    double ka_eff = G.ka * (1 + PX_J);
    double kv_tot = G.ka * G.g.vel_bw;              /* 与生产版相同：Kv 按空载模型定(受噪声限制，不随惯量放大) */
    double kvp = kv_tot - G.kv;
    G.Kv = (kvp > kv_tot / 4) ? kvp : kv_tot / 4;
    G.wv_eff = (G.Kv + G.kv) / ka_eff;              /* 带负载惯量后的实际速度环带宽 */
    G.wp = G.g.pos_bw; if (G.wp > G.wv_eff / 4) G.wp = G.wv_eff / 4;
    G.wi = G.g.int_bw; if (G.wi > G.wp) G.wi = G.wp;
    G.Ki = (G.Kv + (PX_CORR ? G.kv : 0)) * G.wp * G.wi;    /* 位置积分：零点在 ωi */
    G.Kh = G.cal * G.hold_kp / 100000.0;
    G.Kih = G.cal * G.hold_ki / 100000.0;
    /* 保持阻尼：总阻尼(kv + Kd) 达到 2ζ√(Kh·ka_eff)，不够才补 */
    G.Kd = 0;
    if (PX_HOLDD && G.Kh > 0) { double need = 2 * PX_HOLD_ZETA * sqrt(G.Kh * ka_eff) - G.kv; if (need > 0) G.Kd = need; }
}

void C_PosCtrl_Init(const MotorCal_t *cal, int32_t vel_noise, uint16_t obs_bw_hz)
{
    memset(&G, 0, sizeof(G));
    G.ka = cal->ka_nv / 1e6; G.kv = cal->kv_uv / 1e3; G.fc = cal->fc_mv; G.fs = cal->fs_mv;
    G.delay = cal->delay_us / 1e6; G.cal = cal->cal_mv;
    G.noise = cal->noise_cdeg; G.min_step = cal->min_step_cdeg; G.vel_noise = vel_noise;
    G.vel_bw_max = (uint16_t)((cal->delay_us > 333U) ? 1000000UL / (3UL * cal->delay_us) : 1000UL);
    if (G.vel_bw_max > (uint32_t)obs_bw_hz * 6283U / 1000U) G.vel_bw_max = (uint16_t)((uint32_t)obs_bw_hz * 6283U / 1000U);
    C_PosCtrl_Reset();
}

uint8_t C_PosCtrl_SetGains(PosCtrlGains_t *g)
{
    g->vel_bw = (uint16_t)Fx_ClampRange(g->vel_bw, 10, G.vel_bw_max);
    g->pos_bw = (uint16_t)Fx_ClampRange(g->pos_bw, 1, g->vel_bw / 4);
    g->int_bw = (uint16_t)Fx_ClampRange(g->int_bw, 0, g->pos_bw);
    G.g = *g;
    Gains();
    return 0;
}

void C_PosCtrl_Rebuild(void) { Gains(); }   /* 惯量比变化后重算 */

void C_PosCtrl_SetHold(uint16_t precision, uint16_t hold_kp, uint16_t hold_ki)
{
    if (precision == G.precision && hold_kp == G.hold_kp && hold_ki == G.hold_ki) return;
    G.precision = precision; G.w_in = precision; G.hold_kp = hold_kp; G.hold_ki = hold_ki;
    Gains();
}

void C_PosCtrl_Reset(void) { memset(&S, 0, sizeof(S)); S.state = PC_SETTLE; S.still = 0xFFFF; }
uint8_t C_PosCtrl_Reached(void) { return (uint8_t)(S.state == PC_HOLD); }
double C_PosCtrl_Dob(void) { return S.dl; }
double C_PosCtrl_Integral(void) { return S.I; }

static double Stiction(const PosCtrlFeedback_t *fb, int demand, double u)
{
    double need;
    if (fabs(fb->pos - S.anchor) >= G.min_step) { S.anchor = fb->pos; S.still = 0; }
    else if (S.still < 0xFFFF) S.still++;
    if (demand == 0 || S.still < 5 || fabs(fb->vel) >= G.vel_noise)
    { S.boost -= G.fs / 50; if (S.boost < 0) S.boost = 0; return 0; }
    if (S.still >= 25 && S.boost < G.fs) S.boost += G.fs / 50;
    need = S.I + S.dl + demand * G.fs;
    return (((demand > 0) ? (u < need) : (u > need)) ? need - u : 0) + demand * S.boost;
}

int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit_mv, PosCtrlDbg_t *dbg)
{
    const double dt = 1e-3;
    double e = ref->pos - fb->pos, v = fb->vel;
    double ka_eff = G.ka * (1 + PX_J);
    double u_ff = 0, u_fb = 0, u_st = 0, desired, out, di = 0;
    int demand = 0;

    /* 状态：参考在动 MOVE；停下 SETTLE；误差进窗且静止 20ms 捕获 HOLD */
    if (!ref->done)
    {
        if (S.state == PC_HOLD) { S.I += S.hold_p; }   /* 无扰：窗外弹簧并入积分 */
        S.state = PC_MOVE;
    }
    else if (fb->valid)
    {
        if (S.state == PC_MOVE) { S.state = PC_SETTLE; S.cap = 0; }
        else if (S.state == PC_SETTLE)
        {
            if (fabs(v) <= G.vel_noise && fabs(e) <= G.w_in) { if (++S.cap >= CAPTURE_MS) S.state = PC_HOLD; }
            else S.cap = S.cap > 2 ? S.cap - 2 : 0;
        }
    }

    /* 扰动补偿：观测器扰动低通；粘住时不更新(静摩擦带内不可观)；HOLD 中冻结(保持柔顺) */
    if (PX_DOB && S.state != PC_HOLD && !PX_stuck_in) S.dl += (PX_dist_in - S.dl) * PX_DOB_W * dt;
    if (!PX_DOB) S.dl = 0;
    {
        double lim = limit_mv * 0.8;
        if (S.dl > lim) S.dl = lim; if (S.dl < -lim) S.dl = -lim;
    }

    if (S.state != PC_HOLD)
    {
        double eb = dz(e, S.state == PC_MOVE ? G.noise : G.min_step);
        double v_cmd = ref->vel + G.wp * eb;
        double fr = PX_FRIC ? sat1(ref->vel / PX_V0) : sat1(ref->vel / G.vel_noise);
        u_ff = G.kv * (ref->vel + ref->acc * G.delay) + ka_eff * ref->acc + G.fc * fr;
        u_fb = G.Kv * (v_cmd - (S.state == PC_MOVE ? v : dz(v, G.vel_noise / 2))) + (PX_CORR ? G.kv * G.wp * eb : 0);
        if (PX_POSINT) di = G.Ki * eb * dt;
        else di = G.ka * G.g.vel_bw * G.wi * (v_cmd - v) * dt;
        if (S.state == PC_MOVE)
        {
            double vr = PX_FRIC ? PX_V0 : G.vel_noise;
            if (fabs(v_cmd) > vr && ref->acc * ref->vel >= 0 && (v_cmd > 0 ? 1 : -1) * e >= -G.w_in) demand = v_cmd > 0 ? 1 : -1;
        }
        S.hold_p = 0;
    }
    else
    {
        double ex = dz(e, G.w_in);
        S.hold_p = G.Kh * ex;
        u_fb = S.hold_p - G.Kd * v;
        di = G.Kih * ex * dt;
    }
    u_st = Stiction(fb, demand, u_ff + u_fb + S.I + S.dl);
    desired = u_ff + u_fb + S.I + S.dl + u_st;
    if (S.state == PC_HOLD && fabs(desired) < G.fc / 2) desired = 0;
    out = desired > limit_mv ? limit_mv : (desired < -limit_mv ? -limit_mv : desired);
    if (fb->valid && di != 0 && !((di > 0 && desired > out) || (di < 0 && desired < out)))
    {
        double il = limit_mv * 0.8;
        S.I += di; if (S.I > il) S.I = il; if (S.I < -il) S.I = -il;
    }
    PX_static_load = S.I + S.dl;
    if (dbg)
    {
        dbg->e_pos = (int32_t)e; dbg->u_ff = (int16_t)u_ff; dbg->u_fb = (int16_t)u_fb; dbg->u_i = (int16_t)(S.I + S.dl);
        dbg->u_stick = (int16_t)u_st; dbg->u_out = (int16_t)out; dbg->sat = (uint8_t)(fabs(desired - out) > 0.5); dbg->state = (uint8_t)S.state;
    }
    return (int32_t)lround(out);
}
