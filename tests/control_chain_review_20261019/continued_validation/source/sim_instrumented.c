/* 控制链路闭环仿真：把某个工程未修改的生产代码(A_Servo/规划器/控制器/观测器/保护/配置)编译进同一个 TU，
 * 只替换硬件。被控对象 = 电机侧 + 齿轮(刚度/齿隙) + 负载侧惯量 + 重力/外载 + 静/动摩擦 + 纯延迟，
 * 电位器测负载侧，1ms 块均值 + 噪声 + 波纹 + Q4 量化。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "A_Config.h"
static uint8_t flash_mem[65536];
#define FLASH_BASE ((uintptr_t)flash_mem)
#define FLASH_COMPLETE 0
static int FLASH_ROM_ERASE(uint32_t a, uint32_t n) { memset((void *)(uintptr_t)a, 255, n); return 0; }
static int FLASH_ROM_WRITE(uint32_t a, const uint32_t *p, uint32_t n) { memcpy((void *)(uintptr_t)a, p, n); return 0; }
#define CONFIG_IMAGE_TAG() 1234U
#include "flash.c"
#include "A_Config.c"
#include "C_Motor_Model.c"
#include "C_Pos_Ctrl.c"
#include "C_Traj_Planner.c"
#include "C_Speed_Observer.c"
#include "A_Sensor.c"
#include "A_Servo.c"
#include "A_Protect.c"

static const MotorCal_t cal = {.kv_uv = MOTOR_KV_UV, .ka_nv = MOTOR_KA_NV, .fc_mv = MOTOR_FC_MV,
    .fs_mv = MOTOR_FS_MV, .delay_us = MOTOR_DELAY_US, .cal_mv = MOTOR_CAL_MV, .noise_cdeg = MOTOR_NOISE_CDEG,
    .min_step_cdeg = MOTOR_MIN_STEP_CDEG, .sign = MOTOR_SIGN, .rated_vel_dps = MOTOR_RATED_VEL_DPS,
    .rated_acc_dps2 = MOTOR_RATED_ACC_DPS2, .rated_dec_dps2 = MOTOR_RATED_DEC_DPS2};

/* ============================ 被控对象 ============================ */
typedef struct {
    double ka_s, kv_s, fc_s, fs_s, delay_s;   /* 对象相对标定值的比例 */
    double r;              /* 负载惯量 / 电机侧惯量 */
    double bl;             /* 齿隙半宽，厘度 */
    double res_hz;         /* 齿轮-负载谐振频率 */
    double grav;           /* 重力力矩幅值，mV(等效电压)；>0 倒立摆(竖直不稳定)，<0 悬挂 */
    double up;             /* 重力零点(竖直)所在角度，厘度 */
    double fcl, fsl;       /* 负载侧动/静摩擦，mV */
    double noise;          /* 电位器块均值噪声 σ，厘度 */
    double rip_a, rip_p;   /* 电位器波纹幅值/周期，厘度 */
    double vbus;           /* 母线，mV */
    double rbat;           /* 电池内阻，Ω */
    double rw;             /* 绕组电阻，Ω */
} Plant;

static double audit_rw=3.2;
static Plant P;
static double (*load_fn)(double t_ms, double x, double v);  /* 额外外载(mV)，阻碍角度增大为正 */
static double xm, vm, xl, vl, pot_mean, i_avg, vbus_now;
static double uhist[1024];
static unsigned uh_i;
static int duty_now, sim_ms, cur_settle, prev_ph, pwm_mode, pwm_pulse_pending, pwm_pulse_init;
static unsigned rng_state = 1;

static double urand(void) { rng_state = rng_state * 1664525U + 1013904223U; return ((rng_state >> 8) + 0.5) / 16777216.0; }
static double grand(void) { return sqrt(-2 * log(urand())) * cos(6.283185307 * urand()); }

/* 硬件桩 */
uint8_t IAP_HwInfoRead(IAP_HwInfo_t *p) { memset(p, 0, sizeof(*p)); return 0; }
uint8_t IAP_PreservedIdentityRead(uint8_t *a, uint8_t *b) { *a = 0; *b = 5; return 1; }
void D_UART_Enable(uint8_t x) { (void)x; }
void D_PWM_Input_Enable(uint8_t x) { (void)x; }
uint16_t D_PWM_Read(void)
{
    uint16_t p;
    if (!pwm_mode) return 0;
    if (pwm_pulse_init) return (uint16_t)pwm_pulse_init;
    p = (uint16_t)pwm_pulse_pending; pwm_pulse_pending = 0; return p;
}
void Delay_Ms(uint32_t x) { (void)x; }
int16_t D_TMP112_Read_Temp(void) { return 250; }
int16_t D_Motor_Set(int16_t x)
{
    int ph = abs(x) >= (int)D_ADC_CUR_MIN_TICKS ? (x > 0 ? 1 : -1) : 0;
    if (x > 2400) x = 2400;
    if (x < -2400) x = -2400;
    if (ph != prev_ph) cur_settle = 1;
    prev_ph = ph; duty_now = x; return x;
}
void D_Motor_Release(uint8_t h) { (void)h; duty_now = 0; }
uint16_t D_ADC_Voltage_Read(void) { return (uint16_t)lround(vbus_now * 4096. / 16170.); }
uint8_t D_ADC_Encoder_Read(uint16_t *p)
{
    double raw = (ENCODER_POT_ADC_MIN + pot_mean * ENCODER_POT_ADC_SPAN / ENCODER_POT_SPAN_CDEG) * 16;
    *p = (uint16_t)fmin(65520, fmax(0, lround(raw)));
    return 1;
}
uint8_t D_ADC_Current_Read(int32_t *p)
{
    double i_drive = i_avg * (duty_now >= 0 ? 1 : -1) * MOTOR_SIGN;
    *p = (int32_t)lround(i_drive * 65536.0 / CURRENT_FULL_SCALE_MA);
    return (uint8_t)(abs(duty_now) >= (int)D_ADC_CUR_MIN_TICKS && !cur_settle);
}

static double sgn(double x) { return (x > 0) - (x < 0); }

/* 一个控制拍(1ms = 20个PWM周期)的物理过程，10µs 积分 */
static void physics_ms(void)
{
    const double dt = 1e-5;
    double ka_m = cal.ka_nv / 1e6 * P.ka_s, kv = cal.kv_uv / 1e3 * P.kv_s;
    double fc = cal.fc_mv * P.fc_s, fs = cal.fs_mv * P.fs_s;
    double ka_l = cal.ka_nv / 1e6 * P.r;
    int rigid = (P.r <= 0 || (P.bl <= 0 && P.res_hz <= 0));
    double ka_eff = rigid ? 0 : ka_m * ka_l / (ka_m + ka_l);
    double w = 6.283185307 * (P.res_hz > 0 ? P.res_hz : 60.0);
    double kg = ka_eff * w * w, cg = 2 * 0.08 * sqrt(kg * ka_eff);
    int delay = (int)lround(fmax(0, (cal.delay_us - 500.0) * P.delay_s) / 10.0);
    double u_cmd = duty_now * vbus_now / 2400.0 * MOTOR_SIGN, sum = 0, isum = 0;
    int n = 0;

    if (delay > 1000) delay = 1000;
    for (int k = 0; k < 100; k++)
    {
        double u, tg = 0, ext;
        uhist[uh_i] = u_cmd;
        u = uhist[(uh_i + 1024 - delay) % 1024];
        uh_i = (uh_i + 1) % 1024;
        ext = (P.grav != 0 ? -P.grav * sin((xl - P.up) * 3.14159265358979 / 18000.0) : 0)
            + (load_fn ? load_fn(sim_ms + k * 0.01, xl, vl) : 0);
        if (rigid)
        {   /* 电机与负载一体 */
            double ka = ka_m + ka_l, net = u - ext, f_s = fs + P.fsl, f_c = fc + P.fcl;
            if (fabs(vm) < 1e-7 && fabs(net) <= f_s) vm = 0;
            else
            {
                double s = fabs(vm) < 1e-7 ? sgn(net) : sgn(vm);
                double vn = vm + (net - kv * vm - f_c * s) / ka * dt;
                if (fabs(vm) >= 1e-7 && vn * vm < 0) vn = 0;
                vm = vn;
            }
            xm += vm * dt; xl = xm; vl = vm;
        }
        else
        {
            double d = xm - xl, dv = vm - vl, netm, netl;
            if (d > P.bl) { tg = kg * (d - P.bl) + cg * dv; if (tg < 0) tg = 0; }
            else if (d < -P.bl) { tg = kg * (d + P.bl) + cg * dv; if (tg > 0) tg = 0; }
            netm = u - tg;
            if (fabs(vm) < 1e-7 && fabs(netm) <= fs) vm = 0;
            else
            {
                double s = fabs(vm) < 1e-7 ? sgn(netm) : sgn(vm);
                double vn = vm + (netm - kv * vm - fc * s) / ka_m * dt;
                if (fabs(vm) >= 1e-7 && vn * vm < 0) vn = 0;
                vm = vn;
            }
            netl = tg - ext;
            if (fabs(vl) < 1e-7 && fabs(netl) <= P.fsl) vl = 0;
            else
            {
                double s = fabs(vl) < 1e-7 ? sgn(netl) : sgn(vl);
                double vn = vl + (netl - P.fcl * s) / ka_l * dt;
                if (fabs(vl) >= 1e-7 && vn * vl < 0 && P.fcl > 0) vn = 0;
                vl = vn;
            }
            xm += vm * dt; xl += vl * dt;
        }
        isum += (u - kv * vm) / P.rw;
        if (k % 5 == 4) { sum += xl; n++; }
    }
    i_avg = isum / 100;
    pot_mean = sum / n + P.noise * grand() + (P.rip_a ? P.rip_a * sin(6.283185307 * xl / P.rip_p) : 0);
    vbus_now = P.vbus - P.rbat * fabs(i_avg) * fabs(duty_now) / 2400.0;
}

/* ============================ 场景框架 ============================ */
#define DEG(x) ((x) * 100.0)
static int pwm_of(double cdeg) { return (int)lround(500 + cdeg * 2000.0 / 27000.0); }

typedef struct {
    /* 结果 */
    double max_track, overshoot, final_err, max_du, sat_frac, freeze_frac;
    int stops, reverses, peaks, stall_trips, faults, settle_ms;
    double ripple, vmin_ratio, ref_vmin_ratio, vmean, lag;
    int arr_ms;   /* 最后一条指令后到上报到位(arrived)的时间 */
} Res;

static FILE *trace;
static const char *trace_name;
static int stall_prev;

#ifdef PROTO
static double px_J_init; static int px_jtrue;
#endif
static int g_holdki = -1, g_holdkp = -1, g_tmin = -1;
static double g_slowv = 2;      /* slowv 场景的巡航速度，度/秒 */
static void sim_reset(const Plant *pl, int pwm_input, int init_pulse)
{
#ifdef PROTO
    PX_J = px_jtrue ? pl->r : px_J_init;
#endif
    P = *pl;
    memset(flash_mem, 255, sizeof(flash_mem));
    memset(uhist, 0, sizeof(uhist)); uh_i = 0;
    xm = xl = pot_mean = 13500; vm = vl = 0; i_avg = 0; vbus_now = P.vbus;
    duty_now = 0; sim_ms = 0; cur_settle = 0; prev_ph = 0; rng_state = 12345;
    pwm_mode = pwm_input; pwm_pulse_pending = 0; pwm_pulse_init = init_pulse;
    A_Config_Init();
    g_config.boot_mode = SERVO_BOOT_HOLD; g_config.servo_mode = SERVO_MODE_270_CCW;
    C_MotorModel_Init(&cal);
    if (g_holdki >= 0) g_config.hold_ki = g_runtime_config.hold_ki = (uint16_t)g_holdki;
    if (g_holdkp >= 0) g_config.hold_kp = g_runtime_config.hold_kp = (uint16_t)g_holdkp;
    if (g_tmin >= 0) g_config.traj_move_min_ms = g_runtime_config.traj_move_min_ms = (uint16_t)g_tmin;
    A_Servo_Init(); A_Protect_Init();
    pwm_pulse_init = 0;
    stall_prev = 0;
}

static void sim_tick(void)
{
    sim_ms++;
    physics_ms();
    A_Servo_Control();
    if (sim_ms % 10 == 0) A_Protect_Task();
    cur_settle = 0;
}

/* 指令脚本：每拍调用，返回 1 表示本拍发了新目标 */
typedef void (*Script)(int t);
static double g_final_target;
static int g_motion_end;   /* 最后一条指令发出的时刻 */

static void cmd(double cdeg, int T) { g_final_target = cdeg; A_Servo_Submit((uint16_t)pwm_of(cdeg), (uint16_t)T); }
static void cmd_pwm(double cdeg) { g_final_target = cdeg; pwm_pulse_pending = pwm_of(cdeg); }

/* 运行一个场景并计算平顺性指标。ws/we：流式指令的稳态统计窗(ms)，kind 1=斜坡流 2=正弦 */
static int g_ws, g_we, g_kind;
static double g_cmd_now;   /* 本拍的"理想指令"位置(正弦/斜坡)，算跟踪滞后用 */
static Res run(const char *name, Script s, int dur)
{
    Res r; memset(&r, 0, sizeof(r));
    double vf = 0, vbuf[8] = {0}; int vbi = 0;
    int fault_prev = 0;
    int armed = 0, revon = 0, below = 0, lastu = 0, nsat = 0, nfreeze = 0, nmov = 0;
    double vsum = 0; int vn = 0; double vmin = 1e9, vmax = -1e9, rvmin = 1e9, rvmax = -1e9;
    int in_win = 0, last_cmd_seen = -1, approach = 0, ring_sign = 0;
    double ring_ext = 0;
    (void)name;
    r.settle_ms = -1; r.arr_ms = -1;
    for (int t = 0; t < dur; t++)
    {
        s(t);
        if (g_motion_end != last_cmd_seen) { r.arr_ms = -1; in_win=0; r.settle_ms=-1; last_cmd_seen = g_motion_end; approach = (int)sgn(g_final_target - xl); r.overshoot = 0; r.peaks = 0; ring_sign = 0; ring_ext = 0; }
        sim_tick();
        vbuf[vbi] = vl; vbi = (vbi + 1) & 7;
        vf = 0; for (int i = 0; i < 8; i++) vf += vbuf[i]; vf /= 8;
        {
            double ref_v = s_servo.ref.vel, e = s_servo.ref.pos - xl;
            int du = abs(s_servo.dbg.u_out - lastu);
            lastu = s_servo.dbg.u_out;
            if (s_servo.debug_control_valid && du > r.max_du && t > 5) r.max_du = du;
            if (!s_servo.ref.done)
            {
                if (fabs(e) > r.max_track) r.max_track = fabs(e);
                nmov++;
                if (s_servo.dbg.sat) nsat++;
                if (s_servo.dbg.sat && (double)s_servo.dbg.e_pos * ref_v > 0) nfreeze++;
            }
            /* 走走停停：参考在明显运动，实际速度先达到参考的50%，又掉到15%以下持续>=10ms 记一次 */
            if (fabs(ref_v) > 300 && !s_servo.ref.done)
            {
                if (vf * sgn(ref_v) >= 0.5 * fabs(ref_v)) armed = 1;
                if (armed && vf * sgn(ref_v) < 0.15 * fabs(ref_v)) { if (++below == 10) { r.stops++; armed = 0; } }
                else below = 0;
                if (vf * sgn(ref_v) < -150) { if (!revon) r.reverses++; revon = 1; } else revon = 0;
            }
            else { below = 0; armed = 0; revon = 0; }
            if (g_kind == 3)
            {
                static int stick_run;
                double vc = g_slowv * 100;
                if (!s_servo.ref.done && fabs(ref_v) > 0.5 * vc)
                {
                    vsum += fabs(vl); vn++;
                    if (fabs(vl) < 0.05 * vc) { if (++stick_run == 20) r.stops++; } else stick_run = 0;
                    if (fabs(vf) > vmax) vmax = fabs(vf);
                }
                else stick_run = 0;
            }
            /* 流式稳态窗 */
            if (g_kind && t >= g_ws && t < g_we)
            {
                vsum += vf; vn++;
                if (vf < vmin) vmin = vf; if (vf > vmax) vmax = vf;
                if (ref_v < rvmin) rvmin = ref_v; if (ref_v > rvmax) rvmax = ref_v;
                if (g_kind == 2 && fabs(xl - g_cmd_now) > r.lag) r.lag = fabs(xl - g_cmd_now);
            }
            /* 最后一条指令之后、参考停下以后：过冲与来回摆 */
            if (s_servo.ref.done && approach)
            {
                double ex = (xl - g_final_target) * approach;   /* >0 = 冲过头 */
                if (ex > r.overshoot) r.overshoot = ex;
                if (fabs(xl - g_final_target) > 30)
                {
                    int sg = (int)sgn(xl - g_final_target);
                    if (sg != ring_sign) { if (ring_sign) r.peaks++; ring_sign = sg; }
                }
            }
        }
        if (trace && (t % 2 == 0))
            fprintf(trace, "%s,%d,%.1f,%.1f,%d,%d,%d,%.1f,%d,%d,%d,%d,%.0f,%d,%d,%d,%d,%d,%d,%d\n", trace_name, t, xl, vl, s_servo.target_angle,
                    s_servo.ref.pos, s_servo.ref.vel, vf, s_servo.obs.vel, s_servo.dbg.u_out, s_servo.dbg.u_i, s_servo.dbg.state,
                    i_avg, s_servo.out_limit, s_servo.torque, s_servo.enc_state, s_prot.fault, s_prot.stalled, s_servo.paused, duty_now);
        if (s_prot.fault && !fault_prev) r.faults++;
        fault_prev = s_prot.fault;
        if (s_prot.stalled && !stall_prev) r.stall_trips++;
        stall_prev = s_prot.stalled;
        if (t > g_motion_end && r.arr_ms < 0 && s_servo.arrived && t > g_motion_end + 2) r.arr_ms = t - g_motion_end;
        if (t > g_motion_end)
        {
            double ef = xl - g_final_target;
            if (fabs(ef) <= 50) { if (!in_win) { in_win = 1; r.settle_ms = t - g_motion_end; } }
            else { in_win = 0; r.settle_ms = -1; }
        }
    }
    r.final_err = xl - g_final_target;
    r.sat_frac = nmov ? (double)nsat / nmov : 0;
    r.freeze_frac = nmov ? (double)nfreeze / nmov : 0;
    if (g_kind == 3 && vn) { r.vmean = vsum / vn; r.ripple = vmax / (g_slowv * 100); vn = 0; }
    if (vn) { double m = vsum / vn; r.vmean = m; r.vmin_ratio = vmin / m; r.ripple = (vmax - vmin) / m; r.ref_vmin_ratio = rvmin / m; }
    return r;
}

/* ---------- 指令脚本 ---------- */
static void sc_bigstep(int t)          /* +90° 后 -180°，T=0 */
{
    if (t == 0) { cmd(22500, 0); g_motion_end = 0; }
    if (t == 2500) { cmd(4500, 0); g_motion_end = 2500; }
}
static void sc_reverse(int t)          /* 180° 两端每 400ms 换向(中途打断)，3.2s */
{
    if (t < 3200 && t % 400 == 0) { cmd((t / 400) % 2 ? 4500 : 22500, 0); g_motion_end = t; }
}
static void sc_reverse_arrive(int t)   /* 90° 每 1200ms 换向(先到位再换) */
{
    if (t < 6000 && t % 1200 == 0) { cmd((t / 1200) % 2 ? 9000 : 18000, 0); g_motion_end = t; }
}
static void sc_stream_pwm(int t)       /* 50Hz PWM 输入，10°/s 斜坡 4s */
{
    if (t % 20 == 0 && t <= 4000) { cmd_pwm(13500 + 1000.0 * t / 1000); g_motion_end = t; }
    else if (t % 20 == 0) cmd_pwm(g_final_target);
}
static void sc_stream_bus50(int t)     /* 总线每50ms一条，+0.5°(10°/s)，T=0 */
{
    if (t % 50 == 0 && t <= 4000) { cmd(13500 + 1000.0 * t / 1000, 0); g_motion_end = t; }
}
static void sc_stream_bus100T(int t)   /* 总线每100ms一条，+2°(20°/s)，T=100 */
{
    if (t % 100 == 0 && t <= 4000) { cmd(13500 + 2000.0 * t / 1000, 100); g_motion_end = t; }
}
static void sc_stream_bus200T(int t)   /* 总线每200ms一条，+4°(20°/s)，T=200 */
{
    if (t % 200 == 0 && t <= 4000) { cmd(13500 + 2000.0 * t / 1000, 200); g_motion_end = t; }
}
static void sc_sine(int t)             /* 总线 20ms 一条跟踪 ±30°、0.5Hz 正弦，T=20 */
{
    g_cmd_now = 13500 + 3000 * sin(6.283185307 * 0.5 * t / 1000.0);
    if (t % 20 == 0 && t <= 6000) { cmd(g_cmd_now, 20); g_motion_end = t; }
}
static void sc_slow(int t)             /* 10° 用 5s(2°/s) */
{
    if (t == 0) { cmd(14500, 5000); g_motion_end = 0; }
}
static void sc_slowv(int t)            /* 20° 按 g_slowv 度/秒 走(带时间指令) */
{
    if (t == 0) { cmd(15500, (int)(20.0 / g_slowv * 1000)); g_motion_end = 0; }
}
static void sc_tiny(int t)             /* 2µs(0.27°) 小步，每300ms一步×10 */
{
    if (t < 3000 && t % 300 == 0) { cmd(13500 + 27.0 * (t / 300 + 1), 0); g_motion_end = t; }
}
static void sc_timed90(int t)          /* 90° 用 2s，过程中外载阶跃(由负载函数给) */
{
    if (t == 0) { cmd(22500, 2000); g_motion_end = 0; }
}


static void sc_audit_long2(int t) {
    if(t<=4000 && t%100==0){cmd(13500+1000.0*t/1000,2000);g_motion_end=t;}
}
static void sc_audit_long60(int t) {
    if(t<=4000 && t%100==0){cmd(13500+1000.0*t/1000,60000);g_motion_end=t;}
}
static void sc_audit_gap(int t) {
    if(t<=4000 && (t<1000 || t>=2000) && t%100==0){cmd(13500+1000.0*t/1000,0);g_motion_end=t;}
}

/* ---------- 负载函数 ---------- */
static double ld_step(double t, double x, double v) { (void)x; (void)v; return (t > 600 && t < 1300) ? 3000 : 0; }
static double ld_stepneg(double t, double x, double v) { (void)x; (void)v; return (t > 600 && t < 1300) ? -3000 : 0; }
static double ld_push(double t, double x, double v) { (void)x; (void)v; return (t > 1500 && t < 1800) ? 2000 : 0; }
static void sc_hold(int t) { if (t == 0) { cmd(16200, 0); g_motion_end = 0; } }
static double ld_rand_state;
static double ld_random(double t, double x, double v)
{   /* 约2Hz的随机载荷，±1500mV 量级 */
    (void)x; (void)v;
    double a = 1500 * sin(6.283185307 * 1.7 * t / 1000.0) * sin(6.283185307 * 0.37 * t / 1000.0 + 1)
             + 800 * sin(6.283185307 * 3.1 * t / 1000.0 + 2);
    (void)ld_rand_state;
    return a;
}

typedef struct { const char *name; Plant pl; double (*lf)(double, double, double); } LoadCase;
typedef struct { const char *name; Script s; int dur; int pwm; int ws, we, kind; } CmdCase;

static Plant base_plant(void)
{
    Plant p; memset(&p, 0, sizeof(p));
    p.ka_s = p.kv_s = p.fc_s = p.fs_s = p.delay_s = 1;
    p.noise = MOTOR_NOISE_CDEG / 3.0; p.vbus = MOTOR_CAL_MV; p.rw = audit_rw; p.up = 13500;
    p.res_hz = 0;
    return p;
}

static void print_res(const char *v, const char *ld, const char *cm, const Res *r)
{
    printf("%s,%s,%s,track=%.2f,stops=%d,rev=%d,sat=%.2f,freeze=%.2f,maxdu=%.0f,settle=%d,final=%.2f,os=%.2f,ring=%d,"
           "vmean=%.0f,vmin_r=%.2f,ripple=%.2f,refvmin_r=%.2f,lag=%.2f,stall=%d,fault=%d,arr=%d\n",
           v, ld, cm, r->max_track / 100, r->stops, r->reverses, r->sat_frac, r->freeze_frac, r->max_du, r->settle_ms,
           r->final_err / 100, r->overshoot / 100, r->peaks, r->vmean, r->vmin_ratio, r->ripple, r->ref_vmin_ratio, r->lag / 100,
           r->stall_trips, r->faults, r->arr_ms);
}

int main(int argc, char **argv)
{
    const char *variant = argc > 1 ? argv[1] : "?";
    const char *mode = argc > 2 ? argv[2] : "matrix";
    if (argc > 3) { trace = fopen(argv[3], "w"); fprintf(trace, "case,t,x,v,target,ref,refv,vf,obsv,u,ui,state,i,lim,torque,enc,fault,stalled,paused,duty\n"); }

    for(int ai=6;ai<argc;ai++) if(!strncmp(argv[ai],"RW=",3)) audit_rw=atof(argv[ai]+3);
    LoadCase loads[48]; int nl = 0;
    Plant p;
    p = base_plant(); loads[nl++] = (LoadCase){"noload", p, 0};
    p = base_plant(); p.r = 4; loads[nl++] = (LoadCase){"inertia4", p, 0};
    p = base_plant(); p.r = 2; loads[nl++] = (LoadCase){"inertia2", p, 0};
    p = base_plant(); p.r = 8; loads[nl++] = (LoadCase){"inertia8", p, 0};
    p = base_plant(); p.r = 8; loads[nl++] = (LoadCase){"push_r8", p, ld_push};
    p = base_plant(); p.ka_s = 0.7; loads[nl++] = (LoadCase){"mis_ka0.7", p, 0};
    p = base_plant(); p.ka_s = 1.5; loads[nl++] = (LoadCase){"mis_ka1.5", p, 0};
    p = base_plant(); p.kv_s = 0.85; loads[nl++] = (LoadCase){"mis_kv0.85", p, 0};
    p = base_plant(); p.kv_s = 1.15; loads[nl++] = (LoadCase){"mis_kv1.15", p, 0};
    p = base_plant(); p.fc_s = 0.5; p.fs_s = 0.5; loads[nl++] = (LoadCase){"mis_fric0.5", p, 0};
    p = base_plant(); p.fc_s = 1.6; p.fs_s = 1.6; loads[nl++] = (LoadCase){"mis_fric1.6", p, 0};
    p = base_plant(); p.fs_s = 2.0; loads[nl++] = (LoadCase){"mis_fs2.0", p, 0};
    p = base_plant(); p.delay_s = 0.6; loads[nl++] = (LoadCase){"mis_delay0.6", p, 0};
    p = base_plant(); p.delay_s = 1.5; loads[nl++] = (LoadCase){"mis_delay1.5", p, 0};
    p = base_plant(); p.noise *= 2.5; loads[nl++] = (LoadCase){"mis_noise2.5x", p, 0};
    p = base_plant(); p.rip_a = 12; p.rip_p = 400; loads[nl++] = (LoadCase){"pot_ripple", p, 0};
    p = base_plant(); p.r = 4; p.grav = 0.2 * MOTOR_CAL_MV; loads[nl++] = (LoadCase){"grav20%_r4", p, 0};
    p = base_plant(); p.r = 6; p.grav = 0.4 * MOTOR_CAL_MV; loads[nl++] = (LoadCase){"grav40%_r6", p, 0};
    p = base_plant(); p.r = 8; p.grav = 0.6 * MOTOR_CAL_MV; loads[nl++] = (LoadCase){"grav60%_r8", p, 0};
    p = base_plant(); p.r = 4; loads[nl++] = (LoadCase){"stepload+3V", p, ld_step};
    p = base_plant(); p.r = 4; loads[nl++] = (LoadCase){"stepload-3V", p, ld_stepneg};
    p = base_plant(); p.r = 4; loads[nl++] = (LoadCase){"randload", p, ld_random};
    p = base_plant(); p.r = 4; p.grav = 0.3 * MOTOR_CAL_MV; p.bl = 30; p.res_hz = 40; loads[nl++] = (LoadCase){"grav30%_r4_bl0.6", p, 0};
    p = base_plant(); p.r = 4; p.grav = 0.3 * MOTOR_CAL_MV; p.vbus = 6400; loads[nl++] = (LoadCase){"grav30%_r4_6.4V", p, 0};
    p = base_plant(); loads[nl++] = (LoadCase){"push_r0", p, ld_push};
    p = base_plant(); p.r = 4; loads[nl++] = (LoadCase){"push_r4", p, ld_push};
    p = base_plant(); p.r = 4; p.bl = 30; p.res_hz = 40; loads[nl++] = (LoadCase){"push_r4_bl", p, ld_push};

    CmdCase cmds[] = {
        {"audit_long2",sc_audit_long2,8000,0},
        {"audit_long60",sc_audit_long60,66000,0},
        {"audit_gap",sc_audit_gap,6500,0},
        {"bigstep", sc_bigstep, 5000, 0}, {"reverse400", sc_reverse, 5500, 0}, {"reverse_arrive", sc_reverse_arrive, 7500, 0},
        {"stream_pwm", sc_stream_pwm, 5500, 1, 1000, 3900, 1}, {"stream_bus50_T0", sc_stream_bus50, 5500, 0, 1000, 3900, 1},
        {"stream_bus100_T100", sc_stream_bus100T, 5500, 0, 1000, 3900, 1}, {"stream_bus200_T200", sc_stream_bus200T, 5500, 0, 1000, 3900, 1},
        {"sine_T20", sc_sine, 7000, 0, 1000, 6000, 2}, {"slow_2dps", sc_slow, 7000, 0, 1000, 4500, 1}, {"tiny_steps", sc_tiny, 4500, 0},
        {"timed90", sc_timed90, 4000, 0, 400, 1600, 1},
        {"hold", sc_hold, 4000, 0},
        {"slowv", sc_slowv, 12000, 0, 0, 0, 3},
    };
    int nc = sizeof(cmds) / sizeof(cmds[0]);

    printf("CAL,%s,kv=%d,ka=%d,fc=%d,fs=%d,delay=%u,tau=%.1fms,vel_noise=%d,Kv_q16=%d,kp=%d,ki_q20=%d,kh_q16=%d,wv=%u\n", variant,
           cal.kv_uv, cal.ka_nv, cal.fc_mv, cal.fs_mv, cal.delay_us, cal.ka_nv * 1.0 / cal.kv_uv, CTRL_VEL_NOISE_CDPS,
           0, 0, 0, 0, (unsigned)TUNE_VEL_BW_RADS);
    const char *fl = argc > 4 ? argv[4] : 0, *fc = argc > 5 ? argv[5] : 0;
    for (int ai = 6; ai < argc; ai++)
    {
        char k[32]; double val;
        if (sscanf(argv[ai], "%31[^=]=%lf", k, &val) != 2) continue;
        if (!strcmp(k, "HOLDKI")) g_holdki = (int)val;
        if (!strcmp(k, "HOLDKP")) g_holdkp = (int)val;
        if (!strcmp(k, "TMIN")) g_tmin = (int)val;
        if (!strcmp(k, "SLOWV")) g_slowv = val;
#ifdef CONS
#define CK(name) if (!strcmp(k, #name)) name = (int)val;
        CK(CX_J_X10) CK(CX_DOB_BW) CK(CX_DOB_PCT) CK(CX_EMF_PCT) CK(CX_CORR) CK(CX_STILL_MS) CK(CX_HOLD_ZETA)
        CK(CX_STREAM) CK(CX_STREAM_NOTM) CK(CX_ADEF_J) CK(CX_J_FF) CK(CX_J_OBS) CK(CX_J_GAIN) CK(CX_DOB_GATE) CK(CX_DOB_GATE_MV)
#endif
#ifdef PCX
        if (!strcmp(k, "DITHER")) PC_DITHER_PCT = (int)val;
        if (!strcmp(k, "DHALF")) PC_DITHER_HALF = (int)val;
#endif
    }
#ifdef PROTO
    for (int ai = 6; ai < argc; ai++)
    {
        char k[32]; double val;
        if (sscanf(argv[ai], "%31[^=]=%lf", k, &val) != 2) continue;
#define KN(name) if (!strcmp(k, #name)) { if (sizeof(name) == sizeof(double)) *(double *)&name = val; else *(int *)&name = (int)val; }
        KN(PX_J) KN(PX_DOB) KN(PX_DOB_W) KN(PX_CORR) KN(PX_FRIC) KN(PX_POSINT) KN(PX_HOLDD) KN(PX_V0)
        if (!strcmp(k, "JTRUE")) px_jtrue = (int)val;
        KN(PX_STREAM) KN(PX_CAP) KN(PX_ICLAMP) KN(PX_JID) KN(PX_IMAX) KN(PX_R) KN(PX_HOLD_ZETA)
    }
    px_J_init = PX_J;
#endif
    if (!strcmp(mode, "matrix"))
    {
        for (int li = 0; li < nl; li++)
            for (int ci = 0; ci < nc; ci++)
            {
                if (fl && strcmp(fl, "*") && strcmp(fl, loads[li].name)) continue;
                if (fc && strcmp(fc, "*") && strcmp(fc, cmds[ci].name)) continue;
                char nm[96]; Res r;
                load_fn = loads[li].lf;
                sim_reset(&loads[li].pl, cmds[ci].pwm, cmds[ci].pwm ? 1500 : 0);
                snprintf(nm, sizeof(nm), "%s|%s", loads[li].name, cmds[ci].name); trace_name = nm;
                g_motion_end = 0; g_final_target = 13500; g_ws = cmds[ci].ws; g_we = cmds[ci].we; g_kind = cmds[ci].kind;
                r = run(nm, cmds[ci].s, !strcmp(cmds[ci].name,"slowv") ? (int)ceil(20000/g_slowv)+2000 : cmds[ci].dur);
                print_res(variant, loads[li].name, cmds[ci].name, &r);
            }
    }
    if (trace) fclose(trace);
    return 0;
}
