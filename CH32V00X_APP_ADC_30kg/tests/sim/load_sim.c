/*
 * load_sim.c —— 带载工况仿真(30kg 工程专用，不属于四工程共用的 servo_sim.c)
 *
 * 链接生产代码本体(经 servo_sim.c 引入 A_Servo.c 及其桩)，只把被控对象换成更接近
 * 带载实机的模型，用来复现"带负载时抖动"并比较控制链路改动：
 *   - 分方向一阶电机(标定 T1/T2 的斜率、截距、时间常数)，负载惯量按倍率放大时间常数
 *   - Stribeck 摩擦：静止时按出力方向取起转值，滑动后按速度从起转值衰减到动摩擦
 *   - 重力负载：恒定或摆臂 cos 分布(行程中点水平、力矩最大，两端过竖直时力矩换号)
 *   - 可选齿轮柔性 + 齿隙(两质量模型，电位器装在输出轴=非同位测量)
 *   - 可选制动象限增益(标定 T4 的 MODEL_BRAKE_GAIN_Q8 含义：制动比线性模型更猛)
 *   - ADC 按一个控制拍内的平均位置采样，再叠加均匀噪声
 * 仍是简化模型，结论只用于同条件下比较方案，不等于实机保证。
 *
 * 用法: load_sim <场景> [key=value ...]，逐拍 CSV 由 trace=1 打开，汇总指标一行 JSON 打到 stderr。
 */
#define main servo_sim_previous_main
#include "servo_sim.c"
#undef main

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define LS_SUB 40                 /* 每个 1ms 控制拍内的对象积分子步数 */

typedef struct {
    double kv_pos, kv_neg;        /* 稳态速度增益，厘度/秒 每PWM = 65536/slope */
    double fc_pos, fc_neg;        /* 动摩擦(速度直线截距)，按运动方向，PWM */
    double fs_pos, fs_neg;        /* 起转值，按出力方向，PWM */
    double vs;                    /* Stribeck 速度，厘度/秒，0=无(滑动即动摩擦) */
    double tau_pos, tau_neg;      /* 空载机械时间常数，按运动方向，ms */
    double inertia;               /* 外加负载惯量 / 空载总惯量 */
    double f_res;                 /* 空载第一谐振 Hz，0=刚性 */
    double jl0;                   /* 空载时输出侧惯量占比(柔性模型用) */
    double zeta;                  /* 柔性阻尼比 */
    double backlash;              /* 齿隙总量，厘度 */
    double grav;                  /* 重力力矩幅值，PWM 当量，正=把轴往角度减小方向拉 */
    int    grav_mode;             /* 0=恒定，1=摆臂 cos((θ-中点)) */
    double brake_boost;           /* 制动象限(出力与速度反号)的出力倍率，1=线性模型 */
    double noise;                 /* 读数均匀噪声 ±，厘度 */
    int    delay;                 /* 纯延迟，控制拍 */
    double gain;                  /* 整体出力倍率(供电波动) */
} LsPlant;

static LsPlant P = {
    65536.0 / SIM_PLANT_SLOPE_POS_Q16, 65536.0 / SIM_PLANT_SLOPE_NEG_Q16,
    SIM_PLANT_FRIC_POS_PWM, SIM_PLANT_FRIC_NEG_PWM,
    SIM_PLANT_BRK_POS_PWM, SIM_PLANT_BRK_NEG_PWM,
    0.0,
    SIM_PLANT_TAU_POS_MS, SIM_PLANT_TAU_NEG_MS,
    0.0, 0.0, 0.05, 0.1, 0.0,
    0.0, 1, 1.0,
    SIM_NOISE_CDEG, SIM_PLANT_DELAY_TICKS, 1.0,
};

/* 对象状态：m=电机侧(折算到输出轴)，l=输出轴；刚性时 l 跟随 m */
static double m_pos, m_vel, l_pos, l_vel;
static int    m_stuck = 1;
static int    u_hist[16], u_idx;

static double sgn(double x) { return (x > 0) - (x < 0); }

static double ls_load(double theta)
{
    if (P.grav_mode == 1) return P.grav * cos((theta - SIM_MID) * M_PI / 18000.0);
    return P.grav;
}

/* 摩擦：返回动摩擦幅值(随速度从 fs 衰减到 fc) */
static double ls_fric(double v)
{
    double fc = (v >= 0) ? P.fc_pos : P.fc_neg;
    double fs = (v >= 0) ? P.fs_pos : P.fs_neg;
    if (P.vs <= 0 || fs <= fc) return fc;
    return fc + (fs - fc) * exp(-(v / P.vs) * (v / P.vs));
}

/* 一个子步：u=本拍作用在H桥上的PWM，dt 秒 */
static void ls_substep(double u, double dt)
{
    double kv, tau_ms, jtot, jm, jl, k = 0, c = 0, ts = 0, drive, dir;
    int rigid = P.f_res <= 0;

    dir = (fabs(m_vel) > 1e-9) ? sgn(m_vel) : sgn(u);
    kv  = (dir >= 0) ? P.kv_pos : P.kv_neg;
    tau_ms = (dir >= 0) ? P.tau_pos : P.tau_neg;
    jtot = tau_ms / 1000.0 / kv;                  /* 空载总惯量，PWM·s²/厘度 */

    u *= P.gain;
    if (P.brake_boost != 1.0 && u * m_vel < 0) u *= P.brake_boost;

    if (rigid)
    {
        jm = jtot * (1.0 + P.inertia);
        drive = u - m_vel / kv - ls_load(m_pos);
    }
    else
    {
        double jm0 = jtot * (1.0 - P.jl0), jl0 = jtot * P.jl0, w = 2 * M_PI * P.f_res;
        double d = m_pos - l_pos, half = P.backlash / 2, dz;
        jm = jm0;
        jl = jl0 + jtot * P.inertia;
        k  = w * w / (1.0 / jm0 + 1.0 / jl0);
        c  = 2 * P.zeta * sqrt(k * jm0 * jl0 / (jm0 + jl0));
        dz = (d > half) ? d - half : (d < -half) ? d + half : 0;
        ts = (dz != 0) ? k * dz + c * (m_vel - l_vel) : 0;
        drive = u - m_vel / kv - ts;
        /* 输出轴 */
        {
            double a = (ts - ls_load(l_pos)) / jl;
            l_vel += a * dt;
            l_pos += l_vel * dt;
        }
    }

    /* 电机侧带静摩擦的积分 */
    if (m_stuck)
    {
        double fs = (drive >= 0) ? P.fs_pos : P.fs_neg;
        if (fabs(drive) <= fs) m_vel = 0;
        else
        {
            m_stuck = 0;
            m_vel = (drive - sgn(drive) * fs) / jm * dt;
        }
    }
    else
    {
        double s = sgn(m_vel), nv;
        nv = m_vel + (drive - s * ls_fric(m_vel)) / jm * dt;
        if (nv * m_vel <= 0) { m_vel = 0; m_stuck = 1; }
        else m_vel = nv;
    }
    m_pos += m_vel * dt;
    if (rigid) { l_pos = m_pos; l_vel = m_vel; }
}

static void ls_reset(double pos)
{
    m_pos = l_pos = pos; m_vel = l_vel = 0; m_stuck = 1;
    memset(u_hist, 0, sizeof(u_hist)); u_idx = 0;
    /* 带重力时先让对象在该位置自然落定(齿隙贴向受力侧)，避免第一拍的假冲击 */
    if (P.f_res > 0 && P.grav != 0)
    {
        double jtot = P.tau_pos / 1000.0 / P.kv_pos;
        double jm0 = jtot * (1.0 - P.jl0), jl0 = jtot * P.jl0, w = 2 * M_PI * P.f_res;
        double k = w * w / (1.0 / jm0 + 1.0 / jl0);
        double g = ls_load(pos);
        l_pos = pos; m_pos = pos + g / k + sgn(g) * P.backlash / 2;
    }
}

/* 一个控制拍：ADC 取上一拍平均位置 -> 控制 -> 延迟后的PWM作用一整拍 */
static double avg_pos = SIM_MID;
static void ls_tick(void)
{
    int k, u;
    double sum = 0;

    plant_pos = avg_pos;             /* servo_sim 的 ADC 桩读 plant_pos + 噪声 */
    noise = P.noise;
    A_Servo_Control();
    u_hist[u_idx] = applied * SIM_PLANT_MOTOR_SIGN;
    u = u_hist[(u_idx + 16 - P.delay) % 16]; u_idx = (u_idx + 1) % 16;
    for (k = 0; k < LS_SUB; k++)
    {
        ls_substep(u, 0.001 / LS_SUB);
        sum += l_pos;
    }
    avg_pos = sum / LS_SUB;
    plant_vel = l_vel;
    tick++;
}

/* ---------------------------- 场景与指标 ---------------------------- */

#define LS_MAX_EV 256
typedef struct { int t; int pwm; int T; int pwm_in; } LsEvent;     /* pwm_in=1: 走PWM输入 */
typedef struct { int t0, t1; double target; int move_t; int dir; double start; } LsWin;

static LsEvent ev[LS_MAX_EV]; static int n_ev;
static LsWin   win[LS_MAX_EV]; static int n_win;
static int     total_ms;
static int     pwm_mode;           /* 1=整个场景走 PWM 输入(50Hz) */
static int     pwm_jit;            /* PWM 输入线上抖动 ±us */

static void add_ev(int t, int pwm, int T) { ev[n_ev].t = t; ev[n_ev].pwm = pwm; ev[n_ev].T = T; n_ev++; }

static double ang_of(int pwm) { return target_of(pwm); }

/* 场景：返回 0=未知 */
static int build_scenario(const char *s)
{
    int t = 300, i;
    n_ev = 0; total_ms = 0;

    if (strcmp(s, "small") == 0)            /* 小角度：1/2/5/10度 正反来回，每级停1s */
    {
        static const int d[] = {7, 15, 37, 74};
        for (i = 0; i < 4; i++)
        {
            add_ev(t, 1500 + d[i], 0); t += 1000;
            add_ev(t, 1500, 0);        t += 1000;
            add_ev(t, 1500 - d[i], 0); t += 1000;
            add_ev(t, 1500, 0);        t += 1000;
        }
    }
    else if (strcmp(s, "large") == 0)       /* 大角度：90/180/270度，最快 */
    {
        add_ev(t, 2167, 0); t += 2000; add_ev(t, 1500, 0); t += 2000;
        add_ev(t,  833, 0); t += 2000; add_ev(t, 2167, 0); t += 2500;
        add_ev(t,  500, 0); t += 3000; add_ev(t, 2500, 0); t += 3000;
        add_ev(t, 1500, 0); t += 2000;
    }
    else if (strcmp(s, "large_t") == 0)     /* 大角度带时间：T1000/T2000 */
    {
        add_ev(t, 2167, 1000); t += 2000; add_ev(t, 833, 2000); t += 3000;
        add_ev(t, 2500, 2000); t += 3000; add_ev(t, 500, 2000); t += 3000;
        add_ev(t, 1500, 1000); t += 2000;
    }
    else if (strcmp(s, "recip") == 0)       /* 来回转：不等到位就换向，三档幅值/周期 */
    {
        for (i = 0; i < 8; i++) { add_ev(t, (i & 1) ? 1400 : 1600, 0); t += 250; }
        add_ev(t, 1500, 0); t += 1200;
        for (i = 0; i < 6; i++) { add_ev(t, (i & 1) ? 1200 : 1800, 0); t += 500; }
        add_ev(t, 1500, 0); t += 1500;
        for (i = 0; i < 4; i++) { add_ev(t, (i & 1) ? 700 : 2300, 1000); t += 1500; }
        add_ev(t, 1500, 0); t += 2000;
    }
    else if (strcmp(s, "recip_hold") == 0)  /* 来回转且每次到位停稳：看每次到位后的保持 */
    {
        for (i = 0; i < 6; i++) { add_ev(t, (i & 1) ? 1300 : 1700, 0); t += 1200; }
        for (i = 0; i < 4; i++) { add_ev(t, (i & 1) ? 1480 : 1520, 0); t += 800; }
        add_ev(t, 1500, 0); t += 1200;
    }
    else if (strcmp(s, "pwm") == 0)         /* PWM 输入 50Hz，小步 + 大步 */
    {
        pwm_mode = 1;
        add_ev(t, 1500, 0); t += 1000; add_ev(t, 1510, 0); t += 1000;
        add_ev(t, 1540, 0); t += 1000; add_ev(t, 1500, 0); t += 1000;
        add_ev(t, 2000, 0); t += 2000; add_ev(t, 1000, 0); t += 2000;
        add_ev(t, 1500, 0); t += 1500;
    }
    else return 0;
    total_ms = t;
    return 1;
}

/* 运行并统计。每条指令之后到下一条之前是一个窗口：运动指标看全窗，保持指标看窗尾一半 */
typedef struct {
    double hold_travel;     /* 保持段真实轴位移总和 / 保持总时长，厘度/秒(机械抖动) */
    double hold_rev;        /* 保持段位置来回摆次数/秒(回差10厘度) */
    double hold_pp;         /* 保持段位置峰峰最大值，厘度 */
    double hold_err;        /* 保持段 |平均误差| 最大值，厘度 */
    double hold_dpwm;       /* 保持段 |ΔPWM| 平均，PWM/拍(电气嗡嗡) */
    double hold_pwm_sw;     /* 保持段 PWM 符号翻转次数/秒 */
    double hold_on;         /* 保持段 PWM 非零占比 */
    double over_max;        /* 最大超调，厘度 */
    double settle_max;      /* 最慢到位时间(进入±IN_WIN 且此后不出)，ms */
    double move_rev;        /* 运动+到位段速度换向次数(不含指令本身换向)，次/窗口 */
    double move_dpwm;       /* 全程 |ΔPWM| 平均 */
    int    unsettled;       /* 窗口结束仍未到位的次数 */
} LsMetrics;

static int trace;

static void run(LsMetrics *M)
{
    int k, e = 0, w = -1, prev_pwm = 0, prev_sign = 0;
    double hold_time = 0, move_d = 0;
    int    move_n = 0;
    static double pos_log[40000], vel_log[40000]; static int pwm_log[40000]; static unsigned char done_log[40000];
    int t_in_win_start[LS_MAX_EV];

    memset(M, 0, sizeof(*M));
    n_win = 0;
    if (trace) puts("ms,cmd,pos,vel,m_pos,ref_pos,ref_vel,obs_pos,obs_vel,obs_load,pwm,done,state,e_pos,u_i,u_acc,u_fric,u_vel,u_fb,load");

    for (k = 0; k < total_ms && k < 40000; k++)
    {
        while (e < n_ev && ev[e].t == k)
        {
            if (!pwm_mode) A_Servo_Submit((uint16_t)ev[e].pwm, (uint16_t)ev[e].T);
            win[n_win].t0 = k; win[n_win].target = ang_of(ev[e].pwm);
            win[n_win].move_t = ev[e].T; win[n_win].start = l_pos;
            win[n_win].dir = (win[n_win].target > l_pos) ? 1 : -1;
            if (n_win > 0) win[n_win - 1].t1 = k;
            t_in_win_start[n_win] = k;
            n_win++; w = n_win - 1; e++;
        }
        if (pwm_mode && k % 20 == 0 && w >= 0)
        {
            int j = pwm_jit ? (int)(random_unit() * (2 * pwm_jit + 1)) - pwm_jit : 0;
            input = (uint16_t)(ev[e - 1].pwm + j);
        }
        ls_tick();
        pos_log[k] = l_pos; vel_log[k] = l_vel; pwm_log[k] = applied; done_log[k] = C_Traj_Is_Done();
        if (trace)
            printf("%d,%d,%.2f,%.1f,%.2f,%ld,%ld,%ld,%ld,%ld,%d,%u,%u,%ld,%d,%d,%d,%d,%d,%.0f\n",
                   k, (w >= 0) ? ev[e - 1].pwm : 0, l_pos, l_vel, m_pos,
                   (long)s_servo.ref.pos, (long)s_servo.ref.vel, (long)s_servo.obs.pos,
                   (long)s_servo.obs.vel, (long)s_servo.obs.load, applied, C_Traj_Is_Done(),
                   s_servo.dbg.state, (long)s_servo.dbg.e_pos, s_servo.dbg.u_i, s_servo.dbg.u_acc,
                   s_servo.dbg.u_fric, s_servo.dbg.u_vel, s_servo.dbg.u_fb, ls_load(l_pos));
        if (k > 0) { move_d += abs(applied - prev_pwm); move_n++; }
        prev_pwm = applied;
    }
    if (n_win > 0) win[n_win - 1].t1 = k;
    (void)t_in_win_start; (void)prev_sign;

    for (w = 0; w < n_win; w++)
    {
        int t0 = win[w].t0, t1 = win[w].t1, h0, td, i, settle = -1, revs = 0, pv = 0;
        double tgt = win[w].target, lo = 1e9, hi = -1e9, sum = 0, over = 0, trav = 0, dp = 0;
        int sw = 0, on = 0, ps = 0, hv = 0;
        double ext = 0, elo = 0;
        double inwin = CTRL_IN_WIN_CDEG + 10;
        int is_hold;

        /* 保持段 = 轨迹走完 300ms 之后到下一条指令；不够 200ms 的(来回转途中换向)不算保持 */
        /* PWM 输入要等下一帧且连续两帧才生效，从 60ms 后开始找"走完" */
        for (td = t0 + (pwm_mode ? 60 : 1); td < t1 && !done_log[td]; td++) {}
        h0 = td + 300;
        is_hold = (t1 - h0) >= 200;

        for (i = t0; i < t1; i++)
        {
            double ov = win[w].dir * (pos_log[i] - tgt);
            if (ov > over) over = ov;
            if (fabs(pos_log[i] - tgt) > inwin) settle = -1;
            else if (settle < 0) settle = i - t0;
        }
        if (is_hold)
        {
            if (over > M->over_max) M->over_max = over;
            if (settle < 0) M->unsettled++;
            else if (settle > M->settle_max) M->settle_max = settle;
        }
        /* 运动段换向：第一次越过目标之后的速度换向 */
        for (i = t0; i < t1; i++)
        {
            int s = (vel_log[i] > 30) ? 1 : (vel_log[i] < -30) ? -1 : 0;
            if (!s) continue;
            if (pv && s != pv) revs++;
            pv = s;
        }
        if (is_hold) M->move_rev += revs;
        if (!is_hold) continue;
        for (i = h0; i < t1; i++)
        {
            int s;
            if (pos_log[i] < lo) lo = pos_log[i];
            if (pos_log[i] > hi) hi = pos_log[i];
            sum += pos_log[i] - tgt;
            if (i > h0)
            {
                trav += fabs(pos_log[i] - pos_log[i - 1]);
                dp   += abs(pwm_log[i] - pwm_log[i - 1]);
            }
            s = (pwm_log[i] > 0) - (pwm_log[i] < 0);
            if (s) { if (ps && s != ps) sw++; ps = s; on++; }
            /* 位置换向带10厘度回差：只数肉眼可见的来回摆，不数齿轮弹簧上的微振 */
            if (i == h0) { ext = elo = pos_log[i]; hv = 0; }
            if (hv == 0)
            {
                if (pos_log[i] > ext) ext = pos_log[i];
                if (pos_log[i] < elo) elo = pos_log[i];
                if (pos_log[i] < ext - 10) { hv = -1; ext = pos_log[i]; }
                else if (pos_log[i] > elo + 10) { hv = 1; ext = pos_log[i]; }
            }
            else if (hv > 0) { if (pos_log[i] > ext) ext = pos_log[i]; else if (pos_log[i] < ext - 10) { M->hold_rev += 1; hv = -1; ext = pos_log[i]; } }
            else             { if (pos_log[i] < ext) ext = pos_log[i]; else if (pos_log[i] > ext + 10) { M->hold_rev += 1; hv = 1; ext = pos_log[i]; } }
        }
        hold_time += (t1 - h0) / 1000.0;
        M->hold_travel += trav;
        M->hold_dpwm   += dp;
        M->hold_pwm_sw += sw;
        M->hold_on     += on;
        if (hi - lo > M->hold_pp) M->hold_pp = hi - lo;
        if (fabs(sum / (t1 - h0)) > M->hold_err) M->hold_err = fabs(sum / (t1 - h0));
    }
    if (hold_time > 0)
    {
        M->hold_travel /= hold_time;
        M->hold_rev    /= hold_time;
        M->hold_pwm_sw /= hold_time;
        M->hold_dpwm   /= hold_time * 1000.0;
        M->hold_on     /= hold_time * 1000.0;
    }
    M->move_dpwm = move_n ? move_d / move_n : 0;
}

static void set_kv(const char *k, const char *v)
{
    double x = atof(v);
    if      (!strcmp(k, "inertia"))  P.inertia = x;
    else if (!strcmp(k, "grav"))     P.grav = x;
    else if (!strcmp(k, "gmode"))    P.grav_mode = (int)x;
    else if (!strcmp(k, "fres"))     P.f_res = x;
    else if (!strcmp(k, "zeta"))     P.zeta = x;
    else if (!strcmp(k, "jl0"))      P.jl0 = x;
    else if (!strcmp(k, "backlash")) P.backlash = x;
    else if (!strcmp(k, "vs"))       P.vs = x;
    else if (!strcmp(k, "brake"))    P.brake_boost = x;
    else if (!strcmp(k, "noise"))    P.noise = x;
    else if (!strcmp(k, "delay"))    P.delay = (int)x;
    else if (!strcmp(k, "gain"))     P.gain = x;
    else if (!strcmp(k, "fric"))     { P.fc_pos *= x; P.fc_neg *= x; P.fs_pos *= x; P.fs_neg *= x; }
    else if (!strcmp(k, "tau"))      { P.tau_pos *= x; P.tau_neg *= x; }
    else if (!strcmp(k, "pwmjit"))   pwm_jit = (int)x;
    else if (!strcmp(k, "trace"))    trace = (int)x;
    else if (!strcmp(k, "seed"))     rng = (unsigned)x;
    else { fprintf(stderr, "unknown key %s\n", k); exit(2); }
}

int main(int argc, char **argv)
{
    LsMetrics M;
    int i;

    if (argc < 2 || !build_scenario(argv[1])) { fprintf(stderr, "usage: load_sim <small|large|large_t|recip|recip_hold|pwm> [k=v]\n"); return 2; }
    for (i = 2; i < argc; i++)
    {
        char buf[64], *eq;
        strncpy(buf, argv[i], sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
        eq = strchr(buf, '=');
        if (!eq) { fprintf(stderr, "bad arg %s\n", argv[i]); return 2; }
        *eq = 0; set_kv(buf, eq + 1);
    }
    build_scenario(argv[1]);            /* pwm_mode 由场景设定 */

    setup(SIM_MID, pwm_mode);
    ls_reset(SIM_MID); avg_pos = l_pos;
    /* 先原地保持 300ms 让带载对象和积分落定，再开始场景 */
    run(&M);

    fprintf(stderr, "{\"hold_travel\":%.1f,\"hold_rev\":%.2f,\"hold_pp\":%.1f,\"hold_err\":%.1f,"
            "\"hold_dpwm\":%.2f,\"hold_pwm_sw\":%.2f,\"hold_on\":%.3f,\"over_max\":%.1f,"
            "\"settle_max\":%.0f,\"move_rev\":%.0f,\"move_dpwm\":%.2f,\"unsettled\":%d}\n",
            M.hold_travel, M.hold_rev, M.hold_pp, M.hold_err, M.hold_dpwm, M.hold_pwm_sw,
            M.hold_on, M.over_max, M.settle_max, M.move_rev, M.move_dpwm, M.unsettled);
    return 0;
}
