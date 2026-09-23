/*
 * arm_sim.c —— 机械臂肩关节(第二关节，控制整臂前后)工况仿真，30kg 工程专用
 *
 * 链接生产代码本体(经 servo_sim.c 引入 A_Servo.c 及其桩)，被控对象换成"舵机 + 整条手臂"：
 *   舵机侧：电机+齿轮+输出轴折算成一个质量，分方向一阶模型(标定 T1/T2)，Stribeck 静摩擦，
 *           电位器量的是输出轴；
 *   手臂侧：整条手臂(后面四个关节+连杆+负载)是一个质量，经"结构弹簧+间隙"挂在输出轴上
 *           (舵盘花键、支架、下游舵机的齿隙与弹性)，farm=输出轴锁死时手臂自身的摆动频率；
 *   重力：倒立摆形式，手臂竖直时为0，偏离竖直按 sin 增大；lean=手臂重心相对竖直的前倾角
 *         (肘/腕前伸时重心在前面)，grav=手臂水平时的重力力矩(PWM当量，满量程2400)。
 * 指标都按手臂(用户看到的)位置算。简化模型，只用于同条件下比较方案。
 *
 * 用法: arm_sim <场景> [key=value ...]，trace=1 输出逐拍 CSV，汇总 JSON 打到 stderr。
 */
#define main servo_sim_previous_main
#include "servo_sim.c"
#undef main

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define AS_SUB 40                  /* 每个 1ms 控制拍的积分子步 */
#define AS_MAX_MS 40000

typedef struct {
    double kv_pos, kv_neg, fc_pos, fc_neg, fs_pos, fs_neg, vs, tau_pos, tau_neg;
    double inertia;                /* 手臂惯量 / 舵机空载惯量 */
    double farm;                   /* 手臂结构摆动频率(输出轴锁死)，Hz；0=刚性连接 */
    double zarm;                   /* 结构阻尼比 */
    double gap;                    /* 结构间隙总量，厘度 */
    double grav;                   /* 手臂水平时的重力力矩，PWM当量 */
    double lean;                   /* 重心前倾角，厘度(行程中点时手臂偏离竖直的角度) */
    double noise;
    int    delay;
    double gain, brake_boost;
} ArmPlant;

static ArmPlant A = {
    65536.0 / SIM_PLANT_SLOPE_POS_Q16, 65536.0 / SIM_PLANT_SLOPE_NEG_Q16,
    SIM_PLANT_FRIC_POS_PWM, SIM_PLANT_FRIC_NEG_PWM, SIM_PLANT_BRK_POS_PWM, SIM_PLANT_BRK_NEG_PWM,
    300.0, SIM_PLANT_TAU_POS_MS, SIM_PLANT_TAU_NEG_MS,
    3.0, 8.0, 0.08, 30.0, 900.0, 3000.0,
    SIM_NOISE_CDEG, SIM_PLANT_DELAY_TICKS, 1.0, 1.0,
};

static double s_pos, s_vel, a_pos, a_vel;     /* s=舵机输出轴(电位器)，a=手臂 */
static int    s_stuck = 1;
static int    u_hist[16], u_idx;
static double grav_scale = 1.0;               /* 场景用：其它关节动作导致的重力矩变化 */
static double push_f = 0, push_amp = 800;     /* 手推外力(作用在手臂上，正=往角度减小方向推)，push 场景用 */
static int    lock_after = 0;                 /* >0: 该时刻起输出轴锁死(理想舵机)，用作手臂自身衰减的对照 */
static int    ideal_hold = 0, ideal_locked = 0; /* ideal=1: 每段轨迹走完后输出轴锁死，作"舵机无限硬"的对照 */
static int    now_ms;

static double sgn(double x) { return (x > 0) - (x < 0); }

/* 重力矩，正=把轴往角度减小方向拉。倒立摆：偏离竖直越多越往外倒 */
static double arm_load(double theta)
{
    double off = (theta - SIM_MID + A.lean) * M_PI / 18000.0;
    return -A.grav * grav_scale * sin(off) + push_f;
}

static double fric(double v)
{
    double fc = (v >= 0) ? A.fc_pos : A.fc_neg, fs = (v >= 0) ? A.fs_pos : A.fs_neg;
    if (A.vs <= 0 || fs <= fc) return fc;
    return fc + (fs - fc) * exp(-(v / A.vs) * (v / A.vs));
}

static void substep(double u, double dt)
{
    double dir = (fabs(s_vel) > 1e-9) ? sgn(s_vel) : sgn(u);
    double kv = (dir >= 0) ? A.kv_pos : A.kv_neg;
    double jtot = ((dir >= 0) ? A.tau_pos : A.tau_neg) / 1000.0 / kv;
    double js, drive;

    u *= A.gain;
    if (A.brake_boost != 1.0 && u * s_vel < 0) u *= A.brake_boost;

    if (A.farm <= 0)
    {
        js = jtot * (1.0 + A.inertia);
        drive = u - s_vel / kv - arm_load(s_pos);
    }
    else
    {
        double ja = jtot * A.inertia, w = 2 * M_PI * A.farm;
        double ks = w * w * ja, cs = 2 * A.zarm * sqrt(ks * ja);
        double d = s_pos - a_pos, h = A.gap / 2, dz, ts;
        dz = (d > h) ? d - h : (d < -h) ? d + h : 0;
        ts = (dz != 0) ? ks * dz + cs * (s_vel - a_vel) : 0;
        a_vel += (ts - arm_load(a_pos)) / ja * dt;
        a_pos += a_vel * dt;
        js = jtot;
        drive = u - s_vel / kv - ts;
    }

    if (s_stuck)
    {
        double fs = (drive >= 0) ? A.fs_pos : A.fs_neg;
        if (fabs(drive) <= fs) s_vel = 0;
        else { s_stuck = 0; s_vel = (drive - sgn(drive) * fs) / js * dt; }
    }
    else
    {
        double s = sgn(s_vel), nv = s_vel + (drive - s * fric(s_vel)) / js * dt;
        if (nv * s_vel <= 0) { s_vel = 0; s_stuck = 1; }
        else s_vel = nv;
    }
    if (lock_after > 0 && now_ms >= lock_after) s_vel = 0;
    if (ideal_hold && ideal_locked) s_vel = 0;    /* 理想舵机：轨迹走完即把输出轴锁死 */
    s_pos += s_vel * dt;
    if (A.farm <= 0) { a_pos = s_pos; a_vel = s_vel; }
}

static double avg_pos;
static void tick_once(void)
{
    int k, u;
    double sum = 0;
    plant_pos = avg_pos; noise = A.noise;
    A_Servo_Control();
    u_hist[u_idx] = applied * SIM_PLANT_MOTOR_SIGN;
    u = u_hist[(u_idx + 16 - A.delay) % 16]; u_idx = (u_idx + 1) % 16;
    for (k = 0; k < AS_SUB; k++) { substep(u, 0.001 / AS_SUB); sum += s_pos; }
    avg_pos = sum / AS_SUB;
    tick++;
}

static void plant_reset(double pos)
{
    s_pos = a_pos = pos; s_vel = a_vel = 0; s_stuck = 1;
    memset(u_hist, 0, sizeof(u_hist)); u_idx = 0; avg_pos = pos;
    if (A.farm > 0)
    {   /* 手臂按重力压在间隙一侧并压弯结构弹簧 */
        double jtot = A.tau_pos / 1000.0 / A.kv_pos, ja = jtot * A.inertia;
        double ks = 4 * M_PI * M_PI * A.farm * A.farm * ja, g = arm_load(pos);
        a_pos = pos - g / ks - sgn(g) * A.gap / 2;
    }
}

/* ------------------------------ 场景 ------------------------------ */
#define AS_MAX_EV 2048
typedef struct { int t, pwm, T; } Ev;
typedef struct { int t0, t1; int hold; } Win;       /* hold=1 静止窗，0 运动窗 */
static Ev  ev[AS_MAX_EV]; static int n_ev;
static Win wn[64];        static int n_wn;
static int total_ms, pwm_mode, push_mode;
typedef struct { int t; double scale; } Gs;          /* 重力矩倍率变化(其它关节动作) */
static Gs  gs[64]; static int n_gs;

static void E(int t, int pwm, int T) { if (n_ev < AS_MAX_EV) { ev[n_ev].t = t; ev[n_ev].pwm = pwm; ev[n_ev].T = T; n_ev++; } }
static void W(int t0, int t1, int hold) { wn[n_wn].t0 = t0; wn[n_wn].t1 = t1; wn[n_wn].hold = hold; n_wn++; }
static int  deg_pwm(double deg) { return 1500 + (int)lround(deg * 2000.0 / 270.0); }   /* 相对中位的角度 -> 脉宽 */

/* 主机插补：每 period ms 发一条，目标沿正弦走，T 为每条带的时间参数 */
static void stream(int t0, int dur, double amp_deg, int per_ms, int period, int T)
{
    int t;
    for (t = 0; t <= dur; t += period)
        E(t0 + t, deg_pwm(amp_deg * sin(2 * M_PI * t / per_ms)), T);
}

static int build(const char *s)
{
    n_ev = n_wn = n_gs = 0; pwm_mode = 0; push_mode = !strcmp(s, "push");
    if (!strcmp(s, "hold"))              /* 几个姿态各停3s：竖直附近、前倾、后仰、大前倾 */
    {
        static const double pose[] = {0, 20, -20, 45, 0};
        int i, t = 1000;
        for (i = 0; i < 5; i++) { E(t, deg_pwm(pose[i]), 1500); W(t + 2000, t + 4500, 1); t += 4500; }
        total_ms = t;
    }
    else if (!strcmp(s, "slow"))         /* 机械臂常用的慢速带时间运动，含过竖直 */
    {
        E(1000, deg_pwm(40), 2000);  W(1300, 2800, 0); W(4000, 5500, 1);
        E(5500, deg_pwm(-40), 3000); W(5800, 8200, 0); W(9500, 11000, 1);
        E(11000, deg_pwm(10), 1500); W(11300, 12300, 0); W(13500, 15000, 1);
        total_ms = 15000;
    }
    else if (!strcmp(s, "stream"))       /* 主机20ms插补，每条带T=20 */
    {
        stream(1000, 8000, 30, 4000, 20, 20); W(1500, 9000, 0);
        E(9200, deg_pwm(0), 800); W(11000, 13000, 1);
        total_ms = 13000;
    }
    else if (!strcmp(s, "stream0"))      /* 主机20ms插补，T=0 */
    {
        stream(1000, 8000, 30, 4000, 20, 0); W(1500, 9000, 0);
        E(9200, deg_pwm(0), 800); W(11000, 13000, 1);
        total_ms = 13000;
    }
    else if (!strcmp(s, "pwm"))          /* PWM 输入 50Hz，同样的正弦 */
    {
        pwm_mode = 1;
        stream(1000, 8000, 30, 4000, 20, 0); W(1500, 9000, 0);
        E(9200, deg_pwm(0), 0); W(11000, 13000, 1);
        total_ms = 13000;
    }
    else if (!strcmp(s, "push"))         /* 到位后手推手臂：300ms推到 push_amp，保持2.5s，200ms松手，再看3s */
    {
        E(1000, deg_pwm(0), 1500);
        total_ms = 9500;
    }
    else if (!strcmp(s, "disturb"))      /* 停在前倾30度，肘腕动作使重力矩在 0.6~1.4 倍间变化 */
    {
        E(1000, deg_pwm(30), 1500); W(3000, 4000, 1);
        gs[n_gs].t = 4000; gs[n_gs++].scale = 1.4; W(4600, 6000, 1);
        gs[n_gs].t = 6000; gs[n_gs++].scale = 0.6; W(6600, 8000, 1);
        gs[n_gs].t = 8000; gs[n_gs++].scale = 1.0; W(8600, 10000, 1);
        total_ms = 10000;
    }
    else return 0;
    return 1;
}

/* ------------------------------ 运行与指标 ------------------------------ */
static int trace;
static double apos[AS_MAX_MS], avel[AS_MAX_MS], spos[AS_MAX_MS]; static int pwml[AS_MAX_MS];

static void run(void)
{
    int k, e = 0, g = 0, cur = 1500;
    if (trace) puts("ms,cmd,arm,arm_vel,shaft,ref,ref_vel,obs_vel,obs_load,pwm,done,state,e_pos,u_i,u_acc,u_fric,u_vel,u_fb,load");
    for (k = 0; k < total_ms && k < AS_MAX_MS; k++)
    {
        while (e < n_ev && ev[e].t == k)
        {
            cur = ev[e].pwm;
            if (!pwm_mode) A_Servo_Submit((uint16_t)cur, (uint16_t)ev[e].T);
            e++;
        }
        while (g < n_gs && gs[g].t == k)
        {   /* 其它关节动作：重力矩倍率在 300ms 内线性变过去 */
            g++;
        }
        if (g > 0)
        {
            double from = (g >= 2) ? gs[g - 2].scale : 1.0, to = gs[g - 1].scale;
            double r = (k - gs[g - 1].t) / 300.0; if (r > 1) r = 1;
            grav_scale = from + (to - from) * r;
        }
        if (pwm_mode && k % 20 == 0) input = (uint16_t)cur;
        if (push_mode)
            push_f = (k < 4000) ? 0 : (k < 4300) ? push_amp * (k - 4000) / 300.0 : (k < 6800) ? push_amp
                   : (k < 7000) ? push_amp * (7000 - k) / 200.0 : 0;
        now_ms = k;
        if (ideal_hold)
        {
            static int since = 0;
            since = C_Traj_Is_Done() ? since + 1 : 0;
            ideal_locked = since > 50;
        }
        tick_once();
        apos[k] = a_pos; avel[k] = a_vel; spos[k] = s_pos; pwml[k] = applied;
        if (trace)
            printf("%d,%d,%.2f,%.1f,%.2f,%ld,%ld,%ld,%ld,%d,%u,%u,%ld,%d,%d,%d,%d,%d,%.0f\n",
                   k, cur, a_pos, a_vel, s_pos, (long)s_servo.ref.pos, (long)s_servo.ref.vel,
                   (long)s_servo.obs.vel, (long)s_servo.obs.load, applied, C_Traj_Is_Done(),
                   s_servo.dbg.state, (long)s_servo.dbg.e_pos, s_servo.dbg.u_i, s_servo.dbg.u_acc,
                   s_servo.dbg.u_fric, s_servo.dbg.u_vel, s_servo.dbg.u_fb, arm_load(a_pos));
    }
}

/* 50ms 滑动平均之外的手臂速度 = 抖动(>~10Hz)；保持段另看峰峰与摆动次数 */
static void metrics(void)
{
    double hold_pp = 0, hold_swing = 0, hold_jit = 0, hold_pwm_std = 0, hold_dpwm = 0, hold_t = 0;
    double move_jit = 0, move_dpwm = 0, move_t = 0, move_err = 0, end_pp = 0, growth = 0;
    int w;
    for (w = 0; w < n_wn; w++)
    {
        int t0 = wn[w].t0, t1 = wn[w].t1, i, n = t1 - t0, hv = 0;
        double lo = 1e9, hi = -1e9, sj = 0, sp = 0, sp2 = 0, sd = 0, ext = 0, elo = 0, sw = 0, se = 0;
        for (i = t0; i < t1; i++)
        {
            double ma = 0; int j, c = 0;
            for (j = i - 25; j <= i + 25; j++) if (j >= 0 && j < total_ms) { ma += avel[j]; c++; }
            ma /= c;
            sj += (avel[i] - ma) * (avel[i] - ma);
            sp += pwml[i]; sp2 += (double)pwml[i] * pwml[i];
            if (i > t0) sd += abs(pwml[i] - pwml[i - 1]);
            if (apos[i] < lo) lo = apos[i];
            if (apos[i] > hi) hi = apos[i];
            /* 摆动次数(换向次数)，回差5厘度；hv=0 时尚未定出第一个方向，分别跟踪最高/最低点 */
            if (i == t0) { ext = elo = apos[i]; }
            if (hv == 0)
            {
                if (apos[i] > ext) ext = apos[i];
                if (apos[i] < elo) elo = apos[i];
                if (apos[i] < ext - 5) { hv = -1; ext = apos[i]; }
                else if (apos[i] > elo + 5) { hv = 1; ext = apos[i]; }
            }
            else if (hv > 0) { if (apos[i] > ext) ext = apos[i]; else if (apos[i] < ext - 5) { sw++; hv = -1; ext = apos[i]; } }
            else             { if (apos[i] < ext) ext = apos[i]; else if (apos[i] > ext + 5) { sw++; hv = 1; ext = apos[i]; } }
            se += fabs(apos[i] - spos[i]);
        }
        if (wn[w].hold)
        {
            double m = sp / n, lo1 = 1e9, hi1 = -1e9, lo3 = 1e9, hi3 = -1e9;
            int third = n / 3;
            for (i = t0; i < t0 + third; i++) { if (apos[i] < lo1) lo1 = apos[i]; if (apos[i] > hi1) hi1 = apos[i]; }
            for (i = t1 - third; i < t1; i++) { if (apos[i] < lo3) lo3 = apos[i]; if (apos[i] > hi3) hi3 = apos[i]; }
            if (hi3 - lo3 > end_pp) end_pp = hi3 - lo3;
            if (hi1 - lo1 > 10 && (hi3 - lo3) / (hi1 - lo1) > growth) growth = (hi3 - lo3) / (hi1 - lo1);
            if (hi - lo > hold_pp) hold_pp = hi - lo;
            hold_swing += sw; hold_jit += sj; hold_dpwm += sd; hold_t += n;
            hold_pwm_std += (sp2 / n - m * m) * n;
        }
        else { move_jit += sj; move_dpwm += sd; move_t += n; move_err += se; }
    }
    fprintf(stderr, "{\"hold_pp\":%.1f,\"hold_swing\":%.2f,\"hold_jit\":%.1f,\"hold_pwm_std\":%.1f,\"hold_dpwm\":%.2f,"
            "\"move_jit\":%.1f,\"move_dpwm\":%.2f,\"flex\":%.1f,\"end_pp\":%.1f,\"growth\":%.2f}\n",
            hold_pp, hold_t ? hold_swing / (hold_t / 1000.0) : 0, hold_t ? sqrt(hold_jit / hold_t) : 0,
            hold_t ? sqrt(hold_pwm_std / hold_t) : 0, hold_t ? hold_dpwm / hold_t : 0,
            move_t ? sqrt(move_jit / move_t) : 0, move_t ? move_dpwm / move_t : 0,
            move_t ? move_err / move_t : 0, end_pp, growth);
}

/* 手推指标：偏移按推的方向取正，基准为推之前 500ms 的手臂平均位置 */
static void push_metrics(void)
{
    double base = 0, mx = 0, end = 0, rec = 0, fin = 0, sg = (push_amp >= 0) ? 1 : -1, sw = 0, ext = 0, elo = 0, pwm_sd = 0, pm = 0;
    double sbase = 0, smx = 0, send = 0, srec = 0;
    int i, hv = 0, n;
    for (i = 3500; i < 4000; i++) { base += apos[i]; sbase += spos[i]; }
    base /= 500; sbase /= 500;
    /* 舵机自身(输出轴)的让位：手臂结构弹性那部分舵机管不了，单独看 */
    for (i = 4000; i < 7000; i++) { double d = sg * (sbase - spos[i]); if (d > smx) smx = d; }
    for (i = 6500; i < 6800; i++) send += sg * (sbase - spos[i]);
    send /= 300;
    for (i = 7000; i < 9500; i++) { double d = -sg * (sbase - spos[i]); if (d > srec) srec = d; }
    for (i = 4000; i < 7000; i++) { double d = sg * (base - apos[i]); if (d > mx) mx = d; }
    for (i = 6500; i < 6800; i++) end += sg * (base - apos[i]);
    end /= 300;
    for (i = 7000; i < 9500; i++) { double d = -sg * (base - apos[i]); if (d > rec) rec = d; }
    for (i = 9000; i < 9500; i++) fin += fabs(base - apos[i]);
    fin /= 500;
    for (i = 4300; i < 9500; i++)          /* 推住和松手后的来回摆次数，回差5厘度 */
    {
        if (i == 4300) { ext = elo = apos[i]; }
        if (hv == 0) { if (apos[i] > ext) ext = apos[i]; if (apos[i] < elo) elo = apos[i];
                       if (apos[i] < ext - 5) { hv = -1; ext = apos[i]; } else if (apos[i] > elo + 5) { hv = 1; ext = apos[i]; } }
        else if (hv > 0) { if (apos[i] > ext) ext = apos[i]; else if (apos[i] < ext - 5) { sw++; hv = -1; ext = apos[i]; } }
        else             { if (apos[i] < ext) ext = apos[i]; else if (apos[i] > ext + 5) { sw++; hv = 1; ext = apos[i]; } }
    }
    for (n = 0, i = 4500; i < 6800; i++, n++) { pm += pwml[i]; pwm_sd += (double)pwml[i] * pwml[i]; }
    pm /= n; pwm_sd = sqrt(pwm_sd / n - pm * pm);
    fprintf(stderr, "{\"push_max\":%.1f,\"push_end\":%.1f,\"push_recoil\":%.1f,\"push_final\":%.1f,\"push_swing\":%.0f,\"push_pwm_sd\":%.1f,\"shaft_max\":%.1f,\"shaft_end\":%.1f,\"shaft_recoil\":%.1f}\n",
            mx, end, rec, fin, sw, pwm_sd, smx, send, srec);
}

static void set_kv(const char *k, const char *v)
{
    double x = atof(v);
    if      (!strcmp(k, "inertia")) A.inertia = x;
    else if (!strcmp(k, "farm"))    A.farm = x;
    else if (!strcmp(k, "zarm"))    A.zarm = x;
    else if (!strcmp(k, "gap"))     A.gap = x;
    else if (!strcmp(k, "grav"))    A.grav = x;
    else if (!strcmp(k, "lean"))    A.lean = x;
    else if (!strcmp(k, "vs"))      A.vs = x;
    else if (!strcmp(k, "noise"))   A.noise = x;
    else if (!strcmp(k, "delay"))   A.delay = (int)x;
    else if (!strcmp(k, "gain"))    A.gain = x;
    else if (!strcmp(k, "brake"))   A.brake_boost = x;
    else if (!strcmp(k, "fric"))    { A.fc_pos *= x; A.fc_neg *= x; A.fs_pos *= x; A.fs_neg *= x; }
    else if (!strcmp(k, "trace"))   trace = (int)x;
    else if (!strcmp(k, "seed"))    rng = (unsigned)x;
    else if (!strcmp(k, "lock"))    lock_after = (int)x;
    else if (!strcmp(k, "pushf"))   push_amp = x;
    else if (!strcmp(k, "ideal"))   ideal_hold = (int)x;
    else { fprintf(stderr, "unknown key %s\n", k); exit(2); }
}

int main(int argc, char **argv)
{
    int i;
    if (argc < 2 || !build(argv[1])) { fprintf(stderr, "usage: arm_sim <hold|slow|stream|stream0|pwm|disturb|push> [k=v]\n"); return 2; }
    for (i = 2; i < argc; i++)
    {
        char buf[64], *eq;
        strncpy(buf, argv[i], sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
        if (!(eq = strchr(buf, '='))) return 2;
        *eq = 0; set_kv(buf, eq + 1);
    }
    setup(SIM_MID, pwm_mode);
    plant_reset(SIM_MID);
    run();
    if (push_mode) push_metrics(); else metrics();
    return 0;
}
