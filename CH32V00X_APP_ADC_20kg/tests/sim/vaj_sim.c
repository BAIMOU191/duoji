/*
 * vaj_sim.c —— 轨迹三参数(V/A/J)下的单次运动质量仿真，由 tests/vaj_grid.py 调用
 *
 * 复用 servo_sim.c 的被控对象和驱动桩(生产代码本体)，只换一个 main：把轨迹上限设成命令行给的
 * V/A/J，每个位移量正反各走一次(从静止出发，走完后再看1.5秒)，输出所有运动里的最坏指标一行。
 * 用法：vaj_sim V度每秒 A度每秒2 J度每秒3 [增益倍率 tau倍率 摩擦倍率] [v=逐个打印]
 * 指标：over 超调厘度 / err 终点误差厘度 / segs 运动段数(速度先降两成又回升记多一段，即"两段感") /
 *       rev 实际反向次数 / dpwm PWM单拍最大跳变 / tail 轨迹走完到进入到位窗的毫秒数 / small_ms 小位移用时
 */
#define main servo_sim_main
#include "servo_sim.c"
#undef main

typedef struct { double over, err, dpwm, tail, segs, rev, done; } VajMetric_t;

static VajMetric_t vaj_move(int from, int dist, double v, double a, double d, double j)
{
    VajMetric_t m = {0};
    int target, dir = dist > 0 ? 1 : -1, k, done_ms = -1, settle_ms = -1, prev = 0, last_pwm, rising = 1, segs = 1;
    double vf = 0, vmin = 0, vpk = 0;

    setup(from, 0);
    C_Traj_Set_Limits((int32_t)(v * 100), (int32_t)(a * 100), (int32_t)(d * 100), (int32_t)(j * 100));
    for (k = 0; k < 300; k++) step();                      /* 先在起点停稳 */
    target = target_of(pulse_for(from + dist));
    A_Servo_Submit((uint16_t)pulse_for(target), 0);
    last_pwm = applied;
    for (k = 0; k < 40000 && !(done_ms >= 0 && k > done_ms + 1500); k++) {
        step();
        if (done_ms < 0 && C_Traj_Is_Done()) done_ms = k;
        if (dir * (plant_pos - target) > m.over) m.over = dir * (plant_pos - target);
        if (fabs(applied - last_pwm) > m.dpwm) m.dpwm = fabs(applied - last_pwm);
        last_pwm = applied;
        if (fabs(plant_vel) > 300) {                       /* 忽略静止附近的噪声 */
            int s = plant_vel > 0 ? 1 : -1;
            if (prev && s != prev) m.rev++;
            prev = s;
        }
        vf += (dir * plant_vel - vf) * 0.1;                /* 约10ms滤波的行进方向速度 */
        if (vf > vpk) vpk = vf;
        if (rising) { if (vf < vpk * 0.8 && vpk > 200) { rising = 0; vmin = vf; } }
        else {
            if (vf < vmin) vmin = vf;
            if (vf > vmin + 0.25 * vpk && vf > 200) { segs++; rising = 1; vpk = vf; }
        }
        if (settle_ms < 0 && done_ms >= 0 && fabs(plant_pos - target) <= CTRL_IN_WIN_CDEG) settle_ms = k;
        if (settle_ms >= 0 && fabs(plant_pos - target) > CTRL_IN_WIN_CDEG) settle_ms = -1;
    }
    m.err  = fabs(plant_pos - target);
    m.segs = segs;
    m.done = done_ms;
    m.tail = settle_ms < 0 ? 9999 : (settle_ms > done_ms ? settle_ms - done_ms : 0);
    return m;
}

int main(int argc, char **argv)
{
    static const int dists[] = { 27, 60, 200, 1000, 4500, 18000 };
    VajMetric_t w = {0};
    double small_ms = 0, v, a, j, d = 0, g = 1, t = 1, f = 1;
    int verbose = 0;
    unsigned i;
    int s;

    if (argc < 4) { fprintf(stderr, "usage: vaj_sim V A J [gain tau fric] [v] [D]\n"); return 2; }
    v = atof(argv[1]); a = atof(argv[2]); j = atof(argv[3]);
    if (argc > 6) { g = atof(argv[4]); t = atof(argv[5]); f = atof(argv[6]); }
    verbose = (argc > 7 && argv[7][0] == 'v');
    if (argc > 8) d = atof(argv[8]);   /* 最大减速度，0=跟随加速度 */
    for (i = 0; i < sizeof dists / sizeof dists[0]; i++)
        for (s = -1; s <= 1; s += 2) {
            VajMetric_t m;
            gain = g; tau_scale = t; friction_scale = f;
            m = (s > 0) ? vaj_move(4000, dists[i], v, a, d, j) : vaj_move(4000 + dists[i], -dists[i], v, a, d, j);
            if (verbose)
                printf("  d=%6d over=%5.1f err=%5.1f segs=%.0f rev=%.0f dpwm=%4.0f done=%4.0f tail=%4.0f\n",
                       s * dists[i], m.over, m.err, m.segs, m.rev, m.dpwm, m.done, m.tail);
            if (m.over > w.over) w.over = m.over;
            if (m.err > w.err) w.err = m.err;
            if (m.dpwm > w.dpwm) w.dpwm = m.dpwm;
            if (m.tail > w.tail) w.tail = m.tail;
            if (m.segs > w.segs) w.segs = m.segs;
            if (m.rev > w.rev) w.rev = m.rev;
            if (dists[i] <= 60 && m.done > small_ms) small_ms = m.done;
        }
    printf("over=%.1f err=%.1f segs=%.0f rev=%.0f dpwm=%.0f tail=%.0f small_ms=%.0f\n",
           w.over, w.err, w.segs, w.rev, w.dpwm, w.tail, small_ms);
    return 0;
}
