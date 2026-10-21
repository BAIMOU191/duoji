#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "C_Traj_Planner.c"
/* 生产规划器 + 30kg 默认限值(V145 A/D5250 平滑50ms Tm150)，各种连续指令 */
static void run(const char *name, double amp, double hz, int iv, int T, int tm, double A, double sm)
{
    TrajRef_t r; double maxlag = 0, over = 0; int pre_hits = 0, steps = 0;
    C_Traj_Init(13500); C_Traj_SetRange(0, 27000);
    C_Traj_SetLimits(14500, (int32_t)A, (int32_t)A, (int32_t)(A / sm));
    C_Traj_SetMoveMinMs(tm);
    for (int t = 0; t < 8000; t++) {
        double c = 13500 + amp * sin(6.283185307 * hz * t / 1000.0);
        if (t % iv == 0 && t < 6000) C_Traj_Plan((int32_t)lround(c), (uint16_t)T);
        C_Traj_Step(&r, 0);
        if (t > 1000 && t < 6000) {
            double e = fabs(r.pos - c); if (e > maxlag) maxlag = e;
            if (fabs(r.pos - 13500) - amp > over) over = fabs(r.pos - 13500) - amp;
            steps++; if (s_tj.pre_t < s_tj.pre_T) pre_hits++;
        }
    }
    printf("%-28s iv=%3d T=%3d Tm=%3d A=%5.0f sm=%3.0fms | max|ref-cmd| %6.2f deg  overshoot %6.2f deg  takeover %3.0f%% of time\n",
           name, iv, T, tm, A / 100, sm * 1000, maxlag / 100, over / 100, 100.0 * pre_hits / steps);
}
int main(void)
{
    run("sine 30deg 0.5Hz", 3000, 0.5, 20, 20, 150, 525000, 0.05);
    run("sine 30deg 1Hz",   3000, 1.0, 20, 20, 150, 525000, 0.05);
    run("sine 30deg 2Hz",   3000, 2.0, 20, 20, 150, 525000, 0.05);
    run("sine 30deg 2Hz",   3000, 2.0, 20, 20,   0, 525000, 0.05);
    run("sine 15deg 3Hz",   1500, 3.0, 20, 20, 150, 525000, 0.05);
    run("sine 30deg 1Hz iv10", 3000, 1.0, 10, 10, 150, 525000, 0.05);
    run("sine 30deg 1Hz sm100", 3000, 1.0, 20, 20, 150, 525000, 0.10);
    run("sine 30deg 1Hz A2000", 3000, 1.0, 20, 20, 150, 200000, 0.05);
    run("sine 30deg 0.5Hz A1000", 3000, 0.5, 20, 20, 150, 100000, 0.05);
    run("sine 30deg 0.5Hz A1000 iv50", 3000, 0.5, 50, 50, 150, 100000, 0.05);
    run("sine 30deg 0.5Hz A1000 iv100", 3000, 0.5, 100, 100, 150, 100000, 0.05);
    return 0;
}
