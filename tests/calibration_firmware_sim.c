/* 执行真实标定状态机；电压域一阶电机 + 静摩擦 + 输入延迟 + 位置波纹/噪声。 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "../CH32V00X_APP_ADC_30kg/Application/A_Calib_Min.c"

static double sim_pos = 9000, sim_vel;
static double sim_kv = .402, sim_ka = .0098, sim_ripple, sim_brake = 1;
static int sim_voltage = 8000, sim_pwm, sim_delay = 2, sim_index;
static int sim_history[32];
static unsigned sim_rng = 12345;
static void (*sim_stop)(void);

int32_t A_Encoder_Read(void)
{
    sim_rng = 1664525U * sim_rng + 1013904223U;
    double noise = ((sim_rng >> 8) / 16777216.0 * 2 - 1) * 8;
    return (int32_t)lround(sim_pos + sim_ripple * sin(6.283185307179586 * sim_pos / 1000) + noise);
}
uint16_t A_Voltage_Read(void) { return (uint16_t)(sim_voltage / 10); }
int16_t D_Motor_Set(int16_t pwm) { sim_pwm = pwm; return pwm; }
void D_Motor_Release(uint8_t high) { (void)high; sim_pwm = 0; }
void D_IWDG_Feed(void) {}
uint8_t D_UART_Tx_Idle(void) { return 1; }
uint8_t D_UART1_Rx_Get(uint8_t *data) { (void)data; return 0; }
void D_UART_Service(void) {}
uint8_t IAP_Rx_Deal(uint8_t data) { (void)data; return 0; }
void IAP_Service(void) {}
void IAP_SetStopHandler(void (*handler)(void)) { sim_stop = handler; }
void C_Task_Init(Task_t *tasks, uint8_t count) { (void)tasks; (void)count; }

int main(int argc, char **argv)
{
    if (argc > 1) sim_voltage = atoi(argv[1]);
    if (argc > 2) sim_ripple = atof(argv[2]);
    if (argc > 3) sim_ka = atof(argv[3]);
    if (argc > 4) sim_delay = atoi(argv[4]);
    if (argc > 5) sim_brake = atof(argv[5]);
    A_Calib_Min_Init();
    unsigned tick;
    for (tick = 0; tick < 300000 && s_cal.phase != PH_DONE; tick++) {
        A_Calib_Min_Tick();
        sim_history[sim_index] = sim_pwm;
        int pwm = sim_history[(sim_index + 32 - sim_delay) % 32];
        sim_index = (sim_index + 1) % 32;
        double u = pwm * sim_voltage / 2400.0;
        for (int j = 0; j < 20; j++) {
            double drive = (u * sim_vel < 0) ? u * sim_brake : u;
            if (fabs(sim_vel) < 1e-8 && fabs(drive) <= 522) sim_vel = 0;
            else {
                double sign = fabs(sim_vel) < 1e-8 ? (drive > 0 ? 1 : -1) : (sim_vel > 0 ? 1 : -1);
                double next = sim_vel + (drive - sim_kv * sim_vel - 106 * sign) / sim_ka * .00005;
                if (next * sim_vel < 0 && fabs(drive) <= 522) next = 0;
                sim_vel = next;
            }
            sim_pos += sim_vel * .00005;
        }
    }
    if (s_cal.fail || tick == 300000) { fprintf(stderr, "FAIL state=%u code=%u\n", s_cal.phase, s_cal.fail); return 1; }
    sim_stop();
    if (!s_iap_stopped || sim_pwm != 0) return 2;
    fprintf(stderr, "PASS %.1fs, IAP stops output\n", tick / 1000.0);
    return 0;
}
