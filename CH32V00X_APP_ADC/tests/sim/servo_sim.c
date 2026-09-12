/*
 * servo_sim.c —— 主机端舵机仿真
 *
 * 链接的是**生产代码本体**：A_Servo.c / A_Sensor.c / C_Traj_Planner.c /
 * C_Speed_Observer.c / C_Pos_Ctrl.c 一行不改地编译进来。只有 ADC、PWM 捕获和
 * H桥被替换成下面的桩，所以这里跑出来的行为就是板子上的行为。
 *
 * 被控对象用 A_Parameter.h 里那台机器自己的标定值建模，于是同一份仿真代码
 * 在两个工程里分别仿真两台不同的舵机：
 *     dv/dt = (Kv*(u - fc*sign(v)) - v) / tau      Kv = 65536/CAL_SPEED_SLOPE_Q16
 * 静摩擦按 CAL_BREAKAWAY_PWM 建模，并保留实测到的约30%方向不对称——控制器只按
 * 较大的那个方向补偿，所以另一方向天然被过补，这是更难的工况而不是更容易的。
 *
 * 单位：厘度、秒、PWM计数。
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "A_Servo.c"
#include "D_mt6701.h"   /* MT6701_ANGLE_ERROR */

/* 模式1的270度行程，两种编码器一致；仿真的可读范围和噪声底按传感器给。 */
#define SIM_SPAN_CDEG  27000
#if ENCODER_HAS_DEADZONE
#define SIM_TRAVEL_LO  ((int)ENCODER_ANGLE_LO)  /* 碳膜可读下限，负角度 */
#define SIM_TRAVEL_HI  ((int)ENCODER_ANGLE_HI)  /* 碳膜可读上限 */
#define SIM_NOISE_CDEG 4.5                      /* 电位器滑动接触噪声，实测峰值 */
#else
#define SIM_TRAVEL_LO  0                        /* 整圈可测，行程即可读范围 */
#define SIM_TRAVEL_HI  SIM_SPAN_CDEG
#define SIM_NOISE_CDEG 0.8                      /* MT6701 运动中 sigma */
#endif

Config_t g_config;

/* ---- 被控对象状态与可扫描的失配系数 ---- */
static double plant_pos, plant_vel;
static double gain = 1, tau_scale = 1, friction_scale = 1;
static double noise = SIM_NOISE_CDEG, load_pwm;
static int applied, tick, dropout, delay_ticks = MODEL_DELAY_TICKS;
static int history[16], hist_idx;
static uint16_t input;
static uint8_t simulated_fault;
static unsigned rng = 1;

static double random_unit(void) { rng = 1664525U * rng + 1013904223U; return (rng >> 8) / 16777216.0; }

/* ---- 编码器桩：按本工程的 ENCODER_MODE 提供对应的驱动接口 ---- */
#if ENCODER_MODE == 0
uint8_t D_ADC_Encoder_Read(uint16_t *q) {
    double p = plant_pos + noise * (2 * random_unit() - 1);
    /* 抽头转出碳膜或人为制造读取失败：下拉电阻把读数钳到地 */
    if (dropout || p < ENCODER_ANGLE_LO || p > ENCODER_ANGLE_HI) { *q = 64; return 1; }
    *q = (uint16_t)lround((ENCODER_POT_ADC_MIN + p * ENCODER_POT_ADC_SPAN
                           / (double)ENCODER_POT_SPAN_CDEG) * D_ADC_ENCODER_SCALE);
    return 1;
}
uint16_t D_MT6701_Read_Angle(void) { return MT6701_ANGLE_ERROR; }
#else
uint16_t D_MT6701_Read_Angle(void) {
    double p;
    if (dropout) return MT6701_ANGLE_ERROR;
    p = plant_pos + noise * (2 * random_unit() - 1);
    /* 整圈可测，量化到 MT6701 的 2.197 厘度/LSB 再折回 [0,36000) */
    p = lround(p * 16384.0 / 36000.0) * 36000.0 / 16384.0;
    return (uint16_t)(((long)lround(p) % 36000 + 36000) % 36000);
}
#endif

/* ---- 其余驱动桩 ---- */
uint16_t D_ADC_Voltage_Read(void) { return 2000; }
uint8_t  D_ADC_Current_Read(uint16_t *v) { (void)v; return 0; }
int16_t  D_TMP112_Read_Temp(void) { return 250; }
int16_t  D_Motor_Set(int16_t p) { applied = p; return p; }
void     D_Motor_Release(uint8_t h) { (void)h; applied = 0; }
void     D_PWM_Input_Enable(uint8_t en) { (void)en; }
uint16_t D_PWM_Read(void) { uint16_t p = input; input = 0; return p; }
void     D_UART_Enable(uint8_t en) { (void)en; }
uint8_t  D_UART1_Tx_Write(const uint8_t *b, uint8_t n) { (void)b; (void)n; return 1; }
void     Delay_Ms(uint32_t ms) { (void)ms; }
void     A_Config_MarkDirty(void) {}
#ifndef SIM_REAL_PROTECT /* 堵转用例改为链接真实的 A_Protect.c */
uint8_t  A_Protect_Fault(void) { return simulated_fault; }
#endif

#define SIM_MICRO_US 5    /* micro 场景每级的脉宽增量(us)，约67厘度 */
#define SIM_MICRO_MS 600  /* 每级停留时间(ms)，足够走完并停稳 */

/* 行程中点，两种模式的270度行程都落在这里 */
#define SIM_MID   ((int)(SIM_SPAN_CDEG / 2))
#define SIM_LOW   (SIM_TRAVEL_LO - 100)   /* 行程下端之外，电位器模式下即死区 */
#define SIM_HIGH  (SIM_TRAVEL_HI + 100)   /* 行程上端之外 */

static void setup(double p, int pwm_mode) {
    simulated_fault = 0;
    memset(&g_config, 0, sizeof(g_config));
    g_config.servo_mode = 1; g_config.boot_mode = SERVO_BOOT_HOLD;
    plant_pos = p; plant_vel = 0; applied = 0; tick = 0; input = pwm_mode ? 1500 : 0;
    hist_idx = 0; memset(history, 0, sizeof(history)); A_Servo_Init();
}

/* 一个1ms拍：跑一次控制拍，再把输出按纯延迟送进被控对象积分一步 */
static void step(void) {
    A_Servo_Control();
    history[hist_idx] = applied * CTRL_MOTOR_SIGN;
    int u = history[(hist_idx + 16 - delay_ticks) % 16]; hist_idx = (hist_idx + 1) % 16;
    double force = u - load_pwm;
    /* 静摩擦保留实测的方向不对称：控制器按较大值补偿，另一方向被过补 */
    double fs = (force >= 0 ? CAL_BREAKAWAY_PWM : CAL_BREAKAWAY_PWM * 0.77) * friction_scale;
    double fc = CAL_FRICTION_PWM * friction_scale;
    if (fabs(plant_vel) < 1 && fabs(force) < fs) plant_vel = 0;
    else {
        double sign = plant_vel == 0 ? (force >= 0 ? 1 : -1) : (plant_vel > 0 ? 1 : -1);
        double a = (65536.0 / CAL_SPEED_SLOPE_Q16 * gain * (force - fc * sign) - plant_vel)
                 / (MODEL_TAU_MS / 1000.0 * tau_scale);
        double next = plant_vel + a * .001;
        if (next * plant_vel < 0 && fabs(force) < fs) next = 0;
        plant_vel = next;
    }
    plant_pos += plant_vel * .001; tick++;
}

static int failures;
#define CHECK(test) do { if (!(test)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#test); failures++; } } while(0)

/* 目标脉宽 -> 行程坐标，与 Servo_PwmToAngle 同式，供用例写期望值 */
static int target_of(int pwm) {
    return (int)((long)(pwm - SERVO_PWM_MIN) * SIM_SPAN_CDEG / (SERVO_PWM_MAX - SERVO_PWM_MIN));
}

static int regression(void) {
    noise = 0;

    /* ---- 坐标换算：量程内外都不能出现符号翻转或截断错误 ---- */
    int samples[] = { SIM_TRAVEL_LO, SIM_MID / 2, -1, 0, 1, SIM_SPAN_CDEG, SIM_TRAVEL_HI };
    setup(SIM_MID, 0);
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
        int s = samples[i];
#if ENCODER_IS_CIRCULAR
        if (s < 0) continue;               /* 圆周坐标没有负角度 */
#endif
        int want = s < 0 ? 0 : s > SIM_SPAN_CDEG ? SIM_SPAN_CDEG : s;
        plant_pos = s; CHECK(Servo_ReadPosition());
        CHECK(labs((long)s_servo.angle_circ - s) <= 3);
        CHECK(labs((long)s_servo.angle - want) <= 3);
    }

    /* ---- 观测器：静止输入必须收敛到该点且速度归零 ---- */
    SpeedObsOut_t obs; C_SpeedObs_Init(SIM_MID);
    for (int k = 0; k < 100; k++) C_SpeedObs_Update(SIM_MID, 0, &obs);
    CHECK(labs((long)obs.pos - SIM_MID) <= 3); CHECK(labs(obs.vel) <= 10);

    /* ---- 控制器：有正误差时必须朝正方向出力 ---- */
    TrajRef_t ref = { 1000, 0, 0 }; obs.pos = 0; obs.vel = 0; obs.load = 0;
    C_PosCtrl_Init(); CHECK(C_PosCtrl_Update(&ref, &obs, 1, 0) * CTRL_MOTOR_SIGN > 0);

    /* ---- 暂停/继续：目标不能丢，继续后必须重新产生轨迹 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(2000, 0);
    for (int k = 0; k < 50; k++) step();
    A_Servo_Pause(); CHECK(s_servo.target_angle == target_of(2000)); A_Servo_Resume();
    CHECK(!C_Traj_Is_Done());
    A_Servo_Pause(); A_Servo_Submit(2000, 0); CHECK(!C_Traj_Is_Done());

    /* ---- 中值校正与行程收窄 ---- */
    setup(SIM_MID, 0); plant_pos = SIM_MID - 100; CHECK(A_Servo_CalibrateMid());
#if ENCODER_IS_CIRCULAR
    CHECK(labs((long)Servo_Offset() - (CDEG_RANGE - 100)) <= 3); /* 负零点在圆周坐标里存成 36000-100 */
#else
    CHECK(labs((long)Servo_Offset() + 100) <= 3);
#endif
    CHECK(labs((long)s_servo.angle_circ - SIM_MID) <= 3);
    plant_pos = SIM_MID - 3500; CHECK(A_Servo_SetTravelEnd(1));
    CHECK(Servo_ReadPosition()); CHECK(labs((long)s_servo.angle_circ) <= 3);

    /* ---- 大角度往复：必须到位、停稳、静止时不再通电 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(2000, 1000);
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK);
    CHECK(s_servo.target_angle == target_of(2000));
    CHECK(fabs(plant_pos - target_of(2000)) < 100);

    /* ---- PWM输入：单帧毛刺不能生效，连续两帧才算新目标 ---- */
    setup(SIM_MID, 1); A_Servo_Submit(1500, 0);
    for (int k = 0; k < 1000; k++) { if (k % 20 == 0) input = (k % 40 == 0 ? 1497 : 1503); step(); }
    CHECK(fabs(plant_pos - SIM_MID) < 5); CHECK(applied == 0); CHECK(C_Traj_Is_Done());
    int mid_target = s_servo.target_angle;
    input = 2000; step(); CHECK(s_servo.target_angle == mid_target);
    input = 1500; step(); input = 1500; step(); CHECK(s_servo.target_angle == mid_target);
    input = 2000; step(); input = 2000; step(); CHECK(s_servo.target_angle == target_of(2000));

    /* ---- 扭矩上限对闭环链路生效 ---- */
    setup(SIM_MID, 0); A_Servo_SetOutputLimit(300); A_Servo_Submit(500, 0);
    for (int k = 0; k < 300; k++) { step(); CHECK(abs(applied) <= 300); }

#if ENCODER_HAS_DEADZONE
    /* ---- 上电就在死区里：必须开环脱困再执行上电动作 ---- */
    setup(SIM_LOW, 0); A_Servo_Submit(2000, 1000);
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK);
    CHECK(fabs(plant_pos - target_of(2000)) < 100);

    /* ---- 运行中被推进死区：同样要脱困并回到目标 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(2000, 0); plant_pos = SIM_LOW;
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK);
    CHECK(fabs(plant_pos - target_of(2000)) < 100);

    /* ---- 反馈时有时无：两个方向都脱不出来则卸力等人处理 ---- */
    setup(SIM_LOW, 0); plant_pos = SIM_LOW + 200;
    int saw_fault = 0;
    for (int k = 0; k < 4600; k++) { dropout = k % 2; A_Servo_Control();
        if (s_servo.enc_state == SERVO_ENC_FAULT) saw_fault = 1; }
    CHECK(saw_fault); CHECK(applied == 0); CHECK(!A_Servo_TorqueOn()); dropout = 0;
#else
    /* ---- 整圈可测：读失败就是器件坏了，必须停机而不是开环乱转 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(2000, 0);
    for (int k = 0; k < 50; k++) step();
    for (int k = 0; k < 50; k++) { dropout = 1; A_Servo_Control(); }
    CHECK(applied == 0); dropout = 0;

    /* ---- 跨0点：轴停在0点下方一点(圆周坐标读出来接近36000)，目标在0点上方。
     * 位置误差必须按最短路径算成一个小的正值；直接相减会得到 -35900，位置环
     * 立刻朝反方向满舵，实测表现是转速恒定在 -wcorr_max 再也停不下来。 ---- */
    setup(-100, 0); A_Servo_Submit(600, 0);
    for (int k = 0; k < 2000; k++) { step(); CHECK(plant_pos > -2000); } /* 不能反向飞车 */
    CHECK(fabs(plant_pos - target_of(600)) < 100);
#endif

    /* ---- 轴被外力按在行程低端之外：必须朝行程内推，不能朝反方向满舵。
     * 上报坐标在死区里会被投影到端点、误差被抹成0，所以闭环用的必须是未截断反馈。 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(500, 0);
    for (int k = 0; k < 2000; k++) step();          /* 先让轨迹走完，排除跟踪瞬态 */
    CHECK(C_Traj_Is_Done());
    for (int k = 0; k < 300; k++) { plant_pos = -100; plant_vel = 0; A_Servo_Control(); }
    CHECK(applied * CTRL_MOTOR_SIGN > 0);

    printf("regression failures=%d\n", failures); return failures ? 1 : 0;
}

int main(int argc, char **argv) {
    const char *mode = argc > 1 ? argv[1] : "cycle";
    if (strcmp(mode, "regression") == 0) return regression();
    if (argc > 2) gain = atof(argv[2]);
    if (argc > 3) tau_scale = atof(argv[3]);
    if (argc > 4) friction_scale = atof(argv[4]);
    if (argc > 5) noise = atof(argv[5]);
    if (argc > 6) delay_ticks = atoi(argv[6]);
    setup(strcmp(mode, "boot-low") == 0 ? SIM_LOW
        : strcmp(mode, "boot-high") == 0 ? SIM_HIGH : SIM_MID,
          strcmp(mode, "jitter") == 0);
    if (argc > 7) { g_config.servo_mode = (uint8_t)atoi(argv[7]); A_Servo_ApplyConfig(); }

    puts("ms,pwm_target,pos,vel,ref_pos,ref_vel,ref_acc,obs_pos,obs_vel,pwm,done,state,enc_state,error,integral,acc_ff,fric,velff,fb,sat");
    int cmd = 1500;
    for (int k = 0; k < 12000; k++) {
        if (strcmp(mode, "cycle") == 0 || strcmp(mode, "rapid") == 0) {
            int period = strcmp(mode, "rapid") == 0 ? 450 : 3000;
            if (k % period == 0) { cmd = (k / period) % 2 ? 2000 : 500; A_Servo_Submit(cmd, 0); }
        }
        /* micro：每 SIM_MICRO_MS 走一个 SIM_MICRO_US 的小台阶，上10级再下10级。
         * 这是云台微调的真实工况，也是静摩擦前馈和到位死区最容易打架的地方。 */
        if (strcmp(mode, "micro") == 0 && k % SIM_MICRO_MS == 0) {
            int i = (k / SIM_MICRO_MS) % 20;
            cmd = 1500 + SIM_MICRO_US * (i <= 10 ? i : 20 - i);
            A_Servo_Submit((uint16_t)cmd, 0);
        }
        if (strcmp(mode, "jitter") == 0 && k % 20 == 0) input = (uint16_t)(1500 + (int)(random_unit() * 7) - 3);
        if (strcmp(mode, "disturb") == 0 && k == 3000) plant_pos += 300;
        if (strcmp(mode, "load") == 0 && k == 3000) load_pwm = 300;
        if (strcmp(mode, "unload") == 0 && k == 3000) load_pwm = 300;
        if (strcmp(mode, "unload") == 0 && k == 7000) load_pwm = 0;
        step();
        printf("%d,%d,%.4f,%.3f,%ld,%ld,%ld,%ld,%ld,%d,%u,%u,%u,%ld,%d,%d,%d,%d,%d,%u\n",
               k, cmd, plant_pos, plant_vel, (long)s_servo.ref.pos, (long)s_servo.ref.vel,
               (long)s_servo.ref.acc, (long)s_servo.obs.pos, (long)s_servo.obs.vel, applied,
               C_Traj_Is_Done(), s_servo.dbg.state, s_servo.enc_state,
               (long)s_servo.dbg.e_pos, s_servo.dbg.u_i, s_servo.dbg.u_acc,
               s_servo.dbg.u_fric, s_servo.dbg.u_vel, s_servo.dbg.u_fb, s_servo.dbg.sat);
    }
    return 0;
}
