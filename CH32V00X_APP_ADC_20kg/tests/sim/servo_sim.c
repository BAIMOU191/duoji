/*
 * servo_sim.c —— 主机端舵机仿真
 *
 * 链接的是**生产代码本体**：A_Servo.c / A_Sensor.c / C_Traj_Planner.c /
 * C_Speed_Observer.c / C_Pos_Ctrl.c 一行不改地编译进来。只有 ADC、PWM 捕获和
 * H桥被替换成下面的桩，控制算法与板端一致；简化对象的仿真行为不等于硬件保证。
 *
 * 被控对象按 tests/sim/sim_plant.h 里本机的对象参数(分方向)建模，与 A_Parameter.h 的
 * 控制器参数分开，调控制器参数时对象不跟着变。同一份仿真代码四个工程逐字相同：
 *     dv/dt = (Kv*(u - fc*sign(v)) - v) / tau      Kv = 65536/slope
 * slope/fc/tau 按运动方向取，静摩擦按出力方向取；对象参数独立保存，
 * 控制器更新不会改变对象；噪声和增益/惯性/摩擦/延迟可另行扫描。
 *
 * 单位：厘度、秒、PWM计数。
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "A_Servo.c"
#include "D_mt6701.h"   /* MT6701_ANGLE_ERROR */
#include "sim_plant.h"  /* 本机被控对象参数 */

/* 模式1的270度行程，两种编码器一致；仿真的可读范围和噪声底按传感器给。 */
#define SIM_SPAN_CDEG  27000
#if ENCODER_HAS_DEADZONE
#define SIM_TRAVEL_LO  ((int)ENCODER_ANGLE_LO)  /* 碳膜可读下限，负角度 */
#define SIM_TRAVEL_HI  ((int)ENCODER_ANGLE_HI)  /* 碳膜可读上限 */
#else
#define SIM_TRAVEL_LO  0                        /* 整圈可测，行程即可读范围 */
#define SIM_TRAVEL_HI  SIM_SPAN_CDEG
#endif

Config_t g_config;

/* ---- 被控对象状态与可扫描的失配系数 ---- */
static double plant_pos, plant_vel;
#ifndef SIM_PLANT_BRAKE_SCALE
#define SIM_PLANT_BRAKE_SCALE 1.0   /* 本机 sim_plant.h 未给制动增益时按与驱动侧对称处理 */
#endif
static double gain = 1, tau_scale = 1, friction_scale = 1, brake_scale = SIM_PLANT_BRAKE_SCALE;
static double noise = SIM_NOISE_CDEG, load_pwm;
static int applied, tick, dropout, delay_ticks = SIM_PLANT_DELAY_TICKS;
static int history[16], hist_idx;
static uint16_t input;
static uint8_t simulated_fault;
static int jammed;              /* 1=轴被障碍物挡死：位置冻结，与摩擦参数无关(堵转审计用) */
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
/* 桩必须照搬 D_tim.c 的捕获窗口：窗口外的脉宽在驱动层就整帧丢掉、上层根本
 * 收不到，容差带内的则夹进协议量程再上报。不模拟这一层，仿真就会把板子上
 * 永远不会出现的值喂给控制链。三个常数与 D_tim.c 一一对应。 */
#define SIM_PULSE_MIN_US 500
#define SIM_PULSE_MAX_US 2500
#define SIM_PULSE_TOL_US 50
uint16_t D_PWM_Read(void) {
    uint16_t p = input; input = 0;
    if (p < SIM_PULSE_MIN_US - SIM_PULSE_TOL_US) return 0;
    if (p > SIM_PULSE_MAX_US + SIM_PULSE_TOL_US) return 0;
    if (p < SIM_PULSE_MIN_US) p = SIM_PULSE_MIN_US;
    if (p > SIM_PULSE_MAX_US) p = SIM_PULSE_MAX_US;
    return p;
}
void     D_UART_Enable(uint8_t en) { (void)en; }
uint8_t  D_UART1_Tx_Write(const uint8_t *b, uint8_t n) { (void)b; (void)n; return 1; }
void     Delay_Ms(uint32_t ms) { (void)ms; }
void     A_Config_MarkDirty(void) {}
#ifndef SIM_REAL_PROTECT /* 堵转用例改为链接真实的 A_Protect.c */
uint8_t  A_Protect_Fault(void) { return simulated_fault; }
#endif

#define SIM_MICRO_US 5    /* micro 场景每级的脉宽增量(us)，约67厘度 */
#define SIM_MICRO_MS 600  /* 每级停留时间(ms)，足够走完并停稳 */

/* pwm-micro：与 micro 同样的阶梯，但走 PWM 输入这条路(50Hz 帧 + 线上抖动)，
 * 用来量"PWM 输入模式的最小步距"到底比总线粗多少。台阶取 1us，只有真正被
 * 目标死区放行的那些级才会产生位移，未放行的级会并到下一级里。 */
#define SIM_PWM_MICRO_US 1    /* 每级脉宽增量(us) */
#define SIM_PWM_MICRO_MS 600  /* 每级停留时间(ms) */
#define SIM_PWM_FRAME_MS 20   /* 输入帧周期(ms)，50Hz */
static int pwm_jitter;        /* 注入到线上的脉宽抖动幅度(±us)，0=干净信号 */
#define SIM_JITTER_US 3       /* jitter 场景的默认抖动幅度(±us)，可由命令行覆盖 */

/* 行程中点，两种模式的270度行程都落在这里 */
#define SIM_MID   ((int)(SIM_SPAN_CDEG / 2))
#define SIM_LOW   (SIM_TRAVEL_LO - 100)   /* 行程下端之外，电位器模式下即死区 */
#define SIM_HIGH  (SIM_TRAVEL_HI + 100)   /* 行程上端之外 */

static void setup(double p, int pwm_mode) {
    simulated_fault = 0;
    memset(&g_config, 0, sizeof(g_config));
    g_config.servo_mode = 1; g_config.boot_mode = SERVO_BOOT_HOLD;
    /* 出厂默认的脉冲边界。留成0会让 Servo_PulseAllowedQ3 判出一个空窗口，
     * 舵机拒收一切运动指令——这不是被测行为，是仿真自己没初始化。 */
    g_config.pulse_lo = SERVO_PWM_MIN; g_config.pulse_hi = SERVO_PWM_MAX;
    g_config.cal_kind = SERVO_CAL_NONE;
    g_config.traj_vmax_dps = TUNE_VMAX_DPS;
    g_config.traj_amax_dps2 = TUNE_AMAX_DPS2;
    g_config.traj_dmax_dps2 = TUNE_DMAX_DPS2;
    g_config.traj_jmax_dps3 = TUNE_JMAX_DPS3;
    g_config.traj_move_min_ms = TUNE_MOVE_MIN_MS;
    plant_pos = p; plant_vel = 0; applied = 0; tick = 0; input = pwm_mode ? 1500 : 0; jammed = 0;
    hist_idx = 0; memset(history, 0, sizeof(history)); A_Servo_Init();
}

/* 一个1ms拍：跑一次控制拍，再把输出按纯延迟送进被控对象积分一步 */
static void step(void) {
    A_Servo_Control();
    history[hist_idx] = applied * SIM_PLANT_MOTOR_SIGN;
    int u = history[(hist_idx + 16 - delay_ticks) % 16]; hist_idx = (hist_idx + 1) % 16;
    /* 慢衰减H桥制动侧偏弱：出力与转向相反时按标定的制动增益打折(见 sim_plant.h)，外部负载不打折 */
    double drive = (plant_vel * u < 0) ? u * brake_scale : u;
    double force = drive - load_pwm;
    if (jammed) { plant_vel = 0; tick++; return; }
    /* 静摩擦按出力方向、其余按运动方向取本机分向对象值(静止时按出力方向) */
    double plant_sign = fabs(plant_vel) < 1 ? force : plant_vel;
    double fs = (force >= 0 ? SIM_PLANT_BRK_POS_PWM : SIM_PLANT_BRK_NEG_PWM) * friction_scale;
    double fc = (plant_sign >= 0 ? SIM_PLANT_FRIC_POS_PWM : SIM_PLANT_FRIC_NEG_PWM) * friction_scale;
    double plant_slope = plant_sign >= 0 ? SIM_PLANT_SLOPE_POS_Q16 : SIM_PLANT_SLOPE_NEG_Q16;
    double plant_tau = plant_sign >= 0 ? SIM_PLANT_TAU_POS_MS : SIM_PLANT_TAU_NEG_MS;
    if (fabs(plant_vel) < 1 && fabs(force) < fs) plant_vel = 0;
    else {
        double sign = plant_vel == 0 ? (force >= 0 ? 1 : -1) : (plant_vel > 0 ? 1 : -1);
        double a = (65536.0 / plant_slope * gain * (force - fc * sign) - plant_vel)
                 / (plant_tau / 1000.0 * tau_scale);
        double next = plant_vel + a * .001;
        if (next * plant_vel < 0 && fabs(force) < fs) next = 0;
        plant_vel = next;
    }
    plant_pos += plant_vel * .001; tick++;
}

/* 到位验收：配置的静止捕获窗 + 当前噪声 + 3厘度反馈量化/整数取整裕量，随本机分辨率走，不写死精度 */
#define SIM_ARRIVE_TOL_CDEG (CTRL_HOLD_DEADBAND + noise + 3)

static int failures;
#define CHECK(test) do { if (!(test)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#test); failures++; } } while(0)

/* 目标脉宽 -> 行程坐标，与 Servo_PulseToAngle 同式(含就近取整和方向翻转)，
 * 供用例写期望值。必须跟着 s_servo.reverse 走：CFG_DIR_INVERT 把整机旋向翻过来
 * 之后，同一个脉宽对应的是行程另一头，期望值写死就会整片用例假失败。 */
static int target_of(int pwm) {
    long q3 = (long)(pwm - SERVO_PWM_MIN) * 8;
    long span_q3 = (long)(SERVO_PWM_MAX - SERVO_PWM_MIN) * 8;
    long span = s_servo.span_cdeg;   /* 有效行程，校准后可能小于标称量程 */
    int a = (int)((q3 * span + span_q3 / 2) / span_q3);
    return s_servo.reverse ? (int)(span - a) : a;
}

/* target_of 的反函数：给定行程坐标，求能走到那儿的脉宽。用例想表达"往行程低端
 * 走"这类与方向无关的意图时用它，而不是写死某个脉宽。 */
static int pulse_for(int cdeg) {
    long span = s_servo.span_cdeg;
    if (span <= 0) return SERVO_PWM_MID; /* 电机模式没有行程，别在用例里除零 */
    if (s_servo.reverse) cdeg = (int)(span - cdeg);
    return SERVO_PWM_MIN + (int)(((long)cdeg * (SERVO_PWM_MAX - SERVO_PWM_MIN)
                                  + span / 2) / span);
}

/* 沿"脉宽增大"方向的符号，供方向无关的断言用 */
static inline int dir_sign(void) { return s_servo.reverse ? -1 : 1; }

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

    /* ---- 中值校正与行程收窄 ----
     * 校准的对外契约只有一条：校完之后，脚下这个位置就是 1500us 对应的行程中点。
     * 有效行程可能因此收缩(电位器版量程被物理卡死，见 Servo_DeriveCal)，所以
     * 期望值按 span_cdeg 写，不写死标称量程。 */
    setup(SIM_MID, 0); plant_pos = SIM_MID - 100; CHECK(A_Servo_CalibrateMid());
    CHECK(labs((long)s_servo.angle_circ - s_servo.span_cdeg / 2) <= 3);
    CHECK(labs((long)A_Servo_GetPositionPwm() - SERVO_PWM_MID) <= 2);
#if ENCODER_IS_CIRCULAR
    CHECK(s_servo.span_cdeg == SIM_SPAN_CDEG);          /* 整圈可测：不收缩 */
    CHECK(labs((long)Servo_Offset() - (CDEG_RANGE - 100)) <= 3);
#else
    /* 锚点偏低 100 厘度，对称收缩后行程 = 2*(13400-0) = 26800 */
    CHECK(s_servo.span_cdeg == 26800);
    CHECK(labs((long)Servo_Offset()) <= 3);
#endif
    /* AMI 把 500us 端收到当前位置。500us 在行程坐标里是哪一头由方向决定：
     * 正向时是低端(0)，反向时是高端(span)，所以期望值要跟着方向走。 */
    plant_pos = SIM_MID - 3500; CHECK(A_Servo_SetTravelEnd(1));
    CHECK(Servo_ReadPosition());
    CHECK(labs((long)s_servo.angle_circ
               - (s_servo.reverse ? s_servo.span_cdeg : 0)) <= 3);

    /* ---- 大角度往复：必须到位、停稳、静止时不再通电 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(2000, 1000);
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK);
    CHECK(s_servo.target_angle == target_of(2000));
    CHECK(fabs(plant_pos - target_of(2000)) <= SIM_ARRIVE_TOL_CDEG);

    /* ---- PWM输入：单帧毛刺不能生效，连续两帧才算新目标 ---- */
    setup(SIM_MID, 1); A_Servo_Submit(1500, 0);
    for (int k = 0; k < 1000; k++) { if (k % 20 == 0) input = (k % 40 == 0 ? 1497 : 1503); step(); }
    CHECK(fabs(plant_pos - SIM_MID) < 5); CHECK(applied == 0); CHECK(C_Traj_Is_Done());
    int mid_target = s_servo.target_angle;
    input = 2000; step(); CHECK(s_servo.target_angle == mid_target);
    input = 1500; step(); input = 1500; step(); CHECK(s_servo.target_angle == mid_target);
    input = 2000; step(); input = 2000; step(); CHECK(s_servo.target_angle == target_of(2000));

    /* ---- PWM输入：窗口外整帧丢掉，容差带内夹回协议量程 ---- */
    setup(SIM_MID, 1);
    input = 1500; step(); input = 1500; step();
    int keep = s_servo.target_angle;
    /* 窗口之外(±50us 以外)：丢帧，目标一动不动 */
    input = 2600; step(); input = 2600; step(); CHECK(s_servo.target_angle == keep);
    input =  400; step(); input =  400; step(); CHECK(s_servo.target_angle == keep);
    /* 容差带之内：不丢，但夹到端点，绝不允许越过行程 */
    input = 2540; step(); input = 2540; step();
    CHECK(s_servo.target_angle == (s_servo.reverse ? s_servo.traj_lo : s_servo.traj_hi));
    input =  460; step(); input =  460; step();
    CHECK(s_servo.target_angle == (s_servo.reverse ? s_servo.traj_hi : s_servo.traj_lo));
    /* 这一串之后合法脉宽仍须立刻生效——滤波器不能被越界值带偏 */
    input = 2000; step(); input = 2000; step(); CHECK(s_servo.target_angle == target_of(2000));

    /* ---- 扭矩上限对闭环链路生效 ---- */
    setup(SIM_MID, 0); A_Servo_SetOutputLimit(300); A_Servo_Submit(500, 0);
    for (int k = 0; k < 300; k++) { step(); CHECK(abs(applied) <= 300); }

#if ENCODER_HAS_DEADZONE
    /* ---- 上电就在死区里：必须开环脱困再执行上电动作 ---- */
    setup(SIM_LOW, 0); A_Servo_Submit(2000, 1000);
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK);
    CHECK(fabs(plant_pos - target_of(2000)) <= SIM_ARRIVE_TOL_CDEG);

    /* ---- 运行中被推进死区：同样要脱困并回到目标 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(2000, 0); plant_pos = SIM_LOW;
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK);
    CHECK(fabs(plant_pos - target_of(2000)) <= SIM_ARRIVE_TOL_CDEG);

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
    setup(-100, 0);
    { int pwm = pulse_for(1350); A_Servo_Submit(pwm, 0);
      for (int k = 0; k < 2000; k++) { step(); CHECK(plant_pos > -2000); } /* 不能反向飞车 */
      CHECK(fabs(plant_pos - target_of(pwm)) < 100); }
#endif

    /* ---- 轴被外力按在行程低端之外：必须朝行程内推，不能朝反方向满舵。
     * 上报坐标在死区里会被投影到端点、误差被抹成0，所以闭环用的必须是未截断反馈。 ---- */
    setup(SIM_MID, 0); A_Servo_Submit(pulse_for(0), 0);
    for (int k = 0; k < 2000; k++) step();          /* 先让轨迹走完，排除跟踪瞬态 */
    CHECK(C_Traj_Is_Done());
    /* 推离量须超过静音退出窗，否则仍属允许的保持范围；不小于旧用例的100厘度 */
    { double push = (CTRL_QUIET_EXIT_CDEG + 10 > 100) ? CTRL_QUIET_EXIT_CDEG + 10 : 100;
      for (int k = 0; k < 300; k++) { plant_pos = -push; plant_vel = 0; A_Servo_Control(); } }
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
          strcmp(mode, "jitter") == 0 || strncmp(mode, "pwm-micro", 9) == 0);
    if (strcmp(mode, "pwm-micro-jit") == 0 || strcmp(mode, "jitter") == 0)
        pwm_jitter = SIM_JITTER_US;
    if (argc > 8) pwm_jitter = atoi(argv[8]);   /* 抖动幅度扫描 */
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
        if (strcmp(mode, "jitter") == 0 && k % 20 == 0)
            input = (uint16_t)(1500 + (int)(random_unit() * (2 * pwm_jitter + 1)) - pwm_jitter);
        /* bus-micro：同样的 1us 阶梯，但走总线路径，作为 PWM 路径的对照组 */
        if (strcmp(mode, "bus-micro") == 0 && k % SIM_PWM_MICRO_MS == 0) {
            int i = (k / SIM_PWM_MICRO_MS) % 20;
            cmd = 1500 + SIM_PWM_MICRO_US * (i <= 10 ? i : 20 - i);
            A_Servo_Submit((uint16_t)cmd, 0);
        }
        if (strncmp(mode, "pwm-micro", 9) == 0 && k % SIM_PWM_FRAME_MS == 0) {
            int i = (k / SIM_PWM_MICRO_MS) % 20;
            int j = pwm_jitter ? (int)(random_unit() * (2 * pwm_jitter + 1)) - pwm_jitter : 0;
            cmd   = 1500 + SIM_PWM_MICRO_US * (i <= 10 ? i : 20 - i);
            input = (uint16_t)(cmd + j);
        }
        /* flip：先停到P2000，再以T2000在P2500/P1500间每300ms反向，舵机总在两目标中间换向 */
        if (strcmp(mode, "flip") == 0) {
            if (k == 0) A_Servo_Submit(cmd = 2000, 0);
            else if (k >= 1500 && k < 9000 && k % 300 == 0) {
                cmd = ((k - 1500) / 300) % 2 ? 1500 : 2500;
                A_Servo_Submit((uint16_t)cmd, 2000);
            }
        }
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
