/* 状态机审计：复用 servo_sim.c 的被控对象模型，只查状态迁移，不看调参质量。 */
#define main sim_previous_main
#include "../sim/servo_sim.c"
#undef main

/* 换模式 + 中值校正之后，新目标不能被算到可测范围之外去。 */
static void check_mode_range(void) {
    setup(SIM_MID, 0); CHECK(A_Servo_SetMode(3));       /* 180度模式 */
    plant_pos = 18000; CHECK(A_Servo_CalibrateMid());
    CHECK(A_Servo_SetMode(1)); A_Servo_Submit(2500, 0);
    CHECK((int32_t)s_servo.target_angle + Servo_Offset() <= ENCODER_ANGLE_HI
       || (int32_t)s_servo.target_angle + Servo_Offset() - CDEG_RANGE <= ENCODER_ANGLE_HI);
}

/* 读不到位置时不能把一个假位置存成上电目标。 */
static void check_save_invalid(void) {
    setup(SIM_MID, 0); g_config.startup_pwm = 1234; dropout = 1;
    CHECK(!A_Servo_SaveStartup()); CHECK(g_config.startup_pwm == 1234); dropout = 0;
}

/* 故障期间任何指令都不该把扭矩恢复回来。 */
static void check_protection(void) {
    setup(SIM_MID, 0); A_Servo_Release(0); simulated_fault = 1;
    A_Servo_RestoreTorque(); CHECK(!A_Servo_TorqueOn());
    A_Servo_Submit(2000, 0); CHECK(!A_Servo_TorqueOn());
    CHECK(!s_servo.resume_valid); simulated_fault = 0;
}

/* 配置出来的行程与可测范围无交集时，必须拒绝驱动而不是乱转。 */
static void check_invalid_range(void) {
    setup(SIM_MID, 0); g_config.servo_mode = SERVO_MODE_CUSTOM;
    g_config.custom_offset_cdeg = 35000; g_config.custom_span_cdeg = 100;
    A_Servo_ApplyConfig(); A_Servo_Submit(2500, 0);
    A_Servo_Control();
#if ENCODER_HAS_DEADZONE
    CHECK(applied == 0);
#else
    CHECK(s_servo.range_valid);   /* 整圈可测，任何零点都有效 */
#endif
}

#if ENCODER_HAS_DEADZONE
/* 停止指令必须能把正在脱困的轴停下来，且不留下"待恢复"的目标。 */
static void check_stop_escape(void) {
    setup(SIM_LOW, 0); A_Servo_Submit(2000, 0); A_Servo_Stop();
    for (int k = 0; k < 20; k++) step();
    CHECK(applied == 0); CHECK(!s_servo.boot_pending); CHECK(!s_servo.resume_valid);
}

/* 卸力状态下收到运动指令：先恢复扭矩、脱困，再走到目标。 */
static void check_release_escape(void) {
    setup(SIM_LOW, 0); A_Servo_Release(0); A_Servo_Submit(2000, 0);
    for (int k = 0; k < 4000; k++) step();
    CHECK(s_servo.enc_state == SERVO_ENC_OK); CHECK(fabs(plant_pos - target_of(2000)) < 80);
}

/* 脱困途中暂停必须真的停下来，继续之后仍要走到目标。 */
static void check_pause_escape(void) {
    setup(SIM_MID, 0); A_Servo_Submit(2000, 0); plant_pos = SIM_LOW;
    for (int k = 0; k < 5; k++) A_Servo_Control();
    CHECK(s_servo.enc_state == SERVO_ENC_ESCAPING); A_Servo_Pause();
    for (int k = 0; k < 20; k++) step();
    CHECK(applied == 0);
    A_Servo_Resume(); for (int k = 0; k < 4000; k++) step();
    CHECK(fabs(plant_pos - target_of(2000)) < 80);
}
#endif

int main(int argc, char **argv) {
    noise = 0;
    if (argc < 2) return 2;
    if      (!strcmp(argv[1], "range"))         check_mode_range();
    else if (!strcmp(argv[1], "save"))          check_save_invalid();
    else if (!strcmp(argv[1], "protection"))    check_protection();
    else if (!strcmp(argv[1], "invalid_range")) check_invalid_range();
#if ENCODER_HAS_DEADZONE
    else if (!strcmp(argv[1], "stop"))          check_stop_escape();
    else if (!strcmp(argv[1], "release"))       check_release_escape();
    else if (!strcmp(argv[1], "pause"))         check_pause_escape();
#endif
    else return 2;
    printf("%s failures=%d\n", argv[1], failures); return failures ? 1 : 0;
}
