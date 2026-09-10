/* Reuse the physical model; audit state transitions separately from tuning. */
#define main sim_previous_main
#include "../sim/servo_sim.c"
#undef main

static void check_stop_escape(void) {
    setup(-1100,0);A_Servo_Submit(2000,0);A_Servo_Stop();
    for(int k=0;k<20;k++)step();
    CHECK(applied==0);CHECK(!s_servo.boot_pending);CHECK(!s_servo.resume_valid);
}
static void check_release_escape(void) {
    setup(-1100,0);A_Servo_Release(0);A_Servo_Submit(2000,0);
    for(int k=0;k<4000;k++)step();
    CHECK(s_servo.enc_state==SERVO_ENC_OK);CHECK(fabs(plant_pos-20250)<80);
}
static void check_pause_escape(void) {
    setup(13500,0);A_Servo_Submit(2000,0);plant_pos=-1100;
    for(int k=0;k<5;k++)A_Servo_Control();
    CHECK(s_servo.enc_state==SERVO_ENC_ESCAPING);A_Servo_Pause();
    for(int k=0;k<20;k++)step();
    CHECK(applied==0);
    A_Servo_Resume();for(int k=0;k<4000;k++)step();CHECK(fabs(plant_pos-20250)<80);
}
static void check_mode_range(void) {
    setup(13500,0);CHECK(A_Servo_SetMode(3));plant_pos=18000;
    CHECK(A_Servo_CalibrateMid());CHECK(Servo_Offset()==9000);
    CHECK(A_Servo_SetMode(1));A_Servo_Submit(2500,0);
    CHECK((int32_t)s_servo.target_angle+Servo_Offset()<=ENCODER_POT_ANGLE_MAX);
}
static void check_save_invalid(void) {
    setup(13500,0);g_config.startup_pwm=1234;dropout=1;
    CHECK(!A_Servo_SaveStartup());CHECK(g_config.startup_pwm==1234);dropout=0;
}
static void check_protection(void) {
    setup(13500,0);A_Servo_Release(0);simulated_fault=1;
    A_Servo_RestoreTorque();CHECK(!A_Servo_TorqueOn());
    A_Servo_Submit(2000,0);CHECK(!A_Servo_TorqueOn());
    CHECK(!s_servo.resume_valid);simulated_fault=0;
}
static void check_invalid_range(void) {
    setup(13500,0);g_config.servo_mode=11;g_config.custom_offset_cdeg=35000;
    g_config.custom_span_cdeg=100;A_Servo_ApplyConfig();A_Servo_Submit(2500,0);
    A_Servo_Control();CHECK(applied==0);
}
int main(int argc,char **argv) {
    noise=0;
    if(argc<2)return 2;
    if(!strcmp(argv[1],"stop"))check_stop_escape();
    else if(!strcmp(argv[1],"release"))check_release_escape();
    else if(!strcmp(argv[1],"pause"))check_pause_escape();
    else if(!strcmp(argv[1],"range"))check_mode_range();
    else if(!strcmp(argv[1],"save"))check_save_invalid();
    else if(!strcmp(argv[1],"protection"))check_protection();
    else if(!strcmp(argv[1],"invalid_range"))check_invalid_range();
    else return 2;
    printf("%s failures=%d\n",argv[1],failures);return failures?1:0;
}
