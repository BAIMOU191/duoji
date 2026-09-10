#define main original_sim_main
#include "../sim/servo_sim.c"
#undef main
int main(void) {
    setup(4000,0);noise=0;A_Servo_Submit(2500,1500);
    for(int k=0;k<180;k++)step();
    A_Servo_Pause();uint16_t target=s_servo.target_angle;uint32_t remaining=s_servo.remaining;
    for(int k=0;k<500;k++)step();
    CHECK(s_servo.target_angle==target);CHECK(s_servo.remaining==remaining);CHECK(applied==0);
    dropout=1;A_Servo_Resume();CHECK(s_servo.paused);CHECK(applied==0);CHECK(s_servo.target_angle==target);
    A_Servo_Submit(1000,0);CHECK(s_servo.paused);CHECK(s_servo.target_angle==target);dropout=0;
    A_Servo_Resume();CHECK(!s_servo.paused);CHECK(s_servo.target_angle==target);CHECK(s_servo.remaining==remaining);
    A_Servo_Pause();A_Servo_Stop();A_Servo_Resume();CHECK(!s_servo.resume_valid);CHECK(C_Traj_Is_Done());
    printf("pause state failures=%d\n",failures);return failures?1:0;
}
