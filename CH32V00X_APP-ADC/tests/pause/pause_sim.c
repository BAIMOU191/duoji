#define main original_sim_main
#include "../sim/servo_sim.c"
#undef main
int main(int argc,char **argv) {
    int dir=argc>1?atoi(argv[1]):1;
    int before=argc>2?atoi(argv[2]):180, wait=argc>3?atoi(argv[3]):500;
    int timed=argc>4?atoi(argv[4]):0, new_command=argc>5?atoi(argv[5]):0;
    noise=0;setup(dir>0?4000:23000,0);
    A_Servo_Submit(dir>0?2500:500,(uint16_t)timed);
    for(int k=0;k<before;k++)step();
    A_Servo_Pause();double pause_pos=plant_pos;
    for(int k=0;k<wait;k++)step();
    double resume_pos=plant_pos, peak=dir*plant_pos, reverse=0;
    int old_ref=s_servo.ref.pos, old_obs=s_servo.obs.vel;
    if(new_command)A_Servo_Submit(dir>0?2500:500,(uint16_t)timed);
    else A_Servo_Resume();
    int start_ref=s_servo.ref.pos, initial_remaining=s_servo.remaining;
    for(int k=0;k<4000;k++) {
        step();
        if(k<200) {
            double p=dir*plant_pos;if(p>peak)peak=p;
            if(peak-p>reverse)reverse=peak-p;
        }
    }
    printf("{\"dir\":%d,\"before\":%d,\"wait\":%d,\"timed\":%d,\"new_command\":%d,\"pause_pos\":%.4f,\"resume_pos\":%.4f,\"old_ref\":%d,\"old_obs_vel\":%d,\"start_ref\":%d,\"reverse_cdeg\":%.4f,\"final_pos\":%.4f,\"remaining_at_resume\":%d}\n",dir,before,wait,timed,new_command,pause_pos,resume_pos,old_ref,old_obs,start_ref,reverse,plant_pos,initial_remaining);
    return 0;
}
