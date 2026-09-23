/* Exercise the real PWM input/filter/control path with stationary command noise. */
#define main previous_main
#include "../sim/servo_sim.c"
#undef main

static unsigned input_rng;
static unsigned next_input(void) { input_rng=1664525U*input_rng+1013904223U; return input_rng>>8; }

int main(int argc, char **argv)
{
    const char *mode=argc>1?argv[1]:"constant";
    int amp=argc>2?atoi(argv[2]):6;
    int changes=0, late_changes=0, max_pwm=0, last_target, accepted_ms=-1;
    double lo=SIM_MID, hi=SIM_MID, max_speed=0;
    input_rng=123; setup(SIM_MID,1);
    last_target=s_servo.target_angle;
    for (int k=0;k<12000;k++) {
        if (k%20==0) {
            int frame=(k-2000)/20, offset=0;
            if (k>=2000) {
                if (!strcmp(mode,"plateau")) offset=((frame/4)%2?1:-1)*amp;
                if (!strcmp(mode,"random")) offset=(int)(next_input()%(2*amp+1))-amp;
                if (!strcmp(mode,"spike1")) offset=frame%50==10?amp:0;
                if (!strcmp(mode,"spike2")) offset=(frame%50==10||frame%50==11)?amp:0;
                if (!strcmp(mode,"step")) offset=amp;
                if (!strcmp(mode,"ramp")) offset=frame/3;
            }
            input=(uint16_t)(1500+offset);
        }
        step();
        if (k>=2000) {
            if (s_servo.target_angle!=last_target) {
                changes++; if(k>=4000)late_changes++;
                if(accepted_ms<0)accepted_ms=k-2000;
            }
            if(plant_pos<lo)lo=plant_pos;if(plant_pos>hi)hi=plant_pos;
            if(abs(applied)>max_pwm)max_pwm=abs(applied);
            if(fabs(plant_vel)>max_speed)max_speed=fabs(plant_vel);
        }
        last_target=s_servo.target_angle;
    }
    printf("{\"mode\":\"%s\",\"amp_us\":%d,\"target_changes\":%d,\"late_changes\":%d,"
           "\"motion_pp_deg\":%.5f,\"max_pwm\":%d,\"peak_speed_dps\":%.4f,"
           "\"first_command_ms\":%d,\"final_target\":%u}\n",
           mode,amp,changes,late_changes,(hi-lo)/100,max_pwm,max_speed/100,
           accepted_ms,s_servo.target_angle);
    return 0;
}
