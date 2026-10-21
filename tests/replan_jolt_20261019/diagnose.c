#define main old_sim_main
#include "sim.c"
#undef main
int main(void)
{
    Plant pl=base_plant();pl.r=1;load_fn=0;g_kind=0;sim_reset(&pl,0,0);
    cmd(18000,1000);for(int t=0;t<700;t++)sim_tick();
    printf("before actual_v=%.4f ref_v=%.4f ref_a=%.4f remaining=%u\n",vl/100,s_tj.vel_qt*10./65536,s_tj.acc_now/100.,s_servo.remaining);
    cmd(18500,1000);
    printf("new pre_ms=%u first_ms=%d cruise_ms=%d brake_ms=%d total_ms=%d start_v=%.4f cruise_v=%.4f\n",s_tj.pre_T,s_tj.t1,s_tj.tc,s_tj.t3,s_tj.T,s_tj.v0_qt*10./65536,s_tj.vc_qt*10./65536);
    for(int t=0;t<80;t++){sim_tick();if(t%10==0)printf("post=%d ref_v=%.3f ref_a=%.3f actual_v=%.3f\n",t+1,s_servo.ref.vel/100.,s_servo.ref.acc/100.,vl/100.);}
    return 0;
}
