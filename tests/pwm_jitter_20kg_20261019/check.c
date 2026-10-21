#define main original_sim_main
#include "../control_optimization_20261019/source/sim.c"
#undef main

int main(int argc,char **argv)
{
    assert(argc==6);
    int mode=atoi(argv[1]),T=atoi(argv[2]),diag=atoi(argv[4]);double noise=atof(argv[3]);
    Plant pl=base_plant();pl.noise=noise;
    if(diag==4)pl.ka_s=1.3;
    if(diag==5)pl.kv_s=1.15;
    load_fn=0;g_kind=0;sim_reset(&pl,0,0);
    if(diag==1){MotorCal_t c=cal;c.ka_nv=c.ka_nv*(10+TUNE_LOAD_RATIO_X10)/10;C_SpeedObs_Configure(&c,10,CTRL_VEL_NOISE_CDPS);}
    if(diag==2)s_gain.kv_q16=0; /* Attribution only, not a controller proposal. */
    if(diag==3){s_gain.fs_mv=0;} /* Isolate stiction boost without touching plant friction. */
    FILE *f=fopen(argv[5],"w");assert(f);
    fprintf(f,"t,x,v,target,ref,refv,refa,measured,obspos,obsv,dist,stuck,uff,ufb,ui,ustick,u,pwm,applied_mv,bus_cv,state,done,arrived,lim,fault\n");
    int duration=mode==3||mode==4||mode==5?4500:2500;
    for(int t=0;t<duration;t++){
        if(t==0&&mode<=2)cmd(mode==0?22500:mode==1?18000:14000,T);
        if(t==0&&mode>=7)cmd(mode==7?13600:mode==8?13530:13510,T);
        if(mode==3&&t<=1800&&t%50==0)cmd(14500+3*t,T);
        if(mode==4&&t<1600&&t%400==0)cmd((t/400)%2?9000:18000,T);
        if(mode==5&&t==0)cmd(15500,10000);
        sim_tick();
        assert(duty_now==s_servo.applied_pwm);
        fprintf(f,"%d,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",t,xl/100,vl/100,s_servo.target_angle/100.,s_servo.ref.pos/100.,s_servo.ref.vel/100.,s_servo.ref.acc/100.,s_servo.angle_circ/100.,s_servo.obs.pos/100.,s_servo.obs.vel/100.,s_servo.obs.dist,s_servo.obs.stuck,s_servo.dbg.u_ff,s_servo.dbg.u_fb,s_servo.dbg.u_i,s_servo.dbg.u_stick,s_servo.dbg.u_out,s_servo.applied_pwm,s_servo.applied_mv,s_servo.pwm_bus_cv,s_servo.dbg.state,s_servo.ref.done,s_servo.arrived,audit_used_limit,s_prot.fault);
    }
    fclose(f);return 0;
}
