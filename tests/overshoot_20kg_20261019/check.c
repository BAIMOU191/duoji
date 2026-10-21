#define main original_sim_main
#include "../control_optimization_20261019/source/sim.c"
#undef main

int main(int argc,char **argv)
{
    assert(argc>=6);
    int step=atoi(argv[1]),T=atoi(argv[2]),diag=atoi(argv[3]);
    double factor=atof(argv[4]);
    Plant pl=base_plant();
    if(diag==1)pl.noise=0;
    if(diag==2)pl.ka_s=factor;
    if(diag==3)pl.delay_s=factor;
    if(diag==4){pl.fc_s=factor;pl.fs_s=factor;}
    if(diag==5)pl.kv_s=factor;
    if(diag==6)pl.vbus=factor;
    if(diag==9||diag==12||diag==15)pl.ka_s=factor;
    if(diag==10||diag==13)pl.kv_s=factor;
    load_fn=0;g_kind=0;sim_reset(&pl,0,0);
    if(diag==7)s_gain.delay_q16=0; /* Diagnostic only: remove acceleration lead. */
    if(diag==8)s_gain.ki_q20=0;    /* Diagnostic only: motion integral. */
    if(diag==9||diag==10)s_gain.ki_q20=0;
    if(diag==11||diag==12||diag==13)C_SpeedObs_Configure(&cal,TUNE_OBS_BW_HZ,CTRL_VEL_NOISE_CDPS);
    if(diag==14){PosCtrlGains_t g={.pos_bw=TUNE_POS_BW_RADS,.vel_bw=TUNE_VEL_BW_RADS,.int_bw=TUNE_INT_BW_RADS,.load_ratio_x10=0};C_PosCtrl_SetGains(&g);}
    if(diag==15)s_gain.delay_q16=0;
    FILE *f=strcmp(argv[5],"-")?fopen(argv[5],"w"):0;
    if(f)fprintf(f,"t,x,v,target,ref,refv,refa,obs,uff,ufb,ui,ustick,u,state,done,arrived\n");
    double over=0,ref_over=0,peak_v=0,final=0,integ=0;
    int peak_t=-1,done_t=-1,cross=-1,arr=-1,settle=-1,win=0,dir=step>0?1:-1,fault=0,hard=0;
    cmd(13500+step,T);
    for(int t=0;t<4000+(T>2000?T:0);t++){
        sim_tick();
        double e=(xl-s_servo.target_angle)*dir/100.;
        double re=(s_servo.ref.pos-s_servo.target_angle)*dir/100.;
        if(e>over){over=e;peak_t=t;peak_v=vl/100.;integ=s_servo.dbg.u_i;}
        if(re>ref_over)ref_over=re;
        if(e>0 && cross<0)cross=t;
        if(s_servo.ref.done && done_t<0)done_t=t;
        if(s_servo.arrived && arr<0)arr=t;
        if(fabs(e)<=.2){if(!win){win=1;settle=t;}}else{win=0;settle=-1;}
        fault|=s_prot.fault;
        if(s_servo.debug_control_valid&&abs(s_servo.dbg.u_out)>audit_used_limit)hard++;
        final=e;
        if(f)fprintf(f,"%d,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d\n",t,xl/100,vl/100,s_servo.target_angle/100.,s_servo.ref.pos/100.,s_servo.ref.vel/100.,s_servo.ref.acc/100.,s_servo.obs.vel/100.,s_servo.dbg.u_ff,s_servo.dbg.u_fb,s_servo.dbg.u_i,s_servo.dbg.u_stick,s_servo.dbg.u_out,s_servo.dbg.state,s_servo.ref.done,s_servo.arrived);
    }
    if(f)fclose(f);
    printf("step=%d,T=%d,diag=%d,factor=%.4f,over=%.7f,ref_over=%.7f,peak_t=%d,peak_v=%.4f,peak_ui=%.1f,done=%d,cross=%d,arr=%d,settle02=%d,final=%.7f,fault=%d,hard=%d\n",step,T,diag,factor,over,ref_over,peak_t,peak_v,integ,done_t,cross,arr,settle,final,fault,hard);
    return 0;
}

