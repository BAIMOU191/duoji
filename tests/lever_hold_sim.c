/* Reuse actual firmware harness. All changes in this file are test-side. */
#define main latency_audit_main
#include "control_latency_sim.c"
#undef main

static void lever_trace(FILE *f,const char *name,int t) {
    if(!f) return;
    fprintf(f,"%s,%d,%.5f,%.5f,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u\n",
        name,t,(plant_p-s_servo.offset_cdeg)/100.,plant_v/100.,s_servo.target_angle,s_servo.ref.pos,
        s_servo.obs.pos,s_servo.obs.vel,s_servo.dbg.u_out,s_ctrl.integral_q16>>16,
        s_ctrl.state,s_ctrl.probing,s_ctrl.loaded,s_ctrl.push_ms,s_ctrl.slip_ms,s_ctrl.kicks,
        s_ctrl.boost,s_servo.dbg.u_stick,s_servo.obs.dist,s_servo.out_limit,A_Protect_Fault());
}

static void run_lever(FILE *trace_file,FILE *summary,double load,double inertia,double brake,
                      int pulse,int precision,int initial,int noise,int duration,int int_bw,int stiff) {
    reset();load_mv=load;inertia_scale=inertia;brake_scale=brake;noise_amp=noise;
    g_runtime_config.position_resolution_cdeg=precision;
    g_runtime_config.ctrl_int_bw=int_bw;g_runtime_config.hold_stiff=stiff;A_Servo_ApplyCtrlConfig();
    /* initial=0: established load before motion; 1: apply load at command. */
    if(initial==0) for(int i=0;i<5000;i++) step();
    double initial_pos=plant_p;
    command(pulse,duration);
    char name[128];snprintf(name,sizeof(name),"L%.0f_J%.3f_B%.2f_P%d_W%d_init%d_N%d_T%d_I%d_S%d",load,inertia,brake,pulse,precision,initial,noise,duration,int_bw,stiff);
    double lo=1e9,hi=-1e9,sum=0,err2=0;int recover=0,hold=0,zero_i=0,prev_state=s_ctrl.state;
    double tail_lo=1e9,tail_hi=-1e9;
    int periods=0,last_recover=-1;double period_sum=0;
    int before_i=s_ctrl.integral_q16;
    for(int i=0;i<20000;i++) {
        step();lever_trace(trace_file,name,i);
        if(i>=5000) {
            double p=plant_p-s_servo.offset_cdeg;lo=fmin(lo,p);hi=fmax(hi,p);sum+=p;
            err2+=(p-s_servo.target_angle)*(p-s_servo.target_angle);
            if(s_ctrl.state==PC_RECOVER && prev_state!=PC_RECOVER) {
                recover++;if(last_recover>=0) { periods++;period_sum+=i-last_recover; }last_recover=i;
            }
            if(s_ctrl.state==PC_HOLD && prev_state!=PC_HOLD) hold++;
            if(before_i!=0 && s_ctrl.integral_q16==0) zero_i++;
        }
        if(i>=15000) { double p=plant_p-s_servo.offset_cdeg;tail_lo=fmin(tail_lo,p);tail_hi=fmax(tail_hi,p); }
        before_i=s_ctrl.integral_q16;prev_state=s_ctrl.state;
    }
    fprintf(summary,"%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.3f,%u,%.3f,%.3f\n",name,
        (hi-lo)/100.,(sum/15000-s_servo.target_angle)/100.,sqrt(err2/15000)/100.,
        periods?period_sum/periods/1000:0,recover,hold,zero_i,initial_pos/100.,s_ctrl.state,plant_p/100.,(tail_hi-tail_lo)/100.);
}

static void disturbance(FILE *f,FILE *summary,double inertia,double brake,int noise) {
    reset();load_mv=1916.5;inertia_scale=inertia;brake_scale=brake;noise_amp=noise;
    for(int i=0;i<5000;i++) step();
    command(1478,0);
    char name[80];snprintf(name,sizeof(name),"disturb_J%.3f_B%.2f_N%d",inertia,brake,noise);
    double peaks[3]={0},tail_lo=1e9,tail_hi=-1e9;
    for(int i=0;i<26000;i++) {
        if(i==7000) load_mv=0; /* remove weight, unchanged inertia: isolate torque step */
        if(i==12000) load_mv=1916.5;
        if(i==18000) { plant_p-=200;plant_v=0; } /* external 2deg displacement */
        step();lever_trace(f,name,i);
        double e=fabs(plant_p-s_servo.offset_cdeg-s_servo.target_angle)/100.;
        if(i>=7000 && i<12000) peaks[0]=fmax(peaks[0],e);
        if(i>=12000 && i<18000) peaks[1]=fmax(peaks[1],e);
        if(i>=18000) peaks[2]=fmax(peaks[2],e);
        if(i>=22000) { tail_lo=fmin(tail_lo,plant_p);tail_hi=fmax(tail_hi,plant_p); }
    }
    printf("%s remove_load_peak=%.3fdeg restore_load_peak=%.3fdeg push_peak=%.3fdeg tail_pp=%.3fdeg final_error=%.3fdeg\n",name,peaks[0],peaks[1],peaks[2],(tail_hi-tail_lo)/100.,(plant_p-s_servo.offset_cdeg-s_servo.target_angle)/100.);
    (void)summary;
}

int main(int argc,char **argv) {
    if(argc<2) return 2;
    FILE *summary=fopen(argv[1],"w");assert(summary);
    fprintf(summary,"case,peak_peak_deg,mean_error_deg,rms_error_deg,recover_period_s,recover_entries,hold_entries,integral_zero_events,initial_raw_deg,final_state,final_raw_deg,tail_peak_peak_deg\n");
    FILE *trace_file=argc>2?fopen(argv[2],"w"):NULL;
    if(trace_file) fprintf(trace_file,"case,t_ms,actual_deg,actual_dps,target_cdeg,ref_cdeg,obs_pos_cdeg,obs_vel_cdps,out_mv,integral_mv,state,probing,loaded,push_ms,slip_ms,kicks,boost_mv,stick_mv,dist_mv,pwm_limit,fault\n");
    if(trace_file) {
        run_lever(trace_file,summary,1916.5,1.724,1.,1478,50,0,0,0,5,25);
        run_lever(trace_file,summary,1916.5,1.724,.77,1478,50,0,0,0,5,25);
        run_lever(trace_file,summary,1916.5,4.,1.,1478,50,0,0,0,5,25);
        run_lever(trace_file,summary,1916.5,1.724,1.,1522,50,0,0,0,5,25);
        run_lever(trace_file,summary,1916.5,1.724,1.,1478,50,0,6,0,5,25);
        run_lever(trace_file,summary,1916.5,1.724,1.,1478,50,1,0,0,5,25);
        run_lever(trace_file,summary,1916.5,1.724,.77,1478,50,0,0,0,1,25);
        run_lever(trace_file,summary,1916.5,1.724,.77,1478,50,0,0,0,1,10);
        run_lever(trace_file,summary,1916.5,1.724,.77,1478,50,0,0,0,1,20);
        disturbance(trace_file,summary,1.724,1.,0);
        disturbance(trace_file,summary,1.724,.77,0);
        disturbance(trace_file,summary,1.724,.77,6);
    } else {
        for(int load=500;load<=3500;load+=500)
        for(int j=0;j<4;j++)
        for(int b=0;b<2;b++)
        for(int direction=0;direction<2;direction++) {
            double inertia[]={1.,1.724,4.,10.};
            run_lever(NULL,summary,load,inertia[j],b?.77:1.,direction?1522:1478,50,0,0,0,5,25);
        }
        for(int ibw=1;ibw<=5;ibw+=2) for(int stiff=5;stiff<=25;stiff+=10)
            run_lever(NULL,summary,1916.5,1.724,.77,1478,50,0,0,0,ibw,stiff);
    }
    fclose(summary);if(trace_file) fclose(trace_file);return 0;
}
