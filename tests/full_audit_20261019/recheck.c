/* Follow-up: assert public-path failures AND counterexamples; never change production. */
#define main original_audit_main
#include "audit.c"
#undef main

static void reset_entry(unsigned entry){
    if(entry<2){text(entry?"#000PCLE!":"#000PCLE0!");return;}
    uint8_t f[10]={0xaa,0x66,0,entry<4?(entry==2?7:8):3,entry<4?0:2,PARAM_SYSTEM_ACTION,entry==4?2:3};
    unsigned n=f[4];f[5+n]=Bin_Crc8(f+2,3+n);f[6+n]=0x66;f[7+n]=0xaa;
    for(unsigned i=0;i<8+n;i++)Wire_Byte(f[i]);
}
static void reset_matrix(void){
    unsigned mismatch=0,normal=0,blocked_fault=0;
    for(unsigned e=0;e<6;e++)for(unsigned release=0;release<3;release++){
        reset();if(release)A_Servo_Release(release==2);reset_entry(e);ticks(30);
        assert(g_runtime_config.torque_mode==1 && s_prot.fault==0);
        if(release){assert(!A_Servo_TorqueOn());mismatch++;}
        else{assert(A_Servo_TorqueOn());normal++;}
    }
    for(unsigned e=0;e<6;e++){
        reset();voltage=5800;ticks(70);assert(!A_Servo_TorqueOn());reset_entry(e);ticks(30);
        assert(s_prot.fault && !A_Servo_TorqueOn());blocked_fault++;
    }
    printf("RECHECK,reset_matrix,manual_release_mismatches=%u,normal_controls=%u,active_fault_blocks=%u\n",mismatch,normal,blocked_fault);
}
static void detector_matrix(void){
    for(unsigned path=0;path<4;path++){
        reset();current_override=4000;g_runtime_config.prot_current_time_ms=100;
        for(int i=0;i<150;i++){
            if(i%20==0){
                if(path==0)param(PARAM_RT_TORQUE_LIMIT,(i/20)%2?1000:1100,2);
                if(path==1)param(PARAM_TORQUE_LIMIT,(i/20)%2?1000:1100,2);
                if(path==2)param(PARAM_RT_HOLD_KP,(i/20)%2?30:40,2);
            }tick();
        }
        assert(!!(s_prot.fault&PROT_FAULT_OVERCUR)==(path!=0));
        printf("RECHECK,overcurrent_path,path=%u,fault=%u,count=%u\n",path,s_prot.fault,s_prot.cur_cnt);
    }
    for(unsigned path=0;path<2;path++){
        reset();temp=600;g_runtime_config.prot_temp_time_ms=1000;
        for(int i=0;i<2000;i++){
            if(i%100==0)param(path?PARAM_RT_HOLD_KP:PARAM_RT_TORQUE_LIMIT,(i/100)%2?40:50,2);
            tick();
        }
        assert(!!(s_prot.fault&PROT_FAULT_OVERTEMP)==(path==1));
        printf("RECHECK,hot_sensor_path,unrelated_hold_kp=%u,fault=%u,temp_count=%u\n",path,s_prot.fault,s_prot.temp_cnt);
    }
}
static void release_matrix(void){
    for(unsigned fault=0;fault<2;fault++)for(unsigned user=0;user<3;user++){
        reset();if(fault){voltage=5800;ticks(70);}
        if(user)A_Servo_Release(user==2);
        voltage=7670;ticks(600);
        assert(A_Servo_TorqueOn()==(fault || !user));
        printf("RECHECK,release_ownership,fault_history=%u,user_release=%u,torque_on=%u\n",fault,user,A_Servo_TorqueOn());
    }
}
static void public_controller(void){
    reset();run_protect=0;
    C_PosCtrl_Init(&cal,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);
    PosCtrlGains_t g={20,TUNE_VEL_BW_RADS,5};C_PosCtrl_SetGains(&g);C_PosCtrl_SetHold(50,80,0);
    TrajRef_t r={10000,0,0,1};PosCtrlFeedback_t fb={10000,0,1};PosCtrlDbg_t d;
    for(int i=0;i<20;i++)C_PosCtrl_Update(&r,&fb,7670,&d);
    assert(C_PosCtrl_Reached());
    fb.pos=9200;int before=C_PosCtrl_Update(&r,&fb,7670,&d);int oldi=d.u_i;
    r.pos=fb.pos;r.done=0;int after=C_PosCtrl_Update(&r,&fb,7670,&d);
    assert(before>after && oldi==0);
    printf("RECHECK,public_hold_to_move,u_before=%d,u_after=%d,loss_mv=%d\n",before,after,before-after);
    C_PosCtrl_Reset();r=(TrajRef_t){10030,0,0,1};fb=(PosCtrlFeedback_t){10000,300,1};
    for(int i=0;i<19;i++)before=C_PosCtrl_Update(&r,&fb,7670,&d);
    assert(d.state==PC_SETTLE);int fb_term=d.u_fb;
    after=C_PosCtrl_Update(&r,&fb,7670,&d);assert(d.state==PC_HOLD);
    printf("RECHECK,public_settle_to_hold,u_before=%d,u_after=%d,old_feedback=%d\n",before,after,fb_term);
    fb.pos=9200;fb.vel=0;C_PosCtrl_Update(&r,&fb,7670,&d);
    int integral_before=s_ctrl.integral_q16;fb.valid=0;r.done=0;r.pos=9200;
    C_PosCtrl_Update(&r,&fb,7670,&d);
    assert(d.state==PC_MOVE && s_ctrl.integral_q16!=integral_before);
    printf("RECHECK,public_invalid_feedback,state=%u,integral_changed=1\n",d.state);
    C_PosCtrl_Reset();C_PosCtrl_SetHold(500,80,0);r=(TrajRef_t){10000,0,0,1};fb=(PosCtrlFeedback_t){10000,0,1};
    for(int i=0;i<20;i++)C_PosCtrl_Update(&r,&fb,7670,&d);
    fb.pos=9600;before=C_PosCtrl_Update(&r,&fb,7670,&d);C_PosCtrl_SetHold(CFG_RESOLUTION_MIN_CDEG,80,0);
    after=C_PosCtrl_Update(&r,&fb,7670,&d);assert(after-before>2000);
    printf("RECHECK,public_precision_change,u_before=%d,u_after=%d\n",before,after);
}
static void long_hold(void){
    for(unsigned ki=0;ki<=80;ki+=80){
        reset();noise_amp=6;command(2200,0);ticks(2000);assert(s_servo.arrived);
        load=1200;param(PARAM_RT_HOLD_KI,ki,2);ticks(8000);
        double e10=(pos-s_servo.target_angle)/100;int i10=s_ctrl.integral_q16;
        ticks(50000);double e60=(pos-s_servo.target_angle)/100;
        printf("RECHECK,load_long_hold,ki=%u,error_at_10s=%.5f,error_at_60s=%.5f,integral_10s=%d,integral_60s=%d\n",ki,e10,e60,i10>>16,s_ctrl.integral_q16>>16);
        if(ki==0)assert(fabs(e60)>2);
    }
    reset();noise_amp=6;ka_scale=2;kv_scale=1.2;fs_scale=1.5;delay_scale=1.5;command(2200,0);
    int first=-1;double captured=0;
    for(int t=0;t<10000;t++){tick();if(first<0 && s_servo.arrived){first=t;captured=(pos-s_servo.target_angle)/100;}}
    printf("RECHECK,mismatch_capture,first_ms=%d,true_error_at_capture=%.5f,true_error_10s=%.5f\n",first,captured,(pos-s_servo.target_angle)/100);
}
static void bounds_and_recovery(void){
    unsigned accepted=0,empty=0,single=0;
    for(int lo=60;lo<=90;lo++)for(int hi=60;hi<=90;hi++)if(lo+2<=hi){
        reset();assert(param(PARAM_VOLTAGE_MIN,lo,1)==BIN_ST_OK);
        assert(param(PARAM_VOLTAGE_MAX,hi,1)==BIN_ST_OK);
        Config_t stored=g_config;Config_FillDefaults();Config_Repair(&stored);
        assert(stored.voltage_min_dv==lo && stored.voltage_max_dv==hi);
        accepted++;if(lo+2>hi-2)empty++;else if(lo+2==hi-2)single++;
    }
    assert(accepted==435 && empty==57 && single==27);
    printf("RECHECK,voltage_domain,accepted=%u,empty_recovery=%u,single_point_recovery=%u\n",accepted,empty,single);
    for(unsigned good=0;good<2;good++){
        reset();if(!good)voltage=0;A_Servo_Init();voltage=7670;ticks(600);command(1800,0);unsigned goal=s_servo.target_angle;ticks(3000);
        assert((duty==0 && s_servo.to_duty_q16==0)==!good);
        printf("RECHECK,bus_initialization,valid_at_boot=%u,to_duty=%d,submitted_goal=%u,final_goal=%u,work_state=%u,stalled=%u\n",good,s_servo.to_duty_q16,goal,s_servo.target_angle,A_Servo_GetWorkState(),s_prot.stalled);
    }
    reset();adc_bad=1;int start=ms;while(s_servo.enc_state!=SERVO_ENC_FAULT && ms-start<5000)tick();
    assert(s_servo.enc_state==SERVO_ENC_FAULT && ms-start==4505);
    printf("RECHECK,encoder_persistent_loss,elapsed_ms=%d,torque_on=%u\n",ms-start,A_Servo_TorqueOn());
    reset();uint8_t partial[]={0xaa,0x66,0,3,32};for(unsigned i=0;i<sizeof(partial);i++)C_Ring_Buf_Put(&rx,partial[i]);
    A_Uart_Process();command(1800,0);assert(Servo_AngleToPwm(s_servo.target_angle)==1500);
    for(int i=0;i<10;i++)A_Uart_Process();command(1800,0);assert(Servo_AngleToPwm(s_servo.target_angle)==1800);
    puts("RECHECK,partial_frame,swallowed_before_timeout=1,recovered_after_timeout=1");
}
static void jerk_check(void){
    reset();run_protect=0;TrajRef_t r={13500,0,0,1},prev=r;int maxda=0,violations=0;
    for(int t=0;t<200000;t++){
        if(t%7==0){rng=rng*1664525U+1013904223U;C_Traj_Plan(5000+rng%17000,(t%3)?0:5000);}
        C_Traj_Step(&r,0);int da=abs(r.acc-prev.acc);if(da>maxda)maxda=da;
        if(da>s_tj.jmax_cdpss3/1000+2)violations++;prev=r;
    }
    printf("RECHECK,jerk_retarget,max_delta_acc=%d,allowed=%d,violations=%d\n",maxda,s_tj.jmax_cdpss3/1000,violations);
#ifdef CANDIDATE_N1
    assert(maxda>s_tj.jmax_cdpss3/1000+2 && maxda<s_tj.jmax_cdpss3/1000*101/100);
#else
    assert(maxda>s_tj.jmax_cdpss3/1000*19/10);
#endif
}
static void stall_matrix(void){
    for(unsigned slow=0;slow<2;slow++)for(unsigned small=0;small<2;small++){
        reset();blocked=1;if(slow)param(PARAM_RT_TRAJ_VMAX,1,2);
        command(small?1510:2200,small?4000:0);ticks(15000);
        if(!slow && !small)assert(s_prot.stalled);
        if(slow && small)assert(!s_prot.stalled && s_servo.ref.done && s_ctrl.state==PC_SETTLE);
        printf("RECHECK,stall_matrix,slow=%u,small=%u,stalled=%u,ref_done=%u,state=%u,output_mv=%d\n",slow,small,s_prot.stalled,s_servo.ref.done,s_ctrl.state,s_servo.dbg.u_out);
    }
}
int main(void){
    printf("RECHECK,variant,kg=%u\n",FW_TORQUE);
    reset_matrix();detector_matrix();release_matrix();public_controller();long_hold();bounds_and_recovery();jerk_check();stall_matrix();
    puts("PASS: defect and control assertions; candidate residual jerk violations are expected and reported");return 0;
}
