#define main original_claude_main
#include "sim.c"
#undef main
static void knobs(void){
    CX_STILL_MS=200;CX_EMF_PCT=100;CX_STREAM=1;CX_CORR=1;CX_J_X10=20;CX_J_FF=0;
    CX_DOB_BW=30;CX_DOB_GATE=1;CX_DOB_GATE_MV=100;
}
static void controller_init(void){
    C_PosCtrl_Init(&cal,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);PosCtrlGains_t g={20,TUNE_VEL_BW_RADS,5};
    C_PosCtrl_SetGains(&g);C_PosCtrl_SetHold(50,80,0);
}
int main(void){
    knobs();Plant pl=base_plant();sim_reset(&pl,0,0);controller_init();
    PosCtrlFeedback_t fb={.pos=13500,.vel=(int32_t)((int64_t)cal.cal_mv*1100/cal.kv_uv),.valid=1};
    TrajRef_t ref={13500,0,0,1};PosCtrlDbg_t dbg;
    int result=C_PosCtrl_Update(&ref,&fb,100,&dbg);
    assert(abs(result)>100);
    printf("UNIT,emf_empty_intersection,kg=%d,hard_limit=100,output_mv=%d,observed_velocity=%d\n",FW_TORQUE,result,fb.vel);
    controller_init();
    fb=(PosCtrlFeedback_t){.pos=10000,.vel=1000,.valid=1,.dist=cal.cal_mv*3/10,.stuck=0};
    ref=(TrajRef_t){.pos=10300,.vel=1000,.acc=0,.done=0};
    for(int t=0;t<3000;t++){fb.pos++;ref.pos++;C_PosCtrl_Update(&ref,&fb,7670,&dbg);}
    fb.vel=0;ref=(TrajRef_t){.pos=fb.pos,.vel=0,.acc=0,.done=1};
    for(int t=0;t<400;t++)C_PosCtrl_Update(&ref,&fb,7670,&dbg);
    int hold_i=s_ctrl.integral_q16>>16,hold_u=dbg.u_out;
    ref.done=0;C_PosCtrl_Update(&ref,&fb,7670,&dbg);
    printf("UNIT,dob_handoff,kg=%d,hold_integral=%d,half_budget=%d,hold_output=%d,move_output=%d\n",FW_TORQUE,hold_i,7670/2,hold_u,dbg.u_out);
    // A fresh motion can still report Reached from the previous motion.
    printf("UNIT,reached_during_move,kg=%d,state=%d,reached=%d\n",FW_TORQUE,s_ctrl.state,C_PosCtrl_Reached());
    knobs();sim_reset(&pl,0,0);load_fn=0;g_kind=0;g_holdki=-1;
    cmd(13600,0);int found=0;
    for(int t=0;t<1000;t++){
        sim_tick();
        if(s_servo.arrived && s_ctrl.state==PC_SETTLE && abs(s_servo.ref.pos-s_servo.angle)>=2){
            int oldref=s_servo.ref.pos,actual=s_servo.angle;A_Servo_Stop();
            printf("UNIT,stop_during_reported_settle,kg=%d,tick=%d,old_ref=%d,actual=%d,ref_after_stop=%d\n",FW_TORQUE,t,oldref,actual,s_servo.ref.pos);
            assert(s_servo.ref.pos==oldref);found=1;break;
        }
    }
    printf("UNIT,stop_probe_found,kg=%d,found=%d\n",FW_TORQUE,found);
    puts("PASS: boundary probes executed; reported violations are candidate defects, not repair success");return 0;
}
