#define main audit_original_main
#include "sim.c"
#undef main

static void init_controller(void)
{
    C_PosCtrl_Init(&cal,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);
    PosCtrlGains_t gains={TUNE_POS_BW_RADS,TUNE_VEL_BW_RADS,TUNE_INT_BW_RADS,TUNE_LOAD_RATIO_X10};
    C_PosCtrl_SetGains(&gains);C_PosCtrl_SetHold(50,30,0);
}

int main(void)
{
    PosCtrlFeedback_t fb={.pos=13500,.vel=0,.valid=1};
    TrajRef_t ref={.pos=13500,.vel=0,.acc=0,.done=1};
    PosCtrlDbg_t dbg;
    /* A positive but sub-Q16 inertia must not turn into a zero divisor. */
    MotorCal_t tiny=cal;tiny.ka_nv=1;
    C_PosCtrl_Init(&tiny,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);
    PosCtrlGains_t clipped={20,100,5,65535U};
    assert(C_PosCtrl_SetGains(&clipped));
    assert(clipped.load_ratio_x10==POS_CTRL_LOAD_RATIO_MAX_X10 && clipped.pos_bw>=1U);
    puts("PASS small_positive_inertia_and_load_ratio_bounds");
    int tests=0;
    int limits[]={0,1,10,100,500,2000,cal.cal_mv};
    for(unsigned k=0;k<sizeof(limits)/sizeof(limits[0]);k++){
        for(int v=-125;v<=125;v+=5){
            init_controller();fb.vel=(int32_t)((int64_t)cal.cal_mv*1000*v/cal.kv_uv/100);
            for(int direction=-1;direction<=1;direction+=2){
                ref.pos=fb.pos+direction*1000;
                int u=C_PosCtrl_Update(&ref,&fb,limits[k],&dbg);
                assert(abs(u)<=limits[k]);assert(dbg.u_out==u);tests++;
            }
        }
    }
    printf("PASS hard_limit_grid cases=%d\n",tests);
    init_controller();fb.vel=0;ref.pos=fb.pos;
    for(int t=0;t<30;t++)C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(C_PosCtrl_ArrivalQualified());assert(!C_PosCtrl_Reached());
    for(int t=0;t<300;t++)C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(C_PosCtrl_Reached());C_PosCtrl_BeginMotion();assert(!C_PosCtrl_ArrivalQualified());
    C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(C_PosCtrl_Reached());assert(C_PosCtrl_ArrivalQualified());
    C_PosCtrl_BeginMotion();assert(!C_PosCtrl_ArrivalQualified());
    ref.done=0;C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(!C_PosCtrl_Reached());assert(!C_PosCtrl_ArrivalQualified());
    puts("PASS arrival_event_separate_from_holding_and_fresh_motion");
    puts("PASS zero_distance_target_while_already_holding");
    Plant pl=base_plant();sim_reset(&pl,0,0);load_fn=0;g_kind=0;
    cmd(13600,0);int found=0;
    for(int t=0;t<2000;t++){
        sim_tick();
        if(s_servo.arrived && !C_PosCtrl_Reached() && abs(s_servo.ref.pos-s_servo.angle)>=2){
            A_Servo_Stop();assert(s_servo.ref.pos==s_servo.angle);found=1;break;
        }
    }
    assert(found);puts("PASS stop_resynchronizes_while_arrival_event_already_set");
#if 1
    sim_reset(&pl,0,0);load_fn=0;
    for(int t=0;t<1100;t++){
        if(t%200==0)cmd(13500+50*(t/200+1),0);
        sim_tick();
    }
    assert(s_stream.consistent>=2U);
    A_Servo_Pause();assert(s_stream.consistent==0U);
    A_Servo_Resume();assert(s_stream.consistent==0U);
    A_Servo_Stop();assert(s_stream.consistent==0U);
    cmd(14000,2000);assert(s_stream.consistent==0U);
    puts("PASS stream_invalidated_on_pause_resume_stop_and_explicit_time");
#endif
    return 0;
}
