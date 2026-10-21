#define main original_sim_main
#include "../control_optimization_20261019/source/sim.c"
#undef main

static void controller_init(const MotorCal_t *motor)
{
    C_PosCtrl_Init(motor,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);
    PosCtrlGains_t g={TUNE_POS_BW_RADS,TUNE_VEL_BW_RADS,TUNE_INT_BW_RADS,TUNE_LOAD_RATIO_X10};
    C_PosCtrl_SetGains(&g);C_PosCtrl_SetHold(50,80,0);
}
int main(void)
{
    PosCtrlFeedback_t fb={.pos=13500,.vel=0,.valid=1};
    TrajRef_t ref={.pos=13500,.vel=0,.acc=0,.done=1};PosCtrlDbg_t dbg;
    controller_init(&cal);
    for(int t=0;t<400;t++)C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(C_PosCtrl_Reached()&&C_PosCtrl_ArrivalQualified());
    /* A drift outside the configured window must regain active damping; first output matches HOLD. */
    fb.pos=13560;
    int32_t expected=Fx_MulShift(s_gain.kh_q16,-10,16);
    int32_t u=C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(dbg.state==PC_SETTLE&&s_ctrl.hold_blend_ms==HOLD_BLEND_MS);
    assert(u==expected&&C_PosCtrl_ArrivalQualified());
    for(int t=0;t<100;t++){u=C_PosCtrl_Update(&ref,&fb,100,&dbg);assert(abs(u)<=100);}
    fb.pos=13500;
    for(int t=0;t<600;t++)C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(C_PosCtrl_Reached());
    fb.pos=13600;fb.valid=0;C_PosCtrl_Update(&ref,&fb,cal.cal_mv,&dbg);
    assert(C_PosCtrl_Reached()); /* Do not classify drift from an invalid reading. */
    puts("PASS drift_recapture_bumpless_first_output_arrival_latch_and_invalid_sensor");

    int fs[]={0,1,37,486,3000};fb.pos=13500;fb.vel=0;fb.valid=1;
    for(unsigned i=0;i<sizeof(fs)/sizeof(fs[0]);i++){
        MotorCal_t motor=cal;motor.fs_mv=fs[i];controller_init(&motor);
        int32_t previous=0,step=(fs[i]+STICK_SLEW_MS-1)/STICK_SLEW_MS;
        for(int t=0;t<100;t++){
            int demand=t<30?1:t<60?-1:0;
            int32_t now=PosCtrl_Stiction(&fb,demand,0);
            assert(abs(now-previous)<=step);previous=now;
        }
        assert(previous==0);
        C_PosCtrl_Reset();assert(s_ctrl.stick==0&&s_ctrl.vel_error_q8==0);
    }
    puts("PASS stiction_start_reverse_release_slew_bound_and_reset_including_small_fs");
    return 0;
}
