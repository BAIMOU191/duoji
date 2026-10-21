/* Additional current-code invariant probes, separate from load experiments. */
#define main latency_audit_main
#include "control_latency_sim.c"
#undef main
int main(void) {
    TrajRef_t r;
    C_Traj_Init(10000);C_Traj_SetLimits(10000,1000000,1000000,100000000);
    C_Traj_Follow(20000,100);
    for(int i=0;i<30;i++) C_Traj_Step(&r,0);
    unsigned old=s_tj.fir[0].n;
    C_Traj_SetLimits(10000,10000,10000,1000000);
    C_Traj_Follow(10000,100);
    int maxa=0;
    for(int i=0;i<100;i++) { C_Traj_Step(&r,0);maxa=abs(r.acc)>maxa?abs(r.acc):maxa; }
    printf("active Follow limits change: active FIR=%u new requested FIR=%u old FIR=%u peak_acc=%d vs requested 10000 cdeg/s2\n",s_tj.fir[0].n,s_tj.fir_n[0],old,maxa);
    assert(s_tj.fir[0].n==old && maxa>10000);

    C_Traj_Init(10000);C_Traj_SetLimits(10000,1000000,10000,100000000);
    C_Traj_Follow(25000,500);
    for(int i=0;i<50;i++) C_Traj_Step(&r,0);
    C_Traj_Follow(0,500);int maxdec=0,maxjerk=0,last=r.acc;
    for(int i=0;i<100;i++) {
        C_Traj_Step(&r,0);if(-r.acc>maxdec) maxdec=-r.acc;
        int j=abs(r.acc-last)*1000;if(j>maxjerk) maxjerk=j;last=r.acc;
    }
    printf("Follow reversal: requested D=10000 actual_peak_decel=%d cdeg/s2; requested J=100000000 actual_peak_J=%d cdeg/s3\n",maxdec,maxjerk);
    assert(maxdec>10000);

    reset();g_config.pulse_lo=1000;g_config.pulse_hi=2000;
    uint8_t data[4],baud=0;Bin_Put16(data,900);Bin_Put16(data+2,0);
    assert(Bin_ParamWrite(PARAM_MOTION_REQUEST,data,4,&baud)==BIN_ST_OK);
    ServoTelemetry_t t;A_Servo_GetTelemetry(&t);assert(t.target_pwm==1500);
    printf("ACK confirmed: configured pulse 1000..2000 rejects 900 but binary reports OK.\n");

    /* Binary half-frame holds parser exclusive; later text frame is swallowed. */
    reset();uint8_t partial[]={0xaa,0x66,0,3,32};
    for(unsigned i=0;i<sizeof(partial);i++) C_Ring_Buf_Put(&rx,partial[i]);
    A_Uart_Process();
    command(1478,0);A_Servo_GetTelemetry(&t);assert(t.target_pwm==1500);
    for(int i=0;i<10;i++) A_Uart_Process();
    command(1478,0);A_Servo_GetTelemetry(&t);assert(t.target_pwm==1478);
    printf("Partial binary frame confirmed: text command swallowed until 10 UART task calls without bytes reset the parser.\n");

    /* Contract says invalid feedback freezes holding state. MOVE bypasses it. */
    C_PosCtrl_Init(&cal,CTRL_VEL_NOISE_CDPS,TUNE_OBS_BW_HZ);
    PosCtrlGains_t gains={20,120,5};C_PosCtrl_SetGains(&gains);C_PosCtrl_SetHold(50,25,0);
    PosCtrlFeedback_t fb={.pos=10000,.vel=0,.valid=1};r=(TrajRef_t){.pos=10000,.done=1};
    for(int i=0;i<30;i++) C_PosCtrl_Update(&r,&fb,7670,NULL);
    assert(s_ctrl.state==PC_HOLD);s_ctrl.loaded=1;fb.valid=0;r.done=0;
    C_PosCtrl_Update(&r,&fb,7670,NULL);
    printf("invalid-feedback contract: HOLD -> state=%u loaded=%u while feedback invalid.\n",s_ctrl.state,s_ctrl.loaded);
    assert(s_ctrl.state==PC_MOVE && !s_ctrl.loaded);
    return 0;
}
