/* Recompile this TU against EACH unmodified production project. Hardware only is emulated. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "A_Config.h"
static uint8_t flash_mem[65536];
#define FLASH_BASE ((uintptr_t)flash_mem)
#define FLASH_COMPLETE 0
static int cut_after=-1, flash_bytes, erase_count, write_count;
static int flash_byte(uint32_t address,uint8_t v) {
    if(cut_after>=0 && flash_bytes++>=cut_after) return 0;
    *(uint8_t *)(uintptr_t)address=v; return 1;
}
static int FLASH_ROM_ERASE(uint32_t a,uint32_t n) {
    erase_count++;for(uint32_t i=0;i<n;i++)if(!flash_byte(a+i,255))return 1;return 0;
}
static int FLASH_ROM_WRITE(uint32_t a,const uint32_t *p,uint32_t n) {
    write_count++;for(uint32_t i=0;i<n;i++)if(!flash_byte(a+i,((const uint8_t *)p)[i]))return 1;return 0;
}
#define CONFIG_IMAGE_TAG() 1234U
#include "flash.c"
#include "A_Config.c"
#include "C_Motor_Model.c"
#include "C_Pos_Ctrl.c"
#include "C_Traj_Planner.c"
#include "C_Speed_Observer.c"
#include "A_Sensor.c"
#include "A_Servo.c"
#include "A_Protect.c"
#include "A_UartCmd.c"
#include "C_Ring_Buf.c"
static uint32_t __get_MSTATUS(void){return 0;}
static void __disable_irq(void){}
static void __set_MSTATUS(uint32_t n){(void)n;}
#define s_tasks sched_tasks
#include "C_Task_Scheduler.c"
#undef s_tasks

static const MotorCal_t cal={.kv_uv=MOTOR_KV_UV,.ka_nv=MOTOR_KA_NV,.fc_mv=MOTOR_FC_MV,
 .fs_mv=MOTOR_FS_MV,.delay_us=MOTOR_DELAY_US,.cal_mv=MOTOR_CAL_MV,.noise_cdeg=MOTOR_NOISE_CDEG,
 .min_step_cdeg=MOTOR_MIN_STEP_CDEG,.sign=MOTOR_SIGN,.rated_vel_dps=MOTOR_RATED_VEL_DPS,
 .rated_acc_dps2=MOTOR_RATED_ACC_DPS2,.rated_dec_dps2=MOTOR_RATED_DEC_DPS2};
static double pos,vel,load,ka_scale=1,kv_scale=1,fs_scale=1,delay_scale=1;
static double hist[1024], adc_mean;
static unsigned hist_i,rng=123;
static int duty,voltage=7670,adc_bad,noise_amp,ms,blocked,current_override=-1,temp=250;
static int run_protect=1,low_release,current_settle,prev_phase;
static uint8_t tx[128],tx_len;
static RingBuf_t rx;
uint8_t IAP_HwInfoRead(IAP_HwInfo_t *p){memset(p,0,sizeof(*p));return 0;}
uint8_t IAP_PreservedIdentityRead(uint8_t *a,uint8_t *b){*a=0;*b=5;return 1;}
uint8_t IAP_UpgradePending(void){return 0;}
uint8_t IAP_Rx_Deal(uint8_t x){(void)x;return 0;}
void IAP_Rx_Reset(void){}
void D_UART_Enable(uint8_t x){(void)x;}
void D_PWM_Input_Enable(uint8_t x){(void)x;}
uint16_t D_PWM_Read(void){return 0;}
void Delay_Ms(uint32_t x){(void)x;}
uint8_t D_UART1_Tx_Write(const uint8_t *p,uint8_t n){assert(n<=sizeof(tx));memcpy(tx,p,n);tx_len=n;return 1;}
void D_UART_SetBaud_Deferred(uint32_t x){(void)x;}
uint8_t D_UART1_Rx_Get(uint8_t *p){return C_Ring_Buf_Get(&rx,p);}
void D_UART_Service(void){}
uint8_t D_UART_Tx_Idle(void){return 1;}
int16_t D_TMP112_Read_Temp(void){return temp;}
int16_t D_Motor_Set(int16_t x){
    int ph=abs(x)>=D_ADC_CUR_MIN_TICKS?(x>0?1:-1):0;
    if(ph!=prev_phase)current_settle=1;
    prev_phase=ph;
    duty=x;low_release=0;return x;
}
void D_Motor_Release(uint8_t h){duty=0;low_release=!h;}
uint16_t D_ADC_Voltage_Read(void){return (uint16_t)lround(voltage*4096./16170.);}
uint8_t D_ADC_Encoder_Read(uint16_t *p){
    if(adc_bad==2)return 0;
    if(adc_bad==1){*p=0;return 1;}
    rng=rng*1664525U+1013904223U;
    double n=noise_amp?((int)((rng>>8)%(2*noise_amp+1))-noise_amp):0;
    double raw=(ENCODER_POT_ADC_MIN+(adc_mean+n)*ENCODER_POT_ADC_SPAN/ENCODER_POT_SPAN_CDEG)*16;
    *p=(uint16_t)fmin(65520,fmax(0,lround(raw)));return 1;
}
uint8_t D_ADC_Current_Read(int32_t *p){
    int ma=current_override>=0?current_override:(int)fmin(5290,100+fabs(duty*voltage/2400.-cal.kv_uv/1000.*vel)*1900/MOTOR_CAL_MV);
    *p=(int32_t)((int64_t)ma*65536/CURRENT_FULL_SCALE_MA);
    return current_override>=0?1:(abs(duty)>=D_ADC_CUR_MIN_TICKS && !current_settle);
}
static void physics(void){
    double sum=0;unsigned delay=(unsigned)lround(cal.delay_us*delay_scale/50.);
    for(int k=0;k<20;k++){
        hist[hist_i]=duty*voltage/2400.*MOTOR_SIGN;
        double u=hist[(hist_i+1024-delay)%1024]-load;hist_i=(hist_i+1)%1024;
        if(blocked){vel=0;}
        else if(fabs(vel)<1e-9 && fabs(u)<=cal.fs_mv*fs_scale)vel=0;
        else {
            double sign=fabs(vel)<1e-9?(u>0?1:-1):(vel>0?1:-1);
            double next=vel+(u-cal.kv_uv/1000.*kv_scale*vel-cal.fc_mv*sign)/(cal.ka_nv/1e6*ka_scale)*.00005;
            if(next*vel<0 && fabs(u)<=cal.fs_mv*fs_scale)next=0;
            vel=next;
        }
        pos+=vel*.00005;sum+=pos;
    }
    adc_mean=sum/20;
}
static void tick(void){ms++;A_Servo_Control();physics();if(run_protect && ms%10==0)A_Protect_Task();current_settle=0;}
static void ticks(int n){while(n-->0)tick();}
static void reset(void){
    memset(flash_mem,255,sizeof(flash_mem));cut_after=-1;flash_bytes=erase_count=write_count=0;s_dirty=0;s_save_event=0;
    pos=adc_mean=13500;vel=load=duty=0;voltage=7670;adc_bad=noise_amp=ms=blocked=0;hist_i=0;rng=123;
    ka_scale=kv_scale=fs_scale=delay_scale=1;current_override=-1;temp=250;run_protect=1;
    low_release=current_settle=prev_phase=0;memset(hist,0,sizeof(hist));C_Ring_Buf_Init(&rx);s_wire=0;
    A_Config_Init();g_config.boot_mode=SERVO_BOOT_HOLD;g_config.servo_mode=SERVO_MODE_270_CCW;
    C_MotorModel_Init(&cal);A_Servo_Init();A_Protect_Init();ticks(300);
}
static void command(int p,int t){
 if(t>9999){uint8_t f[13]={0xaa,0x66,g_config.servo_id,3,5,PARAM_MOTION_REQUEST};
 Bin_Put16(f+6,p);Bin_Put16(f+8,t);f[10]=Bin_Crc8(f+2,8);f[11]=0x66;f[12]=0xaa;
 for(unsigned i=0;i<sizeof(f);i++)assert(C_Ring_Buf_Put(&rx,f[i]));A_Uart_Process();return;}
 char f[32];snprintf(f,sizeof(f),"#%03dP%04dT%04d!",g_config.servo_id,p,t);
 for(char *c=f;*c;c++)assert(C_Ring_Buf_Put(&rx,*c));
 A_Uart_Process();}
static uint8_t param(uint8_t id,uint32_t value,uint8_t n){uint8_t b[4],baud=0;Bin_PutN(b,value,n);return Bin_ParamWrite(id,b,n,&baud);}
static void text(const char *s){while(*s)Wire_Byte((uint8_t)*s++);}
static FILE *trace;
static void row(const char *name,int t){if(t%5)return;fprintf(trace,"%s,%d,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",name,t,pos,vel,s_servo.target_angle,s_servo.ref.pos,s_servo.ref.vel,s_servo.ref.acc,s_servo.obs.vel,s_servo.dbg.u_out,duty,s_ctrl.state,s_prot.fault);}
static void scenario(const char *name,int kind,int t_cmd,int noisy,int mismatch){
    reset();noise_amp=noisy;if(mismatch){ka_scale=2;kv_scale=1.2;fs_scale=1.5;delay_scale=1.5;}
    int changed=0,first_hold=-1,maxdu=0,lastu=0,transdu=0,laststate=s_ctrl.state;
    double track=0,obs_err=0,tailmin=1e9,tailmax=-1e9;
    if(kind==4)load=1200;
    if(kind==5)voltage=6200;
    if(kind==6)g_runtime_config.position_resolution_cdeg=CFG_RESOLUTION_MIN_CDEG;
    if(kind==9)A_Servo_SetMode(SERVO_MODE_270_CW);
    command(kind==1?1510:2200,t_cmd);
    int duration=t_cmd>10000?t_cmd+4000:10000;
    for(int t=0;t<duration;t++){
        if(kind==2 && t<3000 && t%20==0){command(1500+(t/20)%51,t_cmd);changed=t;first_hold=-1;}
        if(kind==3 && t<3000 && t%250==0){command((t/250)%2?800:2200,t_cmd);changed=t;first_hold=-1;}
        if((kind==7 || kind==8) && t==2000){load=1200;if(kind==8)param(PARAM_RT_HOLD_KI,80,2);}
        tick();row(name,t);
        int du=abs(s_servo.dbg.u_out-lastu);if(du>maxdu)maxdu=du;
        if(laststate!=s_ctrl.state && du>transdu)transdu=du;
        lastu=s_servo.dbg.u_out;laststate=s_ctrl.state;
        if(t>changed && first_hold<0 && s_servo.arrived)first_hold=t;
        double e=fabs(pos-s_servo.ref.pos-s_servo.offset_cdeg);if(e>track)track=e;
        double ve=fabs(vel-s_servo.obs.vel);if(ve>obs_err)obs_err=ve;
        if(t>=duration-1000){tailmin=fmin(tailmin,pos);tailmax=fmax(tailmax,pos);}
    }
    printf("SCENARIO,%s,error_deg=%.5f,tail_pp_deg=%.5f,max_tracking_deg=%.5f,max_observer_error_dps=%.4f,max_du_mv=%d,transition_du_mv=%d,last_command_arrived_ms=%d,final_state=%u,fault=%u,stalled=%u,target=%u\n",name,(pos-s_servo.offset_cdeg-s_servo.target_angle)/100,(tailmax-tailmin)/100,track/100,obs_err/100,maxdu,transdu,first_hold<0?-1:first_hold-changed,s_ctrl.state,s_prot.fault,s_prot.stalled,s_servo.target_angle);
}
static void probes(void){
    reset();A_Servo_Release(0);text("#000PCLE!");ticks(30);
    printf("PROBE,reset_after_manual_release,rt_mode=%u,torque_on=%u,work=%u\n",g_runtime_config.torque_mode,A_Servo_TorqueOn(),A_Servo_GetWorkState());
    reset();command(2200,0);ticks(150);int oldpos=s_servo.ref.pos,oldv=s_servo.ref.vel,oldu=s_servo.dbg.u_out;
    text("#000PCLE!");tick();printf("PROBE,reset_while_moving,ref_jump=%d,vel_before=%d,vel_after=%d,u_before=%d,u_after=%d\n",s_servo.ref.pos-oldpos,oldv,s_servo.ref.vel,oldu,s_servo.dbg.u_out);
    reset();voltage=0;A_Servo_Init();voltage=7670;ticks(1000);command(1800,0);ticks(2000);
    printf("PROBE,zero_bus_at_init,to_duty=%d,duty=%d,position=%.3f,work=%u,fault=%u\n",s_servo.to_duty_q16,duty,pos,A_Servo_GetWorkState(),s_prot.fault);
    reset();param(PARAM_VOLTAGE_MAX,62,1);voltage=5800;ticks(70);voltage=6100;ticks(2000);
    printf("PROBE,tight_voltage_window,fault=%u,recover_lo=%u,recover_hi=%u\n",s_prot.fault,(g_runtime_config.voltage_min_dv+2)*10,(g_runtime_config.voltage_max_dv-2)*10);
    reset();command(2200,0);ticks(200);int oldT=s_tj.T,oldt=s_tj.t;param(PARAM_RT_TRAJ_VMAX,1,2);ticks(10);
    printf("PROBE,live_vmax,new_limit=%d,current_ref_vel=%d,old_plan_T=%d,new_plan_T=%d,old_t=%d\n",s_tj.vmax_cdps,s_servo.ref.vel,oldT,s_tj.T,oldt);
    reset();command(2200,0);ticks(20);int prior_a=s_servo.ref.acc;
    assert(param(PARAM_RT_TRAJ_AMAX,1,4)==0 && param(PARAM_RT_TRAJ_DMAX,1,4)==0 && param(PARAM_RT_TRAJ_SMOOTH,200,2)==0);
    command(1800,0);tick();printf("PROBE,live_lower_limits_retarget,prior_a=%d,next_a=%d,pre_T=%u,new_J=%d,new_A=%d\n",prior_a,s_servo.ref.acc,s_tj.pre_T,s_tj.jmax_cdpss3,s_tj.amax_cdpss);
    reset();C_PosCtrl_Reset();PosCtrlGains_t gains={20,TUNE_VEL_BW_RADS,5};C_PosCtrl_SetGains(&gains);C_PosCtrl_SetHold(50,80,0);
    PosCtrlFeedback_t fb={10000,300,1};TrajRef_t r={10030,0,0,1};PosCtrlDbg_t dbg;
    s_ctrl.state=PC_SETTLE;s_ctrl.cap_ms=19;s_ctrl.integral_q16=1000<<16;
    int before=C_PosCtrl_Update(&r,&fb,7670,&dbg);printf("PROBE,settle_to_hold,u_after=%d,state=%u,previous_law_fb_mv=%d\n",before,s_ctrl.state,(int)(((int64_t)s_gain.kv_q16*(20*(30-cal.min_step_cdeg)-(300-CTRL_VEL_NOISE_CDPS/2)))>>16));
    s_ctrl.state=PC_HOLD;s_ctrl.hold_p=2000;s_ctrl.integral_q16=3000<<16;fb.valid=0;r.done=0;r.vel=0;r.acc=0;
    C_PosCtrl_Update(&r,&fb,7670,&dbg);printf("PROBE,invalid_feedback_hold_move,state=%u,integral_mv=%d\n",s_ctrl.state,s_ctrl.integral_q16>>16);
    reset();blocked=1;param(PARAM_RT_TRAJ_VMAX,1,2);command(1800,4000);ticks(6000);
    printf("PROBE,slow_stall,stalled=%u,state=%u,error_deg=%.3f,ref_done=%u,fault=%u\n",s_prot.stalled,s_ctrl.state,(s_servo.target_angle-pos)/100,s_servo.ref.done,s_prot.fault);
    reset();voltage=5800;ticks(70);A_Servo_Release(0);voltage=7670;ticks(600);
    printf("PROBE,manual_release_during_fault,torque_on=%u,rt_mode=%u\n",A_Servo_TorqueOn(),g_runtime_config.torque_mode);
    reset();current_override=4000;g_runtime_config.prot_current_time_ms=100;
    for(int i=0;i<300;i++){if(i%20==0)param(PARAM_RT_TORQUE_LIMIT,(i/20)%2?1000:1100,2);tick();}
    printf("PROBE,changing_torque_overcurrent,fault=%u,current_counter=%u\n",s_prot.fault,s_prot.cur_cnt);
    reset();g_runtime_config.hold_ki=0;s_ctrl.state=PC_HOLD;s_ctrl.integral_q16=3000<<16;
    fb=(PosCtrlFeedback_t){10000,0,1};r=(TrajRef_t){10000,0,0,1};C_PosCtrl_Update(&r,&fb,100,&dbg);
    printf("PROBE,lower_output_limit,integral_mv=%d,limit_mv=100\n",s_ctrl.integral_q16>>16);
    reset();run_protect=0;g_runtime_config.position_resolution_cdeg=500;Servo_HoldParams();
    s_ctrl.state=PC_HOLD;s_ctrl.integral_q16=0;fb=(PosCtrlFeedback_t){10000,0,1};r=(TrajRef_t){10400,0,0,1};
    int u0=C_PosCtrl_Update(&r,&fb,7670,&dbg);param(PARAM_RT_RESOLUTION,CFG_RESOLUTION_MIN_CDEG,2);Servo_HoldParams();
    int u1=C_PosCtrl_Update(&r,&fb,7670,&dbg);printf("PROBE,live_precision,u_before=%d,u_after=%d,jump_mv=%d\n",u0,u1,u1-u0);
    reset();run_protect=0;uint8_t partial[]={0xaa,0x66,0,3,32};
    for(unsigned i=0;i<sizeof(partial);i++)C_Ring_Buf_Put(&rx,partial[i]);A_Uart_Process();command(1800,0);
    printf("PROBE,partial_frame,target_pwm=%u,wire_state=%u\n",Servo_AngleToPwm(s_servo.target_angle),s_wire);
    reset();command(1500,0);ticks(200);adc_bad=1;ticks(6);
    printf("PROBE,encoder_lost,enc_state=%u,duty=%d,torque_on=%u\n",s_servo.enc_state,duty,A_Servo_TorqueOn());
    reset();current_override=4000;ticks(40);printf("PROBE,baseline_overcurrent,fault=%u,torque_on=%u\n",s_prot.fault,A_Servo_TorqueOn());
    reset();temp=600;ticks(260);printf("PROBE,baseline_overtemp,fault=%u,torque_on=%u\n",s_prot.fault,A_Servo_TorqueOn());
    reset();blocked=1;param(PARAM_RT_TRAJ_VMAX,1,2);command(1510,4000);ticks(15000);
    printf("PROBE,slow_small_stall,stalled=%u,error_deg=%.3f,ref_done=%u,state=%u,output_mv=%d\n",s_prot.stalled,(s_servo.target_angle-pos)/100,s_servo.ref.done,s_ctrl.state,s_servo.dbg.u_out);
}
static void flash_tests(void){
    reset();param(PARAM_HOLD_KP,30,2);assert(A_Config_SaveTask());A_Config_Init();assert(g_config.hold_kp==30);
    param(PARAM_RT_HOLD_KP,5,2);assert(g_config.hold_kp==30 && g_runtime_config.hold_kp==5);A_Config_Init();assert(g_runtime_config.hold_kp==30);
    uint8_t snapshot[512];memcpy(snapshot,flash_mem+0xf400,512);int old=0,new=0;
    for(int n=0;n<=1024;n++){
        memcpy(flash_mem+0xf400,snapshot,512);g_config.hold_kp=60;cut_after=n;flash_bytes=0;
        FLASH_Config_Save(&g_config,sizeof(g_config),k_version);cut_after=-1;
        Config_t c;uint8_t v[3];assert(FLASH_Config_Load(&c,sizeof(c),v)==sizeof(c));
        assert(c.hold_kp==30 || c.hold_kp==60);if(c.hold_kp==30)old++;else new++;
    }
    printf("FLASH,power_cut_positions=1025,old_recovered=%d,new_recovered=%d,invalid=0\n",old,new);
    reset();A_Config_MarkDirty();A_Config_SaveTask();int writes=write_count;param(PARAM_HOLD_KP,80,2);A_Config_SaveTask();printf("FLASH,identical_write,page_writes=%d\n",write_count-writes);
    reset();volatile uint8_t event=0;A_Config_BindSaveEvent(&event);param(PARAM_HOLD_KP,60,2);cut_after=0;event=0;A_Config_SaveTask();assert(s_dirty && event);cut_after=-1;assert(A_Config_SaveTask());A_Config_BindSaveEvent(0);
    puts("FLASH,save_failure_retry=PASS,flash_vs_sram=PASS,same_image_reload=PASS");
}
static void trajectory_fuzz(void){
    reset();run_protect=0;int maxdv=0,maxda=0,maxjump=0,range=0,maxt=0,olda=0,newa=0;TrajRef_t r={13500,0,0,1},prev=r;
    for(int t=0;t<200000;t++){
        if(t%7==0){rng=rng*1664525+1013904223;C_Traj_Plan(5000+(rng%17000),(t%3)?0:5000);}
        C_Traj_Step(&r,0);
        if(abs(r.vel-prev.vel)>maxdv)maxdv=abs(r.vel-prev.vel);
        if(abs(r.acc-prev.acc)>maxda){maxda=abs(r.acc-prev.acc);maxt=t;olda=prev.acc;newa=r.acc;}
        if(abs(r.pos-prev.pos)>maxjump)maxjump=abs(r.pos-prev.pos);
        if(r.pos<0 || r.pos>27000)range++;
        prev=r;
    }
    printf("TRAJECTORY,retarget_7ms_200s,max_dv_cdps=%d,max_da_cdpss=%d,max_position_step_cdeg=%d,range_violations=%d,worst_t=%d,old_a=%d,new_a=%d,j_limit=%d\n",maxdv,maxda,maxjump,range,maxt,olda,newa,s_tj.jmax_cdpss3);
    C_Traj_Init(13500);A_Servo_ApplyTrajConfig();
    s_tj.vel_qt=100*65536/1000;s_tj.acc_now=s_tj.jmax_cdpss3/1000-1;
    int old=s_tj.acc_now;C_Traj_Plan(5000,0);C_Traj_Step(&r,0);
    printf("TRAJECTORY,prefix_skipped,old_acc=%d,new_acc=%d,delta_acc=%d,allowed_delta_acc=%d,pre_T=%u\n",old,r.acc,abs(r.acc-old),s_tj.jmax_cdpss3/1000,s_tj.pre_T);
}
static unsigned control_calls,uart_calls,sim_time_us;
static void cost(unsigned duration){unsigned end=sim_time_us+duration;
 while((sim_time_us/1000+1)*1000<=end){sim_time_us=(sim_time_us/1000+1)*1000;C_Task_Tick_Update();}sim_time_us=end;}
static void cost_control(void){control_calls++;cost(100);}
static void cost_uart(void){uart_calls++;cost(100);}
static void cost_flash(void){cost(4000);}
static void scheduler_probe(void){
    Task_t tasks[]={TASK_DEF(cost_control,1,0),TASK_DEF(cost_uart,1,1),TASK_DEF(cost_flash,0,4)};
    C_TASK_INIT(tasks);control_calls=uart_calls=sim_time_us=0;tasks[2].run=1;
    while(sim_time_us<10000){C_Task_Run();cost(10);}
    printf("SCHEDULER,hypothetical_4ms_block,elapsed_us=%u,control_calls=%u,uart_calls=%u\n",sim_time_us,control_calls,uart_calls);
    C_Task_Init(0,0);
}
static void parameter_matrix(void){
    unsigned checked=0;
    for(unsigned i=0;i<sizeof(k_params)/sizeof(k_params[0]);i++){
        if(k_params[i].min==0 && k_params[i].max==0 && !(k_params[i].flags&PF_TMAX))continue;
        reset();unsigned id=k_params[i].id,n=k_params[i].size;
        uint32_t max=(k_params[i].flags&PF_TMAX)?A_Config_TorqueMax():k_params[i].max;
        assert(param(id,max,n)==BIN_ST_OK);
        uint8_t b[20],len=0;assert(Bin_ParamRead(id,b,&len)==BIN_ST_OK && len==n+1);
        assert(Param_Load(b+1,n)==max);
        if(max+1 < (n==1?256ULL:n==2?65536ULL:4294967296ULL))assert(param(id,max+1,n)==BIN_ST_VALUE);
        if(k_params[i].min>1)assert(param(id,k_params[i].min-1,n)==BIN_ST_VALUE);
        assert(param(id,k_params[i].min,n)==BIN_ST_OK);
        assert(param(id,max,n==1?2:1)==BIN_ST_LENGTH);
        checked++;
    }
    reset();g_config.servo_id=23;g_config.baud_code=6;
    text("#023PCLE0!");assert(g_config.servo_id==23 && g_config.baud_code==5);assert(A_Config_SaveTask());
    A_Config_Init();assert(g_config.servo_id==23 && g_config.hold_kp==CFG_DEF_HOLD_KP);
    text("#023PCLE!");assert(A_Config_SaveTask());A_Config_Init();assert(g_config.servo_id==0);
    printf("PARAMETERS,generic_boundary_matrix=%u,real_flash_reset_reload=PASS\n",checked);
}
int main(int argc,char **argv){
    assert(argc==2);trace=fopen(argv[1],"w");assert(trace);
    fprintf(trace,"scenario,t_ms,pos_cdeg,true_vel_cdps,target_cdeg,ref_cdeg,ref_vel_cdps,ref_acc_cdpss,obs_vel_cdps,u_mv,duty,state,fault\n");
    printf("CAL,kg=%u,kv=%d,ka=%d,fc=%d,fs=%d,delay=%u,tau_ms=%.4f,vel_noise=%d,config_size=%u\n",FW_TORQUE,cal.kv_uv,cal.ka_nv,cal.fc_mv,cal.fs_mv,cal.delay_us,cal.ka_nv*1./cal.kv_uv,CTRL_VEL_NOISE_CDPS,(unsigned)sizeof(Config_t));
    scenario("step_T0",0,0,0,0);scenario("step_T5000",0,5000,0,0);
    scenario("small_T0",1,0,6,0);scenario("small_T5000",1,5000,6,0);
    scenario("stream_T0",2,0,6,0);scenario("stream_T5000",2,5000,6,0);
    scenario("reverse_T0",3,0,6,0);scenario("reverse_T5000",3,5000,6,0);
    scenario("load_1200mV",4,0,6,0);scenario("low_bus_6.2V",5,0,6,0);
    scenario("min_resolution",6,0,6,0);scenario("mismatch_step",0,0,6,1);scenario("mismatch_reverse",3,0,6,1);
    scenario("load_after_arrival_KI0",7,0,6,0);scenario("load_after_arrival_KI80",8,0,6,0);
    scenario("step_T65535",0,65535,6,0);scenario("small_T65535",1,65535,6,0);scenario("stream_T65535",2,65535,6,0);
    scenario("default_direction",9,0,6,0);
    probes();flash_tests();trajectory_fuzz();scheduler_probe();parameter_matrix();fclose(trace);return 0;
}
