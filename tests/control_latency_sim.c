/* Host audit: compile current 30kg production sources, stub hardware only.
 * No firmware source is changed. Plant: delayed voltage, Coulomb/static
 * friction, calibrated first order motor, 50us integration. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CONFIG_IMAGE_TAG() 1234U
#include "../CH32V00X_APP_ADC_30kg/Application/A_Config.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Motor_Model.c"
#ifndef AUDIT_POS_CTRL_SOURCE
#define AUDIT_POS_CTRL_SOURCE "../CH32V00X_APP_ADC_30kg/Common/C_Pos_Ctrl.c"
#endif
#include AUDIT_POS_CTRL_SOURCE
#include "../CH32V00X_APP_ADC_30kg/Common/C_Traj_Planner.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Speed_Observer.c"
#include "../CH32V00X_APP_ADC_30kg/Application/A_Servo.c"
#include "../CH32V00X_APP_ADC_30kg/Application/A_UartCmd.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Ring_Buf.c"
static uint32_t __get_MSTATUS(void) { return 0; }
static void __disable_irq(void) {}
static void __set_MSTATUS(uint32_t n) { (void)n; }
#include "../CH32V00X_APP_ADC_30kg/Common/C_Task_Scheduler.c"

static double plant_p, plant_v, load_mv, fs_scale=1;
static double inertia_scale=1, brake_scale=MOTOR_BRAKE_PCT/100.;
static int plant_duty, invalid_position, tick_no, noise_amp;
static int voltage_cv=767, plant_enabled=1;
static double history[128];
static unsigned history_idx;
static RingBuf_t rx;
static unsigned bytes_dropped, tx_frames;
static double tx_end_us;
static unsigned sim_baud=115200;
static double now_us;
static int emulate_halfduplex;
static const MotorCal_t cal={
    .kv_uv=MOTOR_KV_UV,.ka_nv=MOTOR_KA_NV,.fc_mv=MOTOR_FC_MV,
    .fs_mv=MOTOR_FS_MV,.delay_us=MOTOR_DELAY_US,.cal_mv=MOTOR_CAL_MV,
    .noise_cdeg=MOTOR_NOISE_CDEG,.min_step_cdeg=MOTOR_MIN_STEP_CDEG,
    .sign=MOTOR_SIGN,.brake_pct=MOTOR_BRAKE_PCT,
    .rated_vel_dps=MOTOR_RATED_VEL_DPS,.rated_acc_dps2=MOTOR_RATED_ACC_DPS2,
    .rated_dec_dps2=MOTOR_RATED_DEC_DPS2};
uint8_t IAP_HwInfoRead(IAP_HwInfo_t *p) { memset(p,0,sizeof(*p));return 0; }
uint8_t IAP_PreservedIdentityRead(uint8_t *a,uint8_t *b) { *a=0;*b=5;return 1; }
uint16_t FLASH_Config_Load(void *p,uint16_t n,uint8_t v[3]) { (void)p;(void)n;(void)v;return 0; }
uint8_t FLASH_Config_Save(const void *p,uint16_t n,const uint8_t v[3]) { (void)p;(void)n;(void)v;return 1; }
int32_t A_Encoder_Read(void) { return invalid_position?ENCODER_ANGLE_ERROR:(int32_t)lround(plant_p)+ (noise_amp?((tick_no*37)% (2*noise_amp+1))-noise_amp:0); }
uint16_t A_Voltage_Read(void) { return voltage_cv; }
int16_t A_Temperature_Read(void) { return 250; }
#ifdef AUDIT_REAL_PROTECT
uint8_t A_Current_Read(uint16_t *p) {
    double u=plant_duty*voltage_cv*10.0/2400;
    double emf=cal.kv_uv/1000.*plant_v;
    *p=(uint16_t)fmin(5290,100+fabs(u-emf)*(TORQUE_STALL_CURRENT_MA-100)/MOTOR_CAL_MV);
    return (uint8_t)(abs(plant_duty)>3);
}
#include "../CH32V00X_APP_ADC_30kg/Application/A_Protect.c"
#else
uint8_t A_Protect_CanDrive(void) { return 1; }
uint16_t A_Protect_Fault(void) { return 0; }
uint8_t A_Protect_Stalled(void) { return 0; }
uint16_t A_Protect_GetTorque(void) { return 3000; }
uint16_t A_Protect_GetCurrent(void) { return 0; }
uint8_t A_Protect_SetTorque(uint16_t n) { (void)n;return 1; }
void A_Protect_GetTelemetry(ProtectTelemetry_t *p) { memset(p,0,sizeof(*p)); }
#endif
int16_t D_Motor_Set(int16_t p) { plant_duty=p;return p; }
void D_Motor_Release(uint8_t n) { (void)n;plant_duty=0; }
void D_UART_Enable(uint8_t n) { (void)n; }
void D_PWM_Input_Enable(uint8_t n) { (void)n; }
uint16_t D_PWM_Read(void) { return 0; }
void Delay_Ms(uint32_t n) { (void)n; }
uint8_t D_UART1_Tx_Write(const uint8_t *p,uint8_t n) {
    (void)p;tx_frames++;
    if(emulate_halfduplex) tx_end_us=fmax(tx_end_us,now_us)+ n*10000000.0/sim_baud;
    return 1;
}
void D_UART_SetBaud_Deferred(uint32_t n) { (void)n; }
uint8_t D_UART1_Rx_Get(uint8_t *p) { return C_Ring_Buf_Get(&rx,p); }
void D_UART_Service(void) {}
uint8_t D_UART_Tx_Idle(void) { return now_us>=tx_end_us; }
uint8_t IAP_UpgradePending(void) { return 0; }
uint8_t IAP_Rx_Deal(uint8_t n) { (void)n;return 0; }
void IAP_Rx_Reset(void) {}

static void plant_tick(void) {
    if(!plant_enabled) return;
    for(int j=0;j<20;j++) {
        history[history_idx]=plant_duty*voltage_cv*10.0/2400;
        double u=history[(history_idx+128-64)%128]; /* 3.2ms transport delay */
        history_idx=(history_idx+1)%128;
        if(u*plant_v<0) u*=brake_scale;
        u-=load_mv;
        if(fabs(plant_v)<1e-8 && fabs(u)<=cal.fs_mv*fs_scale) plant_v=0;
        else {
            double sign=fabs(plant_v)<1e-8?(u>0?1:-1):(plant_v>0?1:-1);
            double next=plant_v+(u-cal.kv_uv/1000.*plant_v-cal.fc_mv*sign)/(cal.ka_nv/1000000.*inertia_scale)*.00005;
            if(next*plant_v<0 && fabs(u)<=cal.fs_mv*fs_scale) next=0;
            plant_v=next;
        }
        plant_p+=plant_v*.00005;
    }
}
static void step(void) { tick_no++;now_us+=1000;A_Servo_Control();plant_tick();
#ifdef AUDIT_REAL_PROTECT
    if(tick_no%10==0) A_Protect_Task();
#endif
}
static void reset(void) {
    plant_p=13500;plant_v=0;plant_duty=0;load_mv=0;fs_scale=1;
    inertia_scale=1;brake_scale=MOTOR_BRAKE_PCT/100.;
    invalid_position=noise_amp=0;plant_enabled=1;voltage_cv=767;
    tick_no=0;now_us=tx_end_us=0;bytes_dropped=tx_frames=0;
    history_idx=0;memset(history,0,sizeof(history));C_Ring_Buf_Init(&rx);
    s_wire=s_text_len=s_bin_len=s_wire_idle_ms=0;
    C_MotorModel_Init(&cal);A_Config_Init();
    g_config.boot_mode=SERVO_BOOT_HOLD;
    A_Servo_Init();
#ifdef AUDIT_REAL_PROTECT
    A_Protect_Init();
#endif
    for(int i=0;i<500;i++) step();
}
static void command(int pwm,int time_ms) {
    char f[40];snprintf(f,sizeof(f),"#000P%04dT%04d!",pwm,time_ms);
    for(char *p=f;*p;p++) assert(C_Ring_Buf_Put(&rx,*p));
    A_Uart_Process();
}
static void trace(FILE *f,const char *name,int t) {
    fprintf(f,"%s,%d,%d,%d,%.3f,%d,%d,%d,%d,%d,%d,%d,%.3f\n",name,t,s_servo.target_angle,s_servo.ref.pos,plant_p-s_servo.offset_cdeg,s_servo.ref.vel,s_servo.obs.vel,plant_duty,s_servo.dbg.u_stick,s_servo.dbg.state,s_tj.f_on,s_servo.dbg.sat,plant_v);
}
static void latency(FILE *f,const char *name,int pwm,int duration,int prior_age) {
    reset();
    if(prior_age>=0) { command(1500,1);for(int i=0;i<prior_age;i++) step(); }
    double start=plant_p;
    command(pwm,duration);
    int first_u=-1, first_ref=-1, first_move=-1;double maxerr=0;
    int initial_ref=s_servo.ref.pos;
    for(int i=1;i<=2500;i++) {
        step();trace(f,name,i);
        if(first_u<0 && abs(plant_duty)>=1) first_u=i;
        if(first_ref<0 && s_servo.ref.pos!=initial_ref) first_ref=i;
        if(first_move<0 && fabs(plant_p-start)>=13) first_move=i;
        if(fabs((plant_p-s_servo.offset_cdeg)-s_servo.ref.pos)>maxerr) maxerr=fabs((plant_p-s_servo.offset_cdeg)-s_servo.ref.pos);
    }
    printf("%s: u=%dms ref=%dms move_0.13deg=%dms final_error=%.3fdeg max_tracking=%.3fdeg follow=%u\n",name,first_u,first_ref,first_move,((plant_p-s_servo.offset_cdeg)-s_servo.target_angle)/100,maxerr/100,s_tj.f_on);
}
static int control_calls, uart_calls, scheduler_ticks, clock_us, control_cost, uart_cost;
static void advance_clock(int duration) {
    int until=clock_us+duration;
    while((clock_us/1000+1)*1000<=until) {
        clock_us=(clock_us/1000+1)*1000;C_Task_Tick_Update();scheduler_ticks++;
    }
    clock_us=until;
}
static void slow_control(void) { control_calls++;advance_clock(control_cost); }
static void uart_count(void) { uart_calls++;advance_clock(uart_cost); }

static void retarget(FILE *f,int age,int dest,int force_plan) {
    reset();command(1200,0);for(int i=0;i<age;i++) step();
    double start=plant_p;if(force_plan) s_servo.move_age=0xFFFFU;
    command(dest,0);int change=-1,reverse=-1,ref_reverse=-1;
    int sign=(s_servo.target_angle-(plant_p-s_servo.offset_cdeg))>0?1:-1;
    char name[64];snprintf(name,sizeof(name),"retarget_age%d_to%d%s",age,dest,force_plan?"_force_plan":"");
    int follow=s_tj.f_on;
    for(int i=1;i<=1500;i++) {
        step();trace(f,name,i);
        if(change<0 && (plant_p-start)*sign>=13) change=i;
        if(reverse<0 && plant_v*sign>=378) reverse=i;
        if(ref_reverse<0 && s_servo.ref.vel*sign>=378) ref_reverse=i;
    }
    printf("%s: follow=%d axis_reversed=%dms ref_reversed=%dms displacement_toward_new_target_0.13deg=%dms final_error=%.3fdeg\n",name,follow,reverse,ref_reverse,change,((plant_p-s_servo.offset_cdeg)-s_servo.target_angle)/100);
}

int main(int argc,char **argv) {
    FILE *f=fopen(argc>1?argv[1]:"control_latency_trace.csv","w");assert(f);
    fprintf(f,"case,t_ms,target_cdeg,ref_cdeg,actual_cdeg,ref_cdps,obs_cdps,pwm,stick_mv,ctrl_state,follow,sat,actual_cdps\n");
    latency(f,"single_1deg_fast",1493,0,-1);
    latency(f,"single_10deg_fast",1426,0,-1);
    latency(f,"single_1deg_1000ms",1493,1000,-1);
    latency(f,"second_1deg_age100",1493,0,100);
    latency(f,"second_1deg_age151",1493,0,151);
    latency(f,"second_1deg_t1000_age100",1493,1000,100);
    for(int age=20;age<=200;age+=30) retarget(f,age,1600,0);
    retarget(f,20,1600,1);

    reset();plant_enabled=0;command(1000,0);
    for(int i=0;i<20;i++) step();
    int old=s_servo.ref.pos;command(900,0);int follow=s_tj.f_on;
    for(int i=1;i<=400;i++) { step();trace(f,"retarget_unfinished",i); }
    printf("retarget_unfinished: follow=%d old_ref=%d new_ref_400ms=%d target=%u\n",follow,old,s_servo.ref.pos,s_servo.target_angle);

    /* hold must freeze whole absolute reference, including moving target/FIR. */
    C_Traj_Init(10000);C_Traj_SetLimits(14500,767200,619200,43123000);C_Traj_SetAccelMinMs(40);
    C_Traj_Follow(15000,100);TrajRef_t r;C_Traj_Step(&r,0);old=r.pos;
    for(int i=0;i<100;i++) C_Traj_Step(&r,1);
    printf("follow_hold: pos_before=%d after_100_hold=%d delta=%d\n",old,r.pos,r.pos-old);
    assert(r.pos>old+100); /* Confirm current regression. */

    /* Low A + clipped FIR: observed acceleration must respect SetLimits. */
    C_Traj_Init(10000);C_Traj_SetLimits(10000,10000,10000,1000000);C_Traj_Follow(20000,100);
    int maxa=0;for(int i=0;i<500;i++) { C_Traj_Step(&r,0);if(abs(r.acc)>maxa) maxa=abs(r.acc); }
    printf("low_acc_follow: requested_acc=10000 measured_peak=%d FIR=%u/%u\n",maxa,s_tj.fir_n[0],s_tj.fir_n[1]);
    assert(maxa>10000);

    Task_t tasks[]={TASK_DEF(slow_control,1,0),TASK_DEF(uart_count,1,1)};
    for(control_cost=200;control_cost<=2000;control_cost+=600) {
        C_TASK_INIT(tasks);control_calls=uart_calls=scheduler_ticks=clock_us=0;uart_cost=100;
        while(clock_us<1000000) { C_Task_Run();advance_clock(1); }
        printf("scheduler_control_cost_%dus: control_calls=%d uart_calls=%d tick_count=%d\n",control_cost,control_calls,uart_calls,scheduler_ticks);
        assert(control_calls>0 && uart_calls>0);
    }
    reset();g_config.pulse_lo=1000;int previous=s_servo.target_angle;
    uint8_t data[4],baud=0;Bin_Put16(data,900);Bin_Put16(data+2,0);
    int status=Bin_ParamWrite(PARAM_MOTION_REQUEST,data,4,&baud);
    printf("binary_rejected_pulse: status=%d target_before=%d target_after=%u\n",status,previous,s_servo.target_angle);
    assert(status==BIN_ST_OK && s_servo.target_angle==previous);
    fclose(f);return 0;
}
