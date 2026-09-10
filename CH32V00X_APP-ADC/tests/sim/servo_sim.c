/* Host harness: production servo, sensor, planner, observer and controller.
 * Only ADC, PWM capture and H-bridge are replaced. Units: cdeg, seconds, PWM. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "A_Servo.c"

Config_t g_config;
static double plant_pos, plant_vel, gain=1, tau_scale=1, friction_scale=1;
static double noise=4.5, load_pwm;
static int applied, tick, dropout, delay_ticks=2;
static int history[16], hist_idx;
static uint16_t input;
static uint8_t simulated_fault;
static unsigned rng=1;
static double random_unit(void) { rng=1664525U*rng+1013904223U; return (rng>>8)/16777216.0; }
uint8_t D_ADC_Encoder_Read(uint16_t *q) {
    double p=plant_pos + noise*(2*random_unit()-1);
    if(dropout || p < -1000 || p > 28000) { *q=64; return 1; }
    *q=(uint16_t)lround((350+p*3400/27000)*16); return 1;
}
uint16_t D_ADC_Voltage_Read(void) {return 2000;}
uint8_t D_ADC_Current_Read(uint16_t *v) {(void)v;return 0;}
int16_t D_TMP112_Read_Temp(void) {return 250;}
uint16_t D_MT6701_Read_Angle(void) {return (uint16_t)((int)lround(plant_pos+36000)%36000);}
int16_t D_Motor_Set(int16_t p) { applied=p; return p; }
void D_Motor_Release(uint8_t h) {(void)h;applied=0;}
void D_PWM_Input_Enable(uint8_t en) {(void)en;}
uint16_t D_PWM_Read(void) {uint16_t p=input;input=0;return p;}
void D_UART_Enable(uint8_t en) {(void)en;}
uint8_t D_UART1_Tx_Write(const uint8_t *b,uint8_t n) {(void)b;(void)n;return 1;}
void Delay_Ms(uint32_t ms) {(void)ms;}
void A_Config_MarkDirty(void) {}
#ifndef SIM_REAL_PROTECT /* Stall tests link the real A_Protect.c instead. */
uint8_t A_Protect_Fault(void) {return simulated_fault;}
#endif

static void setup(double p, int pwm_mode) {
    simulated_fault=0;
    memset(&g_config,0,sizeof(g_config));
    g_config.servo_mode=1;g_config.boot_mode=SERVO_BOOT_HOLD;
    plant_pos=p;plant_vel=0;applied=0;tick=0;input=pwm_mode?1500:0;
    hist_idx=0;memset(history,0,sizeof(history));A_Servo_Init();
}
static void step(void) {
    A_Servo_Control();
    history[hist_idx]=applied*CTRL_MOTOR_SIGN;
    int u=history[(hist_idx+16-delay_ticks)%16];hist_idx=(hist_idx+1)%16;
    double force=u-load_pwm;
    double fs=(force>=0?180:236)*friction_scale;
    double fc=57*friction_scale;
    if(fabs(plant_vel)<1 && fabs(force)<fs) plant_vel=0;
    else {
        double sign=plant_vel==0?(force>=0?1:-1):(plant_vel>0?1:-1);
        double a=(65536.0/4716*gain*(force-fc*sign)-plant_vel)/(.052*tau_scale);
        double next=plant_vel+a*.001;
        if(next*plant_vel<0 && fabs(force)<fs) next=0;
        plant_vel=next;
    }
    plant_pos+=plant_vel*.001;tick++;
}
static int failures;
#define CHECK(test) do { if (!(test)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#test); failures++; } } while(0)
static int regression(void) {
    noise=0;
    int samples[]={-1000,-500,-1,0,1,27000,28000};
    setup(13500,0);
    for(unsigned i=0;i<sizeof(samples)/sizeof(samples[0]);i++) {
        plant_pos=samples[i];CHECK(Servo_ReadPosition());
        CHECK(abs((int)s_servo.angle_circ-samples[i])<=1);
        CHECK(s_servo.angle==(samples[i]<0?0:samples[i]>27000?27000:samples[i]));
    }
    SpeedObsOut_t obs;C_SpeedObs_Init(-500);
    for(int k=0;k<100;k++)C_SpeedObs_Update(-500,0,&obs);
    CHECK(abs(obs.pos+500)<=2);CHECK(abs(obs.vel)<=10);
    TrajRef_t ref={20250,0,0};obs.pos=0;obs.vel=0;obs.load=0;
    C_PosCtrl_Init();CHECK(C_PosCtrl_Update(&ref,&obs,1,0)*CTRL_MOTOR_SIGN>0);

    setup(13500,0);A_Servo_Submit(2000,0);
    for(int k=0;k<50;k++)step();
    A_Servo_Pause();CHECK(s_servo.target_angle==20250);A_Servo_Resume();
    CHECK(!C_Traj_Is_Done());
    A_Servo_Pause();A_Servo_Submit(2000,0);CHECK(!C_Traj_Is_Done());

    setup(13500,0);plant_pos=13400;CHECK(A_Servo_CalibrateMid());
    CHECK(Servo_Offset()==-100);CHECK(s_servo.angle_circ==13500);
    plant_pos=1000;CHECK(!A_Servo_CalibrateMid());CHECK(Servo_Offset()==-100);
    plant_pos=10000;CHECK(A_Servo_SetTravelEnd(1));
    CHECK(Servo_Offset()==10000);CHECK(s_servo.span_cdeg==16900);
    CHECK(Servo_ReadPosition());CHECK(abs((int)s_servo.angle_circ)<=1);

    setup(-1100,0);A_Servo_Submit(2000,1000);
    for(int k=0;k<4000;k++)step();
    CHECK(s_servo.enc_state==SERVO_ENC_OK);CHECK(s_servo.target_angle==20250);
    CHECK(fabs(plant_pos-20250)<100);

    setup(13500,0);A_Servo_Submit(2000,0);plant_pos=-1100;
    for(int k=0;k<4000;k++)step();
    CHECK(s_servo.enc_state==SERVO_ENC_OK);CHECK(s_servo.target_angle==20250);
    CHECK(fabs(plant_pos-20250)<100);

    setup(-1100,0);plant_pos=-900;
    int saw_fault=0;
    for(int k=0;k<4600;k++) {dropout=k%2;A_Servo_Control();if(s_servo.enc_state==SERVO_ENC_FAULT)saw_fault=1;}
    CHECK(saw_fault);CHECK(applied==0);CHECK(!A_Servo_TorqueOn());dropout=0;

    setup(13500,1);A_Servo_Submit(1500,0);
    for(int k=0;k<1000;k++) {if(k%20==0)input=(k%40==0?1497:1503);step();}
    CHECK(fabs(plant_pos-13500)<1);CHECK(applied==0);CHECK(C_Traj_Is_Done());
    input=2000;step();CHECK(s_servo.target_angle==13500); /* 单帧毛刺不能生效 */
    input=1500;step();input=1500;step();CHECK(s_servo.target_angle==13500);
    input=2000;step();input=2000;step();CHECK(s_servo.target_angle==20250);

    setup(13500,0);A_Servo_SetOutputLimit(300);A_Servo_Submit(500,0);
    for(int k=0;k<300;k++) {step();CHECK(abs(applied)<=300);}
    printf("regression failures=%d\n",failures);return failures?1:0;
}
int main(int argc,char **argv) {
    const char *mode=argc>1?argv[1]:"cycle";
    if(strcmp(mode,"regression")==0)return regression();
    if(argc>2)gain=atof(argv[2]);
    if(argc>3)tau_scale=atof(argv[3]);
    if(argc>4)friction_scale=atof(argv[4]);
    if(argc>5)noise=atof(argv[5]);
    if(argc>6)delay_ticks=atoi(argv[6]);
    setup(strcmp(mode,"boot-low")==0?-1100:strcmp(mode,"boot-high")==0?28100:13500,strcmp(mode,"jitter")==0);
    if(argc>7) {g_config.servo_mode=(uint8_t)atoi(argv[7]);A_Servo_ApplyConfig();}
    if(strcmp(mode,"coords")==0) {
        noise=0;
        int samples[]={-1000,-500,-1,0,1,27000,28000};
        for(unsigned i=0;i<sizeof(samples)/sizeof(samples[0]);i++) {
            plant_pos=samples[i];Servo_ReadPosition();
            printf("%d,%ld,%u\n",samples[i],(long)s_servo.angle_circ,s_servo.angle);
        }return 0;
    }
    puts("ms,pwm_target,pos,vel,ref_pos,ref_vel,ref_acc,obs_pos,obs_vel,pwm,done,state,enc_state,error,integral,acc_ff");
    int cmd=1500;
    for(int k=0;k<12000;k++) {
        if(strcmp(mode,"cycle")==0 || strcmp(mode,"rapid")==0) {
            int period=strcmp(mode,"rapid")==0?450:3000;
            if(k%period==0) {cmd=(k/period)%2?2000:500;A_Servo_Submit(cmd,0);}
        }
        if(strcmp(mode,"jitter")==0 && k%20==0)input=(uint16_t)(1500+(int)(random_unit()*7)-3);
        if(strcmp(mode,"disturb")==0 && k==3000)plant_pos+=300;
        if(strcmp(mode,"load")==0 && k==3000)load_pwm=300;
        if(strcmp(mode,"unload")==0 && k==3000)load_pwm=300;
        if(strcmp(mode,"unload")==0 && k==7000)load_pwm=0;
        step();
        printf("%d,%d,%.4f,%.3f,%ld,%ld,%ld,%ld,%ld,%d,%u,%u,%u,%ld,%d,%d\n",k,cmd,plant_pos,plant_vel,(long)s_servo.ref.pos,(long)s_servo.ref.vel,(long)s_servo.ref.acc,(long)s_servo.obs.pos,(long)s_servo.obs.vel,applied,C_Traj_Is_Done(),s_servo.dbg.state,s_servo.enc_state,(long)s_servo.dbg.e_pos,s_servo.dbg.u_i,s_servo.dbg.u_acc);
    }
    return 0;
}
