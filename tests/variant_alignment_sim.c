/* Hardware-boundary simulation of the actual per-variant config/protocol/control chain.
 * ALIGN_TEST_CALIBRATED substitutes a synthetic plant's known calibration only in this test TU. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "A_Config.h"
#ifdef ALIGN_TEST_CALIBRATED
#undef MOTOR_KV_UV
#undef MOTOR_KA_NV
#undef MOTOR_FC_MV
#undef MOTOR_FS_MV
#undef MOTOR_DELAY_US
#undef MOTOR_CAL_MV
#undef MOTOR_NOISE_CDEG
#undef MOTOR_MIN_STEP_CDEG
#undef MOTOR_RATED_VEL_DPS
#undef MOTOR_RATED_ACC_DPS2
#undef MOTOR_RATED_DEC_DPS2
#define MOTOR_KV_UV 397
#define MOTOR_KA_NV 7063
#define MOTOR_FC_MV 127
#define MOTOR_FS_MV 413
#define MOTOR_DELAY_US 3211
#define MOTOR_CAL_MV 7666
#define MOTOR_NOISE_CDEG 6
#define MOTOR_MIN_STEP_CDEG 13
#define MOTOR_RATED_VEL_DPS 162
#define MOTOR_RATED_ACC_DPS2 9590
#define MOTOR_RATED_DEC_DPS2 7741
#endif
#define CONFIG_IMAGE_TAG() 1234U
#include "A_Config.c"
#include "C_Motor_Model.c"
#include "C_Pos_Ctrl.c"
#include "C_Traj_Planner.c"
#include "C_Speed_Observer.c"
static uint16_t last_plan_ms;
static void record_plan(int32_t target,uint16_t ms) { last_plan_ms=ms;C_Traj_Plan(target,ms); }
#define C_Traj_Plan record_plan
#include "A_Servo.c"
#undef C_Traj_Plan
#include "A_UartCmd.c"
#ifdef ALIGN_TEST_CALIBRATION_RUN
static uint32_t __get_MSTATUS(void) { return 0; }
static void __disable_irq(void) {}
static void __set_MSTATUS(uint32_t n) { (void)n; }
#define s_tasks alignment_scheduler_tasks
#include "C_Task_Scheduler.c"
#undef s_tasks
#include "A_Calib_Min.c"
void IAP_SetStopHandler(void (*handler)(void)) { (void)handler; }
void IAP_Service(void) {}
void D_IWDG_Feed(void) {}
#endif

static double plant_p, plant_v;
static int16_t plant_duty;
static int encoder_bad, drive_allowed = 1;
static int32_t signed_current_ma;
static uint8_t current_valid;
static uint16_t boot_input_pwm;
static uint8_t last_tx[64], last_tx_len;
static uint32_t deferred_baud;
static unsigned tx_count, baud_after_tx;
static double history[128];
static unsigned history_idx;
static Config_t saved;
static uint8_t saved_valid, save_fail;
uint8_t IAP_HwInfoRead(IAP_HwInfo_t *p) {
    memset(p,0,sizeof(*p));
#ifdef PARAMETER_SCHEMA_PROBE
    p->format=3;p->servo_type=FW_SERVO_TYPE;p->hw_version=FW_HW_VERSION;
    p->voltage=FW_VOLTAGE;p->torque=FW_TORQUE;
    p->max_speed_dps=MOTOR_RATED_VEL_DPS;p->max_accel_dps2=MOTOR_RATED_ACC_DPS2;
    p->max_decel_dps2=MOTOR_RATED_DEC_DPS2;
    p->smooth_ms=(MOTOR_KA_NV+MOTOR_KV_UV/2)/MOTOR_KV_UV;p->min_step_cdeg=MOTOR_MIN_STEP_CDEG;
    return 1;
#else
    return 0;
#endif
}
uint8_t IAP_PreservedIdentityRead(uint8_t *a,uint8_t *b) { *a=0;*b=5;return 1; }
uint16_t FLASH_Config_Load(void *p,uint16_t n,uint8_t v[3]) {
    if(!saved_valid || n<sizeof(saved)) return 0;
    memcpy(p,&saved,sizeof(saved));memcpy(v,k_version,3);return sizeof(saved);
}
uint8_t FLASH_Config_Save(const void *p,uint16_t n,const uint8_t v[3]) {
    (void)v;assert(n==sizeof(saved));if(save_fail)return 0;
    memcpy(&saved,p,n);saved_valid=1;return 1;
}
int32_t A_Encoder_Read(void) {
    if(encoder_bad)return ENCODER_ANGLE_ERROR;
    int32_t p=(int32_t)lround(plant_p);
#if FW_SERVO_TYPE == 2
    while(p<0)p+=CDEG_RANGE;
    while(p>=CDEG_RANGE)p-=CDEG_RANGE;
#endif
    return p;
}
uint16_t A_Voltage_Read(void) { return 767; }
int16_t A_Temperature_Read(void) { return 250; }
uint8_t A_Current_ReadSigned(int32_t *p) { *p=signed_current_ma;return current_valid; }
#ifndef ALIGN_TEST_REAL_PROTECT
uint8_t A_Protect_CanDrive(void) { return drive_allowed; }
uint16_t A_Protect_Fault(void) { return 0; }
uint8_t A_Protect_Stalled(void) { return 0; }
uint16_t A_Protect_GetTorque(void) { return FW_TORQUE*100; }
uint16_t A_Protect_GetCurrent(void) { return 0; }
uint8_t A_Protect_SetTorque(uint16_t n) { (void)n;return 1; }
void A_Protect_GetTelemetry(ProtectTelemetry_t *p) { memset(p,0,sizeof(*p)); }
#endif
int16_t D_Motor_Set(int16_t p) { plant_duty=p;return p; }
void D_Motor_Release(uint8_t n) { (void)n;plant_duty=0; }
void D_UART_Enable(uint8_t n) { (void)n; }
void D_PWM_Input_Enable(uint8_t n) { (void)n; }
uint16_t D_PWM_Read(void) { uint16_t p=boot_input_pwm;boot_input_pwm=0;return p; }
void Delay_Ms(uint32_t n) { (void)n; }
uint8_t D_UART1_Tx_Write(const uint8_t *p,uint8_t n) {
    assert(n<=sizeof(last_tx));memcpy(last_tx,p,n);last_tx_len=n;tx_count++;return 1;
}
void D_UART_SetBaud_Deferred(uint32_t n) { deferred_baud=n;baud_after_tx=tx_count; }
uint8_t D_UART1_Rx_Get(uint8_t *p) { (void)p;return 0; }
void D_UART_Service(void) {}
uint8_t D_UART_Tx_Idle(void) { return 1; }
uint8_t IAP_UpgradePending(void) { return 0; }
uint8_t IAP_Rx_Deal(uint8_t n) { (void)n;return 0; }
void IAP_Rx_Reset(void) {}
#ifdef ALIGN_TEST_REAL_PROTECT
uint8_t A_Current_Read(uint16_t *p) { *p=100;return 1; }
#include "A_Protect.c"
#endif

static const MotorCal_t cal={
    .kv_uv=MOTOR_KV_UV,.ka_nv=MOTOR_KA_NV,.fc_mv=MOTOR_FC_MV,.fs_mv=MOTOR_FS_MV,
    .delay_us=MOTOR_DELAY_US,.cal_mv=MOTOR_CAL_MV,.noise_cdeg=MOTOR_NOISE_CDEG,
    .min_step_cdeg=MOTOR_MIN_STEP_CDEG,.sign=MOTOR_SIGN,
    .rated_vel_dps=MOTOR_RATED_VEL_DPS,.rated_acc_dps2=MOTOR_RATED_ACC_DPS2,
    .rated_dec_dps2=MOTOR_RATED_DEC_DPS2};
static void plant_step(void) {
    for(int j=0;j<20;j++) {
        history[history_idx]=plant_duty*7670.0/2400*MOTOR_SIGN;
        double u=history[(history_idx+128-64)%128];history_idx=(history_idx+1)%128;
        if(fabs(plant_v)<1e-8 && fabs(u)<=cal.fs_mv)plant_v=0;
        else {
            double sign=fabs(plant_v)<1e-8?(u>0?1:-1):(plant_v>0?1:-1);
            double next=plant_v+(u-cal.kv_uv/1000.*plant_v-cal.fc_mv*sign)/(cal.ka_nv/1000000.)*.00005;
            if(next*plant_v<0 && fabs(u)<=cal.fs_mv)next=0;
            plant_v=next;
        }
        plant_p+=plant_v*.00005;
    }
}
static void tick(void) { A_Servo_Control();plant_step(); }
static void run_ticks(int n) { for(int i=0;i<n;i++)tick(); }

static void reset(void) {
    saved_valid=save_fail=0;plant_p=13500;plant_v=0;plant_duty=0;encoder_bad=0;drive_allowed=1;
    boot_input_pwm=0;
    history_idx=0;memset(history,0,sizeof(history));
    A_Config_Init();
    g_config.boot_mode=SERVO_BOOT_HOLD;
    g_config.servo_mode=SERVO_MODE_270_CCW; /* With CFG_DIR_INVERT=1, P increases angle. */
    g_config.cal_kind=SERVO_CAL_ZERO;g_config.cal_anchor_cdeg=0;
    C_MotorModel_Init(&cal);
    A_Servo_Init();
#ifdef ALIGN_TEST_REAL_PROTECT
    A_Protect_Init();
#endif
}
static uint8_t write_param(uint8_t id,uint32_t value,uint8_t len) {
    uint8_t data[4],baud=0;Bin_PutN(data,value,len);return Bin_ParamWrite(id,data,len,&baud);
}
static void parameters(void) {
    reset();assert(A_Config_TorqueMax()==FW_TORQUE*100);
    ServoDebugTelemetry_t debug;
    signed_current_ma=-1234;current_valid=1;A_Servo_GetDebugTelemetry(&debug);
    assert(debug.cur_ma==-1234 && (debug.flags & 8U));
    signed_current_ma=1234;current_valid=0;A_Servo_GetDebugTelemetry(&debug);
    assert(debug.cur_ma==1234 && !(debug.flags & 8U));
    signed_current_ma=40000;A_Servo_GetDebugTelemetry(&debug);assert(debug.cur_ma==32767);
    signed_current_ma=-40000;A_Servo_GetDebugTelemetry(&debug);assert(debug.cur_ma==-32767);
    signed_current_ma=0;
    assert(sizeof(Config_t)==68);
    assert(CFG_SMOOTH_MAX_MS==TRAJ_SMOOTH_MS_MAX && CFG_SMOOTH_MAX_MS==200);
    assert(CFG_MOVE_MIN_MAX_MS==TRAJ_MOVE_MIN_MS_MAX);
    const uint8_t smooth_ids[]={PARAM_TRAJ_SMOOTH,PARAM_RT_TRAJ_SMOOTH};
    for(unsigned i=0;i<sizeof(smooth_ids);i++) {
        uint8_t id=smooth_ids[i];
        assert(write_param(id,0,2)==BIN_ST_OK);
        if(CFG_SMOOTH_MIN_MS>1) assert(write_param(id,CFG_SMOOTH_MIN_MS-1,2)==BIN_ST_VALUE);
        assert(write_param(id,CFG_SMOOTH_MIN_MS,2)==BIN_ST_OK);
        assert(write_param(id,200,2)==BIN_ST_OK);
        assert(write_param(id,201,2)==BIN_ST_VALUE);
    }
    assert(write_param(PARAM_PROT_ENABLE,15,1)==BIN_ST_OK);
    assert(write_param(PARAM_PROT_ENABLE,16,1)==BIN_ST_VALUE);
    assert(write_param(PARAM_PROT_ENABLE,255,1)==BIN_ST_VALUE);
    assert(PROT_VOLTAGE_CFG_MIN_DV==60 && PROT_VOLTAGE_CFG_MAX_DV==90);
    assert(g_config.voltage_min_dv==60 && g_config.voltage_max_dv==90);
    assert(write_param(PARAM_VOLTAGE_MIN,59,1)==BIN_ST_VALUE);
    assert(write_param(PARAM_VOLTAGE_MIN,60,1)==BIN_ST_OK);
    assert(write_param(PARAM_VOLTAGE_MAX,91,1)==BIN_ST_VALUE);
    assert(write_param(PARAM_VOLTAGE_MAX,90,1)==BIN_ST_OK);
    assert(write_param(PARAM_VOLTAGE_MAX,60,1)==BIN_ST_VALUE); /* Preserve 0.2V hysteresis. */
    assert(write_param(PARAM_PROT_STALL_THRESHOLD,PROT_STALL_THRESHOLD_MIN_CDEG-1,2)==BIN_ST_VALUE);
    assert(write_param(PARAM_PROT_STALL_THRESHOLD,PROT_STALL_THRESHOLD_MAX_CDEG,2)==BIN_ST_OK);
    assert(write_param(PARAM_PROT_STALL_THRESHOLD,PROT_STALL_THRESHOLD_MAX_CDEG+1,2)==BIN_ST_VALUE);
    assert(write_param(PARAM_RT_HOLD_KP,CFG_HOLD_KP_MAX,2)==BIN_ST_OK);
    assert(write_param(PARAM_RT_HOLD_KP,CFG_HOLD_KP_MAX+1,2)==BIN_ST_VALUE);
    const uint8_t ki_ids[]={PARAM_HOLD_KI,PARAM_RT_HOLD_KI};
    for(unsigned i=0;i<sizeof(ki_ids);i++) {
        uint8_t value[4],len=0;
        assert(write_param(ki_ids[i],CFG_HOLD_KI_MAX,2)==BIN_ST_OK);
        assert(g_runtime_config.hold_ki==CFG_HOLD_KI_MAX);
        if(i==0) assert(g_config.hold_ki==CFG_HOLD_KI_MAX);
        assert(Bin_ParamRead(ki_ids[i],value,&len)==BIN_ST_OK && len==3);
        assert(value[0]==ki_ids[i] && Bin_Get16(value+1)==CFG_HOLD_KI_MAX);
        assert(write_param(ki_ids[i],CFG_HOLD_KI_MAX+1,2)==BIN_ST_VALUE);
        assert(write_param(ki_ids[i],0,2)==BIN_ST_OK);

    }
    /* Removed D addresses must be rejected rather than aliasing another setting. */
    const uint8_t removed_ids[]={0x39,0x90};
    for(unsigned i=0;i<sizeof(removed_ids);i++) {
        uint8_t value[4],len=0;
        assert(Bin_ParamRead(removed_ids[i],value,&len)==BIN_ST_VALUE);
        assert(write_param(removed_ids[i],10,1)==(removed_ids[i]==0x39?BIN_ST_VALUE:BIN_ST_READONLY));
    }
    g_config.servo_id=23;g_config.baud_code=6;A_Config_Default(1U);
    assert(g_config.servo_id==23 && g_config.baud_code==5);
    volatile uint8_t event=0;A_Config_BindSaveEvent(&event);save_fail=1;A_Config_MarkDirty();
    event=0;A_Config_SaveTask();assert(event==1);save_fail=0;A_Config_SaveTask();assert(!s_dirty);
    A_Config_BindSaveEvent(NULL);
    saved=g_config;saved.image_tag=1234;saved.traj_smooth_ms=201;
    saved.protect_enable=255;
    saved.prot_stall_act_cdeg=PROT_STALL_THRESHOLD_MAX_CDEG+1;
    saved.hold_ki=CFG_HOLD_KI_MAX+1;
    saved.voltage_min_dv=50;saved.voltage_max_dv=90;
    saved_valid=1;A_Config_Init();
    assert(g_config.traj_smooth_ms==CFG_DEF_SMOOTH_MS);
    assert(g_config.protect_enable==PROT_ENABLE_DEFAULT);
    assert(g_config.prot_stall_act_cdeg==PROT_STALL_ACT_DEFAULT_CDEG);
    assert(g_config.hold_ki==CFG_DEF_HOLD_KI && g_runtime_config.hold_ki==CFG_DEF_HOLD_KI);
    assert(g_config.voltage_min_dv==60 && g_config.voltage_max_dv==90);
    saved=g_config;saved.image_tag=1234;saved.servo_mode=5;saved_valid=1;A_Config_Init();
#if FW_SERVO_TYPE == 2
    assert(g_config.servo_mode==5);
#else
    assert(g_config.servo_mode==SERVO_MODE_270_CW);
#endif
}

static void binary_request(uint8_t cmd,const uint8_t *data,uint8_t n) {
    uint8_t f[BIN_FRAME_MAX]={BIN_HEAD1,BIN_HEAD2,23,cmd,n};
    if(n)memcpy(f+5,data,n);
    f[5+n]=Bin_Crc8(f+2,(uint8_t)(3+n));f[6+n]=BIN_TAIL1;f[7+n]=BIN_TAIL2;
    for(unsigned i=0;i<8U+n;i++)Wire_Byte(f[i]);
}

static void reset_commands(void) {
    for(unsigned action=0;action<4;action++) {
        reset();Config_FillDefaults();Config_t expected=g_config;
        A_Config_ReloadRuntime(1U);RuntimeConfig_t rt=g_runtime_config;
        uint8_t keep=(uint8_t)((action%2)==0);
        if(keep)expected.servo_id=23;
        memset(&g_config,0x5a,sizeof(g_config));g_config.servo_id=23;g_config.baud_code=6;
        memset(&g_runtime_config,0x5a,sizeof(g_runtime_config));
        last_tx_len=0;deferred_baud=0;tx_count=baud_after_tx=0;s_wire=WIRE_IDLE;
        if(action<2) {
            const char *f=keep?"#023PCLE0!":"#023PCLE!";
            while(*f)Wire_Byte((uint8_t)*f++);
            const char *ack=keep?"#023POK!":"#000POK!";
            assert(last_tx_len==strlen(ack) && !memcmp(last_tx,ack,last_tx_len));
        } else {
            uint8_t cmd=keep?BIN_CMD_RESET_KEEP:BIN_CMD_RESET_ALL;
            binary_request(cmd,NULL,0);
            assert(last_tx_len==9 && last_tx[2]==expected.servo_id);
            assert(last_tx[3]==(cmd|0x80) && last_tx[5]==BIN_ST_OK);
            assert(last_tx[6]==Bin_Crc8(last_tx+2,4));
        }
        assert(!memcmp(&g_config,&expected,sizeof(expected)));
#define CHECK_RT(f) assert(g_runtime_config.f==rt.f)
        CHECK_RT(torque_limit_ckgcm);CHECK_RT(traj_amax_dps2);CHECK_RT(traj_dmax_dps2);
        CHECK_RT(traj_vmax_dps);CHECK_RT(traj_smooth_ms);CHECK_RT(traj_move_min_ms);
        CHECK_RT(prot_current_limit_ma);CHECK_RT(prot_current_time_ms);CHECK_RT(prot_temp_limit_dC);
        CHECK_RT(prot_temp_time_ms);CHECK_RT(prot_stall_act_cdeg);CHECK_RT(prot_stall_trip);
        CHECK_RT(hold_kp);CHECK_RT(hold_ki);
        CHECK_RT(position_resolution_cdeg);CHECK_RT(protect_enable);CHECK_RT(voltage_min_dv);
        CHECK_RT(voltage_max_dv);CHECK_RT(min_start_torque_ckgcm);CHECK_RT(torque_mode);CHECK_RT(damping_percent);
#undef CHECK_RT
        assert(s_dirty && deferred_baud==115200 && tx_count==1 && baud_after_tx==tx_count);
        A_Config_SaveTask();assert(!s_dirty && !memcmp(&saved,&expected,sizeof(saved)));
        A_Config_Init();assert(!memcmp(&g_config,&expected,sizeof(expected)));
    }
    /* Malformed reset commands must not change parameters or schedule a baud switch. */
    reset();g_config.servo_id=23;g_config.baud_code=6;deferred_baud=0;
    uint8_t data=0;binary_request(BIN_CMD_RESET_ALL,&data,1);
    assert(last_tx[5]==BIN_ST_LENGTH && g_config.servo_id==23 && g_config.baud_code==6 && !deferred_baud);
#ifndef PARAM_SYSTEM_ACTION
    /* The former RAM system-action address must reject every previous action,
     * without resetting settings, reloading runtime values or changing baud. */
    g_config.traj_smooth_ms=80;g_runtime_config.traj_smooth_ms=100;
    Config_t config_before=g_config;
    RuntimeConfig_t runtime_before=g_runtime_config;
    uint8_t dirty_before=s_dirty;
    for(uint8_t action=0;action<=4;action++) {
        for(uint8_t size=1;size<=4;size++) {
            uint8_t bad[5]={0x8C,action,0,0,0};
            binary_request(BIN_CMD_WRITE,bad,(uint8_t)(size+1));
            assert(last_tx_len==9 && last_tx[2]==23 && last_tx[3]==0x83);
            assert(last_tx[5]==BIN_ST_READONLY && !deferred_baud);
            assert(!memcmp(&g_config,&config_before,sizeof(g_config)));
            assert(!memcmp(&g_runtime_config,&runtime_before,sizeof(g_runtime_config)));
            assert(s_dirty==dirty_before);
        }
    }
    uint8_t removed=0x8C;binary_request(BIN_CMD_READ,&removed,1);
    assert(last_tx_len==9 && last_tx[3]==0x82 && last_tx[5]==BIN_ST_VALUE);
#endif
    puts("PASS: text CLE0/CLE and binary 07/08 reset parameters correctly; removed RAM system actions cannot change settings");
}

static void boot_timing(void) {
    reset();g_config.boot_mode=SERVO_BOOT_GOTO_START;g_config.startup_pwm=1800;
    last_plan_ms=0xffff;A_Servo_Init();assert(last_plan_ms==1500 && s_servo.remaining==1500);
    s_servo.move_age=0xffff;assert(A_Servo_Submit(1200,250)==SERVO_SUBMIT_OK);assert(last_plan_ms==250);
    s_servo.move_age=0xffff;assert(A_Servo_Submit(2000,0)==SERVO_SUBMIT_OK);assert(last_plan_ms==0);
    /* An external PWM present at startup has priority, with the same boot time. */
    reset();g_config.boot_mode=SERVO_BOOT_RELEASE;boot_input_pwm=1800;
    last_plan_ms=0xffff;A_Servo_Init();assert(last_plan_ms==1500 && s_servo.input_source==SERVO_INPUT_PWM);
    /* The saved/PWM boot action retains its request time after encoder recovery. */
    for(unsigned pwm=0;pwm<2;pwm++) {
        reset();g_config.boot_mode=SERVO_BOOT_GOTO_START;g_config.startup_pwm=1800;
        boot_input_pwm=pwm?1800:0;encoder_bad=1;last_plan_ms=0xffff;A_Servo_Init();
        assert(last_plan_ms==0xffff);encoder_bad=0;
#if FW_SERVO_TYPE == 2
        A_Servo_Control();
#else
        Servo_EscapeFinish();
#endif
        assert(last_plan_ms==1500);
    }
#if FW_SERVO_TYPE != 2
    /* A new explicit command queued during potentiometer recovery keeps its own T. */
    reset();g_config.boot_mode=SERVO_BOOT_GOTO_START;encoder_bad=1;A_Servo_Init();
    assert(A_Servo_Submit(1800,0)==SERVO_SUBMIT_OK);encoder_bad=0;
    last_plan_ms=0xffff;Servo_EscapeFinish();assert(last_plan_ms==0);
#endif
    for(unsigned mode=SERVO_BOOT_HOLD;mode<=SERVO_BOOT_RELEASE;mode++) {
        reset();g_config.boot_mode=(uint8_t)mode;last_plan_ms=0xffff;A_Servo_Init();assert(last_plan_ms==0xffff);
    }
    /* In motor modes value means turns/seconds, so boot must not inject 1500 there. */
    for(unsigned mode=SERVO_MODE_TURNS_CW;mode<=SERVO_MODE_TIMED_CW;mode+=2) {
        reset();g_config.servo_mode=(uint8_t)mode;g_config.boot_mode=SERVO_BOOT_GOTO_START;
        g_config.startup_pwm=1800;last_plan_ms=0xffff;A_Servo_Init();
        assert(last_plan_ms==0xffff && s_servo.remaining==UINT32_MAX);
    }
    puts("PASS: 1500ms boot planning for saved/PWM/recovered targets; normal commands and motor units retained");
}

int main(void) {
    boot_timing();
    reset_commands();
    parameters();reset();
#ifdef ALIGN_TEST_REAL_PROTECT
    assert(write_param(PARAM_PROT_STALL_THRESHOLD,PROT_STALL_THRESHOLD_MAX_CDEG,2)==BIN_ST_OK);
    A_Protect_Task();
    assert(s_prot.stall_act_cdeg==PROT_STALL_THRESHOLD_MAX_CDEG);
    assert(s_prot.stall_ref_cdeg==s_prot.stall_act_cdeg*PROT_STALL_REF_RATIO);
    assert(write_param(PARAM_PROT_STALL_THRESHOLD,PROT_STALL_THRESHOLD_MIN_CDEG,2)==BIN_ST_OK);
    A_Protect_Task();
    assert(s_prot.stall_act_cdeg==PROT_STALL_THRESHOLD_MIN_CDEG);
    assert(s_prot.stall_ref_cdeg==s_prot.stall_act_cdeg*PROT_STALL_REF_RATIO);
    puts("PASS: real protection module derives reference threshold from actual threshold after protocol writes");
    return 0;
#endif
#ifdef ALIGN_TEST_CALIBRATION_RUN
    A_Calib_Min_Init();
    for(int i=0;i<600000 && s_cal.phase!=PH_DONE;i++) { A_Calib_Min_Tick();plant_step(); }
    assert(s_cal.phase==PH_DONE && !s_cal.fail);
    CalModel_t identified={0};assert(Cal_FitSpeed(&identified));assert(Cal_FitDynamics(&identified));
    assert(fabs(identified.kv*1000-MOTOR_KV_UV)<MOTOR_KV_UV*.08);
    assert(fabs(identified.tau-MOTOR_KA_NV*1.0/MOTOR_KV_UV)<(MOTOR_KA_NV*1.0/MOTOR_KV_UV)*.15);
    printf("PASS: complete on-MCU calibration simulation, kv=%.3f, tau=%.3fms\n",identified.kv,identified.tau);
    return 0;
#endif
    run_ticks(300);
    assert(A_Servo_Submit(1850,0)==SERVO_SUBMIT_OK);run_ticks(2500);
    assert(fabs(plant_p-18225)<60 && A_Servo_GetWorkState()==SERVO_WORK_IDLE);
    assert(A_Servo_Submit(1150,250)==SERVO_SUBMIT_OK);run_ticks(3000);
    assert(fabs(plant_p-8775)<60 && A_Servo_GetWorkState()==SERVO_WORK_IDLE);
    for(int i=0;i<40;i++){assert(A_Servo_Submit(1150+i*5,20)==SERVO_SUBMIT_OK);run_ticks(20);}
    run_ticks(3000);assert(fabs(plant_p-(1150+39*5-500)*13.5)<60);
    uint8_t data[4],baud=0;g_config.pulse_lo=1000;g_config.pulse_hi=2000;
    Bin_Put16(data,900);Bin_Put16(data+2,0);assert(Bin_ParamWrite(PARAM_MOTION_REQUEST,data,4,&baud)==BIN_ST_VALUE);
    drive_allowed=0;Bin_Put16(data,1500);assert(Bin_ParamWrite(PARAM_MOTION_REQUEST,data,4,&baud)==BIN_ST_STATE);drive_allowed=1;
    reset();assert(A_Servo_Submit(2200,0)==SERVO_SUBMIT_OK);run_ticks(100);A_Servo_Pause();run_ticks(500);
    assert(A_Servo_GetWorkState()==SERVO_WORK_PAUSED);double paused=plant_p;run_ticks(200);assert(fabs(plant_p-paused)<60);
    A_Servo_Resume();run_ticks(3000);assert(fabs(plant_p-22950)<60);
#if FW_SERVO_TYPE == 2
    reset();g_config.servo_mode=SERVO_MODE_360_CCW;A_Servo_ApplyConfig();
    plant_p=35950;s_servo.circular_valid=0;assert(Servo_ReadPosition());Servo_Resync(1);
    plant_p=36020;assert(Servo_ReadPosition());assert(s_servo.angle_circ==36020);
    plant_p=35990;assert(Servo_ReadPosition());assert(s_servo.angle_circ==35990);
    reset();plant_p=35990;g_config.cal_kind=SERVO_CAL_MID;g_config.cal_anchor_cdeg=35990;A_Servo_ApplyConfig();
    assert(s_servo.span_cdeg==27000 && s_servo.offset_cdeg==22490);
    assert(A_Servo_Submit(1850,0)==SERVO_SUBMIT_OK);run_ticks(4000);
    assert(fabs(plant_p-40715)<70 && A_Servo_GetWorkState()==SERVO_WORK_IDLE);
    reset();const char *multi="#000P1800N2T0000!";
    Text_Frame((const uint8_t *)multi,(uint8_t)strlen(multi));assert(s_servo.mt_active);
    assert(A_Servo_SubmitTurns(1800,10,0)==SERVO_SUBMIT_VALUE);A_Servo_Stop();
    reset();assert(A_Servo_SubmitTurns(1800,2,0)==SERVO_SUBMIT_OK);run_ticks(12000);
    assert(!s_servo.mt_active && A_Servo_GetWorkState()==SERVO_WORK_IDLE);
    assert(fabs(plant_p-(17550+72000))<70);
    reset();assert(A_Servo_SubmitTurns(1200,2,0)==SERVO_SUBMIT_OK);run_ticks(12000);
    assert(!s_servo.mt_active && fabs(plant_p-(9450-72000))<70);
    reset();assert(A_Servo_SubmitTurns(1800,9,9000)==SERVO_SUBMIT_OK);assert(!s_servo.mt_final);
    run_ticks(2000);A_Servo_Pause();run_ticks(500);assert(s_servo.mt_active);
    A_Servo_Resume();run_ticks(90000);
    assert(!s_servo.mt_active && fabs(plant_p-(17550+324000))<70);
    reset();assert(A_Servo_SubmitTurns(1800,3,0)==SERVO_SUBMIT_OK);run_ticks(500);A_Servo_Stop();
    assert(!s_servo.mt_active);run_ticks(500);assert(A_Servo_GetWorkState()==SERVO_WORK_IDLE);
    reset();assert(A_Servo_SubmitTurns(1800,3,0)==SERVO_SUBMIT_OK);run_ticks(500);
    assert(A_Servo_Submit(1400,0)==SERVO_SUBMIT_OK);assert(!s_servo.mt_active);run_ticks(3000);
    assert(A_Servo_GetWorkState()==SERVO_WORK_IDLE);
    reset();assert(A_Servo_Submit(1800,0)==SERVO_SUBMIT_OK);run_ticks(50);encoder_bad=1;run_ticks(5);
    assert(A_Servo_GetWorkState()==SERVO_WORK_FAULT && plant_duty==0);
    encoder_bad=0;run_ticks(20);assert(!A_Servo_TorqueOn());
    reset();g_config.servo_mode=SERVO_MODE_TURNS_CW;A_Servo_ApplyConfig();
    assert(A_Servo_Submit(2000,2)==SERVO_SUBMIT_OK);encoder_bad=1;run_ticks(5);
    assert(A_Servo_GetWorkState()==SERVO_WORK_FAULT && plant_duty==0);
    reset();encoder_bad=1;A_Servo_Init();run_ticks(20);assert(plant_duty==0);
    encoder_bad=0;run_ticks(50);assert(s_servo.enc_state==SERVO_ENC_OK);
    puts("PASS: IIC 360 wrap, positive/negative multiturn, 9-turn timing, pause/resume, abort and encoder faults");
#else
    reset();encoder_bad=1;run_ticks(5);assert(s_servo.enc_state==SERVO_ENC_ESCAPING);
    run_ticks(1);assert(plant_duty!=0); /* Potentiometer keeps the 30kg escape behavior. */
#endif
    puts("PASS: real voltage-model control chain, small target stream, pause/resume, parameter validation and motion ACK");
    return 0;
}
