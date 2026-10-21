/* 真实配置/协议/控制/规划模块，硬件边界用桩。验证用户参数沿整个控制链生效。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define CONFIG_IMAGE_TAG() 1234U
#include "../CH32V00X_APP_ADC_30kg/Application/A_Config.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Motor_Model.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Pos_Ctrl.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Traj_Planner.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Setpoint.c"
#include "../CH32V00X_APP_ADC_30kg/Common/C_Speed_Observer.c"
#include "../CH32V00X_APP_ADC_30kg/Application/A_Servo.c"
#include "../CH32V00X_APP_ADC_30kg/Application/A_UartCmd.c"

static Config_t saved;
static uint8_t saved_valid;
static uint16_t fault;
uint8_t IAP_HwInfoRead(IAP_HwInfo_t *hw) { memset(hw,0,sizeof(*hw)); return 0; }
uint8_t IAP_PreservedIdentityRead(uint8_t *id,uint8_t *baud) { *id=0;*baud=5;return 1; }
uint16_t FLASH_Config_Load(void *p,uint16_t cap,uint8_t ver[3]) {
    if (!saved_valid || cap<sizeof(saved)) return 0;
    memcpy(p,&saved,sizeof(saved)); memcpy(ver,k_version,3); return sizeof(saved);
}
uint8_t FLASH_Config_Save(const void *p,uint16_t len,const uint8_t ver[3]) {
    assert(len==sizeof(saved)); assert(!memcmp(ver,k_version,3)); memcpy(&saved,p,len);saved_valid=1;return 1;
}
int32_t A_Encoder_Read(void) { return 10000; }
uint16_t A_Voltage_Read(void) { return 800; }
int16_t A_Temperature_Read(void) { return 250; }
uint8_t A_Protect_CanDrive(void) { return 1; }
uint16_t A_Protect_Fault(void) { return fault; }
uint8_t A_Protect_Stalled(void) { return 0; }
uint16_t A_Protect_GetTorque(void) { return 3000; }
uint16_t A_Protect_GetCurrent(void) { return 0; }
uint8_t A_Protect_SetTorque(uint16_t v) { (void)v;return 1; }
void A_Protect_GetTelemetry(ProtectTelemetry_t *p) { memset(p,0,sizeof(*p)); }
int16_t D_Motor_Set(int16_t p) { return p; }
void D_Motor_Release(uint8_t h) { (void)h; }
void D_UART_Enable(uint8_t en) { (void)en; }
void D_PWM_Input_Enable(uint8_t en) { (void)en; }
uint16_t D_PWM_Read(void) { return 0; }
void Delay_Ms(uint32_t t) { (void)t; }
uint8_t D_UART1_Tx_Write(const uint8_t *p,uint8_t n) { (void)p;(void)n;return 1; }
void D_UART_SetBaud_Deferred(uint32_t b) { (void)b; }
uint8_t D_UART1_Rx_Get(uint8_t *p) { (void)p;return 0; }
void D_UART_Service(void) {}
uint8_t IAP_UpgradePending(void) { return 0; }
uint8_t IAP_Rx_Deal(uint8_t b) { (void)b;return 0; }
void IAP_Rx_Reset(void) {}

static uint8_t write_param(uint8_t id,uint32_t value,uint8_t size)
{
    uint8_t p[4],baud=0;
    Bin_PutN(p,value,size);
    return Bin_ParamWrite(id,p,size,&baud);
}

static void control_chain(const MotorCal_t *cal)
{
    double pos=0,vel=0; int32_t u=0,history[32]={0};
    unsigned index=0; int32_t worst=0;
    SpeedObserver_t observer;SpeedObsOut_t observation;
    C_SpeedObs_Configure(cal,21,252);C_SpeedObs_Init(&observer,0);
    C_PosCtrl_Init(cal,120,252);C_PosCtrl_SetPrecision(30,20);
    C_Traj_Init(0);C_Traj_SetLimits(15500,400000,400000,20000000);
    C_Traj_SetMotorCap(800000,24378);C_Setpoint_SetMinTime(75);C_Setpoint_Reset(0);
    C_Setpoint_Move(10000,0);
    for(int tick=0;tick<8000;tick++) {
        if(tick==2000) C_Setpoint_Move(0,0);
        if(tick>=4000 && tick<5000 && tick%20==0) C_Setpoint_Move((tick-4000)*2,0);
        C_SpeedObs_Update(&observer,(int32_t)lround(pos),1,u,&observation);
        C_Setpoint_Tick();TrajRef_t ref;C_Traj_Step(&ref);
        PosCtrlFeedback_t feedback={.pos=(int32_t)lround(pos),.vel=observation.vel,.valid=1};
        u=C_PosCtrl_Update(&ref,C_Traj_Settled(),&feedback,7200,0);
        history[index]=u;int32_t applied=history[(index+30)%32];index=(index+1)%32;
        for(int j=0;j<20;j++) {
            if(fabs(vel)<1e-8 && abs(applied)<=cal->fs_mv) vel=0;
            else {
                double sign=fabs(vel)<1e-8?(applied>0?1:-1):(vel>0?1:-1);
                double next=vel+(applied-.402*vel-106*sign)/.0098*.00005;
                if(next*vel<0 && abs(applied)<=cal->fs_mv) next=0;
                vel=next;
            }
            pos+=vel*.00005;
        }
        if(tick<2000 && pos-10000>worst) worst=(int32_t)(pos-10000);
        if(tick>=2000 && tick<4000 && -pos>worst) worst=(int32_t)-pos;
        if(tick==1999) assert(fabs(pos-10000)<=30);
        if(tick==3999) assert(fabs(pos)<=30);
    }
    assert(worst<150);assert(fabs(pos-1960)<=30);
    printf("PASS: full control chain, step/reverse/20ms target stream; max overshoot %.2fdeg, final error %.2fdeg\n",worst/100.,(pos-1960)/100.);
}

int main(void)
{
    MotorCal_t cal={.kv_uv=402,.ka_nv=9800,.fc_mv=106,.fs_mv=522,.delay_us=2000,
        .noise_cdeg=8,.min_step_cdeg=32,.sign=1,
        .cap={{6000,146,6000,6200},{8000,196,8000,8200}}};
    C_MotorModel_Init(&cal,2400);
    A_Config_Init(); A_Servo_Init();
    MotorCap_t cap;
    C_MotorModel_CapAt(7000,&cap);
    assert(cap.vel==17100 && cap.acc==700000 && cap.dec==720000);
    C_MotorModel_CapAt(5000,&cap); assert(cap.vel==14600);
    assert(g_runtime_config.position_resolution_cdeg==TUNE_RESOLUTION_CDEG);
    assert(write_param(PARAM_TRAJ_VMAX,80,2)==BIN_ST_OK);
    assert(write_param(PARAM_TRAJ_AMAX,1200,4)==BIN_ST_OK);
    assert(write_param(PARAM_TRAJ_DMAX,1600,4)==BIN_ST_OK);
    assert(s_tj.vel_lim==8000*256 && s_tj.acc==120000 && s_tj.dec==160000);
    assert(write_param(PARAM_RESOLUTION,20,2)==BIN_ST_OK);
    assert(g_config.position_resolution_cdeg==20 && g_runtime_config.position_resolution_cdeg==20);
    assert(write_param(PARAM_RT_RESOLUTION,60,2)==BIN_ST_OK);
    assert(g_config.position_resolution_cdeg==20 && g_runtime_config.position_resolution_cdeg==60);
    assert(write_param(PARAM_RESOLUTION,0,2)==BIN_ST_VALUE);
    assert(write_param(PARAM_RESOLUTION,501,2)==BIN_ST_VALUE);
    assert(write_param(PARAM_RESOLUTION,20,1)==BIN_ST_LENGTH);
    A_Config_SaveTask(); A_Config_Init();
    assert(g_runtime_config.position_resolution_cdeg==20);
    assert(g_runtime_config.traj_vmax_dps==80);
    uint8_t data[8],length;
    assert(Bin_ParamRead(PARAM_RESOLUTION,data,&length)==BIN_ST_OK && length==3 && Bin_Get16(data+1)==20);
    uint8_t frame[]={0xAA,0x66,0,0x03,3,0x37,22,0,0,0x66,0xAA};
    frame[2]=g_config.servo_id;frame[8]=Bin_Crc8(frame+2,6);Bin_Frame(frame,sizeof(frame));
    assert(g_config.position_resolution_cdeg==22 && g_runtime_config.position_resolution_cdeg==22);
    frame[6]=25;Bin_Frame(frame,sizeof(frame));assert(g_runtime_config.position_resolution_cdeg==22); /* CRC错不执行 */

    /* 捕获用精度20，退出窗20+30；变化立刻生效，标定的min_step=32不覆盖它。 */
    C_PosCtrl_Init(&cal,120,200); C_PosCtrl_SetPrecision(20,30);
    TrajRef_t ref={.pos=1000}; PosCtrlFeedback_t fb={.pos=985,.valid=1}; PosCtrlDbg_t dbg;
    C_PosCtrl_Update(&ref,1,&fb,7000,&dbg); assert(dbg.v_cmd==0);
    fb.pos=960; C_PosCtrl_Update(&ref,1,&fb,7000,&dbg); assert(dbg.v_cmd==0);
    fb.pos=940; C_PosCtrl_Update(&ref,1,&fb,7000,&dbg); assert(dbg.v_cmd>0);
    C_PosCtrl_SetPrecision(10,0); fb.pos=985;
    C_PosCtrl_Update(&ref,1,&fb,7000,&dbg); assert(dbg.v_cmd>0 && C_PosCtrl_Deadband()==10);
    C_PosCtrl_Reset(); C_PosCtrl_Update(&ref,0,&fb,7000,&dbg); assert(dbg.v_cmd==300);

    /* 首次到位必须连续20ms落在用户精度内；齿隙退出窗不扩大到位判据。 */
    s_servo.arrived=0; s_servo.arrival_ms=0; s_servo.ref.pos=1000;s_servo.angle_circ=975;s_servo.obs.vel=0;
    for(int i=0;i<25;i++) Servo_UpdateArrival(1,1);
    assert(!s_servo.arrived);
    s_servo.angle_circ=985; for(int i=0;i<19;i++) Servo_UpdateArrival(1,1);
    assert(!s_servo.arrived);Servo_UpdateArrival(1,1);assert(s_servo.arrived);
    C_Traj_SetLimits(17000,400000,400000,20000000);C_Traj_SetMotorCap(800000,24000);
    assert(Traj_AccelAt(17000*256)<400000 && Traj_AccelAt(0)==400000);
    C_Setpoint_SetMinTime(75);C_Setpoint_Reset(1000);C_Setpoint_Move(1020,0);
    assert(s_sp.vel_cap>0); for(int i=0;i<10;i++) C_Setpoint_Tick();
    C_Setpoint_Move(1040,0);assert(s_sp.left>0 && s_sp.vel_cap==0);
    MotorCal_t fast=cal;fast.kv_uv=1000;fast.ka_nv=2000;
    C_MotorModel_Init(&fast,2400);s_servo.cap.acc=s_servo.cap.dec=10000000;
    g_runtime_config.traj_amax_dps2=g_runtime_config.traj_dmax_dps2=g_runtime_config.traj_jmax_dps3=0;
    A_Servo_ApplyTrajConfig();assert(s_tj.n_pending>1 && s_tj.acc>1000000);
    C_MotorModel_Init(&cal,2400);
    control_chain(&cal);
    puts("PASS: model interpolation, V/A/D limits, parameter validation/readback, Flash/SRAM separation, precision, arrival, speed-dependent capacity and continuous setpoints");
    return 0;
}
