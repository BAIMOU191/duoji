#include <stdio.h>
#include "A_Protect.c"
Config_t g_config;
static uint16_t current_ma;
static uint8_t current_valid=1, torque_on=1;
static int16_t temperature=250, output_limit;
static int failures;
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);failures++;}}while(0)
void A_Config_MarkDirty(void) {}
uint8_t A_Current_Read(uint16_t *ma) {*ma=current_ma;return current_valid;}
int16_t A_Temperature_Read(void) {return temperature;}
void A_Servo_SetOutputLimit(int16_t limit) {output_limit=limit;}
uint8_t A_Servo_TorqueOn(void) {return torque_on;}
void A_Servo_Release(uint8_t high) {(void)high;torque_on=0;}
void A_Servo_RestoreTorque(void) {torque_on=1;}
static ServoMotion_t motion;   /* What the servo reports for the next sample. */
static int holds;              /* How many times the stall action was issued. */
void A_Servo_SampleMotion(ServoMotion_t *out) {*out=motion;}
void A_Servo_HoldHere(void) {holds++;}
static void ticks(int n) {while(n--)A_Protect_Task();}
/* One detection period's worth of base ticks, with a given sample in place. */
static void windows(int n,int32_t d_ref,int32_t d_act,uint16_t hold,uint8_t valid) {
    motion.ref_delta=d_ref;motion.act_delta=d_act;motion.hold_ticks=hold;motion.valid=valid;
    ticks(n*PROT_STALL_DIV);
}
static void check_stall(void) {
    holds=0;
    windows(PROT_STALL_TRIP-1,PROT_STALL_REF_CDEG,0,0,1);CHECK(!holds);
    windows(1,PROT_STALL_REF_CDEG-1,0,0,1);CHECK(!holds);      /* reference idle */
    windows(PROT_STALL_TRIP-1,PROT_STALL_REF_CDEG,0,0,1);CHECK(!holds); /* recount */
    windows(1,PROT_STALL_REF_CDEG,0,0,1);CHECK(holds==1);CHECK(A_Protect_Stalled());
    windows(PROT_STALL_TRIP,PROT_STALL_REF_CDEG,0,0,0);        /* not driving */
    CHECK(holds==1);CHECK(A_Protect_Stalled()); /* idle time never clears it */
    windows(1,PROT_STALL_REF_CDEG,PROT_STALL_ACT_CDEG+1,0,1);
    CHECK(!A_Protect_Stalled());                               /* it really moved */
    holds=0;
    /* A pinned reference still stalls: the frozen-tick count stands in for it. */
    windows(PROT_STALL_TRIP,0,0,PROT_STALL_HOLD_TICKS,1);CHECK(holds==1);
    holds=0;
    windows(PROT_STALL_TRIP,0,0,PROT_STALL_HOLD_TICKS-1,0);CHECK(!holds);
    windows(PROT_STALL_TRIP,PROT_STALL_REF_CDEG,PROT_STALL_ACT_CDEG+1,0,1);CHECK(!holds);
    windows(1,0,0,0,0);
}
int main(void) {
    g_config.torque_limit=100;A_Protect_Init();CHECK(output_limit==CFG_PWM_FULL);
    current_ma=PROT_CURRENT_LIMIT_MA+1;ticks(PROT_CURRENT_TRIP-1);CHECK(!A_Protect_Fault());
    current_valid=0;ticks(1);current_valid=1;ticks(PROT_CURRENT_TRIP-1);CHECK(!A_Protect_Fault());
    ticks(1);CHECK(A_Protect_Fault()==PROT_FAULT_OVERCUR);CHECK(!torque_on);
    current_ma=0;ticks(PROT_COOLDOWN_TICKS-1);CHECK(!torque_on);ticks(1);CHECK(torque_on);CHECK(!A_Protect_Fault());
    temperature=PROT_TEMP_LIMIT_C*10;ticks(PROT_TEMP_DIV);CHECK(!torque_on);
    temperature=-32768;ticks(PROT_TEMP_DIV);CHECK(!torque_on);
    temperature=(PROT_TEMP_LIMIT_C-PROT_TEMP_HYST_C)*10+1;ticks(PROT_TEMP_DIV);CHECK(!torque_on);
    temperature--;ticks(PROT_TEMP_DIV);CHECK(torque_on);CHECK(!A_Protect_Fault());
    torque_on=0;temperature=PROT_TEMP_LIMIT_C*10;ticks(PROT_TEMP_DIV);
    temperature=250;ticks(PROT_TEMP_DIV);CHECK(!torque_on); /* Preserve an existing user release. */
    CHECK(A_Protect_SetTorque(50));int saved=output_limit;
    current_valid=0;ticks(100);CHECK(output_limit==saved);
    current_valid=1;current_ma=2000;ticks(1);CHECK(output_limit<saved);
    CHECK(!A_Protect_SetTorque(101));CHECK(A_Protect_GetTorque()==50);
    CHECK(A_Protect_SetTorque(0));current_ma=0;ticks(100);
    CHECK(output_limit==PROT_LIMIT_MIN); /* A zero setting must never recover to full PWM. */
    CHECK(A_Protect_SetTorque(100));ticks(1);CHECK(output_limit==CFG_PWM_FULL);
    check_stall();
    printf("protection failures=%d\n",failures);return failures?1:0;
}
