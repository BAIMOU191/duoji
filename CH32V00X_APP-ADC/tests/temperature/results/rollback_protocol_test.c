#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include "D_tmp112.c"
#include "A_UartCmd.c"
Config_t g_config;
static uint16_t raw_temp, written_config;
static uint8_t bus_failure;
static char reply[64];
static int failures;
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);failures++;}}while(0)
uint8_t D_I2C_Read(uint8_t addr,uint8_t reg,uint8_t *p,uint8_t n) {
    CHECK(addr==0x48);CHECK(n==2);
    if(bus_failure)return 1;
    uint16_t value=reg==TMP112_REG_TEMP?raw_temp:0x60A0;
    p[0]=(uint8_t)(value>>8);p[1]=(uint8_t)value;return 0;
}
uint8_t D_I2C_Write(uint8_t addr,uint8_t reg,uint8_t *p,uint8_t n) {
    CHECK(addr==0x48);CHECK(reg==TMP112_REG_CONFIG);CHECK(n==2);
    written_config=(uint16_t)((p[0]<<8)|p[1]);return bus_failure;
}
int16_t A_Temperature_Read(void) {return D_TMP112_Read_Temp();}
uint16_t A_Voltage_Read(void) {return 1234;}
uint8_t D_UART1_Tx_Write(const uint8_t *p,uint8_t n) {CHECK(n<sizeof(reply));memcpy(reply,p,n);reply[n]=0;return 1;}
uint8_t D_UART1_Rx_Get(uint8_t *p) {(void)p;return 0;}
void D_UART_Service(void) {}
void D_UART_SetBaud_Deferred(uint32_t b) {(void)b;}
void IAP_Rx_Deal(uint8_t b) {(void)b;}
void A_Config_MarkDirty(void) {}
void A_Config_Default(uint8_t k) {(void)k;}
uint32_t A_Config_Baudrate(void) {return 115200;}
uint16_t A_Servo_GetPositionPwm(void) {return 1500;}
void A_Servo_Submit(uint16_t p,uint16_t t) {(void)p;(void)t;}
void A_Servo_Release(uint8_t h) {(void)h;}
void A_Servo_RestoreTorque(void) {}
void A_Servo_Pause(void) {}
void A_Servo_Resume(void) {}
void A_Servo_Stop(void) {}
void A_Servo_ApplyConfig(void) {}
uint8_t A_Servo_SetMode(uint8_t m) {(void)m;return 1;}
uint8_t A_Servo_CalibrateMid(void) {return 1;}
uint8_t A_Servo_SetTravelEnd(uint8_t m) {(void)m;return 1;}
uint8_t A_Servo_SaveStartup(void) {return 1;}
uint8_t A_Protect_SetTorque(uint8_t p) {(void)p;return 1;}
uint8_t A_Protect_GetTorque(void) {return 100;}
static void query(void) {const char *p="#000PRTV!";while(*p)Uart_Frame_Byte((uint8_t)*p++);}
int main(void) {
    CHECK(!D_TMP112_Init());CHECK(written_config==0x60A0);
    raw_temp=0x1900;CHECK(D_TMP112_Read_Temp()==250);query();CHECK(!strcmp(reply,"#000PDP1500T25V12.3!"));puts(reply);
    raw_temp=0xF600;CHECK(D_TMP112_Read_Temp()==-100);query();CHECK(!strcmp(reply,"#000PDP1500T-10V12.3!"));
    raw_temp=0;CHECK(D_TMP112_Read_Temp()==0);query();CHECK(!strcmp(reply,"#000PDP1500T0V12.3!"));
    for(int value=-880;value<=2000;value++) {
        raw_temp=(uint16_t)(value*16);
        CHECK(D_TMP112_Read_Temp()==value*625/1000);
    }
    bus_failure=1;CHECK(D_TMP112_Init()==1);CHECK(D_TMP112_Read_Temp()==-32768);
    query();puts(reply);CHECK(!strcmp(reply,"#000PDP1500T0V12.3!"));
    bus_failure=0;raw_temp=0x3200;query();CHECK(!strcmp(reply,"#000PDP1500T50V12.3!"));
    printf("temperature and protocol failures=%d\n",failures);return failures?1:0;
}
