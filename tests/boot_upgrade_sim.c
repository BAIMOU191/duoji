/* Actual Boot protocol/erase/write code, with Flash and UART boundaries in RAM. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "A_Config.h"
#include "../CH32V00X_IAP/System/iap.h"
#define __FLASH_H
#define BOOT_HOST_TEST
#define FLASH_BASE 0U
uint8_t upgrade_flash[0x10000];
#define FLASH_PTR(off) (upgrade_flash+(off))
#define IWDG_FEED() ((void)0)
static uint8_t jumped;
static void Hw_Init(void) {}
static uint8_t Uart_Poll(uint8_t *p) { (void)p;return 0; }
static uint8_t Tick_Poll(void) { return 1; }
static uint8_t Reset_WasSoftware(void) { return 0; }
static void Uart_Send(const uint8_t *p,uint8_t n);
static void Jump_App(void) { jumped=1; }
void FLASH_Unlock_Fast(void) {}
void FLASH_ErasePage_Fast(uint32_t off) {
    assert(!(off%256) && off<=0xF700);memset(upgrade_flash+off,0xff,256);
}
void FLASH_EraseBlock_32K_Fast(uint32_t off) { assert(off==0);memset(upgrade_flash,0xff,0x8000); }
void CH32_IAP_Program(uint32_t off,uint32_t *words) {
    assert(!(off%256) && off<=0xF700);
    for(unsigned i=0;i<256;i++)upgrade_flash[off+i]&=((uint8_t *)words)[i];
}
#include "../CH32V00X_IAP/System/iap.c"
static uint8_t reply_status;
static void Uart_Send(const uint8_t *p,uint8_t n) {
    assert(n>=12 && p[0]==0xaa && p[1]==0x55);reply_status=p[5];
}
static void boot_request(uint8_t cmd,const uint8_t *data,uint16_t n) {
    uint8_t f[11+IAP_DATA_MAX]={0xaa,0x55,cmd,(uint8_t)n,(uint8_t)(n>>8)};
    if(n)memcpy(f+5,data,n);
    Put32(f+5+n,Crc32(f+2,3+n));f[9+n]=0x55;f[10+n]=0xaa;
    unsigned frames=0;
    for(unsigned i=0;i<11U+n;i++)if(Frame_Byte(f[i])) { Frame_Process(0);frames++; }
    assert(frames==1);
}
static void config_page(uint32_t off,uint8_t id,uint8_t baud) {
    uint8_t *p=upgrade_flash+off;
    p[0]=p[1]=p[2]=0;p[3]=id;p[4]=baud;
    uint16_t crc=Crc16(p,CONFIG_CRC_OFFSET);p[250]=(uint8_t)crc;p[251]=(uint8_t)(crc>>8);
    Put32(p+252,CONFIG_COMMITTED);
}
void boot_upgrade_fixture(uint8_t id,uint8_t baud,uint8_t fallback_b) {
    uint8_t image[1024],start[8],page[260];
    memset(upgrade_flash,0x5a,sizeof(upgrade_flash));memset(upgrade_flash+FLAG_OFFSET,0xff,256);
    memset(image,0x3c,sizeof(image));Put32(image+FWINFO_OFFSET,FWINFO_MAGIC);
    const uint8_t hw[]={FW_SERVO_TYPE,FW_HW_VERSION,FW_VOLTAGE,FW_TORQUE};
    memcpy(image+FWINFO_OFFSET+4,hw,4);
    config_page(CONFIG_SLOT_A,id,baud);config_page(CONFIG_SLOT_B,id,baud);
    if(fallback_b)upgrade_flash[CONFIG_SLOT_A+CONFIG_CRC_OFFSET]^=1;
    uint8_t *h=upgrade_flash+HWINFO_OFFSET;Put32(h,HWINFO_MAGIC);h[4]=3;memcpy(h+5,hw,4);Put32(h+28,Crc32(h,28));
    s_pos=0;s_started=0;jumped=0;
    Put32(start,sizeof(image));Put32(start+4,Crc32(image,sizeof(image)));
    boot_request(CMD_START,start,8);assert(reply_status==ST_OK && !App_Bootable());
    for(unsigned i=0;i<APP_DATA_END;i++)assert(upgrade_flash[i]==0xff);
    assert(upgrade_flash[FLAG_OFFSET+FLAG_ID_OFFSET]==id && upgrade_flash[FLAG_OFFSET+FLAG_BAUD_OFFSET]==baud);
    /* Power loss after erase: only the durable snapshot remains. */
    s_pos=0;s_started=0;s_keep_id=s_keep_baud=s_keep_valid=0;s_hw_snapshot_valid=0;
    boot_request(CMD_START,start,8);assert(reply_status==ST_OK);
    assert(s_keep_id==id && s_keep_baud==baud && s_keep_valid);
    /* Incomplete image never becomes bootable. */
    boot_request(CMD_FINISH,NULL,0);assert(reply_status==ST_IMAGE_CRC && !jumped && !App_Bootable());
    for(unsigned off=0;off<sizeof(image);off+=256) {
        Put32(page,off);memcpy(page+4,image+off,256);
        boot_request(CMD_WRITE,page,sizeof(page));assert(reply_status==ST_OK);
    }
    boot_request(CMD_FINISH,NULL,0);assert(reply_status==ST_OK && jumped && App_Bootable());
    assert(!memcmp(upgrade_flash,image,sizeof(image)));
    for(unsigned i=sizeof(image);i<APP_DATA_END;i++)assert(upgrade_flash[i]==0xff);
    for(unsigned i=0xF800;i<sizeof(upgrade_flash);i++)assert(upgrade_flash[i]==0x5a);
}
