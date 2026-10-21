/* Actual APP config loading and IAP snapshot/provisioning after the real Boot fixture. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
extern uint8_t upgrade_flash[0x10000];
void boot_upgrade_fixture(uint8_t id,uint8_t baud,uint8_t fallback_b);
#define FLASH_BASE 0U
#define FLASH_COMPLETE 0U
#define Start_Mode_BOOT 1U
#define IAP_HWINFO_PAGE (upgrade_flash+0xF600)
#define IAP_FLAG_PAGE (upgrade_flash+0xF700)
static uint32_t test_image_tag=1234;
static unsigned reset_count;
static uint8_t tx_idle=1,save_fail;
#define CONFIG_IMAGE_TAG() test_image_tag
uint8_t FLASH_ROM_ERASE(uint32_t address,uint32_t size);
uint8_t FLASH_ROM_WRITE(uint32_t address,const void *data,uint32_t size);
void RCC_ClearFlag(void) {}
void SystemReset_StartMode(uint8_t mode) { (void)mode; }
void NVIC_SystemReset(void) { reset_count++; }
#include "A_Config.c"
#include "iap.c"
static Config_t stored;
static uint8_t stored_valid,stored_version[3];
uint16_t FLASH_Config_Load(void *data,uint16_t n,uint8_t v[3]) {
    if(!stored_valid)return 0;
    assert(n==sizeof(stored));memcpy(data,&stored,n);memcpy(v,stored_version,3);return n;
}
uint8_t FLASH_Config_Save(const void *data,uint16_t n,const uint8_t v[3]) {
    if(save_fail)return 0;
    assert(n==sizeof(stored));memcpy(&stored,data,n);memcpy(stored_version,v,3);stored_valid=1;return 1;
}
uint8_t FLASH_ROM_ERASE(uint32_t address,uint32_t size) {
    assert(address==HW_INFO_ADDR && size==256);memset(upgrade_flash+address,0xff,size);return FLASH_COMPLETE;
}
uint8_t FLASH_ROM_WRITE(uint32_t address,const void *data,uint32_t size) {
    assert(address==HW_INFO_ADDR && size==256);memcpy(upgrade_flash+address,data,size);return FLASH_COMPLETE;
}
void A_Servo_Release(uint8_t mode) { (void)mode; }
uint8_t D_UART1_Tx_Write(const uint8_t *data,uint8_t n) { (void)data;(void)n;return 1; }
uint8_t D_UART_Tx_Idle(void) { return tx_idle; }
int main(void) {
    for(unsigned fallback=0;fallback<2;fallback++) {
        boot_upgrade_fixture(42,8,(uint8_t)fallback);
        uint8_t id=0,baud=0;assert(IAP_PreservedIdentityRead(&id,&baud) && id==42 && baud==8);
        /* Even rewriting the identical image resets config, because Boot erased both pages. */
        stored_valid=0;test_image_tag=1234;A_Config_Init();
        Config_t actual=g_config;Config_FillDefaults();g_config.servo_id=42;g_config.baud_code=8;
        assert(!memcmp(&actual,&g_config,sizeof(actual)) && !memcmp(&stored,&actual,sizeof(stored)));
        IAP_Init();IAP_HwInfo_t hw;assert(IAP_HwInfoRead(&hw));
        assert(hw.servo_type==FW_SERVO_TYPE && hw.torque==FW_TORQUE);
        assert(hw.max_speed_dps==MOTOR_RATED_VEL_DPS && hw.max_accel_dps2==MOTOR_RATED_ACC_DPS2);
        assert(hw.max_decel_dps2==MOTOR_RATED_DEC_DPS2 && hw.min_step_cdeg==MOTOR_MIN_STEP_CDEG);
        /* Same-image power cycles keep user changes; a changed image keeps only ID/baud. */
        g_config.startup_pwm=1700;g_config.hold_kp=12;A_Config_MarkDirty();A_Config_SaveTask();
        A_Config_Init();assert(g_config.startup_pwm==1700 && g_config.hold_kp==12);
        test_image_tag++;A_Config_Init();actual=g_config;
        Config_FillDefaults();g_config.servo_id=42;g_config.baud_code=8;
        assert(!memcmp(&actual,&g_config,sizeof(actual)));
        /* A manual full reset saved after upgrade must survive later power cycles. */
        A_Config_Default(0);A_Config_SaveTask();A_Config_Init();
        assert(g_config.servo_id==0 && g_config.baud_code==5);
        /* An immediate upgrade cannot snapshot an older asynchronously saved identity. */
        g_config.servo_id=53;g_config.baud_code=6;A_Config_MarkDirty();
        s_reset_pending=1;reset_count=0;tx_idle=0;IAP_Service();assert(!reset_count && s_dirty);
        tx_idle=1;save_fail=1;IAP_Service();assert(!reset_count && s_dirty);
        save_fail=0;IAP_Service();assert(reset_count==1 && !s_dirty);
        assert(stored.servo_id==53 && stored.baud_code==6);s_reset_pending=0;
    }
    puts("PASS: Boot erase/write, power-loss retry, A/B identity, APP new defaults, hardware provisioning, full reset and identity flush before upgrade");
    return 0;
}
