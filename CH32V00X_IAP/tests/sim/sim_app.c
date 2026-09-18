/*
 * sim_app.c —— 仿真里的 APP 侧：APP 工程 System/iap.c 原样编译(取电位器ADC版)
 * 硬件信息页落在模拟 Flash 上，串口发送进队列并模拟"发完才空闲"，复位换成通知主程序换模式。
 * 舵机的运动控制不在仿真范围内：这里只跑 IAP 相关的那条串口链路。
 */
#include <string.h>
#include "sim.h"
#include "ch32v00X.h"
#include "A_Config.h"
#include "flash.h"

typedef enum { FLASH_BUSY = 1, FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE, FLASH_TIMEOUT } FLASH_Status;
#define Start_Mode_BOOT ((uint32_t)0x00004000)
#define IAP_HWINFO_PAGE ((const uint8_t *)(sim_flash + 0xF600U))

Config_t g_config;

FLASH_Status FLASH_ROM_ERASE(uint32_t addr, uint32_t len)
{
    memset(sim_flash + (addr - FLASH_BASE), 0xFF, len);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ROM_WRITE(uint32_t addr, uint32_t *buf, uint32_t len)
{
    memcpy(sim_flash + (addr - FLASH_BASE), buf, len);
    return FLASH_COMPLETE;
}

static int s_released;   /* 卸力调用次数，场景脚本可以查 */
void A_Servo_Release(uint8_t high) { (void)high; s_released++; }

void RCC_ClearFlag(void) {}
void SystemReset_StartMode(uint32_t mode) { (void)mode; }
static void NVIC_SystemReset(void) { sim_request_boot(); }

static int s_tx_busy_ms;  /* 应答还要发多久；模拟 115200 下约 11 字节/ms */
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len)
{
    sim_tx_push(data, (int)len);
    s_tx_busy_ms = len / 11 + 1;
    return 1U;
}
uint8_t D_UART_Tx_Idle(void) { return (uint8_t)(s_tx_busy_ms == 0); }

#include "iap.c"

void app_sim_power_on(uint8_t servo_id)
{
    s_pos = 0U;
    s_reset_pending = 0U;
    s_tx_busy_ms = 0;
    memset(&g_config, 0, sizeof(g_config));
    g_config.servo_id  = servo_id;
    g_config.baud_code = 5U;                    /* 参数页里的波特率，仿真不模拟实际速率 */
    IAP_Init();                                 /* 出厂空白时写硬件信息页 */
}

/* 每毫秒一拍：排空接收队列 -> 执行 -> 回复，再看要不要复位进 Boot */
void app_sim_step(int ms)
{
    uint8_t byte;
    int     i;

    for (i = 0; i < ms; i++)
    {
        while (sim_rx_pop(&byte)) IAP_Rx_Deal(byte);
        IAP_Service();
        if (sim_reset_pending()) return;
        if (s_tx_busy_ms > 0) s_tx_busy_ms--;
    }
}
