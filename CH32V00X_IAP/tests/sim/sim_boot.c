/*
 * sim_boot.c —— 仿真里的 Boot 侧：System/iap.c 原样编译
 * 只把硬件换掉：Flash 换成模拟数组(擦除写 0xFF、编程只能把 1 写成 0)，串口换成队列，
 * 毫秒计时换成调用方给的时间预算，跳转 APP 换成通知仿真主程序换模式。
 * Loop 和 IAP_Run 是真实代码，预算用尽时靠 longjmp 把控制权交回主程序。
 */
#include <setjmp.h>
#include <string.h>
#include "sim.h"
#include "ch32v00X.h"

#define BOOT_HOST_TEST
#define __FLASH_H
#define FLASH_PTR(off) ((const uint8_t *)(sim_flash + (off)))
#define IWDG_FEED()    ((void)0)

static int unlocked;

void FLASH_Unlock_Fast(void) { unlocked = 1; }

void FLASH_ErasePage_Fast(uint32_t addr)
{
    if (unlocked) memset(sim_flash + (addr - FLASH_BASE), 0xFF, 256);
}

void FLASH_EraseBlock_32K_Fast(uint32_t addr)
{
    (void)addr;
    if (unlocked) memset(sim_flash, 0xFF, 0x8000);
}

void CH32_IAP_Program(uint32_t adr, uint32_t *buf)
{
    uint8_t  bytes[256];
    uint32_t off = adr - FLASH_BASE;
    int      i;

    if (!unlocked) return;
    memcpy(bytes, buf, sizeof(bytes));
    for (i = 0; i < 256; i++) sim_flash[off + i] &= bytes[i];   /* 编程只能把 1 写成 0 */
}

static jmp_buf s_esc;
static int     s_budget;       /* 本次还能过多少个 1ms */
static int     s_window_left;  /* 上电握手窗口还剩多少 ms */
static uint8_t s_soft;

static void    Hw_Init(void) {}
static uint8_t Reset_WasSoftware(void) { return s_soft; }
static uint8_t Uart_Poll(uint8_t *byte) { return sim_rx_pop(byte); }
static uint8_t Tick_Poll(void) { if (s_budget <= 0) longjmp(s_esc, 1); s_budget--; return 1U; }
static void    Uart_Send(const uint8_t *data, uint8_t len) { sim_tx_push(data, (int)len); }
static void    Jump_App(void) { sim_request_app(); longjmp(s_esc, 2); }

#include "iap.c"

void boot_sim_power_on(int soft)
{
    s_pos = 0U; s_started = 0U; s_img_len = 0U; s_img_crc = 0U;
    unlocked = 0;
    s_soft = (uint8_t)(soft != 0);
    s_window_left = soft ? 0 : (int)HELLO_WINDOW_MS;
}

/* 跑 ms 毫秒：还在窗口期就按剩余窗口调用 Loop，其余时间进升级循环 */
void boot_sim_step(int ms_in)
{
    volatile int ms = ms_in;

    if (setjmp(s_esc) != 0)                       /* 预算用尽，或已经跳去 APP */
    {
        if (s_window_left > 0)
        {
            s_window_left -= ms - s_budget;
            if (s_window_left < 0) s_window_left = 0;
        }
        return;
    }

    s_budget = ms;
    if (s_window_left > 0)
    {
        if (Loop((uint16_t)s_window_left) == 0U)  /* 窗口到时没等到握手 */
        {
            s_window_left = 0;
            if (App_Bootable()) Jump_App();
        }
        else
        {
            s_window_left = 0;                    /* 收到握手，留在 Boot */
        }
    }
    (void)Loop(0U);
}

int boot_sim_flag_state(void) { return (int)Flag_State(); }
