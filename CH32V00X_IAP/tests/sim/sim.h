/*
 * sim.h —— 整机仿真的公共接口
 * 一台舵机 = 一块 62K 模拟 Flash + Boot 协议代码 + APP 协议代码，两侧都是工程里的源码本体。
 */
#pragma once
#include <stdint.h>

#define SIM_FLASH_SIZE 0xF800U

extern uint8_t sim_flash[SIM_FLASH_SIZE];

/* 串口：上位机 -> 舵机、舵机 -> 上位机 */
uint8_t sim_rx_pop(uint8_t *byte);
void    sim_tx_push(const uint8_t *data, int len);

/* 复位请求：APP 请求进 Boot、Boot 请求进 APP */
void    sim_request_boot(void);
void    sim_request_app(void);
int     sim_reset_pending(void);

/* Boot 侧 */
void    boot_sim_power_on(int soft);   /* soft=1：APP 主动复位过来，没有握手窗口 */
void    boot_sim_step(int ms);
int     boot_sim_flag_state(void);

/* APP 侧 */
void    app_sim_power_on(uint8_t servo_id);
void    app_sim_step(int ms);
