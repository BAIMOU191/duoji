#pragma once

/* A_StatusLed.h —— 按故障与工作状态选灯效，只读其他模块状态 */

#define STATUS_LED_PERIOD_MS  20U   /* 调用周期(ms)，彩虹速度按此设计 */

void A_StatusLed_Task(void);        /* 每 STATUS_LED_PERIOD_MS 调一次，刷新一帧 */
