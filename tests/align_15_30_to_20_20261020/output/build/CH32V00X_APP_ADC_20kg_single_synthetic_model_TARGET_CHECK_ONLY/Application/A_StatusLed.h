#pragma once

/* A_StatusLed.h —— 状态指示灯：按保护故障与舵机工作状态选灯效，只读其他模块的状态 */

#define STATUS_LED_PERIOD_MS  20U   /* A_StatusLed_Task 的调用周期，彩虹渐变速度按它设计 */

void A_StatusLed_Task(void);        /* 每 STATUS_LED_PERIOD_MS 调用一次，刷新一帧 */
