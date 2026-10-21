#ifndef __A_CALIB_MIN_H__
#define __A_CALIB_MIN_H__

#include <stdint.h>

/*
 * A_Calib_Min.h —— 标定固件(main.c 定义 APP_MODE_CALIB 启用，上电1秒后自动开始，约3~4分钟)
 * 在额定电压下开环激励电机并在 MCU 上辨识电压域模型，串口(当前配置波特率，默认115200)打印标定块，
 * 整节替换 A_Config.h 第2节即可；自检不过时块里带 #error。
 * 开跑前：输出轴能自由转动且当前位置读得出角度；行程两端各留5度保护带。
 */

void A_Calib_Min_Init(void);       /* 初始化标定状态机 */
void A_Calib_Min_Tasks_Init(void); /* 注册标定、IAP与喂狗任务 */

#endif /* __A_CALIB_MIN_H__ */
