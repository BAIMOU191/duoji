#ifndef __A_CALIB_MIN_H__
#define __A_CALIB_MIN_H__

#include <stdint.h>

/*
 * A_Calib_Min.h —— 标定固件：开环激励电机辨识电压域模型，串口打印标定块整节替换 App_Model.h 对应型号的电机标定区。
 * 全程约50秒；需输出轴可自由转动，行程两端各留保护带。
 */

void A_Calib_Min_Init(void);       /* 初始化标定状态机 */
void A_Calib_Min_Tasks_Init(void); /* 注册标定、IAP与喂狗任务 */

#endif /* __A_CALIB_MIN_H__ */
