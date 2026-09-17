#ifndef __A_CALIB_MIN_H__
#define __A_CALIB_MIN_H__

#include <stdint.h>

/*
 * A_Calib_Min.h —— 自动标定程序(main.c 定义 APP_MODE_CALIB 启用，上电1秒后自动开始，结束后串口打印可粘贴的 #define)
 * 输出速度斜率、动摩擦、起转PWM、装配方向、机械时间常数、纯延迟、制动比，其余参数由 A_Parameter.h 派生。
 * 开跑前：输出轴能自由转动且当前位置读得出角度；电位器版会自动预定位，行程两端各留5度保护带。
 * 中止码：1=满PWM也不动，2=方向测不出，3=编码器读不到，4=冲出安全行程，5=预定位超时。
 */

void A_Calib_Min_Init(void);       /* 初始化标定状态机 */
void A_Calib_Min_Tasks_Init(void); /* 只注册标定拍与喂狗任务 */

#endif /* __A_CALIB_MIN_H__ */
