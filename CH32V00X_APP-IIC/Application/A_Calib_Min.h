#ifndef __A_CALIB_MIN_H__
#define __A_CALIB_MIN_H__

#include <stdint.h>

/*
 * A_Calib_Min.h —— 唯一的自动标定程序(量产约10秒，金样全标约17秒)
 *
 * 工程只保留这一份标定代码。它输出3个单台参数(速度斜率、摩擦、方向)和
 * 3个型号级参数(时间常数、纯延迟、主动制动比)；其余量全部自动派生。
 *
 * 为什么只要这些：v_ss/a0/Ka/观测器增益都能由它们解析算出，
 * 推导与实测校验见 A_Calib_Min.c 文件头和 A_Config.c 的派生层。
 *
 * 使用：main.c 里改用 A_Calib_Min_Init + A_Calib_Min_Tasks_Init，
 * 上电1秒后自动开始，输出轴必须能自由转动。结束后串口打印6行 #define，
 * 直接替换 A_Parameter.h 中的标定参数。
 */

void A_Calib_Min_Init(void);        /* 初始化标定状态机 */
void A_Calib_Min_Tasks_Init(void);  /* 注册标定专用任务表(替代A_Tasks_Init) */

#endif /* __A_CALIB_MIN_H__ */
