#pragma once

/* A_Debug.h —— FireWater 运行诊断输出开关；通道定义见 A_Debug.c */

/* 调试开关：0=关闭；周期单位ms，可用编译宏覆盖 */
#ifndef TASK_FIREWATER_ENABLE
#define TASK_FIREWATER_ENABLE   0
#endif
#ifndef TASK_FIREWATER_PERIOD_MS
#define TASK_FIREWATER_PERIOD_MS  20U
#endif
#if TASK_FIREWATER_ENABLE && (TASK_FIREWATER_PERIOD_MS < 1 || TASK_FIREWATER_PERIOD_MS > 65535)
#error "TASK_FIREWATER_PERIOD_MS must be in [1, 65535]"
#endif

void A_Debug_Task(void);   /* 每 TASK_FIREWATER_PERIOD_MS 调一次，串口空闲时入队一帧 */
