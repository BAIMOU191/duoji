#ifndef __D_IWDG_H__
#define __D_IWDG_H__

#include "ch32v00X.h"

/* 独立看门狗(IWDG)驱动：LSI时钟+256分频 */

#define IWDG_RELOAD_VALUE  1000U  /* 喂狗重装值，超时约2~6s(取决于LSI频率) */

void     D_IWDG_Init(void);        /* 初始化并启动独立看门狗 */
void     D_IWDG_Feed(void);        /* 喂狗(重装计数器) */

#endif /* __D_IWDG_H__ */
