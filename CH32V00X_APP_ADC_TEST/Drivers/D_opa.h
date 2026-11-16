#ifndef __D_OPA_H__
#define __D_OPA_H__

#include "ch32v00X.h"

/* OPA1运算放大器驱动：PA2正输入，8x增益 */

void D_OPA1_Init(void); /* 初始化OPA1：8x增益 + PGA基准VB=VDD/4 */

#endif /* __D_OPA_H__ */
