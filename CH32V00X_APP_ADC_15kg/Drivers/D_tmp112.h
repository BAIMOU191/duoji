#ifndef __D_TMP112_H__
#define __D_TMP112_H__

#include "ch32v00X.h"

/* TMP112数字温度传感器驱动(I2C)，12位精度，0.1°C单位 */

#define TMP112_ADDR         0x48    /* I2C从机地址，ADD0接地 */

#define TMP112_REG_TEMP     0x00    /* 温度寄存器(只读，2字节) */
#define TMP112_REG_CONFIG   0x01    /* 配置寄存器(读写，2字节) */
#define TMP112_CFG_DEFAULT  0x60A0     /* 默认配置：12位，连续，1Hz */

uint8_t D_TMP112_Init(void);                            /* 初始化并写入默认配置 */
int16_t D_TMP112_Read_Temp(void);                        /* 读取温度，0.1°C单位，-32768=失败 */

#endif /* __D_TMP112_H__ */
