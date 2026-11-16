#ifndef __D_MT6701_H__
#define __D_MT6701_H__

#include <stdint.h>

/* MT6701 14位磁编码器驱动(I2C) */

#define MT6701_ADDR     0x06U    /* I2C从机地址 */
#define MT6701_RAW_MAX  16384U   /* 14位满量程 */
#define MT6701_ANGLE_ERROR  0xFFFFU /* I2C读取失败标记 */

uint8_t D_MT6701_Init(void);                     /* 总线探测，0=成功，1=失败 */
uint16_t D_MT6701_Read_Angle(void);               /* 读取角度(厘度)，失败返回MT6701_ANGLE_ERROR */
uint16_t D_MT6701_Read_Raw(void);                 /* 读取原始14位计数[0,16383]，失败返回MT6701_ANGLE_ERROR */

#endif /* __D_MT6701_H__ */
