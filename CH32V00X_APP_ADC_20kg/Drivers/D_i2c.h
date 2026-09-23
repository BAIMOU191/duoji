#ifndef __D_I2C_H__
#define __D_I2C_H__

#include "ch32v00X.h"

/* I2C1总线驱动：PC1=SDA，PC2=SCL，400kHz */

#define I2C_TIMEOUT  1000U   /* 有界等待：避免编码器异常拖垮1ms控制任务 */

void    D_Bus_I2C_Init(void);                                           /* 初始化I2C1总线 */
uint8_t D_I2C_Write(uint8_t devAddr, uint8_t regAddr, uint8_t *pData,
                  uint8_t len);                                       /* 写寄存器，0=成功，1=总线错误 */
uint8_t D_I2C_Read(uint8_t devAddr, uint8_t regAddr, uint8_t *pData,
                 uint8_t len);                                        /* 读寄存器，0=成功，1=总线错误 */

#endif /* __D_I2C_H__ */
