#ifndef __D_SPI_H__
#define __D_SPI_H__

#include "ch32v00X.h"

/* SPI1主机驱动：PC5=SCK，PC6=MOSI，6MHz，Mode0 */

void D_SPI1_Init(void);                          /* 初始化SPI1外设与GPIO */
void D_SPI1_Send_Buf(uint8_t *buf, uint16_t len); /* 阻塞式发送len字节 */

#endif /* __D_SPI_H__ */
