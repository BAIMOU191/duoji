#ifndef __D_WS2812_H__
#define __D_WS2812_H__

#include <stdint.h>

/* WS2812B RGB LED驱动，SPI位流模拟单线协议 */

void D_WS2812_Init(void);                       /* 初始化驱动，发送熄灭帧 */
void D_WS2812_Send(uint8_t r, uint8_t g, uint8_t b); /* 发送RGB颜色帧 */
void D_WS2812_Off(void);                        /* 发送全零帧熄灭LED */

#endif /* __D_WS2812_H__ */
