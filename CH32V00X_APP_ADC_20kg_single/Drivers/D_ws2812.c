/* D_ws2812.c WS2812B RGB LED驱动，6MHz SPI位流模拟单线归零码 */

#include "D_ws2812.h"
#include "D_spi.h"
#include "debug.h"

#define WS_BIT_0     0xC0U   /* bit-0编码：2高+6低 */
#define WS_BIT_1     0xF8U   /* bit-1编码：5高+3低 */
#define WS_FRAME_LEN 24U     /* 每帧24字节：3色字节各展开8位 */

static uint8_t s_frame[WS_FRAME_LEN]; /* SPI发送帧缓冲区 */

/* 颜色字节展开为8个SPI编码字节，高位优先 */
static void WS2812_Encode(uint8_t color, uint8_t *out)
{
    uint8_t i;
    for (i = 0; i < 8U; i++)
    {
        out[i] = (color & 0x80U) ? WS_BIT_1 : WS_BIT_0;
        color <<= 1U;
    }
}

/* 初始化驱动，发送熄灭帧 */
void D_WS2812_Init(void)
{
    D_WS2812_Off();
}

/* 按GRB顺序编码后经SPI发送 */
void D_WS2812_Send(uint8_t r, uint8_t g, uint8_t b)
{
    WS2812_Encode(g, s_frame + 0U);   /* GRB顺序：先绿 */
    WS2812_Encode(r, s_frame + 8U);   /* 再红           */
    WS2812_Encode(b, s_frame + 16U);  /* 最后蓝         */

    D_SPI1_Send_Buf(s_frame, WS_FRAME_LEN);
    Delay_Us(60U); /* 复位锁存延时，须大于50us */
}

/* 发送全零帧熄灭LED */
void D_WS2812_Off(void)
{
    D_WS2812_Send(0U, 0U, 0U);
}
