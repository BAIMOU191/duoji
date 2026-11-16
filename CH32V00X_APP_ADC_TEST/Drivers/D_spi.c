/* D_spi.c SPI1主机驱动：6MHz，Mode0，8位，MSB优先 */

#include "D_spi.h"

#define SPI_WAIT_TIMEOUT  10000U  /* 发送/忙等待超时计数 */

/* 外设异常时关闭重开 */
static void SPI1_Recover(void)
{
    SPI_Cmd(SPI1, DISABLE);
    SPI_Cmd(SPI1, ENABLE);
}

/* 初始化SPI1外设与GPIO */
void D_SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_SPI1, ENABLE);

    /* PC5: SCK，复用推挽 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* PC6: MOSI，复用推挽 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;            /* 空闲低电平 */
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;          /* 第1边沿采样 */
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;            /* 软件NSS管理 */
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; /* 48/8=6MHz  */
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}

/* 阻塞式发送 len 字节，带超时保护 */
void D_SPI1_Send_Buf(uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        uint32_t timeout = SPI_WAIT_TIMEOUT;
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        {
            if (--timeout == 0)
            {
                SPI1_Recover();
                return;
            }
        }
        SPI_I2S_SendData(SPI1, buf[i]);
    }

    {
        uint32_t timeout = SPI_WAIT_TIMEOUT;
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
        {
            if (--timeout == 0)
            {
                SPI1_Recover();
                return;
            }
        }
    }
}
