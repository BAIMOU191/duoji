/* D_mt6701.c MT6701 14位磁编码器驱动，I2C读取原始角度 */

#include "D_mt6701.h"
#include "D_i2c.h"

/* 总线探测，0=成功 */
uint8_t D_MT6701_Init(void)
{
    uint8_t buf[2];
    return D_I2C_Read(MT6701_ADDR, 0x03, buf, 2);
}

/* 读角度(厘度，方向已取反)[0,35999]，失败返回 MT6701_ANGLE_ERROR */
uint16_t D_MT6701_Read_Angle(void)
{
    uint16_t raw = D_MT6701_Read_Raw();

    if (raw == MT6701_ANGLE_ERROR) return MT6701_ANGLE_ERROR;
    return (uint16_t)(((uint32_t)raw * 36000U) / MT6701_RAW_MAX);
}

/* 读原始14位计数(方向已取反)，1计数=2.197厘度；需要亚厘度分辨率(如测噪声)时用它 */
uint16_t D_MT6701_Read_Raw(void)
{
    uint8_t buf[2];
    uint16_t raw;

    if (D_I2C_Read(MT6701_ADDR, 0x03, buf, 2) != 0) return MT6701_ANGLE_ERROR;

    raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);    /* 合并高低字节 */
    return (uint16_t)(MT6701_RAW_MAX - 1U) - raw;      /* 方向取反 */
}
