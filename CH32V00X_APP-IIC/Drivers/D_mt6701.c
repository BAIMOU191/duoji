/* D_mt6701.c MT6701 14位磁编码器驱动，I2C读取原始角度 */

#include "D_mt6701.h"
#include "D_i2c.h"

/*
 * @fn      D_MT6701_Init
 * @brief   总线探测，读取寄存器确认设备应答
 * @param   无
 * @return   0=成功，1=I2C错误
 */
uint8_t D_MT6701_Init(void)
{
    uint8_t buf[2];
    return D_I2C_Read(MT6701_ADDR, 0x03, buf, 2);
}

/*
 * @fn      D_MT6701_Read_Angle
 * @brief   读取并换算角度(厘度)，方向取反
 * @param   无
 * @return  角度[0,35999]，I2C错误返回MT6701_ANGLE_ERROR
 */
uint16_t D_MT6701_Read_Angle(void)
{
    uint16_t raw = D_MT6701_Read_Raw();

    if (raw == MT6701_ANGLE_ERROR) return MT6701_ANGLE_ERROR;
    return (uint16_t)(((uint32_t)raw * 36000U) / MT6701_RAW_MAX);
}

/*
 * @fn      D_MT6701_Read_Raw
 * @brief   读取原始14位计数，方向已取反，不做厘度换算
 * @param   无
 * @return  原始计数[0,16383]，I2C错误返回MT6701_ANGLE_ERROR
 *
 * 一个计数对应 36000/16384 = 2.197 厘度。D_MT6701_Read_Angle 换算成整数厘度
 * 时会把不足1厘度的部分截断，相邻计数的厘度步长变成2/2/2/2/2/3的不均匀序列，
 * 且小于1个计数的传感器抖动会被完全抹平。标定测噪声、以及任何需要判断
 * "传感器到底抖不抖"的场合，必须用本函数取原始计数。
 */
uint16_t D_MT6701_Read_Raw(void)
{
    uint8_t buf[2];
    uint16_t raw;

    if (D_I2C_Read(MT6701_ADDR, 0x03, buf, 2) != 0) return MT6701_ANGLE_ERROR;

    raw = ((uint16_t)buf[0] << 6) | (buf[1] >> 2);    /* 合并高低字节 */
    return (uint16_t)(MT6701_RAW_MAX - 1U) - raw;      /* 方向取反 */
}
