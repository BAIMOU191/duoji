/* D_tmp112.c TMP112温度传感器驱动，I2C读写寄存器，输出0.1°C单位 */

#include "D_tmp112.h"
#include "D_i2c.h"

static uint8_t TMP112_Get_Config(uint16_t *config);
static uint8_t TMP112_Read_Temp_Raw(uint16_t *temp_raw);

/* 读配置验证通信，再写默认配置(12位，连续，1Hz)；0=成功 */
uint8_t D_TMP112_Init(void)
{
    uint8_t  cfg_buf[2];
    uint16_t config;

    if (TMP112_Get_Config(&config) != 0)
        return 1; /* I2C通信失败 */

    config = TMP112_CFG_DEFAULT;
    cfg_buf[0] = (uint8_t)(config >> 8);
    cfg_buf[1] = (uint8_t)(config & 0xFF);

    return D_I2C_Write(TMP112_ADDR, TMP112_REG_CONFIG, cfg_buf, 2);
}

/* 读16位原始温度寄存器，0=成功 */
static uint8_t TMP112_Read_Temp_Raw(uint16_t *temp_raw)
{
    uint8_t buf[2];

    if (temp_raw == NULL) return 1;

    if (D_I2C_Read(TMP112_ADDR, TMP112_REG_TEMP, buf, 2) != 0)
        return 1;

    *temp_raw = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/* 读温度，0.1°C(250=25.0°C)，-32768=失败 */
int16_t D_TMP112_Read_Temp(void)
{
    uint16_t raw;
    int16_t  temp_12bit;
    int32_t  temp_tenths;

    if (TMP112_Read_Temp_Raw(&raw) != 0)
        return -32768;

    temp_12bit  = (int16_t)raw >> 4;                  /* 提取12位有符号温度值 */
    temp_tenths = ((int32_t)temp_12bit * 625) / 1000; /* 转换为0.1°C单位 */

    return (int16_t)temp_tenths;
}

/* 读16位配置寄存器，0=成功 */
static uint8_t TMP112_Get_Config(uint16_t *config)
{
    uint8_t buf[2];

    if (config == NULL) return 1;

    if (D_I2C_Read(TMP112_ADDR, TMP112_REG_CONFIG, buf, 2) != 0)
        return 1;

    *config = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}
