/* A_Sensor.c —— 传感器聚合层：驱动原始计数换算为业务单位 */

#include "A_Sensor.h"
#include "D_mt6701.h"
#include "D_tmp112.h"
#include "D_adc.h"

/* 读磁编码器角度，厘度[0,35999]；MT6701_ANGLE_ERROR 与 ENCODER_ANGLE_ERROR 同值，透传即可 */
int32_t A_Encoder_Read(void)
{
    return (int32_t)D_MT6701_Read_Angle();
}


/* 读电池电压并换算为厘伏；乘积最大 4095*3300*49 ≈ 6.6e8，不会溢出uint32 */
uint16_t A_Voltage_Read(void)
{
    uint16_t raw_adc; /* 最近一次电压注入转换结果，ADC计数 */
    uint32_t mv;      /* 换算出的电池电压，毫伏  */

    raw_adc = D_ADC_Voltage_Read();
    mv = (uint32_t)raw_adc * ADC_VDD_MV * VOLTAGE_DIV_NUM
       / (ADC_FULL_SCALE * VOLTAGE_DIV_DEN);

    return (uint16_t)(mv / 10U); /* 毫伏 -> 厘伏 */
}

/* 读TMP112温度，返回0.1摄氏度；失败返回-32768 */
int16_t A_Temperature_Read(void)
{
    return D_TMP112_Read_Temp();
}

/* 读与PWM同步采到的绕组电流(mA)。值总是写出；返回值=是否采在驱动段(否则读到的0是测不到而非无电流) */
uint8_t A_Current_Read(uint16_t *current_ma)
{
    uint16_t raw_adc = 0; /* 已扣除零点偏置的ADC计数 */
    uint32_t ma;          /* 换算出的电流，毫安      */
    uint8_t  in_window;   /* 本块是否采在驱动段内    */

    if (current_ma == 0) return 0;
    in_window = D_ADC_Current_Read(&raw_adc);

    /* 满量程电流编译期算好，按计数线性插值，无溢出无除零 */
    ma = (uint32_t)raw_adc * CURRENT_FULL_SCALE_MA / ADC_FULL_SCALE;
    if (ma > 0xFFFFU) ma = 0xFFFFU;

    *current_ma = (uint16_t)ma;
    return in_window;
}
