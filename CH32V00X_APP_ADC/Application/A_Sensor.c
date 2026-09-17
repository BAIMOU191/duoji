/* A_Sensor.c —— 传感器聚合层：驱动原始计数换算为业务单位 */

#include "A_Sensor.h"
#include "D_mt6701.h"
#include "D_tmp112.h"
#include "D_adc.h"

#if ENCODER_MODE == 1
/* 读磁编码器角度，厘度[0,35999]；MT6701_ANGLE_ERROR 与 ENCODER_ANGLE_ERROR 同值，透传即可 */
int32_t A_Encoder_Read(void)
{
    return (int32_t)D_MT6701_Read_Angle();
}

#else
/* 读电位器角度，按实测端点线性换算，厘度[ANGLE_MIN,ANGLE_MAX]，可为负；抽头悬空按读取失败处理 */
int32_t A_Encoder_Read(void)
{
    uint16_t raw_q4 = 0; /* 电位器通道的均值，Q4计数 */
    int32_t  num;        /* 线性换算的分子           */
    int32_t  den;        /* 同样放大了SCALE倍的分母  */
    int32_t  cdeg;       /* 换算出的角度，厘度       */

    if (!D_ADC_Encoder_Read(&raw_q4)) return ENCODER_ANGLE_ERROR;

    /* 抽头悬空被下拉钳到地：没有角度，按读取失败处理 */
    if (raw_q4 < (uint16_t)(ENCODER_POT_ADC_FLOAT * D_ADC_ENCODER_SCALE))
        return ENCODER_ANGLE_ERROR;

    /* 标定常数乘SCALE与Q4对齐，分子最大约1.62e9，未越int32 */
    num = ((int32_t)raw_q4
         - (int32_t)(ENCODER_POT_ADC_MIN * D_ADC_ENCODER_SCALE))
        * (int32_t)ENCODER_POT_SPAN_CDEG;
    den = (int32_t)(ENCODER_POT_ADC_SPAN * D_ADC_ENCODER_SCALE);

    /* C的整数除法朝零截断，负半轴要反向补半个除数才是四舍五入 */
    cdeg = ((num >= 0) ? (num + den / 2) : (num - den / 2)) / den;

    if (cdeg < ENCODER_POT_ANGLE_MIN) return ENCODER_POT_ANGLE_MIN;
    if (cdeg > ENCODER_POT_ANGLE_MAX) return ENCODER_POT_ANGLE_MAX;
    return cdeg;
}
#endif

/* 读电池电压并换算为厘伏；乘积最大 4095*3300*49 ≈ 6.6e8，不会溢出uint32 */
uint16_t A_Voltage_Read(void)
{
    uint16_t raw_adc; /* 注入组单次转换的ADC计数 */
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
