/*
 * A_Sensor.c —— 传感器聚合层
 *
 * 职责只有一条：把驱动返回的原始计数换算成业务单位(厘度/厘伏/0.1摄氏度/毫安)，
 * 让上层永远看不到ADC计数、分压比、运放增益这些电路细节。换硬件只需要改
 * A_Sensor.h 里的常量，A_Servo 和协议层一行都不用动。
 */

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

    /* 抽头悬空被下拉钳到地：这不是一个"很小的角度"，而是根本没有角度。
     * 必须和读取失败同样对待，否则上层会拿着一个假位置去闭环。 */
    if (raw_q4 < (uint16_t)(ENCODER_POT_ADC_FLOAT * D_ADC_ENCODER_SCALE))
        return ENCODER_ANGLE_ERROR;

    /* 驱动层给的是Q4定点计数，标定常数在这里统一乘SCALE对齐，分母同样放大，
     * Q4就地约掉，结果仍是整数厘度。分子最大 (4095-350)*16*27000 ≈ 1.62e9，
     * 未越int32，A_Sensor.h 的护栏盯着这一条。 */
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

/* 读PWM同步滤波后的绕组电流并换算为毫安；返回0表示当前没有有效采样窗口 */
uint8_t A_Current_Read(uint16_t *current_ma)
{
    uint16_t raw_adc = 0; /* 已扣除零点偏置的ADC计数 */
    uint32_t ma;          /* 换算出的电流，毫安      */

    if (current_ma == 0 || !D_ADC_Current_Read(&raw_adc)) return 0;

    /* 满量程电流在编译期算好(VDD*1000/(增益*R)=6875mA)，再按计数线性插值。
     * 旧式的 125/(增益/8) 在增益取4倍时会整除为0导致除零，且 raw*VDD*1000
     * 会溢出uint32；本式两个问题都不存在。 */
    ma = (uint32_t)raw_adc * CURRENT_FULL_SCALE_MA / ADC_FULL_SCALE;
    if (ma > 0xFFFFU) ma = 0xFFFFU;

    *current_ma = (uint16_t)ma;
    return 1;
}
