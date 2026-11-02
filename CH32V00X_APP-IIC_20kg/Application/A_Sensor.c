/* A_Sensor.c —— 传感器聚合层：驱动原始计数换算为业务单位 */

#include "A_Sensor.h"
#include "D_tmp112.h"
#include "D_adc.h"

#include "D_mt6701.h"

/* 电压：厘伏 = raw*VDD*NUM/(FULL*DEN*10)，本板约成 raw*1617/4096 */
#define VOLT_CV_K ((uint32_t)ADC_VDD_MV * VOLTAGE_DIV_NUM / (VOLTAGE_DIV_DEN * 10U))
_Static_assert ((uint32_t)VOLT_CV_K * VOLTAGE_DIV_DEN * 10U == (uint32_t)ADC_VDD_MV * VOLTAGE_DIV_NUM
                && ADC_FULL_SCALE == 4096U, "电压换算常数改了：需重新约分，或改回除法");

/* MT6701整圈反馈，读取失败透传ENCODER_ANGLE_ERROR。 */
int32_t A_Encoder_Read(void)
{
    return (int32_t)D_MT6701_Read_Angle();
}

/* 读电池电压并换算为厘伏 */
uint16_t A_Voltage_Read(void)
{
    return (uint16_t)(((uint32_t)D_ADC_Voltage_Read() * VOLT_CV_K) >> 12);
}

uint16_t A_Voltage_ReadRaw(void)
{
    return (uint16_t)(((uint32_t)D_ADC_Voltage_Raw() * VOLT_CV_K) >> 12);
}

/* 读TMP112温度，返回0.1℃；失败返回-32768 */
int16_t A_Temperature_Read(void)
{
    return D_TMP112_Read_Temp();
}

/* 与PWM同步采到的绕组电流(mA)：与施加电压同向为正(出力)，反向为负(被倒拖回馈)。
 * 返回0时(驱动段太窄或卸力)读数是"测不到"而非"没有电流" */
uint8_t A_Current_ReadSigned(int32_t *current_ma)
{
    int32_t q4 = 0;   /* 扣零点、带符号的块均值，Q4计数 */
    uint8_t ok = D_ADC_Current_Read(&q4);

    /* |q4| ≤ 65520，乘满量程不溢出 */
    *current_ma = q4 * (int32_t)CURRENT_FULL_SCALE_MA / (int32_t)(ADC_FULL_SCALE * D_ADC_CURRENT_SCALE);
    return ok;
}

uint8_t A_Current_Read(uint16_t *current_ma)
{
    int32_t ma;
    uint8_t ok = A_Current_ReadSigned(&ma);

    if (ma < 0) ma = -ma;   /* 回馈电流同样计入 */
    *current_ma = (uint16_t)((ma > 0xFFFF) ? 0xFFFF : ma);
    return ok;
}
