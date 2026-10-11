/* A_Sensor.c —— 传感器聚合层：驱动原始计数换算为业务单位 */

#include "A_Sensor.h"
#include "D_tmp112.h"
#include "D_adc.h"

/* 本芯片没有除法指令，每拍都要做的换算一律用"乘倒数再移位"代替除以常数(库函数除法每次一百多条指令)。
 * 电位器：除数 ENC_DEN 落在 [2^15, 2^16)，取 S=47、M=ceil(2^S/ENC_DEN)；被除数 < 2^31 时
 * (u*M)>>S 与 u/ENC_DEN 逐位相同，条件是 (M*ENC_DEN-2^S)*2^31 < 2^S，下面的断言保证。 */
#define ENC_DEN   ((uint32_t)(ENCODER_POT_ADC_SPAN * D_ADC_ENCODER_SCALE)) /** 换算分母，Q4计数 */
#define ENC_DIV_S 47U
#define ENC_DIV_M ((uint32_t)(((1ULL << ENC_DIV_S) + ENC_DEN - 1U) / ENC_DEN))
_Static_assert (ENC_DEN >= (1UL << 15) && ENC_DEN < (1UL << 16),
                "电位器标定跨度变了：ENC_DIV_S 需按新分母重选");
_Static_assert (((uint64_t)ENC_DIV_M * ENC_DEN - (1ULL << ENC_DIV_S)) * (1ULL << 31) < (1ULL << ENC_DIV_S),
                "电位器换算的倒数乘法不再与除法逐位相同");

/* 电压：厘伏 = raw*VDD*NUM/(FULL*DEN*10)，本板常数恰好约成 raw*1617/4096，与原来的两步除法逐位相同 */
#define VOLT_CV_K ((uint32_t)ADC_VDD_MV * VOLTAGE_DIV_NUM / (VOLTAGE_DIV_DEN * 10U))
_Static_assert ((uint32_t)VOLT_CV_K * VOLTAGE_DIV_DEN * 10U == (uint32_t)ADC_VDD_MV * VOLTAGE_DIV_NUM
                && ADC_FULL_SCALE == 4096U, "电压换算常数改了：需重新约分，或改回除法");

/* 读电位器角度，按实测端点线性换算，厘度[ANGLE_MIN,ANGLE_MAX]，可为负；抽头悬空按读取失败处理 */
int32_t A_Encoder_Read(void)
{
    uint16_t raw_q4 = 0; /* 电位器通道的均值，Q4计数 */
    int32_t  num;        /* 线性换算的分子           */
    uint32_t mag;        /* |分子| + 半个分母，四舍五入用 */
    int32_t  cdeg;       /* 换算出的角度，厘度       */

    if (!D_ADC_Encoder_Read(&raw_q4)) return ENCODER_ANGLE_ERROR;

    /* 抽头悬空被下拉钳到地：没有角度，按读取失败处理 */
    if (raw_q4 < (uint16_t)(ENCODER_POT_ADC_FLOAT * D_ADC_ENCODER_SCALE))
        return ENCODER_ANGLE_ERROR;

    /* 标定常数乘SCALE与Q4对齐，分子最大约1.62e9，未越int32 */
    num = ((int32_t)raw_q4
         - (int32_t)(ENCODER_POT_ADC_MIN * D_ADC_ENCODER_SCALE))
        * (int32_t)ENCODER_POT_SPAN_CDEG;

    /* 四舍五入且两半轴对称：|num| 补半个分母后整除，再还原符号 */
    mag  = (uint32_t)((num >= 0) ? num : -num) + ENC_DEN / 2U;
    cdeg = (int32_t)(((uint64_t)mag * ENC_DIV_M) >> ENC_DIV_S);
    if (num < 0) cdeg = -cdeg;

    if (cdeg < ENCODER_POT_ANGLE_MIN) return ENCODER_POT_ANGLE_MIN;
    if (cdeg > ENCODER_POT_ANGLE_MAX) return ENCODER_POT_ANGLE_MAX;
    return cdeg;
}

/* 读电池电压并换算为厘伏；乘积最大 4095*1617 ≈ 6.6e6 */
uint16_t A_Voltage_Read(void)
{
    return (uint16_t)(((uint32_t)D_ADC_Voltage_Read() * VOLT_CV_K) >> 12);
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
