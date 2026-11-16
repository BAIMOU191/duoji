/* A_Sensor.c —— 传感器聚合层：驱动原始计数换算为业务单位 */

#include "A_Sensor.h"
#include "D_tmp112.h"
#include "D_adc.h"

/* 电位器标定换算的分子须放得进int32，写成除法形式避免判据本身溢出 */
_Static_assert((4095L - (long)ENCODER_POT_ADC_MIN) * (long)D_ADC_ENCODER_SCALE < 2147483647L / (long)ENCODER_POT_SPAN_CDEG,
               "电位器标定换算分子超出 int32");

/* 本芯片无除法指令：常数除法用"乘倒数再移位"。
 * ENC_DEN 在 [2^15,2^16)，取 S=47、M=ceil(2^S/ENC_DEN)，断言保证与整除逐位相同 */
#define ENC_DEN   ((uint32_t)(ENCODER_POT_ADC_SPAN * D_ADC_ENCODER_SCALE)) /** 换算分母，Q4计数 */
#define ENC_DIV_S 47U
#define ENC_DIV_M ((uint32_t)(((1ULL << ENC_DIV_S) + ENC_DEN - 1U) / ENC_DEN))
_Static_assert (ENC_DEN >= (1UL << 15) && ENC_DEN < (1UL << 16),
                "电位器标定跨度变了：ENC_DIV_S 需按新分母重选");
_Static_assert (((uint64_t)ENC_DIV_M * ENC_DEN - (1ULL << ENC_DIV_S)) * (1ULL << 31) < (1ULL << ENC_DIV_S),
                "电位器换算的倒数乘法不再与除法逐位相同");

/* 电压：厘伏 = raw*VDD*NUM/(FULL*DEN*10)，本板约成 raw*1617/4096 */
#define VOLT_CV_K ((uint32_t)ADC_VDD_MV * VOLTAGE_DIV_NUM / (VOLTAGE_DIV_DEN * 10U))
_Static_assert ((uint32_t)VOLT_CV_K * VOLTAGE_DIV_DEN * 10U == (uint32_t)ADC_VDD_MV * VOLTAGE_DIV_NUM
                && ADC_FULL_SCALE == 4096U, "电压换算常数改了：需重新约分，或改回除法");

/* 读电位器角度，厘度[ENCODER_ANGLE_LO,ENCODER_ANGLE_HI]，可为负；抽头悬空按读取失败处理 */
int32_t A_Encoder_Read(void)
{
    uint16_t raw_q4 = 0; /* 电位器通道均值，Q4计数 */
    int32_t  num;        /* 线性换算分子 */
    uint32_t mag;        /* |分子| + 半个分母，四舍五入用 */
    int32_t  cdeg;       /* 换算角度，厘度 */

    if (!D_ADC_Encoder_Read(&raw_q4)) return ENCODER_ANGLE_ERROR;

    /* 抽头悬空被下拉钳到地：没有角度，按读取失败处理 */
    if (raw_q4 < (uint16_t)(ENCODER_POT_ADC_FLOAT * D_ADC_ENCODER_SCALE))
        return ENCODER_ANGLE_ERROR;

    /* 标定常数乘SCALE与Q4对齐，分子未越int32 */
    num = ((int32_t)raw_q4
         - (int32_t)(ENCODER_POT_ADC_MIN * D_ADC_ENCODER_SCALE))
        * (int32_t)ENCODER_POT_SPAN_CDEG;

    /* 四舍五入且两半轴对称：补半个分母后整除，再还原符号 */
    mag  = (uint32_t)((num >= 0) ? num : -num) + ENC_DEN / 2U;
    cdeg = (int32_t)(((uint64_t)mag * ENC_DIV_M) >> ENC_DIV_S);
    if (num < 0) cdeg = -cdeg;

    if (cdeg < ENCODER_ANGLE_LO) return ENCODER_ANGLE_LO;
    if (cdeg > ENCODER_ANGLE_HI) return ENCODER_ANGLE_HI;
    return cdeg;
}

/* 诊断用原始计数：电位器块均值(Q4)，只给调试输出 */
int32_t A_Encoder_ReadRaw(void)
{
    uint16_t raw_q4;

    return D_ADC_Encoder_Read(&raw_q4) ? (int32_t)raw_q4 : -1;
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

/* 与PWM同步采到的绕组电流(mA)，硬件方向：正 = 正PWM驱动方向。与施加PWM同号 = 出力，异号 = 被倒拖回馈。
 * 返回0时(驱动段太窄或卸力)读数是"测不到"而非"没有电流" */
uint8_t A_Current_ReadSigned(int32_t *current_ma)
{
    int32_t q4 = 0;   /* 扣零点、带符号的块均值，Q4计数 */
    uint8_t ok = D_ADC_Current_Read(&q4);

    /* |q4| ≤ 65520，乘满量程不溢出 */
    *current_ma = q4 * (int32_t)CURRENT_FULL_SCALE_MA / (int32_t)(ADC_FULL_SCALE * D_ADC_CURRENT_SCALE);
    return ok;
}
