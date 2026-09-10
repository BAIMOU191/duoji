/*
 * A_Sensor.c —— 传感器聚合层
 *
 * 职责只有一条：把驱动返回的**原始计数**换算成**业务单位**(厘度/厘伏/0.1度/毫安)，
 * 让上层永远看不到ADC计数、分压比、运放增益这些电路细节。
 * 换硬件(不同分压电阻、不同检流电阻)只需要改 A_Sensor.h 里的常量，
 * A_Servo 和协议层一行都不用动。
 *
 * 编码器和温度目前是1:1透传，保留在这里是为了让"应用层只认业务单位"这条
 * 边界完整；将来要加滤波、故障计数或单位变换，改动点就在这一层，不会扩散。
 */

#include "A_Sensor.h"
#include "D_mt6701.h"
#include "D_tmp112.h"
#include "D_adc.h"

/*
 * @fn      A_Encoder_Read
 * @brief   单次读取编码器角度，不修改任何全局状态
 * @param   无
 * @return  角度(厘度，0~35999)；ENCODER_ANGLE_ERROR=I2C读取失败
 */
uint16_t A_Encoder_Read(void)
{
    return D_MT6701_Read_Angle();
}

/*
 * @fn      A_Voltage_Read
 * @brief   读取电池电压并换算为厘伏
 * @param   无
 * @return  电压(厘伏，如720表示7.20V)
 *
 * V_bat = ADC / 满量程 * VDD * 分压比，全部用整数算：
 *     先乘后除，乘积最大 4095*3300*49 ≈ 6.6e8，不会溢出uint32。
 */
uint16_t A_Voltage_Read(void)
{
    uint16_t raw_adc; /* 注入组单次转换的ADC计数 */
    uint32_t mv;      /* 换算出的电池电压，毫伏  */

    raw_adc = D_ADC_Voltage_Read();
    mv = (uint32_t)raw_adc * ADC_VDD_MV * VOLTAGE_DIV_NUM
       / (ADC_FULL_SCALE * VOLTAGE_DIV_DEN);

    return (uint16_t)(mv / 10U); /* 毫伏 -> 厘伏 */
}

/*
 * @fn      A_Temperature_Read
 * @brief   读取TMP112温度
 * @param   无
 * @return  温度(0.1摄氏度，如250表示25.0度)；-32768=读取失败
 */
int16_t A_Temperature_Read(void)
{
    return D_TMP112_Read_Temp();
}

/*
 * @fn      A_Current_Read
 * @brief   读取PWM同步滤波后的绕组电流并换算为毫安
 * @param   current_ma 电流结果输出；采样无效时保持调用方原值不变
 * @return  1=结果有效，0=参数无效/当前没有采样窗口/滤波块未就绪
 *
 * 目前正常固件还没有消费者(限流降额功能预留)，只有标定和诊断用得到。
 * 采样链路本身是常开的：TIM2在每个PWM驱动段中点触发一次ADC，DMA攒够
 * 20点(=1个控制周期)由中断求均值，这里只是把结果取走。
 */
uint8_t A_Current_Read(uint16_t *current_ma)
{
    uint16_t raw_adc = 0; /* 已扣除零点偏置的ADC计数 */
    uint32_t ma;          /* 换算出的电流，毫安      */

    if (current_ma == 0 || !D_ADC_Current_Read(&raw_adc)) return 0;

    /* 先在编译期算出满量程电流，再按ADC计数线性插值：
     *     满量程(mA) = VDD(mV)*1000 / (OPA增益 * R(毫欧)) = 6875mA
     * 旧式的 125/(增益/8) 在增益取4倍时会整除为0导致除零，而且
     * raw*VDD*1000 会溢出uint32；本式两个问题都不存在。 */
    ma = (uint32_t)raw_adc * CURRENT_FULL_SCALE_MA / ADC_FULL_SCALE;
    if (ma > 0xFFFFU) ma = 0xFFFFU;

    *current_ma = (uint16_t)ma;
    return 1;
}
