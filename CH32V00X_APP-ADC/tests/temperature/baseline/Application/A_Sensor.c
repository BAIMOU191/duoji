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

#if  ENCODER_MODE ==   1        /*磁编码模式*/
/*
 * @fn      A_Encoder_Read
 * @brief   单次读取编码器角度，不修改任何全局状态
 * @param   无
 * @return  角度(厘度，0~35999)；ENCODER_ANGLE_ERROR=I2C读取失败
 */
int32_t A_Encoder_Read(void)
{
    /* MT6701_ANGLE_ERROR 与 ENCODER_ANGLE_ERROR 同为65535，透传即可 */
    return (int32_t)D_MT6701_Read_Angle();
}

#else                           /*电位器模式*/
/*
 * @fn      A_Encoder_Read
 * @brief   把电位器的ADC计数按实测端点线性换算成角度
 * @param   无
 * @return  角度(厘度，ENCODER_POT_ANGLE_MIN~ENCODER_POT_ANGLE_MAX，可为负)；
 *          ENCODER_ANGLE_ERROR=采样无效，或抽头已经转出碳膜(在死区里)
 *
 * 斜率由两个实测标定点定死，量程按同一斜率向两端各外扩10度，所以0度以下
 * 读出的是负数而不是被压成0：这一段电位器仍在线性区，读数是真的。再往外
 * 才钳到端点，那之外是碳膜端头的电气死区，钳住保证角度单调。
 *
 * 驱动层给的是 Q4 定点计数(见 D_ADC_ENCODER_SCALE)，所以标定常数在这里
 * 统一乘 SCALE 对齐，分母同样放大，Q4 就地约掉，结果仍是整数厘度。1个原始
 * LSB是7.94厘度，Q4之后最细能分辨到0.5厘度——实际精度由平均后的噪声底决定，
 * 不是由这个位宽决定。
 *
 * 分子最大 (4095-350)*16*27000 ≈ 1.62e9，未越int32；上面的编译期护栏盯着
 * 这一条，换更大量程的电位器时会直接报错。
 */
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
