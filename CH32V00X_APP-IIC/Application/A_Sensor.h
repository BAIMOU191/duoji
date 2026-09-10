#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>

/* A_Sensor.h 传感器聚合：编码器/电压/温度/电流读取与换算 */

#define ENCODER_ANGLE_ERROR  0xFFFFU /* 编码器读取失败返回值 */

#define ADC_VDD_MV          3300U  /* ADC参考电压(毫伏) */
#define ADC_FULL_SCALE      4096U  /* ADC满量程计数 */

/* 电压换算：V_bat = ADC × 3300mV × 49 / (4096 × 10) */
#define VOLTAGE_DIV_NUM     49U    /* 分压比分子 */
#define VOLTAGE_DIV_DEN     10U    /* 分压比分母 */

/* 电流检测参数：采样电阻60mΩ接在AT8236的ISEN与地之间，OPA1正输入直接
 * 搭在ISEN节点，单端PGA增益8x。
 *   ADC满量程 = 3300mV/8/60mΩ = 6.875A，1 LSB ≈ 1.68mA
 *   AT8236自身过流点 ITRIP = VREF/(10*R) = 3.3/0.6 = 5.5A，在量程之内
 * 只有PWM驱动段的电流流过该电阻：两路全高的刹车段电流在ISEN节点进出
 * 相消，读数为零，所以短路制动电流在硬件上不可见。 */
#define CURRENT_SENSE_MOHM  60U     /* 采样电阻(毫欧) */
#define CURRENT_SENSE_GAIN   8U     /* OPA1单端PGA增益 */

/* OPA工作在 8x + PGA基准VB=VDD/4：零电流时输出不是0而是约760mV
 * (实测945个ADC计数)，信号骑在这个台阶上。台阶由上电零点标定测出并
 * 在 D_ADC_Current_Read 里扣掉，所以对上层透明，但有两个后果：
 *   可上报的最大电流 = (4096-945) * 1.678mA = 5.29A，正好在AT8236
 *   自身过流点5.5A之下，斩波之前的范围全部可见；
 *   下面的满量程常量仍按整个4096算，只用于计数到毫安的比例换算。 */

/* 满量程电流(mA)，编译期常量 */
#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM))

uint16_t A_Encoder_Read(void);         /* 单次读取编码器角度，失败返回ENCODER_ANGLE_ERROR */
uint16_t A_Voltage_Read(void);        /* 读取电池电压，返回厘伏(V×100) */
int16_t  A_Temperature_Read(void);    /* 读取温度，返回0.1°C，失败返回-32768 */
uint8_t  A_Current_Read(uint16_t *current_ma); /* 读取电流并返回采样有效标志 */

#endif /* __A_SENSOR_H__ */
