#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* ==================== 本板的角度传感器接在哪 ====================
 * 这是**唯一**需要改的模式开关，整个工程的编码器分支都由它派生：
 *   1 = 电位器抽头接在 PC4(IN2)，与电流共用规则组扫描；
 *   0 = 角度来自 I2C 磁编码器(MT6701)，规则组只转电流。
 * A_Sensor.h 的 ENCODER_MODE 直接由它推出，A_Parameter.h 的
 * CFG_WRAP_RANGE_CDEG 必须与之一致，A_Servo.c 里有编译期断言。 */
#define D_ADC_ENCODER_ENABLE 0

/* 电流：PWM驱动段中点硬件触发 + 20点均值 + 一阶滤波，上电自动标定零点；
 * 电位器：与电流同一次触发扫描出来，20点均值，只给原始计数不做换算；
 * 电压：注入组软件单次触发。 */
void     D_ADC_DMA_Init(void);            /* 初始化规则组DMA与电压注入通道 */
void     D_ADC_DMA_ISR(void);             /* DMA1_Channel1_IRQHandler中调用 */
void     D_ADC_Current_Filter_Reset(void);/* 采样窗口变化/换向时使旧样本失效 */
void     D_ADC_Current_Window_Set(uint8_t valid); /* 声明电流采样窗口是否合法 */
uint8_t  D_ADC_Current_Read(uint16_t *value); /* 读取扣零点后的ADC值与有效标志 */
uint16_t D_ADC_Current_Offset_Get(void);  /* 上电标定的零电流ADC偏置 */
uint16_t D_ADC_Voltage_Read(void);        /* 注入组软件触发，返回ADC原始值 */

#if D_ADC_ENCODER_ENABLE
/* 电位器通道的定点小数位。整块20点的均值保留4位小数再发布：抽头电压上有
 * 大于1个LSB的抖动(实测静止时读数在相邻3个LSB之间跳)，20点平均本身就能把
 * 分辨率恢复到亚LSB，四舍五入回整数计数等于把这部分信息白扔掉。
 * 4095*16 = 65520，正好还装得进 uint16。 */
#define D_ADC_ENCODER_FRAC_BITS 4U
#define D_ADC_ENCODER_SCALE     (1U << D_ADC_ENCODER_FRAC_BITS)

uint8_t  D_ADC_Encoder_Read(uint16_t *value_q4); /* 电位器均值，Q4(计数×16) */
#endif

#endif /* __D_ADC_H__ */
