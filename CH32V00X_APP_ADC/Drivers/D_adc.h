#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* 本板角度传感器：1=电位器接PC4(IN2)与电流共用规则组，0=I2C磁编码器(MT6701)。
 * 全工程编码器分支由它派生；A_Parameter.h 的 CFG_WRAP_RANGE_CDEG 须与之一致(有编译期断言)。 */
#define D_ADC_ENCODER_ENABLE 1

/* 电流：驱动段中点硬件触发 + 20点均值 + alpha=1/2 一阶滤波，上电标定零点；
 * 电位器：同次触发20点均值；电压：注入组软件单次触发。 */
void     D_ADC_DMA_Init(void);            /* 初始化规则组DMA与电压注入通道 */
void     D_ADC_DMA_ISR(void);             /* DMA1_Channel1_IRQHandler中调用 */
void     D_ADC_Current_Window_Set(uint8_t valid); /* 声明采样点在不在驱动段内 */
uint8_t  D_ADC_Current_Read(uint16_t *value); /* 读最新块的扣零点ADC值；返回值=是否采在驱动段 */
uint16_t D_ADC_Current_Offset_Get(void);  /* 上电标定的零电流ADC偏置 */
uint16_t D_ADC_Voltage_Read(void);        /* 注入组软件触发，返回ADC原始值 */

#if D_ADC_ENCODER_ENABLE
/* 电位器均值保留4位小数(亚LSB分辨率)，4095*16=65520 装得进 uint16 */
#define D_ADC_ENCODER_FRAC_BITS 4U
#define D_ADC_ENCODER_SCALE     (1U << D_ADC_ENCODER_FRAC_BITS)

uint8_t  D_ADC_Encoder_Read(uint16_t *value_q4); /* 电位器均值，Q4(计数×16) */
#endif

#endif /* __D_ADC_H__ */
