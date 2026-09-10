#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* 电流：PWM驱动段中点硬件触发 + 20点均值 + 一阶滤波，上电自动标定零点；
 * 电压：注入组软件单次触发。 */
void     D_ADC_DMA_Init(void);            /* 初始化电流DMA与电压注入通道 */
void     D_ADC_DMA_ISR(void);             /* DMA1_Channel1_IRQHandler中调用 */
void     D_ADC_Current_Filter_Reset(void);/* 采样窗口变化/换向时使旧样本失效 */
uint8_t  D_ADC_Current_Read(uint16_t *value); /* 读取扣零点后的ADC值与有效标志 */
uint16_t D_ADC_Current_Offset_Get(void);  /* 上电标定的零电流ADC偏置 */
uint16_t D_ADC_Voltage_Read(void);        /* 注入组软件触发，返回ADC原始值 */

#endif /* __D_ADC_H__ */
