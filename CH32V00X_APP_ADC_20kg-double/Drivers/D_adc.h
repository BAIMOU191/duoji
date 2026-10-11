#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* 本板角度传感器：电位器接PC4(IN2)，与电流共用规则组 */

/* 电流：驱动段中点硬件触发 + 20点均值 + alpha=1/2 一阶滤波，上电标定零点；
 * 电位器：同次触发20点均值；电压：注入组跟在每次规则组之后自动转换(每0.5ms取一次结果)，读取只取缓存。 */
void     D_ADC_DMA_Init(void);            /* 初始化规则组DMA与电压注入通道 */
void     D_ADC_DMA_ISR(void);             /* DMA1_Channel1_IRQHandler中调用 */
void     D_ADC_Current_Window_Set(uint8_t valid); /* 声明采样点在不在驱动段内 */
uint8_t  D_ADC_Current_Read(uint16_t *value); /* 读最新块的扣零点ADC值；返回值=是否采在驱动段 */
uint16_t D_ADC_Voltage_Read(void);        /* 最近一次电池电压(ADC计数)，只读缓存，不启动转换 */

/* 电位器均值保留4位小数(亚LSB分辨率)，4095*16=65520 装得进 uint16 */
#define D_ADC_ENCODER_FRAC_BITS 4U
#define D_ADC_ENCODER_SCALE     (1U << D_ADC_ENCODER_FRAC_BITS)

uint8_t  D_ADC_Encoder_Read(uint16_t *value_q4); /* 电位器均值，Q4(计数×16) */

#endif /* __D_ADC_H__ */
