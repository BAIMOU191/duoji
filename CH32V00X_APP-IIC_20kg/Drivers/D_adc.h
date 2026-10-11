#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* 本板角度传感器：1=电位器接PC4(IN2)与电流共用规则组，0=I2C磁编码器(MT6701)。
 * 全工程编码器分支由它派生；A_Config.h 的 CFG_WRAP_RANGE_CDEG 须与之一致(有编译期断言)。 */
#define D_ADC_ENCODER_ENABLE 0

/* 电流：驱动段中点硬件触发 + 20点均值 + alpha=1/2 一阶滤波，上电标定零点；
 * 电压：注入组在规则组之后自动采样，读取只取缓存。 */
void     D_ADC_DMA_Init(void);            /* 初始化规则组DMA与电压注入通道 */
void     D_ADC_DMA_ISR(void);             /* DMA1_Channel1_IRQHandler中调用 */
void     D_ADC_Current_Window_Set(uint8_t valid); /* 声明采样点在不在驱动段内 */
uint8_t  D_ADC_Current_Read(uint16_t *value); /* 读最新块的扣零点ADC值；返回值=是否采在驱动段 */
uint16_t D_ADC_Voltage_Read(void);        /* 最近一次电池电压(ADC计数)，只读缓存，不启动转换 */


#endif /* __D_ADC_H__ */
