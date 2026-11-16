#ifndef __D_TIM_H__
#define __D_TIM_H__

#include "ch32v00X.h"

/* 定时器：TIM1/TIM2 均 48MHz/2400=20kHz 向上计数，TIM2 由 TIM1 更新复位，计数同相；比较值预装载，周期边界同时生效。
 * TIM1_CH1 -> PD2、TIM2_CH2 -> PD3 电机PWM；TIM1_CH2 <- PA1 舵机捕获；TIM2_CH4 不出引脚，经TRGO每周期触发一次ADC */
#define TIM_PWM_RESOLUTION  2400                      /* PWM周期计数 */
#define TIM_TRIG_PARK       (TIM_PWM_RESOLUTION / 2)  /* ADC触发停放点：周期中间，电流不可测时采零点 */

void     D_TIM2_PWM_Init(void);          /* TIM2：PD3 PWM + CH4 ADC触发 */
void     D_TIM1_PWM_IC_Init(void);       /* TIM1：PD2 PWM + PA1捕获 + 更新中断 */
uint16_t D_PWM_Read(void);               /* 最新捕获脉宽(us)，读后清零，无新帧返回0 */
void     D_PWM_Input_Enable(uint8_t en); /* 使能捕获；失能时PA1开漏置高释放共线信号 */
void     D_TIM1_CC_ISR(void);            /* TIM1捕获中断，在ISR中调用 */
void     D_TIM1_UP_ISR(void);            /* TIM1更新中断，在ISR中调用 */

#endif /* __D_TIM_H__ */
