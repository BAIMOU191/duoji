#ifndef __D_TIM_H__
#define __D_TIM_H__

#include "ch32v00X.h"

/* 定时器驱动：TIM1/TIM2 PWM输出、输入捕获与ADC同步触发 */

#define TIM_PWM_RESOLUTION  2400  /* PWM周期计数值，48MHz/2400=20kHz */

/* ===== 电流采样时序：D_tim.c 与 D_adc.c 共用的唯一配置处 =====
 * 慢衰减下采样窗口必须塞进周期尾部驱动段，可采样的最低幅值
 *     mag_min = tBLANK(2.2us，芯片硬地板) + 采样时间 + 转换时间 + 尾余量
 * 1个ADCCLK = ADC_CLK_DIV 个TIM拍；24MHz/Mode3 下 mag_min=212拍(8.8%占空比)。
 * 换更短采样档须同时改 ADC_SAMPLE_HALFCYCLES 和 D_adc.c 的映射(未映射会编译报错)。 */
#define ADC_CLK_DIV               2U   /* ADCCLK = PCLK2 / ADC_CLK_DIV = 24MHz */
#define ADC_SAMPLE_HALFCYCLES    57U   /* 采样保持的ADCCLK周期数×2 (Mode3=28.5) */
#define ADC_CONV_HALFCYCLES      25U   /* 采样结束到转换结束×2 (12位SAR约12.5周期) */
#define ADC_TRIG_LATENCY_CYCLES   2U   /* 外部触发到开始采样的ADCCLK周期数 */
#define ADC_BLANK_TICKS         106U   /* AT8236 tBLANK=2.2us，按48MHz计数 */
#define ADC_TAIL_MARGIN_TICKS    24U   /* 转换结束到周期末尾的余量(0.5us) */

#define ADC_SAMPLE_TICKS   (ADC_SAMPLE_HALFCYCLES * ADC_CLK_DIV / 2U)
#define ADC_CONV_TICKS     (ADC_CONV_HALFCYCLES   * ADC_CLK_DIV / 2U)
#define ADC_LATENCY_TICKS  (ADC_TRIG_LATENCY_CYCLES * ADC_CLK_DIV)

/* 低于该幅值放不下采样窗口，触发点停到续流段并由上层判为无效 */
#define ADC_TRIGGER_MIN_PWM_TICKS  (ADC_BLANK_TICKS + ADC_SAMPLE_TICKS \
                                  + ADC_CONV_TICKS + ADC_TAIL_MARGIN_TICKS)

void     D_TIM2_PWM_Init(void);         /* TIM2：20kHz PWM + CH4电流采样标记 */
void     D_TIM1_PWM_IC_Init(void);      /* TIM1：20kHz PWM + 捕获 + 调度时基更新中断 */
uint8_t  D_TIM2_ADC_Trigger_Set(uint16_t pwm_ticks); /* 按幅值定位驱动段采样点 */
uint16_t D_PWM_Read(void);              /* 读取最新捕获脉宽(us)，消费后无新帧返回0 */
void     D_PWM_Input_Enable(uint8_t en);/* 使能捕获；失能时PA1开漏置高释放 */
void     D_TIM1_CC_ISR(void);           /* TIM1捕获中断处理，在ISR中调用 */
void     D_TIM1_UP_ISR(void);           /* TIM1溢出中断处理，在ISR中调用 */

#endif /* __D_TIM_H__ */
