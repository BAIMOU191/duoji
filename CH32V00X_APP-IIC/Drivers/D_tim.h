#ifndef __D_TIM_H__
#define __D_TIM_H__

#include "ch32v00X.h"

/* 定时器驱动：TIM1/TIM2 PWM输出、输入捕获与ADC同步触发 */

#define TIM_PWM_RESOLUTION  2400  /* PWM周期计数值，48MHz/2400=20kHz */

/* ============ 电流采样时序：D_tim.c 与 D_adc.c 共用的唯一配置处 ============
 *
 * 慢衰减下采样窗口必须整个塞进周期尾部的驱动段 [ARR-mag, ARR)，且满足：
 *   孔径前沿距开关沿 >= AT8236 的 tBLANK = 2.2us
 *       (手册用它屏蔽自身OCP，等价于告诉我们ISEN振铃能持续这么久)
 *   SAR转换结束 <= 周期末尾再留一点余量，避免转换过程被下一个开关沿干扰
 * 于是能采样的最低幅值：
 *       mag_min = tBLANK + 采样时间 + 转换时间 + 尾余量
 * tBLANK是芯片给的硬地板(106拍 = 4.4%占空比)，只能压缩后面三项。
 *
 * PCLK2=48MHz且TIM也按48MHz计数，所以 1个ADCCLK 正好 = ADC_CLK_DIV 个TIM拍。
 *
 *   ADCCLK  采样档   采样拍  转换拍  mag_min  最低占空比
 *   12MHz   Mode3     114     50      294      12.3%
 *   24MHz   Mode3      57     25      212       8.8%   <= 当前
 *   24MHz   更短档       9     25      164       6.8%
 *
 * 想再往下压就得换更短的采样档：先在参考手册里查清该型号各档的实际周期数
 * 与ADCCLK上限，改 ADC_SAMPLE_HALFCYCLES，再到 D_adc.c 的 #if 映射里补上
 * 对应的 ADC_SampleTime_CyclesModeX（没补会直接编译报错，不会静默跑偏）。
 * 采样档缩短的前提是源阻抗足够低——这里源是OPA输出，本来就是低阻。 */
#define ADC_CLK_DIV               2U   /* ADCCLK = PCLK2 / ADC_CLK_DIV = 24MHz */
#define ADC_SAMPLE_HALFCYCLES    57U   /* 采样保持的ADCCLK周期数×2 (Mode3=28.5) */
#define ADC_CONV_HALFCYCLES      25U   /* 采样结束到转换结束×2 (12位SAR约12.5周期) */
#define ADC_TRIG_LATENCY_CYCLES   2U   /* 外部触发到开始采样的ADCCLK周期数 */
#define ADC_BLANK_TICKS         106U   /* AT8236 tBLANK=2.2us，按48MHz计数 */
#define ADC_TAIL_MARGIN_TICKS    24U   /* 转换结束到周期末尾的余量(0.5us) */

#define ADC_SAMPLE_TICKS   (ADC_SAMPLE_HALFCYCLES * ADC_CLK_DIV / 2U)
#define ADC_CONV_TICKS     (ADC_CONV_HALFCYCLES   * ADC_CLK_DIV / 2U)
#define ADC_LATENCY_TICKS  (ADC_TRIG_LATENCY_CYCLES * ADC_CLK_DIV)

/* 低于该幅值无论怎么摆都放不下采样窗口，关闭触发并由上层判为无效 */
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
