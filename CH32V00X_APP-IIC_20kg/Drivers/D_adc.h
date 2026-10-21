#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* ADC1：TIM2_CH4 每个PWM周期触发一次规则组 [电流(运放内部输出IN9)]，
 * 规则组转完硬件自动接一次注入组电池电压(PD6)。DMA 攒满20个周期(=1ms)发布一块均值，电流保留4位小数(Q4)。
 * 每发布一块就是一个1ms控制拍(见 ch32v00X_it.c)：控制拿到的总是刚出炉的一块，在拍头改的占空比从下一块开头生效。
 * 电流块均值去掉最大、最小各一个采样再平均，挡掉单个尖峰。
 *
 * 电流：低边检流电阻只在PWM驱动段有电流，采样点由 D_motor 放在驱动段正中(孔径居中)。
 *   驱动段太窄(含0)时采样点放在续流段，检流电阻上没有电流，读数就是零点，拿来跟踪温漂。
 *   电机驱动每次改占空比都用 D_ADC_Current_Phase 声明采样点在哪一段；段变了，紧接着的一块作废。 */

/* 电流采样时序，单位为定时器拍(48MHz)；ADC时钟24MHz，1个ADC周期 = 2拍 */
#define D_ADC_CUR_SETTLE_TICKS   106U  /* 驱动段开通到检流电压稳定(开通尖峰+运放建立)：按2.2us取，偏保守，实测后可减 */
#define D_ADC_CUR_APERTURE_TICKS  57U  /* 采样孔径：28.5个ADC周期(CyclesMode3)，改采样档要一起改 */
#define D_ADC_CUR_LEAD_TICKS     (4U + D_ADC_CUR_APERTURE_TICKS / 2U) /* 触发到孔径中点：触发延迟2个ADC周期 + 半孔径 */
/* 孔径居中、前沿离开通沿至少一个稳定时间所需的最短驱动段(约11%占空比)；更窄时测不准 */
#define D_ADC_CUR_MIN_TICKS      (2U * (D_ADC_CUR_SETTLE_TICKS + D_ADC_CUR_APERTURE_TICKS / 2U))
/* 一次触发的全部转换：触发延迟 + 电流(28.5+12.5) + 电压(239.5+12.5)，ADC周期×2 */
#define D_ADC_SCAN_TICKS         (4U + 82U + 504U)

/* 电流采样点所在段，由电机驱动随占空比声明 */
#define D_CUR_NONE 0U   /* 卸力：读数无意义 */
#define D_CUR_ZERO 1U   /* 续流段：检流电阻无电流，读数即零点 */
#define D_CUR_FWD  2U   /* 正向驱动段：读数 = 绕组电流 */
#define D_CUR_REV  3U   /* 反向驱动段：读数 = −绕组电流 */

#define D_ADC_CURRENT_SCALE     16U                              /* 电流 Q4 */

void     D_ADC_DMA_Init(void);                  /* 配置ADC与DMA，并在电机首次驱动前标定电流零点 */
uint8_t  D_ADC_DMA_ISR(void);                   /* DMA1_Channel1_IRQHandler 中调用；返回1=刚发布一块 */
void     D_ADC_Current_Phase(uint8_t phase);    /* 声明采样点所在段，D_CUR_xxx */
/* 最新块电流：扣零点、与施加电压同向为正(被倒拖回馈为负)，Q4计数。返回1=整块都采在同一方向的驱动段 */
uint8_t  D_ADC_Current_Read(int32_t *value_q4);
uint16_t D_ADC_Voltage_Read(void);               /* 最近一次电池电压(ADC计数)，上电后尚无结果时为0 */

#endif /* __D_ADC_H__ */
