#ifndef __D_ADC_H__
#define __D_ADC_H__

#include "ch32v00X.h"

/* D_adc.h ADC1：TIM2_CH4每PWM周期触发规则组[电流(运放IN9)]，自动注入组电池电压(PD6)；
 * DMA攒20周期(=1ms)发布一块均值(去最大最小各一采样)，电流Q4，每块即一个1ms控制拍。
 * 电流采样点由 D_motor 经 D_ADC_Current_Phase 按段声明：驱动段孔径居中读电流，续流段读零点跟踪温漂，段变后相邻块作废。 */

/* 电流采样时序，单位为定时器拍(48MHz)；ADC时钟24MHz，1个ADC周期 = 2拍 */
#define D_ADC_CUR_SETTLE_TICKS   106U  /* 驱动段开通到检流电压稳定(开通尖峰+运放建立)：按2.2us取，偏保守 */
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
uint16_t D_ADC_Voltage_Read(void);               /* 电池电压(ADC计数)，约4ms一阶低通；上电后尚无结果时为0 */
uint16_t D_ADC_Voltage_Raw(void);                /* 最近一次电池电压单点(ADC计数)，含开关尖峰：只给要抢时间的回馈钳位 */

#endif /* __D_ADC_H__ */
