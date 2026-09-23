#ifndef __C_SPEED_OBSERVER_H__
#define __C_SPEED_OBSERVER_H__

#include <stdint.h>
#include "A_Parameter.h"

/*
 * C_Speed_Observer.h —— 3状态模型观测器(位置/速度/负载扰动)
 *
 * 编码器单源求速度降噪必加滞后，这里引入第二个信息源：已下发PWM + 标定的一阶电机模型
 *     dv/dt = (Kv*(u - d) - v) / tau,  dp/dt = v
 * 每拍模型预测，再用编码器残差按固定增益 L1/L2/L3 修正。增益由 A_Parameter.h 的
 * OBSG_* 宏编译期做三重极点配置(极点 exp(-2*pi*TUNE_OBS_BW_HZ*T))算出。
 *
 * 实现要点：输入按纯延迟对齐并乘 CAL_MOTOR_SIGN；输入先扣库仑摩擦(零速附近连续过渡)；
 * d 只在观测器内部用，不前馈到PWM(负载补偿归速度环积分)。状态Q8，每拍无除法。
 */

typedef struct {
    int32_t pos;  /* 滤波后位置，厘度 */
    int32_t vel;  /* 速度估计，厘度/秒，无滞后 */
    int32_t load; /* 负载扰动估计，等效PWM计数，仅供调试，不要用作前馈 */
} SpeedObsOut_t;

/* 初始化，init_pos 为当前编码器角度(厘度) */
void C_SpeedObs_Init(int32_t init_pos);

/* 1ms调用一次；pwm_applied 为上一拍实际施加的PWM，纯延迟对齐在内部完成 */
void C_SpeedObs_Update(int32_t measured_pos, int16_t pwm_applied, SpeedObsOut_t *out);

#endif /* __C_SPEED_OBSERVER_H__ */
