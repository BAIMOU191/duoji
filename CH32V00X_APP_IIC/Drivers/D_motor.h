#ifndef __D_MOTOR_H__
#define __D_MOTOR_H__

#include <stdint.h>
#include "D_tim.h"

/* AT8236 H桥，固定慢衰减。占空比 pwm ∈ [-MOTOR_PWM_MAX, MOTOR_PWM_MAX]，正=正转，0=两路全高短路制动。
 * 设占空比的同时把本周期的电流采样点放好(见 D_motor.c)。 */
#define MOTOR_PWM_MAX  TIM_PWM_RESOLUTION

int16_t D_Motor_Set(int16_t pwm);                 /* 返回限幅后的占空比 */
void    D_Motor_Release(uint8_t high_resistance); /* 卸力：0=两路低(高阻)，1=两路高(短路阻尼) */

#endif /* __D_MOTOR_H__ */
