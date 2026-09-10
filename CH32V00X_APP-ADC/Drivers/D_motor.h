#ifndef __D_MOTOR_H__
#define __D_MOTOR_H__

#include <stdint.h>
#include "D_tim.h"

/* AT8236固定使用慢衰减：一路常高，另一路输出反相PWM。
 * D_Motor_Set(0) 对应两路全高，电机绕组短路制动。 */
#define MOTOR_PWM_MAX  TIM_PWM_RESOLUTION

int16_t D_Motor_Set(int16_t pwm);
void D_Motor_Release(uint8_t high_resistance); /* 0=两路低，1=两路高 */

#endif /* __D_MOTOR_H__ */
