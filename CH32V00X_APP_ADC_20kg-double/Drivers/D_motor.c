/* D_motor.c AT8236固定慢衰减电机驱动 */

#include "D_motor.h"
#include "D_adc.h"

static int8_t  s_motor_direction;
static int16_t s_motor_pwm = MOTOR_PWM_MAX + 1;

/* 设置慢衰减H桥输出，返回限幅后的PWM：正转IN1恒高、IN2反相PWM，反转相反；pwm=0 两路全高短路制动 */
int16_t D_Motor_Set(int16_t pwm)
{
    int8_t direction;
    int8_t old_direction;
    uint16_t magnitude;
    uint16_t inverted;

    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    if (pwm == s_motor_pwm) return pwm;

    direction = (pwm > 0) ? 1 : ((pwm < 0) ? -1 : 0);
    magnitude = (uint16_t)((pwm < 0) ? -pwm : pwm);
    inverted = (uint16_t)(MOTOR_PWM_MAX - magnitude);
    old_direction = s_motor_direction;

    /* 按本次幅值重定位电流采样点(与PWM比较值同一周期边界生效，不错拍)；换向不丢样本 */
    D_ADC_Current_Window_Set(D_TIM2_ADC_Trigger_Set(magnitude));

    /* 换向时先清掉两路比较值；预装载会在PWM周期边界同步更新。 */
    if (direction != old_direction)
    {
        TIM_SetCompare2(TIM2, 0);
        TIM_SetCompare1(TIM1, 0);
    }

    if (pwm >= 0)
    {
        TIM_SetCompare2(TIM2, MOTOR_PWM_MAX);
        TIM_SetCompare1(TIM1, inverted);
    }
    else
    {
        TIM_SetCompare1(TIM1, MOTOR_PWM_MAX);
        TIM_SetCompare2(TIM2, inverted);
    }

    s_motor_direction = direction;
    s_motor_pwm = pwm;
    return pwm;
}

void D_Motor_Release(uint8_t high_resistance)
{
    D_ADC_Current_Window_Set(D_TIM2_ADC_Trigger_Set(0U));
    if (high_resistance)
    {
        TIM_SetCompare1(TIM1, MOTOR_PWM_MAX);
        TIM_SetCompare2(TIM2, MOTOR_PWM_MAX);
    }
    else
    {
        TIM_SetCompare1(TIM1, 0U);
        TIM_SetCompare2(TIM2, 0U);
    }
    s_motor_direction = 0;
    s_motor_pwm = MOTOR_PWM_MAX + 1; /* 强制下次Set重新配置硬件 */
}
