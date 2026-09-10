/* D_motor.c AT8236固定慢衰减电机驱动 */

#include "D_motor.h"
#include "D_adc.h"

static int8_t  s_motor_direction;
static int16_t s_motor_pwm = MOTOR_PWM_MAX + 1;
static uint8_t s_adc_window;   /* 上一次设置后电流采样窗口是否有效 */

/*
 * @fn      D_Motor_Set
 * @brief   设置固定慢衰减H桥输出
 * @param   pwm 输出比较值，范围[-MOTOR_PWM_MAX, MOTOR_PWM_MAX]
 * @return  实际施加的限幅后PWM
 *
 * 正转：IN1恒高，IN2输出反相PWM；反转：IN2恒高，IN1输出反相PWM。
 * |pwm|=MOTOR_PWM_MAX时全驱动，pwm=0时两路全高并短路制动。
 */
int16_t D_Motor_Set(int16_t pwm)
{
    int8_t direction;
    int8_t old_direction;
    uint16_t magnitude;
    uint16_t inverted;
    uint8_t  window;

    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    if (pwm == s_motor_pwm) return pwm;

    direction = (pwm > 0) ? 1 : ((pwm < 0) ? -1 : 0);
    magnitude = (uint16_t)((pwm < 0) ? -pwm : pwm);
    inverted = (uint16_t)(MOTOR_PWM_MAX - magnitude);
    old_direction = s_motor_direction;

    /* 慢衰减下驱动段在周期尾部，采样点必须按本次幅值重新定位。
     * 传入的是幅值而不是反相后的比较值：D_TIM2_ADC_Trigger_Set 自己
     * 换算成 RES - mag/2 的中点。 */
    window = D_TIM2_ADC_Trigger_Set(magnitude);
    D_ADC_Current_Window_Set(window);

    /* 三种情况下必须让已有电流样本失效并丢弃下一块：
     *   换向        —— 前后两块样本来自不同的电流方向
     *   窗口关闭    —— 触发点移到了续流段，采到的不再是绕组电流
     *   窗口重新打开—— 那一块会混进窗口关闭期间采的续流段样本 */
    if (direction != old_direction || window != s_adc_window)
        D_ADC_Current_Filter_Reset();
    s_adc_window = window;

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
    D_ADC_Current_Filter_Reset();
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
    s_adc_window = 0U;
}
