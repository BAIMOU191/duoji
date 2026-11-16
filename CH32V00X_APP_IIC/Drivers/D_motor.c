/* D_motor.c AT8236 H桥(慢衰减)，mag=|pwm|：周期内 [0,周期-mag) 续流段(两路全高，电流不经过低边检流电阻)，[周期-mag,周期) 驱动段(正转PD3常高PD2拉低，反转反之)，占空比与触发点同为预装载、同周期边界生效。
 * TIM2_CH4每周期触发ADC：驱动段够宽则孔径居中(电感使电流近似线性，中点=周期平均电流)，不够宽(含0)则停在续流段读零点。 */

#include "D_motor.h"
#include "D_adc.h"

/* 一次触发的整套转换(规则组+注入组，时长 D_ADC_SCAN_TICKS 见 D_adc.h)要在下一次触发前做完。触发点最晚 = 最窄可测驱动段的中点，
 * 最早 = 满占空比驱动段的中点(停放点在两者之间)：最晚 + 转换时长 < 一个周期 + 最早 */
_Static_assert((MOTOR_PWM_MAX - D_ADC_CUR_MIN_TICKS / 2 - D_ADC_CUR_LEAD_TICKS) + D_ADC_SCAN_TICKS
               < MOTOR_PWM_MAX + (MOTOR_PWM_MAX / 2 - D_ADC_CUR_LEAD_TICKS), "ADC scan overruns the next trigger");
_Static_assert(TIM_TRIG_PARK >= MOTOR_PWM_MAX / 2 - D_ADC_CUR_LEAD_TICKS
               && TIM_TRIG_PARK <= MOTOR_PWM_MAX - D_ADC_CUR_MIN_TICKS / 2 - D_ADC_CUR_LEAD_TICKS
               && TIM_TRIG_PARK < MOTOR_PWM_MAX - D_ADC_CUR_MIN_TICKS, "park point must sit in the freewheel part");

static int16_t s_pwm = MOTOR_PWM_MAX + 1;   /* 当前占空比；越界值 = 下次 Set 必须重配 */

int16_t D_Motor_Set(int16_t pwm)
{
    uint16_t mag;

    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;
    if (pwm == s_pwm) return pwm;
    s_pwm = pwm;
    mag = (uint16_t)((pwm < 0) ? -pwm : pwm);

    /* 比较值取满 = 全周期高；PWM 的那一路在周期末 mag 拍拉低 */
    TIM_SetCompare1(TIM1, (uint16_t)((pwm >= 0) ? MOTOR_PWM_MAX - mag : MOTOR_PWM_MAX));   /* PD2 */
    TIM_SetCompare2(TIM2, (uint16_t)((pwm >= 0) ? MOTOR_PWM_MAX : MOTOR_PWM_MAX - mag));   /* PD3 */

    if (mag >= D_ADC_CUR_MIN_TICKS)
    {
        TIM_SetCompare4(TIM2, (uint16_t)(MOTOR_PWM_MAX - mag / 2U - D_ADC_CUR_LEAD_TICKS));
        D_ADC_Current_Phase((pwm > 0) ? D_CUR_FWD : D_CUR_REV);
    }
    else
    {
        TIM_SetCompare4(TIM2, TIM_TRIG_PARK);
        D_ADC_Current_Phase(D_CUR_ZERO);
    }
    return pwm;
}

void D_Motor_Release(uint8_t high_resistance)
{
    uint16_t level = high_resistance ? MOTOR_PWM_MAX : 0U;   /* 比较值取满 = 常高，取0 = 常低 */

    TIM_SetCompare1(TIM1, level);
    TIM_SetCompare2(TIM2, level);
    TIM_SetCompare4(TIM2, TIM_TRIG_PARK);
    D_ADC_Current_Phase(D_CUR_NONE);
    s_pwm = MOTOR_PWM_MAX + 1;
}
