/* D_motor.c AT8236 H桥(慢衰减)与电流采样点
 *
 * 一个PWM周期(TIM_PWM_RESOLUTION拍)，mag = |pwm|：
 *   [0, 周期-mag)    续流段：两路全高，绕组短路续流，电流不经过低边检流电阻
 *   [周期-mag, 周期) 驱动段：正转 PD3常高、PD2拉低；反转 PD2常高、PD3拉低。绕组电流流过检流电阻
 * 电流采样：TIM2_CH4 每个周期触发一次ADC。
 *   驱动段够宽(≥ D_ADC_CUR_MIN_TICKS)：触发点让采样孔径落在驱动段正中。绕组电感使周期内电流近似线性，
 *     驱动段中点的电流就是整个周期的平均电流。
 *   不够宽(含0)：触发点停在周期中间的续流段，读数是零点。
 * 占空比与触发点同为预装载，在同一个周期边界生效。 */

#include "D_motor.h"
#include "D_adc.h"

/* 一次触发的整套转换(电流+电位器+电压)要在下一次触发前做完。触发点最晚 = 最窄可测驱动段的中点，
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
