/* 电压域电机模型，纯算法。
 * U = kv·ω + ka·α + fc·sgn(ω)；ω=0 且 |U|<fs 时粘住。
 * U 为绕组平均电压(mV)，与母线电压无关；占空比换算在应用层。
 * 单位：厘度、秒、mV。 */
#ifndef __C_MOTOR_MODEL_H__
#define __C_MOTOR_MODEL_H__

#include <stdint.h>

typedef struct {
    int32_t  kv_uv;              /* 反电势斜率 µV/(厘度/s) */
    int32_t  ka_nv;              /* 加速系数 nV/(厘度/s^2) */
    int16_t  fc_mv, fs_mv;       /* 动摩擦、起转电压 mV */
    uint16_t delay_us;           /* 输入→响应纯延迟 µs */
    uint16_t cal_mv;             /* 标定电压 mV，额定能力按它算 */
    uint8_t  noise_cdeg;         /* 静止噪声幅值 ±厘度 */
    uint8_t  min_step_cdeg;      /* 最小可靠位移 厘度 */
    int8_t   sign;               /* 正电压使角度增大=1 */
    uint16_t rated_vel_dps;      /* 标定电压额定角速度(已留余量) dps */
    uint32_t rated_acc_dps2, rated_dec_dps2;
} MotorCal_t;

void     C_MotorModel_Init(const MotorCal_t *cal);
const MotorCal_t *C_MotorModel_Cal(void);
int32_t  C_MotorModel_Feedforward(int32_t vel, int32_t acc); /* kv·vel+ka·acc mV */
int32_t  C_MotorModel_TauUs(void);                           /* 机械时间常数 ka/kv µs */

#endif /* __C_MOTOR_MODEL_H__ */
