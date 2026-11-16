/* 电压域电机模型：除法只在初始化时做 */
#include "C_Motor_Model.h"
#include "C_Fixed.h"

static struct {
    MotorCal_t cal;
    int32_t kv_q16;   /* mV/(厘度/s)，Q16 */
    int32_t ka_q24;   /* mV/(厘度/s^2)，Q24 */
} s_model;

void C_MotorModel_Init(const MotorCal_t *cal)
{
    s_model.cal    = *cal;
    s_model.kv_q16 = (int32_t)(((int64_t)cal->kv_uv << 16) / 1000);
    s_model.ka_q24 = (int32_t)(((int64_t)cal->ka_nv << 24) / 1000000);
}

const MotorCal_t *C_MotorModel_Cal(void) { return &s_model.cal; }

int32_t C_MotorModel_Feedforward(int32_t vel, int32_t acc)
{
    return Fx_MulShift(s_model.kv_q16, vel, 16) + Fx_MulShift(s_model.ka_q24, acc, 24);
}

int32_t C_MotorModel_TauUs(void)
{
    return (int32_t)(((int64_t)s_model.cal.ka_nv * 1000) / s_model.cal.kv_uv);
}
