/* 三状态模型观测器(位置/速度/扰动电压)，固定1ms，纯算法。
 * 预测：v'=v+α·(g·(u−d−摩擦)−v)，α=T·kv/ka，g=1/kv；p'=p+T·v；d'=d。
 * 校正：位置残差(半拍前)修三状态；极点 bw_hz(位置/速度)、dist_hz(扰动)。
 * 摩擦连续过渡，避免卡住估出假速度。 */
#ifndef __C_SPEED_OBSERVER_H__
#define __C_SPEED_OBSERVER_H__

#include <stdint.h>
#include "C_Motor_Model.h"

#define SPEED_OBS_HISTORY 8U   /* 延迟对齐历史长度，延迟须小于它 */

typedef struct {
    int32_t pos, vel;   /* 估计位置厘度、速度厘度/s */
    int32_t dist;       /* 扰动估计 mV，阻碍正向为正；仅供诊断 */
    uint8_t stuck;      /* 1=净电压不足起转且速度未过渡到动摩擦(诊断) */
} SpeedObsOut_t;

/* 实例由调用者持有 */
typedef struct {
    int32_t pos_q8, vel_q8, dist_q8;
    int16_t history[SPEED_OBS_HISTORY];
    uint8_t index;
} SpeedObserver_t;

/* 计算增益，所有实例共用；bw_hz=位置/速度极点，dist_hz=扰动极点(≤bw_hz)；
 * vel_noise=速度噪声底厘度/s，静摩擦在2噪声底内过渡到动摩擦 */
void C_SpeedObs_Configure(const MotorCal_t *cal, uint16_t bw_hz, uint16_t dist_hz, int32_t vel_noise);
void C_SpeedObs_Init(SpeedObserver_t *obs, int32_t pos);
/* 每1ms调用；valid=0只预测；u_mv=上一拍施加电压(角度增大为正) */
void C_SpeedObs_Update(SpeedObserver_t *obs, int32_t pos, uint8_t valid, int32_t u_mv, SpeedObsOut_t *out);

#endif /* __C_SPEED_OBSERVER_H__ */
