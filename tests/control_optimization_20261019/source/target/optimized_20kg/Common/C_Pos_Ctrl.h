/* C_Pos_Ctrl.h 位置控制器，固定1ms，纯算法
 *
 * 运动律(参考在动，MOVE)：
 *   u = kv·(v_r + a_r·延迟) + ka·a_r + fc·sat(v_r)       模型前馈
 *     + Kv·(v_r + Kp·e − v̂) + I                           位置 P / 速度 PI
 *     + 起转补偿(参考起步/匀速时卡住才顶)
 * 收尾(参考 done，SETTLE)：u = Kv·(Kp·dz(e, 最小可靠位移) − v̂) + I，同一个积分器，不顶起。
 *   误差在到位窗内且静止约20ms → HOLD 并上报到位(到位只看到位精度)；没有时间限制，进不了到位精度就一直尝试。
 * 保持(HOLD，直到下一段运动)：对超出到位窗的量 ex = dz(e, 到位窗) 做 PI，积分从收尾原样继承
 *   窗内：u = I(只有到位时的托举力，没有回正力，积分不动)
 *   窗外：u = I + K_h·ex，积分 I 每秒增加 K_ih·ex(I=0 时就是从窗口边沿长出来的弹簧)
 *     P/I 是相对扭矩增益，以标定电压为满量程(电压驱动下出力 ∝ 电压)，同一组数在不同电机上相对出力一致：
 *       P(‰/°)     ：超出 1° 输出标定电压的 P‰
 *       I(‰/(°·s)) ：超出 1° 持续 1s，输出再加标定电压的 I‰(0=积分不动，只用到位时的托举力)
 *     不加 D：电压驱动下反电势就是阻尼
 *   |u| < fc/2 时短路制动；下一段运动开始时 K_h·ex 并入积分(无扰切换)。
 * 只有一个窗口：到位窗 = 到位精度(配置层保证 ≥ 1.5×最小可靠位移)，到位和保持共用，超出就拉回。
 * 增益由三个带宽(固件常量)经模型换算(ka·ω_v = kv + Kv)；积分以输出电压保存，改增益输出连续。单位：厘度、秒、mV。 */
#ifndef __C_POS_CTRL_H__
#define __C_POS_CTRL_H__

#include <stdint.h>
#include "C_Motor_Model.h"
#include "C_Traj_Planner.h"

typedef struct {
    int32_t pos;    /* 观测器位置，厘度 */
    int32_t vel;    /* 观测器速度，厘度/s */
    uint8_t valid;  /* 0=本拍位置无效：积分与状态冻结 */
} PosCtrlFeedback_t;

typedef struct {
    uint16_t pos_bw;   /* 位置环带宽 ω_p，rad/s */
    uint16_t vel_bw;   /* 速度环带宽 ω_v，rad/s */
    uint16_t int_bw;   /* 速度积分零点 ω_i，rad/s */
} PosCtrlGains_t;

typedef enum { PC_MOVE = 0, PC_SETTLE, PC_HOLD } PosCtrlState_t;

typedef struct {
    int32_t e_pos;
    int16_t u_ff, u_fb, u_i, u_stick, u_out;   /* mV */
    uint8_t sat, state;
} PosCtrlDbg_t;

/* vel_noise=观测器速度噪声底(厘度/s)；obs_bw_hz=观测器带宽，速度环带宽不超过它 */
void    C_PosCtrl_Init(const MotorCal_t *cal, int32_t vel_noise, uint16_t obs_bw_hz);
/* 按约束钳位后生效并写回：ω_v ≤ min(1/(3·延迟), 观测器带宽)，ω_p ≤ ω_v/4，ω_i ≤ ω_p。返回1=有钳位 */
uint8_t C_PosCtrl_SetGains(PosCtrlGains_t *g);
/* precision=到位精度(厘度，到位窗兼保持窗)，保持 P(‰/°)、I(‰/(°·s))，‰ 指标定电压的千分之一 */
void    C_PosCtrl_SetHold(uint16_t precision, uint16_t hold_kp, uint16_t hold_ki);
void    C_PosCtrl_Reset(void);                  /* 失能/故障/坐标变化：清积分与状态 */
/* limit_mv=本拍可用电压；返回 mV，角度增大为正 */
int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit_mv, PosCtrlDbg_t *dbg);
uint8_t C_PosCtrl_Reached(void);                /* 1 = 本段已按到位精度到位，正在保持 */

/* Qualified arrival is separate from the currently holding state used by Stop. */
uint8_t C_PosCtrl_ArrivalQualified(void);
void C_PosCtrl_BeginMotion(void);

#endif /* __C_POS_CTRL_H__ */
