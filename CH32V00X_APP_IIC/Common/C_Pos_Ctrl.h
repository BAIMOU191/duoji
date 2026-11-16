/* C_Pos_Ctrl.h 位置控制器，固定1ms，纯算法
 * 三态：MOVE(运动)→SETTLE(收尾)→HOLD(保持)。MOVE 用模型前馈+位置P/速度反馈+积分；
 * SETTLE 无记忆 P+阻尼+锚在目标的弹簧，积分只当静载托举力；HOLD 对超出到位窗的 ex 做 PI。
 * 积分分账：运动中动态份额(∝|v|,|a|)停下后渐撤，静载留下。只有一个窗：到位窗=保持窗。
 * 单位：厘度、秒、mV；保持 P/I 相对标定电压(‰)，实际效果×1.5。 */
#ifndef __C_POS_CTRL_H__
#define __C_POS_CTRL_H__

#include <stdint.h>
#include "C_Motor_Model.h"
#include "C_Traj_Planner.h"

#define POS_CTRL_LOAD_RATIO_MAX_X10 40U /* J_load/J_motor，0~4.0，×10 */

typedef struct {
    int32_t pos;    /* 观测器位置，厘度 */
    int32_t vel;    /* 观测器速度，厘度/s */
    int32_t dist;   /* 观测器扰动估计，mV，阻碍角度增大为正；负载惯量比>0 时运动段前馈抵消 */
    uint8_t valid;  /* 0=本拍位置无效：积分与状态冻结 */
} PosCtrlFeedback_t;

typedef struct {
    uint16_t pos_bw;   /* 位置环带宽 ω_p，rad/s */
    uint16_t vel_bw;   /* 速度环带宽 ω_v，rad/s */
    uint16_t int_bw;   /* 速度积分零点 ω_i，rad/s */
    uint16_t load_ratio_x10;    /* 带宽用负载惯量比×10：外环带宽按它收敛，按可能的最大负载取 */
    uint16_t ff_load_ratio_x10; /* 用户负载惯量比×10：加速度前馈用(须与观测器模型一致)；0=扰动前馈关闭 */
} PosCtrlGains_t;

typedef enum { PC_MOVE = 0, PC_SETTLE, PC_HOLD } PosCtrlState_t;

typedef struct {
    int32_t e_pos;
    int16_t u_ff, u_fb, u_i, u_stick, u_out;   /* mV */
    uint8_t sat, state;
} PosCtrlDbg_t;

/* vel_noise=速度噪声底(厘度/s)；obs_bw_hz=观测器带宽，ω_v 不超过它 */
void    C_PosCtrl_Init(const MotorCal_t *cal, int32_t vel_noise, uint16_t obs_bw_hz);
/* 钳位后生效并写回：ω_v≤min(1/(3·延迟),观测器带宽)，ω_p≤ω_v/4，ω_i≤ω_p。返回1=有钳位 */
uint8_t C_PosCtrl_SetGains(PosCtrlGains_t *g);
/* precision=到位精度(厘度，到位窗兼保持窗)；hold_kp/ki：‰/°、‰/(°·s) */
void    C_PosCtrl_SetHold(uint16_t precision, uint16_t hold_kp, uint16_t hold_ki);
void    C_PosCtrl_Reset(void);                  /* 失能/故障/坐标变化：清积分与状态 */
void    C_PosCtrl_StallHold(void);              /* 堵转保持：清托举力，直接进 HOLD */
/* limit_mv=可用电压；返回 mV，角度增大为正 */
int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit_mv, PosCtrlDbg_t *dbg);
uint8_t C_PosCtrl_Reached(void);                /* 1=已到位并在保持 */
uint8_t C_PosCtrl_ArrivalQualified(void);       /* 本条运动满足过到位条件；未必已进 HOLD */
void    C_PosCtrl_SetMoveSpan(int32_t span_cdeg); /* 新指令的运动距离：运动段增益按距离在 ×1~上限 间取 */
void    C_PosCtrl_BeginMotion(void);            /* 新规划清到位资格，保留积分及交接状态 */
void    C_PosCtrl_SetLoadLearn(uint8_t enable); /* 位置负载自学习开关；关闭时模型并入积分后清零，输出不跳 */
void    C_PosCtrl_LoadLearnReset(void);         /* 坐标变化(切模式/校准)：清掉学到的模型 */

#endif /* __C_POS_CTRL_H__ */
