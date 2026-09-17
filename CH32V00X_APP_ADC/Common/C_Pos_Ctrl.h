#pragma once

#include <stdint.h>
#include "C_Traj_Planner.h"
#include "C_Speed_Observer.h"

/*
 * C_Pos_Ctrl.h —— 级联位置控制器：位置P + 速度PI + 对象前馈，纯算法可在PC上测试
 *
 *   e_pos = pos_ref - pos_obs(含相对速度预测)
 *   w_cmd = vel_ref + Kp_pos*e_pos + acc_ref*delay
 *   e_vel = vel_ref + Kp_pos*e_pos - vel_obs
 *   u     = 摩擦前馈 + Kv*w_cmd + Ka*acc_ref + Kp_vel*e_vel + ∫Ki*e_vel
 * 模型只用于前馈，模型误差由速度PI消除。输出最后乘 CAL_MOTOR_SIGN 换到H桥物理方向。
 */

typedef enum {
    POSCTRL_RUN  = 0,   /* 跟随轨迹 */
    POSCTRL_HOLD = 1    /* 已到位保持：放宽死区，误差进窗后输出静音(短路制动) */
} PosCtrlState_t;

/* 遥测：分项输出，便于区分模型/反馈/积分/饱和问题 */
typedef struct {
    int32_t e_pos;      /* 位置误差，厘度 */
    int32_t e_vel;      /* 速度误差，厘度/秒 */
    int32_t w_corr;     /* 位置环速度修正，厘度/秒 */
    int32_t w_cmd;      /* 速度指令，厘度/秒 */
    int16_t u_fric;     /* 摩擦前馈，PWM */
    int16_t u_vel;      /* 速度前馈，PWM */
    int16_t u_acc;      /* 加速度前馈，PWM */
    int16_t u_fb;       /* 速度P反馈，PWM */
    int16_t u_i;        /* 速度PI积分项，PWM */
    int16_t u_out;      /* 算法坐标输出，尚未乘方向符号 */
    uint8_t state;      /* PosCtrlState_t */
    uint8_t sat;        /* 1=本拍输出饱和 */
    uint8_t braking;    /* 1=减速/残余运动抑制阶段 */
} PosCtrlDbg_t;

/* 初始化并清零速度PI积分与到位状态。 */
void C_PosCtrl_Init(void);

/* 复位状态保留参数，用于失能/故障恢复后接管；正常换指令不要调用(清积分会力矩突变) */
void C_PosCtrl_Reset(void);

/* 推进一拍，返回H桥物理方向PWM(已乘方向并限幅)；dbg 可传0 */
int16_t C_PosCtrl_Update(const TrajRef_t *ref, const SpeedObsOut_t *obs,
                         uint8_t traj_done, PosCtrlDbg_t *dbg);

/* 设置对称输出上限(限功率用)，0或超上限恢复默认；积分抗饱和跟随此上限 */
void C_PosCtrl_Set_Output_Limit(int16_t limit);

uint8_t C_PosCtrl_In_Position(void);
