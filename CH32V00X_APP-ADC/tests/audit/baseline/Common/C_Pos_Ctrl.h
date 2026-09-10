#pragma once

#include <stdint.h>
#include "C_Traj_Planner.h"
#include "C_Speed_Observer.h"

/*
 * C_Pos_Ctrl.h —— 标准级联位置控制器：位置P + 速度PI + 对象前馈
 *
 * 输入参考轨迹与观测值，输出PWM计数，不依赖硬件寄存器，可在PC上测试。
 * 每拍的信号关系为：
 *
 *   e_pos = pos_ref - pos_obs（含1拍相对速度预测）
 *   w_cmd = vel_ref + Kp_pos*e_pos + acc_ref*delay
 *   e_vel = vel_ref + Kp_pos*e_pos - vel_obs
 *   u = 摩擦前馈 + Kv*w_cmd + Ka*acc_ref + Kp_vel*e_vel + ∫Ki*e_vel
 *
 * 核心是成熟的P-PI级联，不是某台舵机专用的经验控制律。模型只用于前馈；
 * 模型有误时速度PI消除低频稳态误差。工程保护只有输出限幅、积分抗饱和(条件积分)、
 * 到位滞环、大加速度时暂停积分，以及保持态的输出静音。
 *
 * 制动方向仍使用相同结构和目标带宽，只用型号级MODEL_BRAKE_GAIN_Q8补偿
 * 慢衰减H桥的方向增益差异。控制器最后才乘CAL_MOTOR_SIGN，把“编码器角度
 * 增大为正”的算法坐标换成H桥物理方向。
 *
 * 平滑度只改变参考轨迹，不改变控制器增益。smooth=0按硬件上限最快运动；
 * smooth>0的加速度从0爬升并在段尾回到0，数值越大，jerk越小、时长越长。
 */

typedef enum {
    POSCTRL_RUN  = 0,   /* 跟随轨迹 */
    POSCTRL_HOLD = 1    /* 已到位保持：放宽位置死区 + 速度反馈噪声底
                         * 死区；误差进死区后输出直接静音(短路制动)，防蜂鸣 */
} PosCtrlState_t;

/* 遥测：分项输出便于区分模型、反馈、积分和饱和问题。 */
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

/*
 * 复位状态但保留参数。用于失能、故障恢复、传感器错误后重新接管。
 * 正常收到新指令时不要调用，否则带载时清零积分会产生力矩突变。
 */
void C_PosCtrl_Reset(void);

/*
 * 推进一拍，返回H桥物理方向PWM（已乘方向符号并限幅）。
 * ref/obs分别是本拍参考与观测；traj_done用于到位判据；dbg可传0。
 */
int16_t C_PosCtrl_Update(const TrajRef_t *ref, const SpeedObsOut_t *obs,
                         uint8_t traj_done, PosCtrlDbg_t *dbg);

/*
 * 设置本拍对称输出上限，用于电流/功率降额。传0或超过硬件上限时恢复默认值。
 * 抗积分饱和跟随此上限，限功率时积分项不会继续充电。
 */
void C_PosCtrl_Set_Output_Limit(int16_t limit);

uint8_t C_PosCtrl_In_Position(void);
