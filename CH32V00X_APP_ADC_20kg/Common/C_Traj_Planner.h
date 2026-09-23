#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>
#include "A_Parameter.h"

/*
 * C_Traj_Planner —— 在线jerk受限S曲线规划器
 *
 * 加速过渡—匀速—减速过渡，受最大速度/加速度/减速度/加加速度四个上限约束(标准七段式的在线形式)：
 * 速度差够大时过渡段加速度为梯形，否则为三角形。加速侧还受硬件可用加速度、减速侧受硬件制动上限约束。
 * 每次Plan以当前参考状态为起点求最短轨迹，请求时间更长时整体拉伸(速度/lambda，加速度/lambda^2)。
 * 单位：位置厘度，速度厘度/秒，加速度厘度/秒^2。每拍只做乘加移位，除法和开方只在Plan里。
 */

typedef struct {
    int32_t pos;
    int32_t vel;
    int32_t acc;
} TrajRef_t;

/* 初始化并停在 init_pos，轨迹上限取调参面板(TUNE_VMAX/AMAX/JMAX) */
void C_Traj_Init(int32_t init_pos);

/* 设置轨迹上限：速度厘度/秒、加速度与减速度厘度/秒^2(减速<=0则跟随加速)、加加速度厘度/秒^3；
 * 超出可设范围自动夹紧，下一次Plan生效 */
void C_Traj_Set_Limits(int32_t vmax_cdps, int32_t amax_cdpss, int32_t dmax_cdpss, int32_t jmax_cdpss3);
void C_Traj_Set_MoveMinMs(uint16_t move_min_ms); /* 0=关闭，最大200ms，下一次Plan生效 */

/* 设置不可跨越的行程[lo,hi]，参考被夹入范围并停住；不走最短路径，不穿越死区 */
void C_Traj_Set_Range(int32_t lo, int32_t hi);
/* 同上，但当前参考已在新范围内时不停住轨迹，供多圈指令临时放开行程用 */
void C_Traj_Set_Range_Open(int32_t lo, int32_t hi);
void C_Traj_Hold(int32_t pos); /* 立即在指定位置停住，用于暂停/停止 */

/* 从当前参考规划到target_pos；time_ms=0或不可达时取最快，更长时均匀拉伸 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms);

/* 推进1拍；hold非0时冻结轨迹时钟(饱和且落后)，减速段不冻结 */
void C_Traj_Step(TrajRef_t *out, uint8_t hold);

uint8_t C_Traj_Is_Done(void);
int32_t C_Traj_Get_Pos(void); /* 当前参考位置，即下一次Plan的段起点(厘度) */
uint16_t C_Traj_Get_Ticks(void);

#endif /* __C_TRAJ_PLANNER_H__ */
