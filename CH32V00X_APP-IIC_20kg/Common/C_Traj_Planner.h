#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>

/*
 * C_Traj_Planner —— 在线jerk受限S曲线规划器
 *
 * 加速过渡—匀速—减速过渡，受最大速度/加速度/减速度/加加速度四个上限约束(标准七段式的在线形式)：
 * 速度差够大时过渡段加速度为梯形，否则为三角形。加速侧还受硬件可用加速度、减速侧受硬件制动上限约束。
 * 每次Plan以当前参考状态为起点求最短轨迹，请求时间更长时整体拉伸(速度/lambda，加速度/lambda^2)。
 * 单位：位置厘度，速度厘度/秒，加速度厘度/秒^2。每拍只做乘加移位，除法和开方只在Plan里。
 */

/* C_Traj_Set_Limits 接受的范围，超出自动夹紧。速度上限是本机硬件值 PLANT_VMAX_HW_CDPS。 */
#define TRAJ_AMAX_MIN_CDPSS    100L        /* 1度/秒^2 */
#define TRAJ_AMAX_MAX_CDPSS   10000000L    /* 100000度/秒^2 */
#define TRAJ_JMAX_MIN_CDPSS3    100L       /* 1度/秒^3 */
#define TRAJ_JMAX_MAX_CDPSS3  200000000L   /* 2000000度/秒^3，此时斜坡短于1拍=加速度阶跃 */

typedef struct {
    int32_t pos;
    int32_t vel;
    int32_t acc;
} TrajRef_t;

/* 初始化并停在 init_pos，轨迹上限取 A_Config.h 的出厂默认值(随后由 A_Servo 按 Flash 参数下发) */
void C_Traj_Init(int32_t init_pos);

/* 设置轨迹上限：速度厘度/秒、加速度与减速度厘度/秒^2、加加速度厘度/秒^3；后三项为0时取对应硬件最大值，
 * 超出可设范围自动夹紧，下一次Plan生效 */
void C_Traj_Set_Limits(int32_t vmax_cdps, int32_t amax_cdpss, int32_t dmax_cdpss, int32_t jmax_cdpss3);
void C_Traj_Set_MoveMinMs(uint16_t move_min_ms); /* 0=关闭，最大200ms，下一次Plan生效 */

/* 对象加速度增益比(Q12，4096=标定值，>1 表示负载更重)：硬件加速度/制动上限按它缩小，下一次 Plan 起生效 */
void C_Traj_SetPlant(int32_t ka_q12);

/* 设置不可跨越的行程[lo,hi]，参考被夹入范围并停住；不走最短路径，不穿越死区 */
void C_Traj_Set_Range(int32_t lo, int32_t hi);
void C_Traj_Hold(int32_t pos); /* 立即在指定位置停住，用于暂停/停止 */

/* 从当前参考规划到target_pos；time_ms=0或不可达时取最快，更长时均匀拉伸 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms);

/* 推进1拍；hold非0时冻结轨迹时钟(饱和且落后)，减速段不冻结 */
void C_Traj_Step(TrajRef_t *out, uint8_t hold);

uint8_t C_Traj_Is_Done(void);
uint16_t C_Traj_Get_Ticks(void);

/* 多圈轨迹放开范围，当前位置仍在范围内时保留运行状态。 */
void C_Traj_Set_Range_Open(int32_t lo, int32_t hi);
int32_t C_Traj_Get_Pos(void);

#endif /* __C_TRAJ_PLANNER_H__ */
