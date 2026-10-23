/* C_Traj_Planner.h 在线轨迹规划器，固定1ms，纯算法
 *
 * 两级，点到点、带时间、连续轨迹共用同一套：
 *   1. 跟踪器：速度朝终点走，受 速度上限/加速度/减速度 约束，按制动曲线 v ≤ √(2·D·剩余距离) 刚好停在终点，
 *      新目标随时接上(从当前位置、速度继续，不先停)。
 *   2. 平滑：对跟踪器速度做长度=平滑时间的滑动平均。加速度变成斜坡(加加速度 ≤ A/平滑时间)，
 *      曲线单调、终点不变、不会过冲；代价是整段多走一个平滑时间。
 * 三种用法只是第1级的参数不同：
 *   Plan(点到点/带时间)：时长取 max(T, 最短运动时间)，比最快走法长时 V/λ、A/λ²、D/λ² 整体放慢。
 *   Follow(连续轨迹)：速度 = 平均指令速度 + (剩余距离 − 速度·展望)/(2·展望)，A、D 用全值；指令到达早晚不均时速度也平稳。
 *   Stop：按减速度刹停，不回拉。
 * 只认用户上限，不含电机能力；运行中由 hold(饱和且落后)冻结时钟兜底。
 * 每拍只有乘加移位(制动段多一次整数开方)，除法只在 Plan/Follow/SetLimits 里。单位：厘度、厘度/秒、厘度/秒^2。 */
#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>

#define TRAJ_VMAX_MIN_CDPS       1L
#define TRAJ_VMAX_MAX_CDPS  100000L        /* 1000度/秒 */
#define TRAJ_AMAX_MIN_CDPSS    100L        /* 1度/秒^2 */
#define TRAJ_AMAX_MAX_CDPSS 10000000L      /* 100000度/秒^2 */
#define TRAJ_MOVE_MIN_MS_MAX    500U
#define TRAJ_SMOOTH_MS_MAX      200U       /* 平滑时间上限，ms：滑动平均窗口长度(占 2 字节/ms 的RAM) */

typedef struct {
    int32_t pos, vel, acc;
    uint8_t done;           /* 1 = 参考已静止在终点 */
} TrajRef_t;

void    C_Traj_Init(int32_t pos);
/* 速度、加速度、减速度上限超出范围自动夹紧，下一次 Plan 生效；平滑时间(ms，1~200)在参考静止时生效 */
void    C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, uint16_t smooth_ms);
/* 最短运动时间(ms，0=关闭，含平滑时间)：每段 Plan 至少走这么久(带时间的取两者较大)，按整体拉伸降低 V/A/D */
void    C_Traj_SetMoveMinMs(uint16_t ms);
void    C_Traj_SetRange(int32_t lo, int32_t hi);   /* 不可跨越的行程；参考夹进范围并停住 */
void    C_Traj_SetRangeOpen(int32_t lo, int32_t hi); /* 扩展范围，不打断当前参考 */
int32_t C_Traj_GetPos(void);                       /* 第1级(跟踪器)当前位置：新规划的起点 */
void    C_Traj_Hold(int32_t pos);                  /* 立即静止在 pos，清空平滑窗口 */
void    C_Traj_Plan(int32_t target, uint16_t time_ms);   /* 点到点；time_ms=到达时间(0=最快)，含平滑时间 */
void    C_Traj_Follow(int32_t target, int32_t vel_cdps, uint16_t horizon_ms); /* 连续轨迹：按平均指令速度走，滞后偏离 速度·horizon 的部分慢慢追平 */
void    C_Traj_Stop(void);                         /* 按减速度刹停，不回拉 */
void    C_Traj_Step(TrajRef_t *out, uint8_t hold); /* hold=1：非减速时冻结时钟一拍 */
uint8_t C_Traj_Is_Done(void);

#endif /* __C_TRAJ_PLANNER_H__ */
