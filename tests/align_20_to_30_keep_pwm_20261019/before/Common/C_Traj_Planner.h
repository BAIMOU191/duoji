/* C_Traj_Planner.h 在线 jerk 受限 S 曲线规划器，固定1ms，纯算法
 *
 * 加速过渡—匀速—减速过渡，受 V/A/D/J 四个上限约束(七段式的在线形式)：速度差够大时过渡段加速度为梯形，
 * 否则为三角形。每次 Plan 以当前参考的位置、速度为起点求最短轨迹；带时间时整体拉伸(v/λ、a/λ²、j/λ³)。
 * 最短运动时间 Tm：每段至少走 Tm(带时间的取两者较大)，同样按整体拉伸，所以越小的动作 V/A/J 降得越多，
 * 最快也要走 Tm 以上的大动作不受影响。普通 Plan 从当前参考状态接着规划；
 * 显式带时间的 PlanTimed 在同向运动且约束可行时，用两段三次速度过渡和巡航接续当前位置/速度/加速度。
 * 接续失败回到普通 Plan，T=0展望与Stop保持原路径。每条新目标求解一次，每1ms只计算已规划曲线。
 * 连续目标的间隔约在 Tm/2 以内时，新目标在参考停下前到达，运动连贯。
 * 只认用户上限，不含电机能力；能力约束由调用方在设置时完成，运行中由 hold(饱和且落后)冻结时钟兜底。
 * 每拍只有乘加移位，除法和开方只在 Plan/SetLimits 里。单位：厘度、厘度/秒、厘度/秒^2、厘度/秒^3。 */
#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>

#define TRAJ_VMAX_MIN_CDPS       1L
#define TRAJ_VMAX_MAX_CDPS  100000L        /* 1000度/秒 */
#define TRAJ_AMAX_MIN_CDPSS    100L        /* 1度/秒^2 */
#define TRAJ_AMAX_MAX_CDPSS 10000000L      /* 100000度/秒^2 */
#define TRAJ_JMAX_MIN_CDPSS3   100L        /* 1度/秒^3 */
#define TRAJ_JMAX_MAX_CDPSS3 200000000L    /* 2000000度/秒^3，斜坡短于1拍=加速度阶跃 */
#define TRAJ_MOVE_MIN_MS_MAX    500U
#define TRAJ_SMOOTH_MS_MAX      200U   /* 平滑时间(A/J)上限，ms：重规划时的加速度接管段按拍预算，最长这么多拍 */

typedef struct {
    int32_t pos, vel, acc;
    uint8_t done;           /* 1 = 参考已静止在终点 */
} TrajRef_t;

void    C_Traj_Init(int32_t pos);
/* 上限超出范围自动夹紧，下一次 Plan 生效 */
void    C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, int32_t jerk);
/* 最短运动时间(ms，0=关闭)：每段 Plan 至少走这么久(带时间的取两者较大)，按整体拉伸降低 V/A/J */
void    C_Traj_SetMoveMinMs(uint16_t ms);
void    C_Traj_SetRange(int32_t lo, int32_t hi);   /* 不可跨越的行程；参考夹进范围并停住 */
void    C_Traj_SetRangeOpen(int32_t lo, int32_t hi); /* IIC多圈扩展范围，不打断当前参考 */
int32_t C_Traj_GetPos(void);                       /* 当前参考位置，包含接管段 */
void    C_Traj_Hold(int32_t pos);                  /* 立即静止在 pos */
void    C_Traj_Plan(int32_t target, uint16_t time_ms); /* 时长取 time_ms 与最短运动时间较大者，比最速长时均匀拉伸 */
void    C_Traj_PlanTimed(int32_t target, uint16_t time_ms); /* 显式带时间指令：运动中优先连续接续 */
void    C_Traj_Stop(void);                         /* 按减速度刹停，不回拉 */
void    C_Traj_Step(TrajRef_t *out, uint8_t hold); /* hold=1：加速/匀速段冻结时钟一拍 */
uint8_t C_Traj_Is_Done(void);

#endif /* __C_TRAJ_PLANNER_H__ */
