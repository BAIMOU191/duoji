/* C_Traj_Planner.h 在线轨迹规划器，固定1ms，纯算法
 *
 * 只有一个运动入口 C_Traj_Command(目标, 时间)：不管上位机是点到点、带时间、连续流式还是 T=0，每条指令都是
 * "从参考此刻的位置出发，用 max(T, 最短运动时间) 到达这个目标"，新指令随时覆盖旧的。三级：
 *   1. 期望路径：从当前参考位置匀速走到目标，用时 = max(T, 最短运动时间)(平滑窗口的半窗延迟不扣，到达约晚半个窗口)。
 *      连续发来的指令各自起算：顺路时从旧路径当前点接着走，路径首尾相接；T 比指令间隔长时参考按 T 慢慢追，
 *      T 不比间隔长时按指令本身的节奏走；T=0 的稀疏指令按最短运动时间一步到位后停住。
 *   2. 跟踪器：期望速度 = 路径速度 + 追平路径误差的速度(路径在走时按25ms时间常数，到头后按制动曲线 √(2·D·误差))，
 *      再受"在目标前刹得住"与 V 限制，
 *      按 A(变快)/D(变慢、掉头)逼近：路径走不了的(超过 V/A/D)由它限住，反向时立即按全减速度掉头。
 *   3. 平滑：对跟踪器速度做长度=平滑时间的滑动平均，加速度变成斜坡(加加速度 ≤ A/平滑时间)，终点不变、不过冲。
 * 只认用户上限，不含电机能力；运行中由 hold(饱和且落后)冻结时钟兜底。
 * 每拍只有乘加移位(制动段多一两次整数开方)，除法只在 Command/SetLimits 里。单位：厘度、厘度/秒、厘度/秒^2。 */
#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>

#define TRAJ_VMAX_MIN_CDPS       1L
#define TRAJ_VMAX_MAX_CDPS  100000L        /* 1000度/秒 */
#define TRAJ_AMAX_MIN_CDPSS    100L        /* 1度/秒^2 */
#define TRAJ_AMAX_MAX_CDPSS 10000000L      /* 100000度/秒^2 */
#define TRAJ_MOVE_MIN_MS_MAX    500U       /* 最短运动时间上限，ms */
#define TRAJ_SMOOTH_MS_MAX      200U       /* 平滑时间上限，ms：滑动平均窗口长度(占 2 字节/ms 的RAM) */

typedef struct {
    int32_t pos, vel, acc;
    uint8_t done;           /* 1 = 参考已静止在终点 */
} TrajRef_t;

void    C_Traj_Init(int32_t pos);
/* 速度、加速度、减速度上限超出范围自动夹紧，立即生效；平滑时间(ms，1~200)在参考静止时生效 */
void    C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, uint16_t smooth_ms);
/* 最短运动时间(ms，0=关闭，含平滑时间)：每条指令的路径至少走这么久(带时间的取两者较大) */
void    C_Traj_SetMoveMinMs(uint16_t ms);
void    C_Traj_SetRange(int32_t lo, int32_t hi);   /* 不可跨越的行程；参考夹进范围并停住 */
void    C_Traj_SetRangeOpen(int32_t lo, int32_t hi); /* 扩展范围，不打断当前参考 */
int32_t C_Traj_GetPos(void);                       /* 第1级(跟踪器)当前位置：新规划的起点 */
void    C_Traj_Hold(int32_t pos);                  /* 立即静止在 pos，清空平滑窗口 */
void    C_Traj_Command(int32_t target, uint16_t time_ms); /* 唯一运动入口：time_ms=到达时间(0=按最短运动时间/最快)，含平滑时间 */
void    C_Traj_Stop(void);                         /* 按减速度刹停，不回拉 */
void    C_Traj_Step(TrajRef_t *out, uint8_t hold); /* hold=1：非减速时冻结时钟一拍 */
uint8_t C_Traj_Is_Done(void);

#endif /* __C_TRAJ_PLANNER_H__ */
