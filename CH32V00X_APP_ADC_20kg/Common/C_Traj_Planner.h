/* C_Traj_Planner.h 在线轨迹规划器，固定1ms，纯算法
 * 入口只有 C_Traj_Command(目标,时间)：从参考当前位置以 max(T,最短运动时间) 走到目标，新指令随时覆盖旧的。
 * 三级：匀速期望路径 → V/A/D 受限跟踪器 → 速度滑动平均(加速度变斜坡，终点不变不过冲)。
 * 只认用户上限；hold 冻结时钟兜底。单位：厘度、厘度/秒、厘度/秒^2。 */
#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>

#define TRAJ_VMAX_MIN_CDPS       1L
#define TRAJ_VMAX_MAX_CDPS  100000L        /* 1000度/秒 */
#define TRAJ_AMAX_MIN_CDPSS    100L        /* 1度/秒^2 */
#define TRAJ_AMAX_MAX_CDPSS 10000000L      /* 100000度/秒^2 */
#define TRAJ_MOVE_MIN_MS_MAX    500U       /* 最短运动时间上限，ms */
#define TRAJ_SMOOTH_MS_MAX      200U       /* 平滑时间上限，ms；窗口占2字节/ms RAM */

typedef struct {
    int32_t pos, vel, acc;
    uint8_t done;           /* 1=参考已静止在终点 */
} TrajRef_t;

void    C_Traj_Init(int32_t pos);
/* 速度/加/减速度超出范围自动夹紧；平滑时间(1~200ms)在参考静止时生效 */
void    C_Traj_SetLimits(int32_t vel, int32_t acc, int32_t dec, uint16_t smooth_ms);
/* 最短运动时间(ms，0=关闭，含平滑时间) */
void    C_Traj_SetMoveMinMs(uint16_t ms);
void    C_Traj_SetRange(int32_t lo, int32_t hi);   /* 不可跨越的行程；超出则夹紧并停住 */
void    C_Traj_SetRangeOpen(int32_t lo, int32_t hi); /* 扩展范围，不打断当前参考 */
int32_t C_Traj_GetPos(void);                       /* 跟踪器当前位置：新规划起点 */
void    C_Traj_Hold(int32_t pos);                  /* 立即静止在 pos，清空平滑窗口 */
void    C_Traj_Command(int32_t target, uint16_t time_ms); /* 唯一运动入口；time_ms=0 按最短时间/最快 */
void    C_Traj_Stop(void);                         /* 按减速度刹停，不回拉 */
void    C_Traj_Step(TrajRef_t *out, uint8_t hold); /* hold=1：非减速时冻结时钟一拍 */
uint8_t C_Traj_Is_Done(void);

#endif /* __C_TRAJ_PLANNER_H__ */
