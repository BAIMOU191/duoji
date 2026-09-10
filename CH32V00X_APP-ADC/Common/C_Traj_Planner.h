#ifndef __C_TRAJ_PLANNER_H__
#define __C_TRAJ_PLANNER_H__

#include <stdint.h>
#include "A_Parameter.h"

/*
 * C_Traj_Planner —— 在线jerk受限S曲线规划器
 *
 * 轨迹由“加速过渡—匀速—减速过渡”组成；每个过渡内部的加速度为
 * “线性爬升—恒定—线性回落”，展开后就是经典七段式S曲线。
 *
 * 平滑度s映射为：
 *     r = 0.5*s/s_max              加速度爬升、回落各占过渡段的比例
 *     duration_ratio = 1/(1-r)     相对恒加速度段的时长倍率，1~2
 *
 * s=0严格退化为恒定最大加速度，时间最短；s>0时位置、速度、加速度在时间上
 * 连续，只有jerk在子段边界改变。随着s增大，段时长和整条位置/速度曲线连续
 * 变化。必须诚实说明：因为s=0按定义允许加速度瞬时跳到最大值，而任意s>0
 * 都从零加速度起步，所以“加速度在端点关于s也连续”与“0档始终最大加速度”
 * 两个要求不能同时满足；当前实现优先遵守0档最快的物理定义。
 *
 * 每次Plan根据距离、当前参考速度、速度上限和可用加减速度解析求最短轨迹。
 * 若用户给的时间更长，则整体均匀拉伸：速度按1/lambda、加速度按1/lambda^2
 * 降低。在线改目标时仍以规划器自身的参考状态为起点，位置和速度不跳变。
 *
 * 小位移约束只用一个直观参数TUNE_MOVE_MIN_MS(最短移动时长)。它只限速度不限加速度，D低于交叉点时，速度和
 * 加速度上限乘：
 *     floor + (1-floor)*(3x^2-2x^3), x=D/threshold
 * 它在x=1处数值和一阶导数都与“不开约束”连续，不含阈值阶跃。
 *
 * 定点约定：位置=厘度，速度=厘度/秒，加速度=厘度/秒^2；内部速度为
 * Q16厘度/拍。C_Traj_Step每拍只做乘加与移位，开方和一般除法只在Plan执行。
 */

typedef struct {
    int32_t pos;
    int32_t vel;
    int32_t acc;
} TrajRef_t;

/*
 * 初始化并设置加、减速平滑度。范围0~TRAJ_SMOOTH_MAX，越大越柔。
 */
void C_Traj_Init(int32_t init_pos, uint8_t accel_smooth, uint8_t decel_smooth);

/*
 * 设置不可跨越的机械行程[lo,hi]。当前参考会被夹入范围并停住。
 * 模块不自行走360°最短路径，避免穿越机械死区。
 */
void C_Traj_Set_Range(int32_t lo, int32_t hi);
void C_Traj_Hold(int32_t pos); /* 立即在指定位置停住，用于暂停/停止 */

/*
 * 从当前参考状态规划到target_pos。time_ms=0表示硬件允许的最快轨迹；
 * 请求时间小于最短时间时也使用最短时间，大于时均匀时间缩放。
 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms);

/*
 * 推进1拍。hold非0时，在饱和且实际落后参考的工况暂停轨迹时钟；
 * 已进入减速段时不会冻结，以免错过制动。
 */
void C_Traj_Step(TrajRef_t *out, uint8_t hold);

uint8_t C_Traj_Is_Done(void);
uint16_t C_Traj_Get_Ticks(void);

#endif /* __C_TRAJ_PLANNER_H__ */
