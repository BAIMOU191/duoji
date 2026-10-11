/* C_Friction.h 摩擦补偿(静摩擦顶起 + 动摩擦)与实测位移速度，纯算法，由位置控制器每拍调用 */
#ifndef __C_FRICTION_H__
#define __C_FRICTION_H__

#include <stdint.h>
#include "A_Config.h"

/* 速度噪声底(A_Config.h 第6节)：观测器带宽 * 最小可靠位移 * 3/8。控制器和摩擦补偿的速度阈值都以它为刻度，
 * 降观测器带宽时阈值自动变小；只改到位精度 TUNE_RESOLUTION_CDEG 不会动到它们。 */
#define CTRL_VEL_NOISE_CDPS PLANT_VEL_NOISE_CDPS
#define CTRL_VEL_DB_CDPS (CTRL_VEL_NOISE_CDPS * 2)          /** 静止判据的速度死区(1倍会漏过一半噪声) */
#define CTRL_FRIC_MOVE_CDPS (CTRL_VEL_NOISE_CDPS * 4)       /** 判"已挣脱静摩擦"的速度门限 */
/* 起转值。本机静摩擦与动摩擦差好几倍，所以必须连续过渡而不是二值切换 */
#define CTRL_FRIC_STATIC ((CAL_BREAKAWAY_PWM > CAL_FRICTION_PWM) ? CAL_BREAKAWAY_PWM : CAL_FRICTION_PWM)

void    C_Fric_Reset(void);                 /* 回到"粘住"状态，第一次修正才有起转力 */
int32_t C_Fric_MeasVel(int32_t pos);        /* 每拍先调：实测位移速度(厘度/秒，绝对值) */
/* 返回摩擦前馈PWM。u_rest=其余各项之和；ahead=轴已超到参考前面 */
int32_t C_Fric_Update(int32_t w_cmd, int32_t av_meas, int32_t pos, int32_t u_rest, uint8_t ahead);

#endif /* __C_FRICTION_H__ */
