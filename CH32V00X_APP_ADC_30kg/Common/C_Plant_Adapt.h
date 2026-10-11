/* C_Plant_Adapt.h 对象参数自学习：每段运动结束估计负载惯量比 rho 与速度增益比 k(相对标定，Q12)
 *
 * 标定是空载做的，装上负载后惯量变大、电机个体/电压不同速度增益也会变，模型一失配，
 * 前馈、速度环带宽和观测器全部跟着偏(仿真惯量x2过冲3度、x3过冲7度)。
 * 模型惯量或速度增益不对时，观测器的负载估计 d 会和"加速用的驱动量"z=u-v/Kv、"反电势"w=v/Kv 成比例：
 *     d = s*z + g*w + c    =>  惯量比 rho' = 1/(g+1-s)，速度增益比 k' = (1-s)*rho'
 * 每段运动结束解一次这个回归(常数项 c 吸收摩擦/重力等恒定偏差)，不动点就是真实值。
 * 两个运动方向分开估计，只采用两边一致的那部分偏差(都偏大取小的、都偏小取大的、一大一小不改)：
 * 负载惯量对两个方向一样，方向本身的不对称不该被当成负载来回翻。
 * 学到的值由 A_Servo 同步给观测器、控制器、规划器；上电从标定值开始，运行中重同步不清。 */
#ifndef __C_PLANT_ADAPT_H__
#define __C_PLANT_ADAPT_H__

#include <stdint.h>
#include "C_Speed_Observer.h"

void    C_PlantAdapt_Reset(void);                                     /* rho=k=1，丢弃样本 */
void    C_PlantAdapt_Abort(void);                                     /* 丢弃本段还没结算的样本 */
void    C_PlantAdapt_Sample(const SpeedObsOut_t *obs, uint8_t moving); /* 每拍，观测器更新之后 */
uint8_t C_PlantAdapt_MoveEnd(void);                                   /* 一段运动结束；返回1=rho或k变了 */
int32_t C_PlantAdapt_Rho(void);                                       /* 负载惯量比，Q12 */
int32_t C_PlantAdapt_Kv(void);                                        /* 速度增益比，Q12 */

#endif /* __C_PLANT_ADAPT_H__ */
