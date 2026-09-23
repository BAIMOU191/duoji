#ifndef __A_PROTECT_H__
#define __A_PROTECT_H__

#include <stdint.h>

/*
 * A_Protect.h —— 保护与功率限制，一个10ms任务内分频：
 *     10ms  过流检测      连续3拍 > 3A -> 卸力 + 冷却500ms
 *     10ms  功率限制回路  扭矩% x 3A 当限流值，超了压PWM上限
 *     20ms  堵转检测      参考在走而实际不动，连续5次 -> 就地保持
 *     250ms 过温检测      >= 50℃卸力，回落到45℃恢复
 * 阈值均可用 -DPROT_xxx 覆盖。
 */

#define PROT_TEMP_LIMIT_C 50 /** 过温阈值(摄氏度) */
#define PROT_TEMP_HYST_C   5 /** 回差(摄氏度)：回落到 阈值-回差 才恢复 */

/* 3A软件跳闸先于硬件保护(可上报上限5.29A，AT8236斩波5.5A) */
#define PROT_CURRENT_LIMIT_MA  3000 /** 过流阈值(mA)，同时是扭矩100%对应的限流值 */
#define PROT_CURRENT_TRIP         3 /** 连续这么多拍超阈值才判过流，滤启动/换向尖峰 */
#define PROT_CURRENT_COOLDOWN_MS 500 /** 过流后强制卸力的冷却时长(ms) */

/* 只在Flash参数无效或CLE恢复出厂时生效，已烧参数的板子须发 #000PSP100! */
#define PROT_TORQUE_DEFAULT 100 /** 出厂扭矩上限百分比 */
#define PROT_TORQUE_MAX     100 /** 协议规定的扭矩百分比上限，不可改 */

/* 堵转：每 PROT_STALL_PERIOD_MS 采一次参考与实际位移，连续 PROT_STALL_TRIP 次"参考在走(或轨迹被冻结)而实际没动"才判定。
 * REF 取 ACT 的4倍避免含糊地带。触发后就地保持，不卸力、不记故障位(被按住或撞限位是可恢复事件)。 */
#define PROT_STALL_PERIOD_MS  20 /** 堵转检测周期(ms)，须是10ms基拍的整数倍 */
#define PROT_STALL_TRIP        5 /** 连续这么多次成立才判堵转，5x20=100ms */
#define PROT_STALL_REF_CDEG  128 /** 一个周期内参考位移超过它才算"轨迹在变" */
#define PROT_STALL_ACT_CDEG   32 /** 一个周期内实际位移不超过它才算"没动" */
#define PROT_STALL_HOLD_TICKS 15 /** 周期内轨迹被冻结这么多拍，同样算"轨迹在变" */

#define PROT_FAULT_NONE     0x00U /** 无故障 */
#define PROT_FAULT_OVERTEMP 0x01U /** 过温故障位 */
#define PROT_FAULT_OVERCUR  0x02U /** 过流故障位，堵转不在其中，见上 */

void    A_Protect_Init(void);                 /* 须在A_Servo_Init之后调用 */
void    A_Protect_Task(void);                 /* 10ms任务入口 */
uint8_t A_Protect_SetTorque(uint8_t percent); /* 设扭矩上限并落盘，0~100 */
uint8_t A_Protect_GetTorque(void);            /* 读当前扭矩上限 */
uint8_t A_Protect_Fault(void);                /* 读故障位掩码，供诊断 */
uint8_t A_Protect_Stalled(void);              /* 1=最近一次动作以堵转收场，供诊断 */
uint16_t A_Protect_GetCurrent(void);          /* 读最近一块绕组电流(mA)，供RIV查询 */

#endif /* __A_PROTECT_H__ */
