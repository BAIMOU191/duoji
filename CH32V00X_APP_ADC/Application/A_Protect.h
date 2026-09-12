#ifndef __A_PROTECT_H__
#define __A_PROTECT_H__

#include <stdint.h>

/*
 * A_Protect.h —— 保护与功率限制模块
 *
 * 一个10ms任务，内部再分频出三条检测链和一条限流回路：
 *     10ms  过流检测      连续3拍(30ms) > 3A -> 卸力 + 冷却500ms
 *     10ms  功率限制回路  扭矩% x 3A 当限流值，超了压PWM上限，低了回升
 *     20ms  堵转检测      参考在走而实际不动，连续5次(100ms) -> 就地保持
 *     250ms 过温检测      >= 50摄氏度卸力，回落到45度自动恢复
 *
 * 下面的阈值全部是可覆盖的宏：编译命令行 -DPROT_xxx=yyy 或在本文件上方先行
 * 定义都能改，不需要动 .c。各阈值的实测依据见 A_Parameter.h 的文件头。
 */

#define PROT_TEMP_LIMIT_C 50 /** 过温阈值(摄氏度) */
#define PROT_TEMP_HYST_C   5 /** 回差(摄氏度)：回落到 阈值-回差 才恢复 */

/* 硬件上可上报的最大电流是5.29A(见A_Sensor.h的OPA零点说明)，AT8236自身的斩波点
 * 是5.5A，所以3A的软件跳闸永远排在硬件保护之前生效。 */
#define PROT_CURRENT_LIMIT_MA  3000 /** 过流阈值(mA)，同时是扭矩100%对应的限流值 */
#define PROT_CURRENT_TRIP         3 /** 连续这么多拍超阈值才判过流，滤启动/换向尖峰 */
#define PROT_CURRENT_COOLDOWN_MS 500 /** 过流后强制卸力的冷却时长(ms) */

/* 扭矩百分比同时决定限流值和PWM上限的起步值。取100的理由和安全前提见
 * A_Parameter.h 的文件头；本宏只在Flash参数无效或CLE恢复出厂时生效，已烧过
 * 参数的板子里存的还是旧值，必须发一次 #000PSP100! 才会改过来。 */
#define PROT_TORQUE_DEFAULT 100 /** 出厂扭矩上限百分比 */
#define PROT_TORQUE_MAX     100 /** 协议规定的扭矩百分比上限，不可改 */

/* 堵转判据是"目标轨迹还在往前走，实际位置却不动"。每 PROT_STALL_PERIOD_MS 采一次
 * 两者的位移，连续 PROT_STALL_TRIP 次成立才判堵转，中间只要有一次不成立就清零，
 * 和过流的连续计数是同一套思路。
 *
 * 两个位移阈值必须拉开档次：REF 取 ACT 的4倍，"参考在走"和"实际没动"之间隔着4倍
 * 速度，不存在两个条件同时勉强成立的含糊地带。ACT 取 TUNE_RESOLUTION_CDEG，它本
 * 来就是这套机构的最小可靠位移，比它更小的位移和传感器噪声分不开，判"没动"是诚实的。
 *
 * 比 REF 更慢的指令(带很长T参数的慢速运动)不会走"参考在变"这条判据，但只要它真堵
 * 住，输出照样会饱和，由 HOLD_TICKS 冻结判据接住，所以慢速动作并没有失去保护。
 *
 * 触发后的动作是"把当前实际位置设成目标并保持"，**不卸力、不记故障位**：堵转常常
 * 是被人按住或撞到限位，卸力会让负载掉下去；记故障位则会连累 A_Servo_Submit 拒收
 * 后续指令，把一次可恢复的事件变成死锁。 */
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

#endif /* __A_PROTECT_H__ */
