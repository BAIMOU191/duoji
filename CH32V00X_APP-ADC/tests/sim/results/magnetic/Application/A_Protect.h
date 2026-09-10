#ifndef __A_PROTECT_H__
#define __A_PROTECT_H__

#include <stdint.h>

/*
 * A_Protect.h —— 保护与功率限制模块
 *
 * 一个10ms任务，内部再分频出两条检测链和一条限流回路：
 *
 *     10ms   过流检测      连续3拍(30ms) > 3A  -> 卸力 + 冷却500ms
 *     10ms   功率限制回路  扭矩% x 3A 当限流值，超了压PWM上限，低了回升
 *     250ms  过温检测      >= 50°C 卸力，回落到45°C自动恢复
 *
 * 下面的阈值全部是可覆盖的宏：在编译命令行 -DPROT_xxx=yyy 或在本文件上方
 * 先行定义都能改，不需要动 .c。
 */

/* ---- 过温 ---- */
#define PROT_TEMP_LIMIT_C        50   /* 过温阈值(摄氏度) */
#define PROT_TEMP_HYST_C          5   /* 回差(摄氏度)：回落到 阈值-回差 才恢复 */

/* ---- 过流 ----
 * 硬件上可上报的最大电流是5.29A(见A_Sensor.h的OPA零点说明)，AT8236自身的
 * 斩波点是5.5A，所以3A的软件跳闸永远排在硬件保护之前生效。 */
#define PROT_CURRENT_LIMIT_MA  3000   /* 过流阈值(mA)，同时是扭矩100%对应的限流值 */
#define PROT_CURRENT_TRIP         3   /* 连续这么多拍超阈值才判过流，滤启动/换向尖峰 */
#define PROT_CURRENT_COOLDOWN_MS 500  /* 过流后强制卸力的冷却时长(ms) */

/* ---- 功率(扭矩)限制 ---- */
/* 出厂扭矩上限百分比，对应 #000PSP100!。
 *
 * 50 -> 100：这个数同时决定两件事——限流值(pct x 3A)和**PWM上限的开环估计
 * pwm_ff(pct x 2400)**。50% 时 pwm_ff=1200，而 A_Parameter.h 里的 v_ss/a0/
 * VMAX 全部按 CFG_PWM_FULL=2400 推导，规划器要的速度是执行器能力的两倍，
 * 整条闭环退化成开环 bang-bang(2026-09-08 实测：规划 29854、实测 15500)。
 *
 * 在这台机器上 100% 是安全的：实测空载满PWM绕组电流约 120mA，离
 * PROT_CURRENT_LIMIT_MA=3000 的跳闸点还有 25 倍余量，限流回路根本不会介入。
 * 换更大扭力的机构、或者堵转工况占比高的应用，要重新核这笔账。
 *
 * 注意：本宏只在 Flash 参数无效/CLE恢复出厂时生效。已经烧过参数的板子里
 * 存的还是旧值，必须发一次 #000PSP100! 才会改过来。 */
#define PROT_TORQUE_DEFAULT     100
#define PROT_TORQUE_MAX         100   /* 协议规定的扭矩百分比上限，不可改 */

/* 故障位，A_Protect_Fault返回值。 */
#define PROT_FAULT_NONE      0x00U
#define PROT_FAULT_OVERTEMP  0x01U
#define PROT_FAULT_OVERCUR   0x02U

void    A_Protect_Init(void);                  /* 须在A_Servo_Init之后调用 */
void    A_Protect_Task(void);                  /* 10ms任务入口 */
uint8_t A_Protect_SetTorque(uint8_t percent);  /* 设扭矩上限并落盘，0~100 */
uint8_t A_Protect_GetTorque(void);             /* 读当前扭矩上限 */
uint8_t A_Protect_Fault(void);                 /* 读故障位掩码，供诊断 */

#endif /* __A_PROTECT_H__ */
