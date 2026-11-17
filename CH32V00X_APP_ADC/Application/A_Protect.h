#ifndef __A_PROTECT_H__
#define __A_PROTECT_H__

#include <stdint.h>
#include "A_Config.h"   /* 保护参数的出厂值与范围(第5节)、额定堵转电流(第2节) */

/* A_Protect.h —— 保护与功率限制，一个10ms任务内分频。可设参数的出厂值与范围在 A_Config.h 第5节，
 * 检测周期、回差、冷却时间等内部常量在 A_Protect.c。 */

/* 扭矩统一用百分比(0-100)，分母是额定堵转电流 TORQUE_STALL_CURRENT_MA：100% 就是它对应的输出。
 * 不引入 kgf·cm 换算系数，也不扣减空载电流（与 Dynamixel Max Torque、Feetech Torque Limit 同法）。
 * 电流超过额定堵转电流时按 100% 上报，不做线性外推；过流另有过流保护与原始电流上报。 */

/* 电流换算扭矩百分比，带符号(方向同电流)，按幅值四舍五入；超额定堵转电流钳位到±满量程；
 * 无有效采样(未采在驱动段)报0。查询路径用 */
static inline int16_t A_Torque_Estimate(int16_t current_ma, uint8_t valid)
{
    uint32_t mag, pct;
    if (!valid) return 0;
    mag = (uint32_t)((current_ma < 0) ? -(int32_t)current_ma : current_ma);
    pct = (mag * TORQUE_MAX_PERCENT + TORQUE_STALL_CURRENT_MA / 2U) / TORQUE_STALL_CURRENT_MA;
    if (pct > TORQUE_MAX_PERCENT) pct = TORQUE_MAX_PERCENT;
    return (int16_t)((current_ma < 0) ? -(int32_t)pct : (int32_t)pct);
}

/* 故障位(协议 0xC1)；堵转就地保持不算故障，bit2 保留恒为0 */
#define PROT_FAULT_NONE         0x0000U /** 无故障 */
#define PROT_FAULT_OVERTEMP     0x0001U /** 过温 */
#define PROT_FAULT_OVERCUR      0x0002U /** 过流 */
#define PROT_FAULT_STALL        0x0004U /** 保留：堵转就地保持，不报故障 */
#define PROT_FAULT_UNDERVOLT    0x0008U /** 欠压 */
#define PROT_FAULT_OVERVOLT     0x0010U /** 过压 */
#define PROT_FAULT_VOLT_INVALID 0x0020U /** 电压采样尚无有效值 */
#define PROT_FAULT_VOLTAGE      (PROT_FAULT_UNDERVOLT | PROT_FAULT_OVERVOLT | PROT_FAULT_VOLT_INVALID)

/* 周期采样缓存供集合查询复用，查询本身不访问ADC或温度传感器。 */
typedef struct {
    int16_t  current_ma;    /* 最近电流，mA；位置值增大方向为正 */
    uint16_t voltage_cv;    /* 最近电压，0.01V；0=尚无有效值 */
    int16_t  temperature_dc; /* 最近读成功的温度，0.1℃；-32768=还没读成功过 */
    uint8_t  current_valid; /* 1=电流采样落在有效驱动段 */
} ProtectTelemetry_t;

void     A_Protect_Init(void);                 /* 须在电机输出入口初始化后调用 */
void     A_Protect_Task(void);                 /* 10ms任务入口 */
void     A_Protect_Refresh(void);              /* 运行参数改动后立即重算阈值与限流(否则最迟下一拍跟上) */
uint16_t A_Protect_GetTorque(void);            /* 读已生效扭矩上限，百分比0-100 */
uint16_t A_Protect_Fault(void);                /* 读故障位掩码，供诊断 */
uint8_t  A_Protect_Stalled(void);              /* 1=最近一次动作以堵转收场(仍在堵转保持中)，下一条运动指令清0 */
void     A_Protect_EnterStallHold(void);       /* 堵转保持：限流到堵转保持电流 */
void     A_Protect_ExitStallHold(void);        /* 新指令等：恢复扭矩限流 */
void     A_Protect_GetTelemetry(ProtectTelemetry_t *out); /* 仅读SRAM快照 */
int16_t  A_Protect_GetTemperature(void);       /* 最近读成功的温度，0.1℃；-32768=还没读成功过；不访问I2C */
int16_t  A_Protect_GetCurrent(void);           /* 读最近绕组电流(mA)，带符号，位置值增大方向为正 */
uint8_t  A_Protect_CanDrive(void);             /* 1=无停机故障且满足最小启动扭矩 */

#endif /* __A_PROTECT_H__ */
