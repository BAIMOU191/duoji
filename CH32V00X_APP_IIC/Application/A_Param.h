#pragma once

#include <stdint.h>
#include "A_Config.h"

/* A_Param.h —— 虚拟寄存器参数表(二进制协议 CMD 02/03 的地址空间)：地址、线上宽度、范围、写后联动都在 A_Param.c 一张表里；
 * Flash 参数上电修复也用同一张表，范围只写一处。协议层只做帧，不碰参数字段。
 * 地址分区：00~0F 硬件信息(只读)，10~1F 固件，20~7F Flash 参数(掉电保存)，80~BF SRAM 参数与动作，C0~ 只读实时状态。
 * 对外数值一律是不超过2字节的整数物理量(°/s、°/s²、ms、%、0.01°、mA、0.1V、0.1℃…)，内部定点数不出现在表里。 */

/* ---- 00~10 硬件与固件信息(只读) ---- */
#define PARAM_HW_TYPE              0x00U
#define PARAM_HW_VERSION           0x01U
#define PARAM_HW_VOLTAGE           0x02U
#define PARAM_HW_TORQUE            0x03U
#define PARAM_HW_VMAX              0x04U  /* 额定最大速度 °/s，2B */
#define PARAM_HW_AMAX              0x05U  /* 额定最大加速度 °/s²，2B */
#define PARAM_HW_DMAX              0x06U  /* 额定最大减速度 °/s²，2B */
#define PARAM_HW_SMOOTH            0x07U  /* 自动平滑时间 τ，ms，1B */
#define PARAM_HW_MIN_STEP          0x08U  /* 最小可靠位移，厘度，1B */
#define PARAM_FW_VERSION           0x10U
/* ---- 20~4B Flash 参数 ---- */
#define PARAM_SERVO_ID             0x20U
#define PARAM_BAUD_CODE            0x21U
#define PARAM_SERVO_MODE           0x22U
#define PARAM_BOOT_MODE            0x23U
#define PARAM_TORQUE_LIMIT         0x24U
#define PARAM_STARTUP_POS          0x25U  /* 上电位置，位置码 0~4095 */
#define PARAM_TRAJ_VMAX            0x30U
#define PARAM_TRAJ_AMAX            0x31U
#define PARAM_TRAJ_DMAX            0x32U
#define PARAM_TRAJ_SMOOTH          0x33U
#define PARAM_MOVE_MIN             0x34U
#define PARAM_HOLD_KP              0x35U
#define PARAM_RESOLUTION           0x37U
#define PARAM_HOLD_KI              0x38U
#define PARAM_PROT_CURRENT_LIMIT   0x40U
#define PARAM_PROT_CURRENT_TIME    0x41U
#define PARAM_PROT_TEMP_LIMIT      0x42U
#define PARAM_PROT_TEMP_TIME       0x43U
#define PARAM_PROT_STALL_THRESHOLD 0x44U
#define PARAM_PROT_STALL_TRIP      0x45U
#define PARAM_PROT_ENABLE          0x46U
#define PARAM_VOLTAGE_MIN          0x47U
#define PARAM_VOLTAGE_MAX          0x48U
#define PARAM_MIN_START_TORQUE     0x49U
#define PARAM_LOAD_RATIO           0x4AU
#define PARAM_LOAD_LEARN           0x4BU
/* ---- 80~91 SRAM 参数与动作 ---- */
#define PARAM_MOTION_REQUEST       0x80U  /* 只写：位置码2B + 时间2B */
#define PARAM_MOTION_ACTION        0x81U  /* 只写：1=继续 2=暂停 3=停止 */
#define PARAM_TORQUE_MODE          0x82U
#define PARAM_DAMPING              0x83U
#define PARAM_RT_TORQUE_LIMIT      0x84U
#define PARAM_RT_TRAJ_VMAX         0x85U
#define PARAM_RT_TRAJ_AMAX         0x86U
#define PARAM_RT_TRAJ_DMAX         0x87U
#define PARAM_RT_TRAJ_SMOOTH       0x88U
#define PARAM_RT_MOVE_MIN          0x89U
#define PARAM_RT_HOLD_KP           0x8AU
#define PARAM_CAL_ACTION           0x8BU  /* 只写：1=SCK 2=SCZ 3=AMI 4=AMX 5=SMI 6=SMX 7=CSD */
#define PARAM_SYSTEM_ACTION        0x8CU  /* 只写：1=SRAM参数重载 2=保留ID恢复默认 3=全部恢复默认 */
#define PARAM_RT_RESOLUTION        0x8EU
#define PARAM_RT_HOLD_KI           0x8FU
#define PARAM_RT_LOAD_RATIO        0x90U
#define PARAM_RT_LOAD_LEARN        0x91U
/* ---- C0~ 只读实时状态 ---- */
#define PARAM_WORK_STATE           0xC0U
#define PARAM_FAULT                0xC1U
#define PARAM_POSITION             0xC2U  /* 位置码 0~4095 */
#define PARAM_SPEED                0xC3U  /* int16 0.1°/s，位置值增大方向为正；观测器不在更新报0 */
#define PARAM_TEMPERATURE          0xC4U  /* int16 0.1℃，最近一次读成功的值 */
#define PARAM_VOLTAGE              0xC5U
#define PARAM_CURRENT              0xC6U  /* int16 mA，位置值增大方向为正 */
#define PARAM_TORQUE               0xC7U  /* int16 %，方向同电流；测不到报0 */
#define PARAM_STALLED              0xC8U
#define PARAM_TELEMETRY            0xC9U  /* 七项实时状态集合 */
#define PARAM_TELEMETRY_BYTES       14U

#define PARAM_VALUE_MAX             16U   /* 单个参数值最长字节数(集合状态) */

/* 读写结果码，即二进制应答状态码 */
#define PARAM_ST_OK                 0U
#define PARAM_ST_CMD                1U    /* 命令不存在(协议层用) */
#define PARAM_ST_LENGTH             2U
#define PARAM_ST_VALUE              3U
#define PARAM_ST_STATE              4U
#define PARAM_ST_FAILED             5U
#define PARAM_ST_READONLY           6U
#define PARAM_ST_UNSUP              7U

/* 写后需要协议层收尾的事：切波特率(等应答发完) */
#define PARAM_FX_BAUD             0x01U

/* 位置码：数据帧 0~4095(12位4096码，中位2047.5无整数码)，端点精确：0=500us、4095=2500us，码级往返恒等 */
#define PARAM_POS_MAX             4095U
#define PARAM_POS_TO_Q3(u)  ((uint16_t)(Q3_MIN + (((uint32_t)(u) * Q3_SPAN + PARAM_POS_MAX / 2U) / PARAM_POS_MAX)))
#define Q3_TO_PARAM_POS(q)  ((uint16_t)((((uint32_t)(q) - Q3_MIN) * PARAM_POS_MAX + Q3_SPAN / 2U) / Q3_SPAN))

uint8_t A_Param_Read(uint8_t addr, uint8_t *out, uint8_t *len);          /* 值按小端写 out，*len=字节数；返回 PARAM_ST_* */
uint8_t A_Param_Write(uint8_t addr, const uint8_t *data, uint8_t len, uint8_t *fx); /* 小端值；*fx 置 PARAM_FX_* */
uint8_t A_Param_WriteValue(uint8_t addr, uint32_t value, uint8_t *fx);   /* 同上，值已解码(文本协议用) */
void    A_Param_Repair(Config_t *c, const Config_t *def);               /* 非法字段改回 def 中的值(上电加载用) */
uint8_t A_Param_VoltageRangeValid(uint16_t lo_dv, uint16_t hi_dv);       /* 工作电压上下限组合是否合法 */
