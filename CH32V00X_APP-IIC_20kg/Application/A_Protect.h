#ifndef __A_PROTECT_H__
#define __A_PROTECT_H__

#include <stdint.h>
#include "A_Config.h"   /* 堵转阈值上限按额定速度推出 */

/* A_Protect.h —— 保护与功率限制，一个10ms任务内分频。 */

/* 扭矩统一用0.01 kgf·cm；换算参数实测后在此调整。 */
#define TORQUE_IDLE_CURRENT_MA   100U  /* 空载损耗电流(mA) */
#define TORQUE_STALL_CURRENT_MA 2000U  /* 堵转参考电流(mA) */
#define TORQUE_INVALID        0xFFFFU /* 无有效采样；不允许作为设置值 */
#define TORQUE_SCALE            100U  /* 1 kgf·cm=100个协议单位 */

_Static_assert(TORQUE_STALL_CURRENT_MA > TORQUE_IDLE_CURRENT_MA
               && TORQUE_STALL_CURRENT_MA <= UINT16_MAX,
               "堵转电流必须大于空载电流，且不超过电流字段量程");

/* 电流估算扭矩，四舍五入；超参考电流继续线性，仅在字段量程处饱和。 */
static inline uint16_t A_Torque_Estimate(uint16_t current_ma, uint8_t valid,
                                      uint16_t rated_ckgcm)
{
    const uint32_t span = TORQUE_STALL_CURRENT_MA - TORQUE_IDLE_CURRENT_MA;
    uint32_t torque;
    if (!valid || rated_ckgcm == 0U) return TORQUE_INVALID;
    if (current_ma <= TORQUE_IDLE_CURRENT_MA) return 0U;
    torque = ((uint32_t)rated_ckgcm * (current_ma - TORQUE_IDLE_CURRENT_MA)
             + span / 2U) / span;
    return (uint16_t)((torque >= TORQUE_INVALID) ? TORQUE_INVALID - 1U : torque);
}

/* 目标扭矩反算限流值；零扭矩禁止驱动，不能反算为空载电流后继续供电。 */
static inline uint16_t A_Torque_CurrentLimit(uint16_t torque_ckgcm,
                                          uint16_t rated_ckgcm)
{
    const uint32_t span = TORQUE_STALL_CURRENT_MA - TORQUE_IDLE_CURRENT_MA;
    if (torque_ckgcm == 0U || rated_ckgcm == 0U) return 0U;
    if (torque_ckgcm > rated_ckgcm) torque_ckgcm = rated_ckgcm;
    return (uint16_t)(TORQUE_IDLE_CURRENT_MA
           + ((uint32_t)torque_ckgcm * span + rated_ckgcm / 2U) / rated_ckgcm);
}

#define PROT_TEMP_SAMPLE_MS       250U  /** 温度检测周期(ms) */
#define PROT_TEMP_LIMIT_DEFAULT_DC 500  /** 出厂过温阈值，0.1℃ */
#define PROT_TEMP_LIMIT_MIN_DC     300  /** 用户可设下限，30.0℃ */
#define PROT_TEMP_LIMIT_MAX_DC    1000  /** 用户可设上限，100.0℃ */
#define PROT_TEMP_HYST_DC           50  /** 回差，5.0℃ */
#define PROT_TEMP_TIME_DEFAULT_MS  250U /** 出厂过温持续时间(ms) */
#define PROT_TEMP_TIME_MIN_MS      250U /** 用户可设最短时间(ms) */
#define PROT_TEMP_TIME_MAX_MS    10000U /** 用户可设最长时间(ms) */

/* 3A软件跳闸先于硬件保护(可上报上限5.29A，AT8236斩波5.5A) */
#define PROT_CURRENT_LIMIT_DEFAULT_MA 3000U /** 出厂过流阈值(mA)，独立于扭矩限流 */
#define PROT_CURRENT_LIMIT_MIN_MA      500U /** 用户可设下限(mA) */
#define PROT_CURRENT_LIMIT_MAX_MA     5000U /** 用户可设上限(mA)，低于硬件保护 */
#define PROT_CURRENT_TIME_DEFAULT_MS    30U /** 出厂过流持续时间(ms) */
#define PROT_CURRENT_TIME_MIN_MS        10U /** 用户可设最短时间(ms) */
#define PROT_CURRENT_TIME_MAX_MS      1000U /** 用户可设最长时间(ms) */
#define PROT_CURRENT_COOLDOWN_MS        500 /** 过流后强制卸力冷却(ms) */

/* 保护功能开关位，出厂全开 */
#define PROT_ENABLE_TEMP     0x01U
#define PROT_ENABLE_CURRENT  0x02U
#define PROT_ENABLE_STALL    0x04U
#define PROT_ENABLE_VOLTAGE  0x08U
#define PROT_ENABLE_IMPLEMENTED 0x0FU   /** 已实现功能位，也是对外上限 */
#define PROT_ENABLE_DEFAULT     0x0FU

/* 工作电压单位0.1V。用户可设6.0~9.0V；静止时超出设置值上下各0.5V才动作，
 * 出厂动作线5.5V/9.5V。运动中的起步压降、回馈泵升不判，由输出级限速和钳位处理。 */
#define PROT_VOLTAGE_MIN_DEFAULT_DV 60U
#define PROT_VOLTAGE_MAX_DEFAULT_DV 90U
#define PROT_VOLTAGE_CFG_MIN_DV     60U
#define PROT_VOLTAGE_CFG_MAX_DV     90U
#define PROT_VOLTAGE_TOL_LO_DV       5U   /** 欠压动作线 = 设置下限 - 本值 */
#define PROT_VOLTAGE_TOL_HI_DV       5U   /** 过压动作线 = 设置上限 + 本值 */
/* 采样可信范围(驱动电压换算用)，超出视为采样无效；下限比动作线宽：运动中压降可低于动作线 */
#define PROT_VOLTAGE_ABS_MIN_DV     (PROT_VOLTAGE_CFG_MIN_DV - 10U)
#define PROT_VOLTAGE_ABS_MAX_DV     (PROT_VOLTAGE_CFG_MAX_DV + PROT_VOLTAGE_TOL_HI_DV)
#define PROT_VOLTAGE_HYST_DV         2U
#define PROT_VOLTAGE_TRIP_MS       100U   /** 静止时超限(或读数为0)持续此时长报电压故障 */
#define PROT_VOLTAGE_RECOVER_MS    500U

/* 上下留回差后仍须有恢复区间。协议与开机修复共用。 */
static inline uint8_t A_Protect_VoltageRangeValid(uint16_t lo, uint16_t hi)
{
    return (uint8_t)(lo >= PROT_VOLTAGE_CFG_MIN_DV && hi <= PROT_VOLTAGE_CFG_MAX_DV
                 && lo + 2U * PROT_VOLTAGE_HYST_DV < hi);
}

/* 堵转：每周期比较参考与实际位移，连续达配置次数才判定。参考阈值=实际×PROT_STALL_REF_RATIO。
 * 触发后就地保持，不卸力、不记故障位(被按住/撞限位可恢复)。 */
#define PROT_STALL_PERIOD_MS          20U /** 堵转检测周期(ms)，须为10ms基拍整数倍 */
#define PROT_STALL_REF_RATIO           4U /** 参考位移阈值 = 实际位移阈值 × 它 */
#define PROT_STALL_ACT_DEFAULT_CDEG   32U /** 出厂实际位移阈值，厘度/周期 */
#define PROT_STALL_THRESHOLD_MIN_CDEG 10U /** 对外阈值(实际位移)下限 */
/** 对外阈值上限：参考阈值不能超过额定速度一个周期的位移，否则"参考在走"永不成立 */
#define PROT_STALL_THRESHOLD_MAX_CDEG ((uint16_t)((uint32_t)MOTOR_RATED_VEL_DPS * 100U * PROT_STALL_PERIOD_MS / 1000U \
                                                  / PROT_STALL_REF_RATIO))
#define PROT_STALL_TRIP_DEFAULT        5U /** 出厂连续判定次数，5×20=100ms */
#define PROT_STALL_TRIP_MIN            1U /** 用户可设连续次数下限 */
#define PROT_STALL_TRIP_MAX          100U /** 用户可设连续次数上限 */
#define PROT_STALL_HOLD_TICKS 15 /** 周期内轨迹被冻结这么多拍，同样算"轨迹在变" */
#define PROT_STALL_HOLD_LIMIT_MA   100U /** 堵转保持限流(mA)，新指令恢复 */

#define PROT_FAULT_NONE        0x0000U /** 无故障 */
#define PROT_FAULT_OVERTEMP    0x0001U /** 过温 */
#define PROT_FAULT_OVERCUR     0x0002U /** 过流 */
#define PROT_FAULT_STALL       0x0004U /** 堵转 */
#define PROT_FAULT_UNDERVOLT   0x0008U /** 欠压 */
#define PROT_FAULT_OVERVOLT    0x0010U /** 过压 */
#define PROT_FAULT_VOLT_INVALID 0x0020U /** 电压采样尚无有效值 */

void    A_Protect_Init(void);                 /* 须在电机输出入口初始化后调用 */
void    A_Protect_Task(void);                 /* 10ms任务入口 */
uint8_t A_Protect_SetTorque(uint16_t torque_ckgcm); /* 设扭矩上限并落盘，0.01 kgf·cm */
uint16_t A_Protect_GetTorque(void);          /* 读已生效扭矩上限，0.01 kgf·cm */
uint16_t A_Protect_Fault(void);               /* 读故障位掩码，供诊断 */
uint8_t A_Protect_Stalled(void);              /* 1=最近动作以堵转收场，供诊断 */
void    A_Protect_EnterStallHold(void);       /* 堵转保持：限流到 PROT_STALL_HOLD_LIMIT_MA */
void    A_Protect_ExitStallHold(void);        /* 新指令等：恢复扭矩限流 */
/* 周期采样缓存供集合查询复用，查询本身不访问ADC或温度传感器。 */
typedef struct {
    uint16_t current_ma;    /* 最近电流，mA */
    uint16_t voltage_cv;    /* 最近电压，0.01V；0=尚无有效值 */
    int16_t temperature_dc; /* 最近温度，0.1℃；-32768=未采样或失败 */
    uint8_t current_valid; /* 1=电流采样落在有效驱动段 */
} ProtectTelemetry_t;
void A_Protect_GetTelemetry(ProtectTelemetry_t *out); /* 仅读SRAM快照 */
uint16_t A_Protect_GetCurrent(void);          /* 读最近绕组电流(mA)，供RIV查询 */
uint8_t A_Protect_CanDrive(void);             /* 1=无停机故障且满足最小启动扭矩 */

#endif /* __A_PROTECT_H__ */
