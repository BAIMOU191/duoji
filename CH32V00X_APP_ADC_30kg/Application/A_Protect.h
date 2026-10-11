#ifndef __A_PROTECT_H__
#define __A_PROTECT_H__

#include <stdint.h>

/* A_Protect.h —— 保护与功率限制，一个10ms任务内分频；详见 docs/ALGORITHM_NOTES.md。 */

/* 设置、上报与启动门槛统一使用0.01 kgf·cm；换算参数实测后在此调整。 */
#define TORQUE_IDLE_CURRENT_MA   100U  /* 空载损耗电流，暂取0.1A */
#define TORQUE_STALL_CURRENT_MA 2000U  /* 堵转参考电流，暂取2A */
#define TORQUE_INVALID        0xFFFFU /* 无有效采样；不允许作为设置值 */
#define TORQUE_SCALE            100U  /* 1 kgf·cm对应100个协议单位 */

_Static_assert(TORQUE_STALL_CURRENT_MA > TORQUE_IDLE_CURRENT_MA
               && TORQUE_STALL_CURRENT_MA <= UINT16_MAX,
               "堵转电流必须大于空载电流，且不超过电流字段量程");

/* 电流转估算扭矩，四舍五入；超参考电流继续线性计算，仅在字段量程处饱和。 */
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
#define PROT_TEMP_LIMIT_DEFAULT_DC 500  /** 出厂过温阈值，0.1摄氏度 */
#define PROT_TEMP_LIMIT_MIN_DC     300  /** 用户可设下限，30.0摄氏度 */
#define PROT_TEMP_LIMIT_MAX_DC    1000  /** 用户可设上限，100.0摄氏度 */
#define PROT_TEMP_HYST_DC           50  /** 回差，5.0摄氏度 */
#define PROT_TEMP_TIME_DEFAULT_MS  250U /** 出厂过温持续时间(ms) */
#define PROT_TEMP_TIME_MIN_MS      250U /** 用户可设最短时间(ms) */
#define PROT_TEMP_TIME_MAX_MS    10000U /** 用户可设最长时间(ms) */

/* 3A软件跳闸先于硬件保护(可上报上限5.29A，AT8236斩波5.5A) */
#define PROT_CURRENT_LIMIT_DEFAULT_MA 3000U /** 出厂过流保护阈值(mA)，独立于扭矩限流换算 */
#define PROT_CURRENT_LIMIT_MIN_MA      500U /** 用户可设下限(mA) */
#define PROT_CURRENT_LIMIT_MAX_MA     5000U /** 用户可设上限(mA)，低于硬件保护 */
#define PROT_CURRENT_TIME_DEFAULT_MS    30U /** 出厂过流持续时间(ms) */
#define PROT_CURRENT_TIME_MIN_MS        10U /** 用户可设最短时间(ms) */
#define PROT_CURRENT_TIME_MAX_MS      1000U /** 用户可设最长时间(ms) */
#define PROT_CURRENT_COOLDOWN_MS 500 /** 过流后强制卸力的冷却时长(ms) */

/* 保护功能开关：每一位独立控制，出厂全部开启。 */
#define PROT_ENABLE_TEMP     0x01U
#define PROT_ENABLE_CURRENT  0x02U
#define PROT_ENABLE_STALL    0x04U
#define PROT_ENABLE_VOLTAGE  0x08U
#define PROT_ENABLE_IMPLEMENTED 0x0FU
#define PROT_ENABLE_DEFAULT     0x0FU

/* 工作电压单位为0.1V；用户设置范围为5.0~9.0V，采样边界额外保留0.2V内部容差。 */
#define PROT_VOLTAGE_MIN_DEFAULT_DV 50U
#define PROT_VOLTAGE_MAX_DEFAULT_DV 90U
#define PROT_VOLTAGE_CFG_MIN_DV     50U
#define PROT_VOLTAGE_CFG_MAX_DV     90U
#define PROT_VOLTAGE_ABS_MIN_DV     48U
#define PROT_VOLTAGE_ABS_MAX_DV     92U
#define PROT_VOLTAGE_HYST_DV         2U
#define PROT_VOLTAGE_TRIP_MS        50U
#define PROT_VOLTAGE_RECOVER_MS    500U

/* 堵转：每 PROT_STALL_PERIOD_MS 采一次参考与实际位移，连续达到配置次数才判定。
 * REF 取 ACT 的4倍避免含糊地带。触发后就地保持，不卸力、不记故障位(被按住或撞限位是可恢复事件)。 */
#define PROT_STALL_PERIOD_MS          20U /** 堵转检测周期(ms)，须是10ms基拍的整数倍 */
#define PROT_STALL_REF_DEFAULT_CDEG  128U /** 出厂参考位移阈值，厘度/周期 */
#define PROT_STALL_REF_MIN_CDEG       40U /** 用户可设参考位移阈值下限 */
#define PROT_STALL_REF_MAX_CDEG     1000U /** 用户可设参考位移阈值上限 */
#define PROT_STALL_ACT_DEFAULT_CDEG   32U /** 出厂实际位移阈值，厘度/周期 */
#define PROT_STALL_ACT_MIN_CDEG        1U /** 用户可设实际位移阈值下限 */
#define PROT_STALL_ACT_MAX_CDEG      500U /** 用户可设实际位移阈值上限 */
#define PROT_STALL_THRESHOLD_DEFAULT_CDEG PROT_STALL_ACT_DEFAULT_CDEG
#define PROT_STALL_THRESHOLD_MIN_CDEG     10U /** 对外单一阈值；内部参考阈值固定为它的4倍 */
#define PROT_STALL_THRESHOLD_MAX_CDEG    250U
#define PROT_STALL_TRIP_DEFAULT        5U /** 出厂连续判定次数，5x20=100ms */
#define PROT_STALL_TRIP_MIN            1U /** 用户可设连续次数下限 */
#define PROT_STALL_TRIP_MAX          100U /** 用户可设连续次数上限 */
#define PROT_STALL_HOLD_TICKS 15 /** 周期内轨迹被冻结这么多拍，同样算"轨迹在变" */

#define PROT_FAULT_NONE        0x0000U /** 无故障 */
#define PROT_FAULT_OVERTEMP    0x0001U /** 过温 */
#define PROT_FAULT_OVERCUR     0x0002U /** 过流 */
#define PROT_FAULT_STALL       0x0004U /** 堵转 */
#define PROT_FAULT_UNDERVOLT   0x0008U /** 欠压 */
#define PROT_FAULT_OVERVOLT    0x0010U /** 过压 */
#define PROT_FAULT_VOLT_INVALID 0x0020U /** 电压采样尚无有效值 */

void    A_Protect_Init(void);                 /* 须在A_Servo_Init之后调用 */
void    A_Protect_Task(void);                 /* 10ms任务入口 */
uint8_t A_Protect_SetTorque(uint16_t torque_ckgcm); /* 设扭矩上限并落盘，单位0.01 kgf·cm */
uint16_t A_Protect_GetTorque(void);          /* 读已生效扭矩上限，0.01 kgf·cm */
uint16_t A_Protect_Fault(void);               /* 读故障位掩码，供诊断 */
uint8_t A_Protect_Stalled(void);              /* 1=最近一次动作以堵转收场，供诊断 */
/* 周期采样缓存供集合查询复用，查询本身不访问ADC或温度传感器。 */
typedef struct {
    uint16_t current_ma;    /* 最近一次电流值，mA */
    uint16_t voltage_cv;    /* 最近一次电压值，0.01V；0=尚无有效值 */
    int16_t temperature_dc; /* 最近一次温度值，0.1摄氏度；-32768=未采样或失败 */
    uint8_t current_valid; /* 1=电流采样落在有效驱动段 */
} ProtectTelemetry_t;
void A_Protect_GetTelemetry(ProtectTelemetry_t *out); /* 仅读SRAM快照 */
uint16_t A_Protect_GetCurrent(void);          /* 读最近一块绕组电流(mA)，供RIV查询 */
uint8_t A_Protect_CanDrive(void);             /* 1=无停机故障且满足最小启动扭矩 */

#endif /* __A_PROTECT_H__ */
