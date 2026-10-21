#pragma once

#include <stdint.h>
#include "C_Traj_Planner.h"   /* 平滑时间、最短运动时间的上限以规划器为准 */

/* 固件版本 主.次.修订，各0~255；0.0.0表示未发布版本。 */
#define SERVO_VERSION_MAJOR 0U
#define SERVO_VERSION_MINOR 0U
#define SERVO_VERSION_PATCH 0U

/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时上位机和 Boot 逐项与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC版 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     20U       /** 扭力型号 20kg */

/* ==================== 2. 电机标定（标定固件打印，整节替换） ====================
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。
 * 本节由 A_Calib_Min 标定固件实测打印，换电机或减速箱后重新标定并整节替换。 */
#define MOTOR_CAL_MV              7683   /* 标定电压，mV */
#define MOTOR_KV_UV                349   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV              10285   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                 37   /* 动摩擦，mV */
#define MOTOR_FS_MV                486   /* 起转电压，mV */
#define MOTOR_DELAY_US            1655   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             3   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          9   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        191   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      6000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      5000   /* 额定低速减速度，度/秒² */
/* ==================== 电机标定结束 ==================== */

/* ==================== 3. 控制器整定(固件常量，不对外) ====================
 * 带宽越高越硬、越跟手，但电位器噪声放大越多；约束见 C_Pos_Ctrl.h，生效时仍按约束钳位。 */
#define TUNE_OBS_BW_HZ          21   /** 观测器三重极点带宽(Hz)，速度环带宽不超过它 */
/* 速度环带宽受纯延迟限制 ω_v·延迟 ≤ 1/3(与 C_PosCtrl_Init 同一公式)，取值不超过它 */
#define CTRL_VEL_BW_DELAY_MAX  ((MOTOR_DELAY_US > 333) ? 1000000U / (3U * MOTOR_DELAY_US) : 1000U)
#define TUNE_VEL_BW_RADS       ((CTRL_VEL_BW_DELAY_MAX < 120U) ? CTRL_VEL_BW_DELAY_MAX : 120U)  /** 速度环带宽 ω_v，rad/s */
#define TUNE_POS_BW_RADS        20U  /** 位置环带宽 ω_p，rad/s，≤ ω_v/4 */
#define TUNE_INT_BW_RADS         5U  /** 速度积分零点 ω_i，rad/s，≤ ω_p */
/* 实测版新增唯一负载整定量：J_load/J_motor×10，10=负载惯量等于电机自身，总惯量2倍。
 * 仅供观测器/闭环带宽适配，不修改轨迹限值或加速度前馈；改后重新编译，不占Flash/SRAM协议地址。 */
#define TUNE_LOAD_RATIO_X10     10U

/* ==================== 4. 运行参数出厂值 ====================
 * 精度、齿隙、刚度、V/A/D、平滑时间、最短运动时间是运行参数(Flash/SRAM 两份，可由协议修改)，这里只是出厂值。 */
#define CFG_DEF_RESOLUTION_CDEG 50U  /** 到位精度：误差在它以内且静止才算到位，厘度 */
#define CFG_DEF_HOLD_KP         80  /** 保持 P，‰/°：超出到位窗 1° 输出标定电压的多少‰(相对扭矩)；*/
#define CFG_DEF_HOLD_KI           0  /** 保持 I，‰/(°·s)：超出 1° 持续 1s，输出再加标定电压的多少‰；0=积分不动，只用到位时继承的托举力 */
/* 恢复原运动默认值：V/A/D取额定能力的90%/70%/70%，具体值由本型号标定参数计算。
 * 控制器优化独立保留；实际限值仍由现有Flash/SRAM参数决定，0表示按额定。 */
#define CFG_DEF_VMAX_DPS       (MOTOR_RATED_VEL_DPS * 9U / 10U)
#define CFG_DEF_AMAX_DPS2      (MOTOR_RATED_ACC_DPS2 * 7U / 10U)
#define CFG_DEF_DMAX_DPS2      (MOTOR_RATED_DEC_DPS2 * 7U / 10U)
#define CFG_DEF_SMOOTH_MS        50  /** S曲线平滑时间(加速度从0升到A的时间)，ms；0=自动取机械时间常数τ */
#define CFG_DEF_MOVE_MIN_MS    150U  /** 最短运动时间，ms；0=关闭：每段运动至少走这么久(带时间的取较大者)，小动作按比例降低 V/A/J */

/* ==================== 5. 硬件量程与旋向 ==================== */
#define CDEG_RANGE          36000 /** 整圈角度，厘度 */
#define CFG_PWM_FULL         2400 /** PWM满量程，必须与驱动层实际限制一致 */
#define CFG_TICK_HZ          1000 /** 控制任务频率(Hz) */
#define CFG_DIR_INVERT          1 /** 1=反转整机标称旋向 */
#define SERVO_PWM_MIN         500 /** 协议最小脉宽，也是电机模式负向满速 */
#define SERVO_PWM_MID        1500 /** 电机模式零速指令 */
#define SERVO_PWM_MAX        2500 /** 协议最大脉宽，也是电机模式正向满速 */

/* ==================== 6. 运行参数范围 ====================
 * 范围即生效范围：写入越界的值被拒绝，读回的就是实际生效值(V/A/D 的 0 表示按额定，平滑时间的 0 表示自动取 τ)。
 * 有物理或实现约束的上下限都由约束本身推出，不另写数字。 */
#define CFG_TRAJ_VMAX_MAX_DPS      MOTOR_RATED_VEL_DPS   /** 速度上限=额定能力 */
#define CFG_TRAJ_AMAX_MAX_DPS2     MOTOR_RATED_ACC_DPS2
#define CFG_TRAJ_DMAX_MAX_DPS2     MOTOR_RATED_DEC_DPS2
#define CFG_SMOOTH_MAX_MS          TRAJ_SMOOTH_MS_MAX    /** 平滑时间上限，ms：重规划时加速度接管段的最长拍数 */
/** 平滑时间下限，ms(0 另外允许=自动)：再短则 J = max(A,D)/平滑时间 在额定加减速度下超过规划器的 J 上限，设了也不生效 */
#define CFG_SMOOTH_MIN_MS          ((uint16_t)(((uint64_t)((MOTOR_RATED_ACC_DPS2 > MOTOR_RATED_DEC_DPS2) ? MOTOR_RATED_ACC_DPS2 \
                                    : MOTOR_RATED_DEC_DPS2) * 100000U + TRAJ_JMAX_MAX_CDPSS3 - 1) / TRAJ_JMAX_MAX_CDPSS3))
#define CFG_MOVE_MIN_MAX_MS        TRAJ_MOVE_MIN_MS_MAX  /** 最短运动时间上限，ms */
#define CFG_HOLD_KP_MAX            100U  /** 保持 P 上限，‰/°：超出 1° 给 10% 标定电压，30kg 约等于旧单位的 23 */
#define CFG_HOLD_KI_MAX            100U  /** 保持 I 上限，‰/(°·s) */
#define CFG_RESOLUTION_MIN_CDEG    (MOTOR_MIN_STEP_CDEG * 3U / 2U)  /** 到位精度下限=1.5×最小可靠位移(与 C_PosCtrl 捕获窗同一公式) */
#define CFG_RESOLUTION_MAX_CDEG    500U

/* 速度噪声底：观测器把读数噪声微分成速度噪声，所有静止判据、速度死区以它为刻度。
 * 白噪声下速度噪声峰值 ≈ 2 × 观测器带宽 × 读数噪声(3σ)；实机读数有尖峰、且噪声随电位器位置变化(静止时观测速度
 * 偶到300~400)，取3倍。另不低于按最小可靠位移估的量。只按最小步长算时，步长标得小(13)会把它压到100左右，
 * 低于实际噪声，静止判据长期不成立：进不了保持，积分追着噪声来回挪 */
#define CTRL_VEL_NOISE_STEP (TUNE_OBS_BW_HZ * MOTOR_MIN_STEP_CDEG * 3 / 8)
#define CTRL_VEL_NOISE_READ (TUNE_OBS_BW_HZ * MOTOR_NOISE_CDEG * 3)
#define CTRL_VEL_NOISE_CDPS ((CTRL_VEL_NOISE_STEP > CTRL_VEL_NOISE_READ) ? CTRL_VEL_NOISE_STEP : CTRL_VEL_NOISE_READ)

_Static_assert(MOTOR_KV_UV > 0 && MOTOR_KA_NV > 0 && MOTOR_FS_MV >= MOTOR_FC_MV && MOTOR_DELAY_US < 7000
               && MOTOR_CAL_MV > 0 && MOTOR_MIN_STEP_CDEG >= 1, "Invalid motor calibration");
_Static_assert(MOTOR_SIGN == 1 || MOTOR_SIGN == -1, "Invalid motor direction");
_Static_assert(TUNE_POS_BW_RADS * 4U <= TUNE_VEL_BW_RADS && TUNE_INT_BW_RADS <= TUNE_POS_BW_RADS && TUNE_INT_BW_RADS >= 1U
               && TUNE_VEL_BW_RADS * 1000U <= TUNE_OBS_BW_HZ * 6283U, "Controller bandwidths violate constraints");
_Static_assert(CFG_DEF_RESOLUTION_CDEG >= CFG_RESOLUTION_MIN_CDEG && CFG_DEF_RESOLUTION_CDEG <= CFG_RESOLUTION_MAX_CDEG
               && CFG_RESOLUTION_MIN_CDEG >= 1U
               && CFG_DEF_HOLD_KP <= CFG_HOLD_KP_MAX
               && CFG_DEF_HOLD_KI <= CFG_HOLD_KI_MAX
               && (CFG_DEF_SMOOTH_MS == 0U || (CFG_DEF_SMOOTH_MS >= CFG_SMOOTH_MIN_MS && CFG_DEF_SMOOTH_MS <= CFG_SMOOTH_MAX_MS))
               && CFG_DEF_MOVE_MIN_MS <= CFG_MOVE_MIN_MAX_MS,
               "Invalid default motion/hold parameters");
/* 平滑时间：下限要在上限之内；自动值 τ = ka/kv 也不能超过接管段上限 */
_Static_assert(CFG_SMOOTH_MIN_MS >= 1U && CFG_SMOOTH_MIN_MS <= CFG_SMOOTH_MAX_MS
               && (MOTOR_KA_NV + MOTOR_KV_UV - 1) / MOTOR_KV_UV <= CFG_SMOOTH_MAX_MS, "Invalid smoothing time range");

/*
 * ==================== 8. 掉电保存参数与访问接口 ====================
 * 模式编号是对外协议，不重新编号：11(自定义)只能由 AMI/AMX 进入；5/6 是整圈编码器的360度模式，
 * 电位器有死区不支持，编号空着，Flash 里遗留的5/6上电修回270度。
 */

typedef enum {
    SERVO_MODE_270_CW = 1, /* 270度位置，P增大时CW */
    SERVO_MODE_270_CCW,    /* 270度位置，P增大时CCW */
    SERVO_MODE_180_CW,     /* 180度位置，P增大时CW */
    SERVO_MODE_180_CCW,    /* 180度位置，P增大时CCW */
    SERVO_MODE_TURNS_CW = 7, /* CW定圈，T=0无限 */
    SERVO_MODE_TURNS_CCW,  /* CCW定圈，T=0无限 */
    SERVO_MODE_TIMED_CW,   /* CW定时，T=0无限 */
    SERVO_MODE_TIMED_CCW,  /* CCW定时，T=0无限 */
    SERVO_MODE_CUSTOM      /* 自定义行程位置模式，只能由AMI/AMX进入 */
} ServoMode_t;

#define SERVO_MODE_IS_SUPPORTED(m) \
    (((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_180_CCW) || ((m) >= SERVO_MODE_TURNS_CW && (m) <= SERVO_MODE_CUSTOM))

#define SERVO_CUSTOM_SPAN_MIN 100U /** 自定义行程下限(厘度)，再小每个脉宽档都低于机构分辨率 */

/* 校准记账：Flash 存校准时的编码器原始角度(锚点)，有效行程与零点每次由"锚点+模式标称行程"现推，
 * 所以反复校准、切换180/270度都不会叠加。 */
typedef enum {
    SERVO_CAL_NONE = 0, /* 未校准，零点取 Flash 里的历史偏移值 */
    SERVO_CAL_MID,      /* SCK：锚点是行程中点(1500us 对应的角度) */
    SERVO_CAL_ZERO      /* SCZ：锚点是行程起点(500us 对应的角度) */
} ServoCalKind_t;

typedef enum {
    SERVO_BOOT_GOTO_START = 1, /* 上电转到startup_pwm */
    SERVO_BOOT_HOLD,           /* 上电保持实际位置 */
    SERVO_BOOT_RELEASE         /* 上电小阻力释放，首条控制指令恢复 */
} ServoBootMode_t;

/* Flash掉电保存参数：2.x起扭矩字段改为16位物理量；升级沿用重置参数并保留ID/波特率的规则。 */
typedef struct {
    uint8_t  servo_id;             /** 总线ID，0~254 */
    uint8_t  baud_code;            /** 1~8，默认5=115200 */
    uint8_t  servo_mode;           /** 工作模式，见ServoMode_t，1~11 */
    uint8_t  boot_mode;            /** 上电动作，见ServoBootMode_t，1~3 */
    uint16_t torque_limit_ckgcm;   /** 扭矩上限，0.01 kgf·cm，最大为本机硬件扭矩 */
    uint8_t  custom_reverse;       /** 自定义模式方向(未经 CFG_DIR_INVERT 翻转的原始值) */
    uint8_t  cal_kind;             /** ServoCalKind_t，0=未校准 */
    uint16_t startup_pwm;          /** 上电目标脉宽，500~2500 */
    uint16_t position_offset_cdeg; /** 标准模式零点偏移(厘度)；已校准时由锚点现推 */
    uint16_t custom_offset_cdeg;   /** 自定义行程零点对应的编码器角度，厘度 */
    uint16_t custom_span_cdeg;     /** 自定义行程标称长度，厘度，只由 AMI/AMX 写 */
    uint16_t pulse_lo;             /** SMI 标定的脉冲下界，500~2500 */
    uint16_t pulse_hi;             /** SMX 标定的脉冲上界，500~2500，须 > pulse_lo */
    uint16_t cal_anchor_cdeg;      /** 校准锚点的编码器原始角度，厘度 */
    /* ---- 新增字段从这里往下追加 ---- */
    uint32_t traj_amax_dps2;       /** 轨迹最大加速度，度/秒^2，≤额定；0=按额定 */
    uint32_t traj_dmax_dps2;       /** 轨迹最大减速度，度/秒^2，≤额定；0=按额定 */
    uint16_t traj_vmax_dps;        /** 轨迹最大速度，度/秒，≤额定；0=按额定 */
    uint16_t traj_smooth_ms;       /** S曲线平滑时间(加速度0→A)，ms；0=自动(τ)，或 CFG_SMOOTH_MIN_MS~200 */
    uint16_t traj_move_min_ms;     /** 最短运动时间，ms；0=关闭。每段运动至少走这么久，小动作按比例降低 V/A/J */
    uint16_t prot_current_limit_ma;/** 过流保护阈值，mA */
    uint16_t prot_current_time_ms; /** 过流超过阈值后持续多久触发，ms */
    uint16_t prot_temp_limit_dC;   /** 过温保护阈值，0.1摄氏度 */
    uint16_t prot_temp_time_ms;    /** 过温超过阈值后持续多久触发，ms */
    uint16_t prot_stall_act_cdeg;  /** 堵转判定：实际位移上限，厘度/20ms；参考位移阈值固定为它的 PROT_STALL_REF_RATIO 倍 */
    uint16_t prot_stall_trip;      /** 连续多少次成立后判堵转 */
    uint16_t position_resolution_cdeg; /** 到位精度，厘度，CFG_RESOLUTION_MIN_CDEG~500 */
    uint16_t hold_kp;              /** 保持 P，‰/°，0~CFG_HOLD_KP_MAX */
    uint16_t reserved0;            /** 保留(原齿隙，布局不变) */
    uint16_t hold_ki;              /** 保持 I，‰/(°·s)，0~CFG_HOLD_KI_MAX；0=积分不动 */
    uint8_t  protect_enable;       /** 保护功能位：bit0温度 bit1电流 bit2堵转 bit3电压，0~15 */
    uint8_t  voltage_min_dv;       /** 最低工作电压，0.1V */
    uint8_t  voltage_max_dv;       /** 最高工作电压，0.1V */
    uint8_t  reserved1;            /** 对齐保留 */
    uint16_t min_start_torque_ckgcm; /** 最小启动扭矩，0.01 kgf·cm；0=关闭门槛 */
    uint32_t image_tag;            /** 写入这份参数的固件镜像指纹；与当前固件不同=刚换过固件，见 A_Config_Init */
} Config_t;

/* SRAM 实时参数。前半部分由 Flash 参数在上电时复制，运行中可单独修改且不落盘；
 * torque_mode/damping_percent 只有 SRAM 值，掉电恢复为 1/100。 */
typedef struct {
    uint16_t torque_limit_ckgcm;   /* 扭矩上限，0.01 kgf·cm */
    uint32_t traj_amax_dps2;
    uint32_t traj_dmax_dps2;
    uint16_t traj_vmax_dps;
    uint16_t traj_smooth_ms;
    uint16_t traj_move_min_ms;
    uint16_t prot_current_limit_ma;
    uint16_t prot_current_time_ms;
    uint16_t prot_temp_limit_dC;
    uint16_t prot_temp_time_ms;
    uint16_t prot_stall_act_cdeg;
    uint16_t prot_stall_trip;
    uint16_t hold_kp;
    uint16_t hold_ki;
    uint8_t  protect_enable;
    uint8_t  voltage_min_dv;
    uint8_t  voltage_max_dv;
    uint16_t min_start_torque_ckgcm; /* 最小启动扭矩，0.01 kgf·cm */
    uint8_t  torque_mode;          /** 0=自由释放，1=正常，2=自定义阻尼 */
    uint8_t  damping_percent;      /** 自定义阻尼占空比，0~100 */
    uint16_t position_resolution_cdeg;
} RuntimeConfig_t;

extern Config_t        g_config;         /** Flash 参数实例，修改后须 MarkDirty */
extern RuntimeConfig_t g_runtime_config; /** SRAM 实时参数实例，不自动保存 */

uint16_t A_Config_TorqueMax(void); /* 硬件扭矩上限，0.01 kgf·cm；无有效硬件页时按FW_TORQUE */
void     A_Config_Init(void);                             /* 上电加载；同一固件非法字段单独修复，换过固件或坏页回默认并保留ID与波特率 */
void     A_Config_Default(uint8_t keep_id);                /* 用户恢复默认：非0仅保留ID；0全部恢复；波特率均回115200 */
void     A_Config_MarkDirty(void);                        /* 登记改动，唤醒保存任务，不在指令路径写Flash */
uint8_t  A_Config_SaveTask(void);                         /* 参数变脏时落盘；1=已保存/无待保存，0=失败待重试 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定保存任务的run标志 */
uint32_t A_Config_Baudrate(void);                         /* baud_code换算为实际波特率 */
void     A_Config_ReloadRuntime(uint8_t reset_transient); /* Flash参数复制到SRAM；非0同时恢复运行模式默认值 */
