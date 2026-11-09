#pragma once

#include <stdint.h>
#include "C_Traj_Planner.h"   /* 平滑时间、最短运动时间的上限以规划器为准 */

/* 固件版本 主.次.修订，各0~255；0.0.0表示未发布版本。 */
#define SERVO_VERSION_MAJOR 1U
#define SERVO_VERSION_MINOR 0U
#define SERVO_VERSION_PATCH 0U

/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC版 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     30U       /** 扭力型号 30kg；仅作型号标识，不参与扭矩换算 */

/* ==================== 2. 电机标定（标定固件打印，整节替换） ====================
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。
 * 本节由 A_Calib_Min 标定固件实测打印，换电机或减速箱后重新标定并整节替换。 */
#define MOTOR_CAL_MV              7666   /* 标定电压，mV */
#define MOTOR_KV_UV                397   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV              7063   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                 127   /* 动摩擦，mV */
#define MOTOR_FS_MV                413   /* 起转电压，mV */
#define MOTOR_DELAY_US            3211   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             6   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          13   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        162   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      7500   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      7500   /* 额定低速减速度，度/秒² */
/* ==================== 电机标定结束 ==================== */

/* ==================== 3. 控制器整定(固件常量，不对外) ====================
 * 带宽越高越硬，但电位器噪声放大越多；约束见 C_Pos_Ctrl.h。
 * 位置/速度极点与扰动极点分开配：速度估计多靠电压模型、少靠电位器微分。
 * 速度环带宽不超过观测器带宽。 */
#define TUNE_OBS_BW_HZ          12   /** 观测器位置/速度极点(Hz) */
#define TUNE_OBS_DIST_HZ         5   /** 观测器扰动极点(Hz)，≤ TUNE_OBS_BW_HZ */
/* 速度环带宽受纯延迟限制 ω_v·延迟 ≤ 1/3(与 C_PosCtrl_Init 同一公式) */
#define CTRL_VEL_BW_DELAY_MAX  ((MOTOR_DELAY_US > 333) ? 1000000U / (3U * MOTOR_DELAY_US) : 1000U)
#define TUNE_VEL_BW_RADS       ((CTRL_VEL_BW_DELAY_MAX < 70U) ? CTRL_VEL_BW_DELAY_MAX : 70U)  /** 速度环带宽 ω_v，rad/s */
#define TUNE_POS_BW_RADS       ((TUNE_VEL_BW_RADS / 4U < 20U) ? TUNE_VEL_BW_RADS / 4U : 20U)  /** 位置环带宽 ω_p，rad/s：≤ ω_v/4 */
#define TUNE_INT_BW_RADS         5U  /** 速度积分零点 ω_i，rad/s，≤ ω_p */
/* 负载惯量比：J_load/J_motor×10，10=负载惯量等于电机自身。只供观测器模型和位置环带宽收敛，不改轨迹限值和前馈 */
#define TUNE_LOAD_RATIO_X10     20U

/* ==================== 4. 运行参数出厂值 ====================
 * 精度、V/A/D、平滑时间等是Flash/SRAM运行参数，这里只是出厂值。 */
#define CFG_DEF_RESOLUTION_CDEG 50U  /** 到位精度：误差在它以内且静止才算到位，厘度 */
#define CFG_DEF_HOLD_KP         80  /** 保持 P，‰/°：超出到位窗1°输出标定电压的1.5×P‰ */
#define CFG_DEF_HOLD_KI           0  /** 保持 I，‰/(°·s)；0=积分不动，只用到位时继承的托举力 */
#define CFG_DEF_LOAD_RATIO_X10    0U  /** 负载惯量比×10；0=空载/轻载，扰动前馈关闭 */
#define CFG_DEF_LOAD_LEARN        1U  /** 位置负载(重力)自学习：1=开 0=关 */
/* V/A/D取额定能力的90%/70%/70%；实际限值由现有Flash/SRAM参数决定 */
#define CFG_DEF_VMAX_DPS       (MOTOR_RATED_VEL_DPS * 9U / 10U)
#define CFG_DEF_AMAX_DPS2      (MOTOR_RATED_ACC_DPS2 * 7U / 10U)
#define CFG_DEF_DMAX_DPS2      (MOTOR_RATED_DEC_DPS2 * 7U / 10U)
#define CFG_DEF_SMOOTH_MS       100  /** 平滑时间(加速度0→A，即速度滑动平均窗口)，ms；0=自动取机械时间常数τ */
#define CFG_DEF_MOVE_MIN_MS    150U  /** 最短运动时间，ms；0=关闭。每条指令至少用这么久到达(带时间取较大者) */

/* ==================== 5. 硬件量程与旋向 ==================== */
#define CDEG_RANGE          36000 /** 整圈角度，厘度 */
#define CFG_PWM_FULL         2400 /** PWM满量程，须与驱动层一致 */
#define CFG_TICK_HZ          1000 /** 控制任务频率(Hz) */
#define CFG_DIR_INVERT          1 /** 1=反转整机标称旋向 */
#define SERVO_PWM_MIN         500 /** 协议最小脉宽，也是电机模式负向满速 */
#define SERVO_PWM_MID        1500 /** 电机模式零速指令 */
#define SERVO_PWM_MAX        2500 /** 协议最大脉宽，也是电机模式正向满速 */

/* ==================== 6. 运行参数范围 ====================
 * 范围即生效范围：越界拒绝，读回实际生效值(V/A/D的0=按额定，平滑0=自动取τ)。 */
#define CFG_TRAJ_VMAX_MAX_DPS      MOTOR_RATED_VEL_DPS   /** 速度上限=额定能力 */
#define CFG_TRAJ_AMAX_MAX_DPS2     MOTOR_RATED_ACC_DPS2
#define CFG_TRAJ_DMAX_MAX_DPS2     MOTOR_RATED_DEC_DPS2
#define CFG_SMOOTH_MAX_MS          TRAJ_SMOOTH_MS_MAX    /** 平滑时间上限，ms */
#define CFG_SMOOTH_MIN_MS          1U                    /** 平滑时间下限，ms(0另允许=自动τ) */
#define CFG_MOVE_MIN_MAX_MS        TRAJ_MOVE_MIN_MS_MAX  /** 最短运动时间上限，ms */
#define CFG_HOLD_KP_MAX            100U  /** 保持 P 上限，‰/° */
#define CFG_HOLD_KI_MAX            100U  /** 保持 I 上限，‰/(°·s) */
#define CFG_LOAD_RATIO_MAX_X10      40U  /** 负载惯量比上限×10(4.0) */
#define CFG_RESOLUTION_MIN_CDEG    (MOTOR_MIN_STEP_CDEG * 3U / 2U)  /** 到位精度下限=1.5×最小可靠位移 */
#define CFG_RESOLUTION_MAX_CDEG    500U

/* 速度噪声底：观测器把读数噪声微分成速度噪声，静止判据以它为刻度。
 * 取读数噪声与最小步长估计的较大者，避免步长标得小时静止判据长期不成立 */
#define CTRL_VEL_NOISE_STEP (TUNE_OBS_BW_HZ * MOTOR_MIN_STEP_CDEG * 3 / 8)
#define CTRL_VEL_NOISE_READ (TUNE_OBS_BW_HZ * MOTOR_NOISE_CDEG * 3)
#define CTRL_VEL_NOISE_CDPS ((CTRL_VEL_NOISE_STEP > CTRL_VEL_NOISE_READ) ? CTRL_VEL_NOISE_STEP : CTRL_VEL_NOISE_READ)

_Static_assert(MOTOR_KV_UV > 0 && MOTOR_KA_NV > 0 && MOTOR_FS_MV >= MOTOR_FC_MV && MOTOR_DELAY_US < 7000
               && MOTOR_CAL_MV > 0 && MOTOR_MIN_STEP_CDEG >= 1, "Invalid motor calibration");
_Static_assert(MOTOR_SIGN == 1 || MOTOR_SIGN == -1, "Invalid motor direction");
_Static_assert(TUNE_POS_BW_RADS * 4U <= TUNE_VEL_BW_RADS && TUNE_INT_BW_RADS <= TUNE_POS_BW_RADS && TUNE_INT_BW_RADS >= 1U
               && TUNE_VEL_BW_RADS * 1000U <= TUNE_OBS_BW_HZ * 6283U
               && TUNE_OBS_DIST_HZ >= 1U && TUNE_OBS_DIST_HZ <= TUNE_OBS_BW_HZ, "Controller bandwidths violate constraints");
_Static_assert(CFG_DEF_RESOLUTION_CDEG >= CFG_RESOLUTION_MIN_CDEG && CFG_DEF_RESOLUTION_CDEG <= CFG_RESOLUTION_MAX_CDEG
               && CFG_RESOLUTION_MIN_CDEG >= 1U
               && CFG_DEF_HOLD_KP <= CFG_HOLD_KP_MAX
               && CFG_DEF_HOLD_KI <= CFG_HOLD_KI_MAX && CFG_DEF_LOAD_RATIO_X10 <= CFG_LOAD_RATIO_MAX_X10
               && (CFG_DEF_SMOOTH_MS == 0U || (CFG_DEF_SMOOTH_MS >= CFG_SMOOTH_MIN_MS && CFG_DEF_SMOOTH_MS <= CFG_SMOOTH_MAX_MS))
               && CFG_DEF_MOVE_MIN_MS <= CFG_MOVE_MIN_MAX_MS,
               "Invalid default motion/hold parameters");
/* 平滑时间自动值 τ = ka/kv 也不能超过上限 */
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

#define SERVO_CUSTOM_SPAN_MIN 100U /** 自定义行程下限(厘度)，再小每档低于机构分辨率 */

/* 校准记账：Flash存锚点，有效行程与零点由"锚点+模式标称行程"现推，反复校准不叠加 */
typedef enum {
    SERVO_CAL_NONE = 0, /* 未校准，零点取 Flash 历史偏移值 */
    SERVO_CAL_MID,      /* SCK：锚点是行程中点(1500us对应角度) */
    SERVO_CAL_ZERO      /* SCZ：锚点是行程起点(500us对应角度) */
} ServoCalKind_t;

typedef enum {
    SERVO_BOOT_GOTO_START = 1, /* 上电转到startup_pwm */
    SERVO_BOOT_HOLD,           /* 上电保持实际位置 */
    SERVO_BOOT_RELEASE         /* 上电小阻力释放，首条控制指令恢复 */
} ServoBootMode_t;

/* Flash掉电保存参数：扭矩字段用百分比0-100；升级沿用重置并保留ID/波特率的规则。 */
typedef struct {
    uint8_t  servo_id;             /** 总线ID，0~254 */
    uint8_t  baud_code;            /** 1~8，默认5=115200 */
    uint8_t  servo_mode;           /** 工作模式，见ServoMode_t，1~11 */
    uint8_t  boot_mode;            /** 上电动作，见ServoBootMode_t，1~3 */
    uint16_t torque_limit_pct;     /** 扭矩上限，百分比0-100；分母是额定堵转电流 */
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
    uint16_t traj_smooth_ms;       /** S曲线平滑时间(加速度0→A)，ms；0=自动(τ) */
    uint16_t traj_move_min_ms;     /** 最短运动时间，ms；0=关闭 */
    uint16_t prot_current_limit_ma;/** 过流保护阈值，mA */
    uint16_t prot_current_time_ms; /** 过流超阈后持续多久触发，ms */
    uint16_t prot_temp_limit_dC;   /** 过温保护阈值，0.1℃ */
    uint16_t prot_temp_time_ms;    /** 过温超阈后持续多久触发，ms */
    uint16_t prot_stall_act_cdeg;  /** 堵转：实际位移上限，厘度/20ms；参考阈值固定为它的PROT_STALL_REF_RATIO倍 */
    uint16_t prot_stall_trip;      /** 连续多少次成立后判堵转 */
    uint16_t position_resolution_cdeg; /** 到位精度，厘度，CFG_RESOLUTION_MIN_CDEG~500 */
    uint16_t hold_kp;              /** 保持 P，‰/°，0~CFG_HOLD_KP_MAX */
    uint16_t load_learn;           /** 位置负载(重力)自学习：1=开 0=关(占原齿隙位置，布局不变) */
    uint16_t hold_ki;              /** 保持 I，‰/(°·s)，0~CFG_HOLD_KI_MAX；0=积分不动 */
    uint8_t  protect_enable;       /** 保护功能位：bit0温度 bit1电流 bit2堵转 bit3电压 */
    uint8_t  voltage_min_dv;       /** 最低工作电压，0.1V */
    uint8_t  voltage_max_dv;       /** 最高工作电压，0.1V */
    uint8_t  load_ratio_x10;       /** 负载惯量比 J负载/J电机×10，0~CFG_LOAD_RATIO_MAX_X10；0=扰动前馈关闭 */
    uint16_t min_start_torque_pct;   /** 最小启动扭矩，百分比0-100；0=关闭门槛 */
    uint32_t image_tag;            /** 写入这份参数的固件镜像指纹；与当前不同=刚换过固件 */
} Config_t;

/* SRAM 实时参数：前半由Flash上电复制，运行中可改不落盘；
 * torque_mode/damping_percent 只有SRAM值，掉电恢复1/100。 */
typedef struct {
    uint16_t torque_limit_pct;     /* 扭矩上限，百分比0-100 */
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
    uint16_t min_start_torque_pct; /* 最小启动扭矩，百分比0-100 */
    uint8_t  torque_mode;          /** 0=自由释放，1=正常，2=自定义阻尼 */
    uint8_t  damping_percent;      /** 自定义阻尼占空比，0~100 */
    uint16_t position_resolution_cdeg;
    uint8_t  load_ratio_x10;
    uint16_t load_learn;
} RuntimeConfig_t;

extern Config_t        g_config;         /** Flash 参数实例，修改后须 MarkDirty */
extern RuntimeConfig_t g_runtime_config; /** SRAM 实时参数实例，不自动保存 */

void     A_Config_Init(void);                             /* 上电加载；同固件非法字段修复，换固件或坏页回默认并保留ID/波特率 */
void     A_Config_Default(uint8_t keep_id);                /* 恢复默认：非0仅保留ID；0全部恢复；波特率回115200 */
void     A_Config_MarkDirty(void);                        /* 登记改动，唤醒保存任务，不在指令路径写Flash */
uint8_t  A_Config_SaveTask(void);                         /* 变脏时落盘；1=已保存/无待保存，0=失败待重试 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定保存任务的run标志 */
uint32_t A_Config_Baudrate(void);                         /* baud_code换算为实际波特率 */
void     A_Config_ReloadRuntime(uint8_t reset_transient); /* Flash参数复制到SRAM；非0同时恢复运行模式默认值 */
