#pragma once

#include <stdint.h>
#include "C_Traj_Planner.h"   /* 平滑时间、最短运动时间的上限以规划器为准 */

/* A_Config.h —— 固件常量与掉电保存参数。电位器版与磁编码版此文件相同，随编码器/型号变化的数据只在 App_Model.h。
 *   1. 版本与型号            5. 用户参数：出厂值与范围(对外，参数表 20~49 / 80~91)
 *   2. 板级硬件              6. 工作模式、校准、上电动作(对外编号)
 *   3. 内部单位与坐标        7. 参数存储结构与接口
 *   4. 控制器整定(不对外)
 * 只放"值"；推导量跟着使用它的模块走；编译期校验在 A_Config.c 开头；寄存器地址与读写规则在 A_Param.c。 */

/* ==================== 1. 版本与型号 ==================== */
/* 固件版本 主.次.修订，各0~255；0.0.0表示未发布版本。 */
#define SERVO_VERSION_MAJOR 1U
#define SERVO_VERSION_MINOR 0U
#define SERVO_VERSION_PATCH 0U

#include "App_Model.h"   /* 型号选择、编码器常量、型号四参数、电机标定、工作电压范围 */


/* ==================== 2. 板级硬件(换板时核对) ==================== */
/* ---- 母线电压采样：分压10k+39k，满量程16.17V ---- */
#define ADC_VDD_MV                 3300U  /** ADC参考电压(mV) */
#define ADC_FULL_SCALE             4096U  /** ADC满量程计数 */
#define VOLTAGE_DIV_NUM              49U  /** 母线分压比分子，V_bat = ADC*3300mV*49/(4096*10) */
#define VOLTAGE_DIV_DEN              10U  /** 母线分压比分母 */
#define HW_BUS_SAFE_MV            15000   /** 母线短时允许上限(mV)：回馈钳位硬阈值，须低于器件耐压与电机端TVS导通电压 */
#define HW_BUS_MEAS_MAX_MV        16000   /** 母线采样可信上限(mV)，更高读数已饱和 */
#define HW_BUS_MEAS_MIN_MV  ((PROT_VOLTAGE_CFG_MIN_DV - 10U) * 100U) /** 采样可信下限(mV)：比欠压动作线宽，运动中压降可低于动作线 */
/* ---- 电流检测：60mΩ采样电阻+OPA1单端8x，满量程6.875A，1LSB≈1.68mA；只有驱动段电流流过采样电阻 ----
 * 正向可上报最大5.29A(低于AT8236斩波点5.5A)，倒拖回馈也可测(约-1.6A以内)。 */
#define CURRENT_SENSE_MOHM           60U  /** 采样电阻(毫欧) */
#define CURRENT_SENSE_GAIN            8U  /** OPA1单端PGA增益 */
#define TORQUE_STALL_CURRENT_MA    2000U  /** 额定堵转电流(mA)：扭矩百分比的分母，也是满PWM对应的电流 */
/* ---- 电机输出与时基 ---- */
#define CFG_PWM_FULL               2400   /** 电机PWM满量程，须与驱动层 TIM_PWM_RESOLUTION 一致 */
#define CFG_TICK_HZ                1000   /** 控制任务频率(Hz)，1拍=1ms */
#define CFG_DIR_INVERT                1   /** 1=反转整机标称旋向 */


/* ==================== 3. 内部单位与坐标(固件内部，不对外) ====================
 * 跨模块接口一律用整数物理单位：角度 厘度(0.01°)、速度 厘度/s、加速度 厘度/s²、电压 mV、电流 mA、时间 ms。
 * 定点数(Qn)只在各算法模块内部(C_*.c、输出级)使用，不出模块、不进参数表；
 * 参数表单位(°/s、°/s²、0.1V、%…)只在 A_Param.c 出入口与 A_Servo_Ctrl 的 ApplyTrajConfig 换算一次。
 * 周期路径(1ms/10ms 任务)不做运行时除法：常数除法用 C_Fixed.h 的 Fx_UDivR/Fx_DivR，变量除法放到配置变化时预算。 */
#define CDEG_PER_DEG                100   /** 1度 = 100厘度 */
#define CDEG_RANGE                36000   /** 整圈角度，厘度 */
/* ---- 协议脉宽量程 ---- */
#define SERVO_PWM_MIN               500   /** 协议最小脉宽，也是电机模式负向满速 */
#define SERVO_PWM_MID              1500   /** 电机模式零速指令 */
#define SERVO_PWM_MAX              2500   /** 协议最大脉宽，也是电机模式正向满速 */
/* 内部指令坐标 = 脉宽 1/8us(Q3)，4000~20000 共16000级：Flash 里的脉宽字段、PWM输入滤波、行程换算都用它；
 * 文本协议(us)只在 A_UartCmd.c、数据帧位置码(0~4095)只在 A_Param.c 出入口换算 */
#define Q3(us)       ((uint16_t)((uint16_t)(us) << 3))
#define Q3_MIN       Q3(SERVO_PWM_MIN)
#define Q3_MID       Q3(SERVO_PWM_MID)
#define Q3_MAX       Q3(SERVO_PWM_MAX)
#define Q3_SPAN      (Q3_MAX - Q3_MIN)


/* ==================== 4. 控制器整定(固件常量，不对外) ====================
 * 带宽越高越硬，但读数噪声/齿隙在环路里放大越多；约束见 C_Pos_Ctrl.h 与 A_Config.c 的编译期校验。
 * 位置/速度极点与扰动极点分开配：速度估计多靠电压模型、少靠位置微分。速度环带宽不超过观测器带宽。 */
#define TUNE_OBS_BW_HZ         ENCODER_TUNE_OBS_BW_HZ   /** 观测器位置/速度极点(Hz)，随编码器见 App_Model.h */
#define TUNE_OBS_DIST_HZ         5   /** 观测器扰动极点(Hz)，≤ TUNE_OBS_BW_HZ */
/* 速度环带宽受纯延迟限制 ω_v·延迟 ≤ 1/3(与 C_PosCtrl_Init 同一公式)，另受编码器带宽上限 */
#define TUNE_VEL_BW_DELAY_MAX  ((MOTOR_DELAY_US > 333) ? 1000000U / (3U * MOTOR_DELAY_US) : 1000U)
#define TUNE_VEL_BW_RADS       ((TUNE_VEL_BW_DELAY_MAX < ENCODER_TUNE_VEL_BW_CAP) ? TUNE_VEL_BW_DELAY_MAX : ENCODER_TUNE_VEL_BW_CAP)  /** 速度环带宽 ω_v，rad/s */
/* 位置环带宽随编码器与型号见 App_Model.h；运动段位置增益另按距离 ×1~×2(见 C_Pos_Ctrl.c MOVE_POS_BW_X) */
#define TUNE_POS_BW_RADS       ENCODER_TUNE_POS_BW_RADS  /** 位置环带宽 ω_p，rad/s：≤ ω_v/4 */
#define TUNE_INT_BW_RADS         5U  /** 速度积分零点 ω_i，rad/s，≤ ω_p */
/* 观测器名义惯量 = 电机惯量×(1 + 本值/10)：只影响速度估计，扰动估计不进输出 */
#define TUNE_OBS_LOAD_RATIO_X10 20U


/* ==================== 5. 用户参数：出厂值与范围(对外) ====================
 * 范围即生效范围：越界拒绝，读回实际生效值。每项注明参数表地址(Flash/RAM)；线上字节数 = 存储字段宽度。 */
/* ---- 20~23 通信与模式 ---- */
#define CFG_ID_MAX                 254U   /** 20 舵机ID 0~254，255为广播 */
#define CFG_BAUD_DEFAULT             5U   /** 21 波特率档位 1~8，5=115200 */
#define CFG_BAUD_MIN                 1U
#define CFG_BAUD_MAX                 8U
/* ---- 24/84 扭矩上限、49 最小启动扭矩：百分比0-100，分母是额定堵转电流 TORQUE_STALL_CURRENT_MA ---- */
#define TORQUE_MAX_PERCENT         100U   /** 扭矩百分比满量程，也是出厂值与可设上限 */
/* ---- 30~38 运动(V/A/D 写0=按额定；平滑写0=自动取τ) ---- */
#define CFG_TRAJ_VMAX_MAX_DPS      MOTOR_RATED_VEL_DPS   /** 30 最大速度上限=额定能力，°/s */
#define CFG_TRAJ_AMAX_MAX_DPS2     MOTOR_RATED_ACC_DPS2  /** 31 °/s² */
#define CFG_TRAJ_DMAX_MAX_DPS2     MOTOR_RATED_DEC_DPS2  /** 32 °/s² */
#define CFG_DEF_VMAX_DPS          (MOTOR_RATED_VEL_DPS * 9U / 10U)  /** 出厂取额定的90%/70%/70% */
#define CFG_DEF_AMAX_DPS2         (MOTOR_RATED_ACC_DPS2 * 7U / 10U)
#define CFG_DEF_DMAX_DPS2         (MOTOR_RATED_DEC_DPS2 * 7U / 10U)
#define CFG_DEF_SMOOTH_MS          100U  /** 33 平滑时间(加速度0→A，即速度滑动平均窗口)，ms */
#define CFG_SMOOTH_MIN_MS            1U  /**    下限(0另允许=自动τ) */
#define CFG_SMOOTH_MAX_MS          TRAJ_SMOOTH_MS_MAX
#define CFG_DEF_MOVE_MIN_MS        150U  /** 34 最短运动时间，ms；0=关闭。每条指令至少用这么久到达(带时间取较大者) */
#define CFG_MOVE_MIN_MAX_MS        TRAJ_MOVE_MIN_MS_MAX
#define CFG_DEF_HOLD_KP             80U  /** 35 保持 P，‰/°：超出到位窗1°输出标定电压的1.5×P‰ */
#define CFG_HOLD_KP_MAX            100U
#define CFG_DEF_RESOLUTION_CDEG    ENCODER_DEF_RESOLUTION_CDEG       /** 37 到位精度，厘度，随编码器 */
#define CFG_RESOLUTION_MIN_CDEG    (MOTOR_MIN_STEP_CDEG * 3U / 2U)   /**    下限=1.5×最小可靠位移 */
#define CFG_RESOLUTION_MAX_CDEG    500U
#define CFG_DEF_HOLD_KI              0U  /** 38 保持 I，‰/(°·s)；0=积分不动，只用到位时继承的托举力 */
#define CFG_HOLD_KI_MAX            100U
/* ---- 40~41 过流：3A软件跳闸先于硬件保护(可上报上限5.29A，AT8236斩波5.5A)，独立于扭矩限流 ---- */
#define PROT_CURRENT_LIMIT_DEFAULT_MA 3000U
#define PROT_CURRENT_LIMIT_MIN_MA      500U
#define PROT_CURRENT_LIMIT_MAX_MA     5000U  /** 低于硬件保护 */
#define PROT_CURRENT_TIME_DEFAULT_MS    30U
#define PROT_CURRENT_TIME_MIN_MS        10U
#define PROT_CURRENT_TIME_MAX_MS      1000U
/* ---- 42~43 过温，0.1℃ ---- */
#define PROT_TEMP_LIMIT_DEFAULT_DC     500U  /** 50.0℃ */
#define PROT_TEMP_LIMIT_MIN_DC         300U
#define PROT_TEMP_LIMIT_MAX_DC        1000U
#define PROT_TEMP_TIME_DEFAULT_MS      500U
#define PROT_TEMP_TIME_MIN_MS          250U  /** 不短于一个检测周期 */
#define PROT_TEMP_TIME_MAX_MS        10000U
/* ---- 44~45 堵转：每 PROT_STALL_PERIOD_MS 比较参考与实际位移，连续成立达设定次数就地保持(不卸力、不记故障位) ---- */
#define PROT_STALL_PERIOD_MS            20U  /** 检测周期(ms)，也是44的位移单位：厘度/周期 */
#define PROT_STALL_REF_RATIO             4U  /** 参考位移阈值 = 实际位移阈值 × 它 */
#define PROT_STALL_ACT_DEFAULT_CDEG     32U
#define PROT_STALL_THRESHOLD_MIN_CDEG   10U
/** 上限：参考阈值不能超过额定速度一个周期的位移，否则"参考在走"永不成立 */
#define PROT_STALL_THRESHOLD_MAX_CDEG ((uint16_t)((uint32_t)MOTOR_RATED_VEL_DPS * CDEG_PER_DEG * PROT_STALL_PERIOD_MS \
                                                  / 1000U / PROT_STALL_REF_RATIO))
#define PROT_STALL_TRIP_DEFAULT          5U  /** 5×20=100ms */
#define PROT_STALL_TRIP_MIN              1U
#define PROT_STALL_TRIP_MAX            100U
/* ---- 46 保护开关位，出厂全开 ---- */
#define PROT_ENABLE_TEMP              0x01U
#define PROT_ENABLE_CURRENT           0x02U
#define PROT_ENABLE_STALL             0x04U
#define PROT_ENABLE_VOLTAGE           0x08U
#define PROT_ENABLE_IMPLEMENTED       0x0FU  /** 已实现功能位，也是对外上限 */
#define PROT_ENABLE_DEFAULT           0x0FU
/* ---- 47~48 工作电压，0.1V：可设范围与出厂值 PROT_VOLTAGE_CFG_MIN/MAX_DV 随型号见 App_Model.h ----
 * 静止时超出设置值上下各一个容差才动作(8V型出厂动作线5.9V/9.1V)；运动中的起步压降、回馈泵升不判，由输出级处理。
 * 上下限各留回差后仍须有恢复区间：lo + 2×回差 < hi(即至少相差0.5V)。 */
#define PROT_VOLTAGE_TOL_LO_DV           1U  /** 欠压动作线 = 设置下限 - 本值 */
#define PROT_VOLTAGE_TOL_HI_DV           1U  /** 过压动作线 = 设置上限 + 本值 */
#define PROT_VOLTAGE_HYST_DV             2U  /** 恢复回差 */


/* ==================== 6. 工作模式、校准、上电动作(对外编号) ====================
 * 模式编号是对外协议：90度(5/6)紧挨180度(3/4)；7/8 是整圈编码器的360度模式(电位器有死区不支持，编号空着，
 * Flash 里遗留的7/8上电修回270度)；13(自定义)只能由 AMI/AMX 进入。 */
typedef enum {
    SERVO_MODE_270_CW = 1, /* 270度位置，P增大时CW */
    SERVO_MODE_270_CCW,    /* 270度位置，P增大时CCW */
    SERVO_MODE_180_CW,     /* 180度位置，P增大时CW */
    SERVO_MODE_180_CCW,    /* 180度位置，P增大时CCW */
    SERVO_MODE_90_CW,      /* 90度位置，P增大时CW */
    SERVO_MODE_90_CCW,     /* 90度位置，P增大时CCW */
    SERVO_MODE_360_CW,     /* 360度整圈位置，仅整圈编码器 */
    SERVO_MODE_360_CCW,
    SERVO_MODE_TURNS_CW,   /* CW定圈，T=0无限 */
    SERVO_MODE_TURNS_CCW,  /* CCW定圈，T=0无限 */
    SERVO_MODE_TIMED_CW,   /* CW定时，T=0无限 */
    SERVO_MODE_TIMED_CCW,  /* CCW定时，T=0无限 */
    SERVO_MODE_CUSTOM      /* 自定义行程位置模式，只能由AMI/AMX进入 */
} ServoMode_t;

#if ENCODER_IS_CIRCULAR
#define SERVO_MODE_IS_SUPPORTED(m)  ((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_CUSTOM)
#else
#define SERVO_MODE_IS_SUPPORTED(m)  (((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_90_CCW) \
                                     || ((m) >= SERVO_MODE_TURNS_CW && (m) <= SERVO_MODE_CUSTOM))
#endif
#define SERVO_MODE_USER_SETTABLE(m) (SERVO_MODE_IS_SUPPORTED(m) && (m) != SERVO_MODE_CUSTOM)   /* 协议可直接设 */

#define SERVO_CUSTOM_SPAN_MIN 100U /** 自定义行程下限(厘度)，再小每档低于机构分辨率 */

/* 校准记账：Flash存锚点，有效行程与零点由"锚点+模式标称行程"现推，反复校准不叠加 */
typedef enum {
    SERVO_CAL_NONE = 0, /* 未校准，零点取 Flash 历史偏移值 */
    SERVO_CAL_MID,      /* SCK：锚点是行程中点(1500us对应角度) */
    SERVO_CAL_ZERO      /* SCZ：锚点是行程起点(500us对应角度) */
} ServoCalKind_t;

typedef enum {
    SERVO_BOOT_GOTO_START = 1, /* 上电转到startup_q3 */
    SERVO_BOOT_HOLD,           /* 上电保持实际位置 */
    SERVO_BOOT_RELEASE         /* 上电小阻力释放，首条控制指令恢复 */
} ServoBootMode_t;


/* ==================== 7. 参数存储结构与接口 ==================== */

/* 运行参数：Flash 与 SRAM 各一份、布局相同；上电/重载时 Flash→SRAM 整块复制，运行中只读 SRAM 份。
 * 字段宽度就是参数表线上字节数：取值不超过255的用1字节。单位即参数表单位，内部单位由使用模块换算。 */
typedef struct {
    uint16_t traj_vmax_dps;            /** 30/85 最大速度，°/s，≤额定；0=按额定 */
    uint16_t traj_amax_dps2;           /** 31/86 最大加速度，°/s²，≤额定；0=按额定 */
    uint16_t traj_dmax_dps2;           /** 32/87 最大减速度，°/s²，≤额定；0=按额定 */
    uint16_t traj_move_min_ms;         /** 34/89 最短运动时间，ms；0=关闭 */
    uint16_t position_resolution_cdeg; /** 37/8E 到位精度，厘度 */
    uint16_t prot_current_limit_ma;    /** 40 过流阈值，mA */
    uint16_t prot_current_time_ms;     /** 41 过流持续时间，ms */
    uint16_t prot_temp_limit_dC;       /** 42 过温阈值，0.1℃ */
    uint16_t prot_temp_time_ms;        /** 43 过温持续时间，ms */
    uint16_t prot_stall_act_cdeg;      /** 44 堵转实际位移阈值，厘度/20ms；参考阈值为它的 PROT_STALL_REF_RATIO 倍 */
    uint8_t  torque_limit_pct;         /** 24/84 扭矩上限，%；0=禁止主动驱动 */
    uint8_t  traj_smooth_ms;           /** 33/88 S曲线平滑时间，ms；0=自动(τ) */
    uint8_t  hold_kp;                  /** 35/8A 保持 P，‰/° */
    uint8_t  hold_ki;                  /** 38/8F 保持 I，‰/(°·s)；0=积分不动 */
    uint8_t  prot_stall_trip;          /** 45 堵转连续次数 */
    uint8_t  protect_enable;           /** 46 保护功能位：bit0温度 bit1电流 bit2堵转 bit3电压 */
    uint8_t  voltage_min_dv;           /** 47 最低工作电压，0.1V */
    uint8_t  voltage_max_dv;           /** 48 最高工作电压，0.1V */
    uint8_t  min_start_torque_pct;     /** 49 最小启动扭矩，%；0=关闭门槛 */
    uint8_t  reserved;                 /** 补齐偶数字节，恒为0 */
} RunParams_t;

/* Flash掉电保存参数。ID/波特率必须在最前(IAP 换固件时按偏移读回)；换固件(镜像指纹变)回默认并保留ID/波特率，
 * 所以字段可按需重排，不必只在末尾追加。 */
typedef struct {
    uint8_t  servo_id;             /** 20 总线ID，0~254 */
    uint8_t  baud_code;            /** 21 1~8，默认5=115200 */
    uint8_t  servo_mode;           /** 22 工作模式，见ServoMode_t */
    uint8_t  boot_mode;            /** 23 上电动作，见ServoBootMode_t，1~3 */
    uint8_t  cal_kind;             /** ServoCalKind_t，0=未校准 */
    uint8_t  custom_reverse;       /** 自定义模式方向(未经 CFG_DIR_INVERT 翻转的原始值) */
    uint8_t  slave_mode;           /** 从机模式(二进制CMD 04)：1=明文/二进制一律不应答；07恢复默认保留，换固件回0 */
    uint8_t  reserved_cfg;         /** 补齐偶数字节(image_tag前无填充)，恒为0 */
    uint16_t startup_q3;           /** 25 上电目标脉宽，Q3(1/8us)，Q3_MIN~Q3_MAX */
    uint16_t pulse_lo_q3;          /** SMI 标定的脉冲下界，Q3 */
    uint16_t pulse_hi_q3;          /** SMX 标定的脉冲上界，Q3，须 > pulse_lo_q3 */
    uint16_t position_offset_cdeg; /** 标准模式零点偏移(厘度，[0,36000) 存放)；已校准时由锚点现推 */
    uint16_t cal_anchor_cdeg;      /** 校准锚点的编码器原始角度，厘度 */
    uint16_t custom_offset_cdeg;   /** 自定义行程零点对应的编码器角度，厘度 */
    uint16_t custom_span_cdeg;     /** 自定义行程标称长度，厘度，只由 AMI/AMX 写 */
    RunParams_t run;               /** 运行参数的 Flash 份 */
    uint32_t image_tag;            /** 写入这份参数的固件镜像指纹；与当前不同=刚换过固件 */
} Config_t;

/* SRAM 实时参数：run 由 Flash 上电复制，运行中可改不落盘；torque_mode/damping_percent 只有SRAM值，掉电恢复1/100。 */
typedef struct {
    RunParams_t run;               /** 运行参数的 SRAM 份(生效值) */
    uint8_t  torque_mode;          /** 82 0=自由释放，1=正常，2=自定义阻尼 */
    uint8_t  damping_percent;      /** 83 自定义阻尼占空比，0~100 */
} RuntimeConfig_t;

extern Config_t        g_config;         /** Flash 参数实例，修改后须 MarkDirty */
extern RuntimeConfig_t g_runtime_config; /** SRAM 实时参数实例，不自动保存 */

void     A_Config_Init(void);                             /* 上电加载；同固件非法字段修复，换固件或坏页回默认并保留ID/波特率 */
void     A_Config_Default(uint8_t keep_id);                /* 恢复默认：非0仅保留ID；0全部恢复；波特率回115200 */
void     A_Config_Defaults(Config_t *c);                  /* 出厂值填进 *c(不登记保存)；参数修复以它为准 */
void     A_Config_MarkDirty(void);                        /* 登记改动，唤醒保存任务，不在指令路径写Flash */
uint8_t  A_Config_SaveTask(void);                         /* 变脏时落盘；1=已保存/无待保存，0=失败待重试 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定保存任务的run标志 */
uint32_t A_Config_Baudrate(void);                         /* baud_code换算为实际波特率 */
void     A_Config_ReloadRuntime(uint8_t reset_transient); /* Flash运行参数复制到SRAM；非0同时恢复扭矩模式/阻尼默认值 */
