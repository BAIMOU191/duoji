#pragma once

#include <stdint.h>

/* 固件版本 主.次.修订，各0~255。 */
#define SERVO_VERSION_MAJOR 2U
#define SERVO_VERSION_MINOR 0U
#define SERVO_VERSION_PATCH 0U

/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时上位机和 Boot 逐项与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC版 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     20U       /** 扭力型号 20kg */

/* ==================== 2. 本机标定 ====================
 * 全部由 A_Calib_Min 标定固件打印，照抄这一节即可。换舵机只改这8个数。
 * 本机：15kg 电位器版，下面的值由标定程序打印后抄回。 */
#define CAL_SPEED_SLOPE_Q16  4765
#define CAL_FRICTION_PWM      42
#define CAL_BREAKAWAY_PWM     201
#define CAL_MOTOR_SIGN          -1
#define MODEL_TAU_MS           46
#define MODEL_DELAY_TICKS       2
#define MODEL_BRAKE_GAIN_Q8   338
#define CAL_MIN_STEP_CDEG      40  /** 最小可靠位移(厘度)：速度噪声底、摩擦与到位零速判据由它派生，与到位精度无关 */

/* ==================== 3. 调参 ====================
 * 改这一节要重跑 tests/run.py。带宽取"够用的最低"：电位器抽头的滑动接触噪声很大，
 * 环路一快就把噪声放大成抖动。速度环至少比位置环快2倍(第7节断言)；观测器至少比速度环快3倍、
 * 积分零点不高于位置环带宽——这两条在第6节自动满足(按需抬高观测器带宽/压低积分零点)，
 * 单独改其中一个不会编译失败。负载惯量与电机增益偏差由观测器在线自适应(C_Speed_Observer.h)，
 * 不需要为带载重调这一节。 */
#define TUNE_STIFFNESS_RADS   9  /** 位置环带宽(rad/s) */
#define TUNE_DAMPING         25  /** 速度环带宽 / 位置环带宽 的10倍 */
#define TUNE_INTEGRAL_RADS    8  /** 速度积分零点(rad/s)，不得高于位置环带宽 */
#define TUNE_OBS_BW_HZ       11  /** 观测器带宽(Hz)，受纯延迟约束 */
#define TUNE_RESOLUTION_CDEG 40  /** 到位精度标度(厘度)：位置死区与各到位窗口，实际停住误差约1.5倍；只影响位置窗口 */
#define CFG_DEF_HOLD_STIFF   12 /** 到位保持刚度出厂值(PWM/厘度)，运行时可在0~12内修改 */
#define CFG_HOLD_STIFF_MAX   12U /** 对外统一上限；0=关闭保持弹簧 */
#define TUNE_HOLD_DAMP_X      0  /** 保持弹簧的速度阻尼 = 该值x速度环P增益；0=不带阻尼(本机型无手臂负载) */
#define TUNE_ACCEL_MIN_MS    45  /** 小位移加速段最短时间(ms)，约1倍tau；短于纯延迟没有意义 */

/* ==================== 4. 硬件量程与旋向 ==================== */
#define CDEG_RANGE          36000 /** 整圈角度，厘度 */
#define CFG_PWM_FULL         2400 /** PWM满量程，必须与驱动层实际限制一致 */
#define CFG_TICK_HZ          1000 /** 控制任务频率(Hz) */
#define CFG_DIR_INVERT          1 /** 1=反转整机标称旋向 */
#define SERVO_PWM_MIN         500 /** 协议最小脉宽，也是电机模式负向满速 */
#define SERVO_PWM_MID        1500 /** 电机模式零速指令 */
#define SERVO_PWM_MAX        2500 /** 协议最大脉宽，也是电机模式正向满速 */

/* ==================== 5. 轨迹出厂默认值 ====================
 * 这五个是运行时参数，存在 Flash 里、可由总线指令改(见下面的 Config_t)，这里只是出厂值。
 * 单位是真实物理量，不是百分比或档位；实际生效值 = min(设定值, 本机硬件能力)。*/
#define CFG_DEF_VMAX_DPS         280
#define CFG_DEF_AMAX_DPS2       4000
#define CFG_DEF_DMAX_DPS2       0
#define CFG_DEF_JMAX_DPS3    2000000
#define CFG_DEF_MOVE_MIN_MS      99

/* ==================== 6. 对象模型(派生量，不要手改) ====================
 * 一阶电机模型 dv/dt = (Kv*(u-d) - v)/tau，三个算法文件都要用。 */
#define PLANT_NET_PWM (((CFG_PWM_FULL - CAL_FRICTION_PWM) > 0) ? (CFG_PWM_FULL - CAL_FRICTION_PWM) : 1) /** 扣除摩擦后的净PWM */
#define PLANT_VSS_CDPS ((int32_t)(((int64_t)PLANT_NET_PWM << 16) / CAL_SPEED_SLOPE_Q16))                /** 满PWM稳态速度，厘度/秒 */
#define PLANT_A0_CDPSS ((int32_t)(((int64_t)PLANT_VSS_CDPS * 1000) / MODEL_TAU_MS))                     /** 零速起步加速度，厘度/秒^2 */
#define PLANT_KA_Q8_RAW ((int32_t)(((int64_t)65536 * 1000 * 256) / ((int64_t)CAL_SPEED_SLOPE_Q16 * MODEL_TAU_MS)))
#define PLANT_KA_Q8 ((PLANT_KA_Q8_RAW > 0) ? PLANT_KA_Q8_RAW : 1)                                       /** 加速度增益，Q8 */
#define PLANT_FF_MARGIN_PWM 250                                                                         /** 前馈之外必须留给反馈的PWM余量 */
/* 留足反馈余量后的最大速度：速度上限、速度指令限幅都取它，不随轨迹设定变 */
#define PLANT_VMAX_HW_CDPS ((int32_t)(((int64_t)PLANT_VSS_CDPS * (CFG_PWM_FULL - PLANT_FF_MARGIN_PWM - CAL_FRICTION_PWM)) / PLANT_NET_PWM))
/* 对外公布的轨迹能力。计算值不低于已经验证过的出厂参数，且与硬件信息页使用同一组定义。 */
#define PLANT_VMAX_RATIO_Q15 ((int32_t)(((int64_t)PLANT_VMAX_HW_CDPS << 15) / PLANT_VSS_CDPS))
#define PLANT_AMAX_HW_RAW_DPS2 ((uint32_t)((((int64_t)PLANT_A0_CDPSS * (32768L - PLANT_VMAX_RATIO_Q15 / 2)) >> 15) / 100))
#define PLANT_AMAX_HW_DPS2 ((PLANT_AMAX_HW_RAW_DPS2 > CFG_DEF_AMAX_DPS2) ? PLANT_AMAX_HW_RAW_DPS2 : CFG_DEF_AMAX_DPS2)
#define PLANT_DMAX_HW_RAW_DPS2 ((uint32_t)(((int64_t)(CFG_PWM_FULL - PLANT_FF_MARGIN_PWM) * 65536 * 1000 * 256) / ((int64_t)CAL_SPEED_SLOPE_Q16 * MODEL_TAU_MS * MODEL_BRAKE_GAIN_Q8) / 100))
#define PLANT_DMAX_HW_DPS2 ((PLANT_DMAX_HW_RAW_DPS2 > CFG_DEF_DMAX_DPS2) ? PLANT_DMAX_HW_RAW_DPS2 : CFG_DEF_DMAX_DPS2)
#define PLANT_JMAX_HW_RAW_DPS3 ((uint32_t)PLANT_AMAX_HW_DPS2 * 500U)
#define PLANT_JMAX_HW_MODEL_DPS3 ((PLANT_JMAX_HW_RAW_DPS3 > 2000000U) ? 2000000U : PLANT_JMAX_HW_RAW_DPS3)
#define PLANT_JMAX_HW_DPS3 ((PLANT_JMAX_HW_MODEL_DPS3 > CFG_DEF_JMAX_DPS3) ? PLANT_JMAX_HW_MODEL_DPS3 : CFG_DEF_JMAX_DPS3)
/* 最小有意义的速度指令：比它还小的指令分辨不出方向，摩擦前馈和观测器都拿它当零速刻度 */
#define PLANT_WCMD_MIN_CDPS (TUNE_STIFFNESS_RADS * CAL_MIN_STEP_CDEG / 4)
/* 调参约束自动满足：积分零点不高于位置环带宽，观测器至少比速度环快3倍(不够就抬，受第7节纯延迟上限约束) */
#define PLANT_INTEGRAL_RADS ((TUNE_INTEGRAL_RADS < TUNE_STIFFNESS_RADS) ? TUNE_INTEGRAL_RADS : TUNE_STIFFNESS_RADS)
#define PLANT_VEL_BW_RADS   (TUNE_STIFFNESS_RADS * TUNE_DAMPING / 10)
#define PLANT_OBS_BW_MIN_HZ ((PLANT_VEL_BW_RADS * 30000L + 62831L) / 62832L)
#define PLANT_OBS_BW_HZ     ((TUNE_OBS_BW_HZ > PLANT_OBS_BW_MIN_HZ) ? TUNE_OBS_BW_HZ : PLANT_OBS_BW_MIN_HZ)
/* 观测器把位置抖动微分成速度噪声，噪声底 ≈ 观测器带宽 * 最小可靠位移 * 3/8；所有速度阈值以它为刻度 */
#define PLANT_VEL_NOISE_CDPS (PLANT_OBS_BW_HZ * CAL_MIN_STEP_CDEG * 3 / 8)

/* ==================== 7. 编译期断言 ==================== */
/* 观测器带宽上限：纯延迟 L 的相位裕度要求 2*pi*f*(2L+1)*T <= 1 */
_Static_assert (PLANT_OBS_BW_HZ * 62832L * (2 * MODEL_DELAY_TICKS + 1) <= CFG_TICK_HZ * 10000L,
                "观测器带宽超过纯延迟允许的上限(位置环/速度环带宽太高，观测器被自动抬到了延迟上限之外)");
_Static_assert (TUNE_DAMPING >= 20, "速度环没有比位置环快2倍");
_Static_assert (CAL_MIN_STEP_CDEG >= 1, "最小可靠位移必须>=1厘度");
_Static_assert (TUNE_ACCEL_MIN_MS * 2 >= 2 * MODEL_DELAY_TICKS + 1, "加速段短于纯延迟，等于加速度阶跃");
_Static_assert (CFG_DEF_HOLD_STIFF <= CFG_HOLD_STIFF_MAX, "保持刚度默认值超出 0~12");
_Static_assert (TUNE_RESOLUTION_CDEG >= 1 && MODEL_TAU_MS > 0 && CAL_SPEED_SLOPE_Q16 > 0,
                "标定值非法(分辨率/时间常数/速度斜率)");
_Static_assert (CAL_MOTOR_SIGN == 1 || CAL_MOTOR_SIGN == -1, "电机符号只能是 +1 或 -1");
_Static_assert (CAL_FRICTION_PWM < CFG_PWM_FULL && CAL_BREAKAWAY_PWM < CFG_PWM_FULL,
                "摩擦/起转PWM超过满量程");
/* 默认最大速度不得超过本机硬件上限，否则上电时 Config_Repair 会把它修回出厂值，
 * 而出厂值本身又非法。判据必须和 Config_Repair 里的那一条逐字相同。 */
_Static_assert ((int32_t)CFG_DEF_VMAX_DPS * 100 <= PLANT_VMAX_HW_CDPS,
                "CFG_DEF_VMAX_DPS 超过本机硬件上限(PLANT_VMAX_HW_CDPS/100 度/秒)");

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
    uint32_t traj_amax_dps2;       /** 轨迹最大加速度，度/秒^2；0=硬件最大值 */
    uint32_t traj_dmax_dps2;       /** 轨迹最大减速度，度/秒^2；0=硬件最大值 */
    uint32_t traj_jmax_dps3;       /** 轨迹最大加加速度，度/秒^3；0=硬件最大值 */
    uint16_t traj_vmax_dps;        /** 轨迹最大速度，度/秒；0=硬件最大值 */
    uint16_t traj_move_min_ms;     /** 小位移最短运动时间，ms；0=关闭 */
    uint16_t prot_current_limit_ma;/** 过流保护阈值，mA */
    uint16_t prot_current_time_ms; /** 过流超过阈值后持续多久触发，ms */
    uint16_t prot_temp_limit_dC;   /** 过温保护阈值，0.1摄氏度 */
    uint16_t prot_temp_time_ms;    /** 过温超过阈值后持续多久触发，ms */
    uint16_t prot_stall_ref_cdeg;  /** 堵转判定：参考位移下限，厘度/20ms */
    uint16_t prot_stall_act_cdeg;  /** 堵转判定：实际位移上限，厘度/20ms */
    uint16_t prot_stall_trip;      /** 连续多少次成立后判堵转 */
    uint16_t reserved0;            /** 显式占满32位对齐尾部，固定为0 */
    uint8_t  hold_stiff;           /** 到位保持刚度，0~12 PWM/厘度，0=关闭 */
    uint8_t  reserved1[3];         /** 尾部对齐占位，全部固定为0 */
    uint8_t  protect_enable;       /** 保护功能位：温度/电流/堵转/电压 */
    uint8_t  voltage_min_dv;       /** 最低工作电压，0.1V */
    uint8_t  voltage_max_dv;       /** 最高工作电压，0.1V */
    uint16_t min_start_torque_ckgcm; /** 最小启动扭矩，0.01 kgf·cm；0=关闭门槛 */
    uint32_t image_tag;            /** 写入这份参数的固件镜像指纹；与当前固件不同=刚换过固件，见 A_Config_Init */
} Config_t;

/* SRAM 实时参数。前半部分由 Flash 参数在上电时复制，运行中可单独修改且不落盘；
 * torque_mode/damping_percent 只有 SRAM 值，掉电恢复为 1/100。 */
typedef struct {
    uint16_t torque_limit_ckgcm;   /* 扭矩上限，0.01 kgf·cm */
    uint32_t traj_amax_dps2;
    uint32_t traj_dmax_dps2;
    uint32_t traj_jmax_dps3;
    uint16_t traj_vmax_dps;
    uint16_t traj_move_min_ms;
    uint16_t prot_current_limit_ma;
    uint16_t prot_current_time_ms;
    uint16_t prot_temp_limit_dC;
    uint16_t prot_temp_time_ms;
    uint16_t prot_stall_ref_cdeg;
    uint16_t prot_stall_act_cdeg;
    uint16_t prot_stall_trip;
    uint8_t  hold_stiff;
    uint8_t  protect_enable;
    uint8_t  voltage_min_dv;
    uint8_t  voltage_max_dv;
    uint16_t min_start_torque_ckgcm; /* 最小启动扭矩，0.01 kgf·cm */
    uint8_t  torque_mode;          /** 0=自由释放，1=正常，2=自定义阻尼 */
    uint8_t  damping_percent;      /** 自定义阻尼占空比，0~100 */
} RuntimeConfig_t;

extern Config_t        g_config;         /** Flash 参数实例，修改后须 MarkDirty */
extern RuntimeConfig_t g_runtime_config; /** SRAM 实时参数实例，不自动保存 */

uint16_t A_Config_TorqueMax(void); /* 硬件扭矩上限，0.01 kgf·cm；无有效硬件页时按FW_TORQUE */
void     A_Config_Init(void);                             /* 上电加载；同一固件非法字段单独修复，换过固件或坏页回默认并保留ID与波特率 */
void     A_Config_Default(uint8_t keep_id);               /* 恢复默认，keep_id=1保留ID */
void     A_Config_MarkDirty(void);                        /* 登记改动，唤醒保存任务，不在指令路径写Flash */
void     A_Config_SaveTask(void);                         /* 最低优先级任务：参数变脏时落盘 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定保存任务的run标志 */
uint32_t A_Config_Baudrate(void);                         /* baud_code换算为实际波特率 */
void     A_Config_ReloadRuntime(uint8_t reset_transient); /* Flash参数复制到SRAM；非0同时恢复运行模式默认值 */
