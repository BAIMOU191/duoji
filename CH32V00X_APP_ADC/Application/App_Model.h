#pragma once

/* App_Model.h —— 随工程(编码器类型)与型号变化的全部数据，A_Config.h 两个工程相同、只引用这里。
 *   1. 型号选择
 *   2. 编码器相关常量(本工程所有型号共用)
 *   3. 型号表：四参数(写入固件信息块，升级时与舵机硬件信息页比对，0xFF=不限)、电机标定、电位器标定、工作电压范围
 * 型号宏名 = 四个型号参数：T{舵机类型}_H{硬件版本}_V{电压型号}_Q{扭力型号}。
 * 电机标定由 A_Calib_Min 标定固件实测打印，换电机或减速箱后重新标定并按型号整节替换。 */

/* ==================== 1. 型号选择(只启用一个) ==================== */
/* 型号编号：仅用于 #if 选择比较，不承载参数 */
#define APP_MODEL_T1_H10_V8_Q15     1   /* 电位器 8V 15kg */
#define APP_MODEL_T1_H10_V8_Q20     2   /* 电位器 8V 20kg */
#define APP_MODEL_T1_H10_V8_Q30     3   /* 电位器 8V 30kg */
#define APP_MODEL_T1_H10_V12_Q35    4   /* 电位器 12V 35kg */
#define APP_MODEL_T3_H10_V8_Q15     5   /* 电位器 8V 15kg */

#define APP_MODEL  APP_MODEL_T3_H10_V8_Q15


/* ==================== 2. 编码器相关常量(本工程共用) ====================
 * 电位器接ADC：带符号线性行程(不回绕)，抽头脱离碳膜为死区，悬空时下拉钳地，据此判定死区并开环脱困。 */
#define ENCODER_IS_CIRCULAR          0   /** 1=反馈坐标整圈回绕：支持360度位置模式与多圈运动 */
#define ENCODER_HAS_DEADZONE         1   /** 1=有测不到角度的区间(读失败时开环脱困)，0=读失败即故障 */
#define ENCODER_POT_MARGIN_CDEG   1000   /** 标定行程两端各外扩的可读角度(厘度)，只兜端点超调和脱困 */
#define ENCODER_POT_ADC_FLOAT     100U   /** 低于它=抽头悬空=在死区里(整数计数) */
/* 控制器整定里随编码器变化的部分(A_Config.h 第4节引用)：带宽越高电位器噪声放大越多 */
#define ENCODER_TUNE_OBS_BW_HZ      12   /** 观测器位置/速度极点(Hz) */
#define ENCODER_TUNE_VEL_BW_CAP     70U  /** 速度环带宽上限(rad/s)，另受纯延迟限制 */
#define ENCODER_DEF_RESOLUTION_CDEG 50U  /** 到位精度出厂值(厘度)：电位器噪声大，取得比磁编码宽 */


/* ==================== 3. 型号表 ==================== */

#if APP_MODEL == APP_MODEL_T1_H10_V8_Q15
/* ==================== T1 电位器 8V 15kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     15U       /** 扭力型号 15kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV              7640   /* 标定电压，mV */
#define MOTOR_KV_UV                250   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               6829   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                313   /* 动摩擦，mV */
#define MOTOR_FS_MV                546   /* 起转电压，mV */
#define MOTOR_DELAY_US            3206   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             6   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          9   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        260   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      8000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      8000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */

#elif APP_MODEL == APP_MODEL_T1_H10_V8_Q20
/* ==================== T1 电位器 8V 20kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     20U       /** 扭力型号 20kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
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

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */

#elif APP_MODEL == APP_MODEL_T1_H10_V8_Q30
/* ==================== T1 电位器 8V 30kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     30U       /** 扭力型号 30kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV              7666   /* 标定电压，mV */
#define MOTOR_KV_UV                397   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               7063   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                127   /* 动摩擦，mV */
#define MOTOR_FS_MV                413   /* 起转电压，mV */
#define MOTOR_DELAY_US            3211   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             6   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         13   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        162   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      7500   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      7500   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */

#elif APP_MODEL == APP_MODEL_T1_H10_V12_Q35
/* ==================== T1 电位器 12V 35kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    12U       /** 电压型号 12V */
#define FW_TORQUE     35U       /** 扭力型号 35kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV             12101   /* 标定电压，mV */
#define MOTOR_KV_UV                468   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               7364   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                532   /* 动摩擦，mV */
#define MOTOR_FS_MV                750   /* 起转电压，mV */
#define MOTOR_DELAY_US            2395   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             4   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         11   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        215   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2     12000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2     12000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      90U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV     130U  /** 用户可设的工作电压上限(也是出厂值) */


#elif APP_MODEL == APP_MODEL_T3_H10_V8_Q15
/* ==================== T3 电位器 8V 15kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 3U        /** 舵机类型：3=电位器ADC新版(机械结构与T1不同) */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     15U       /** 扭力型号 15kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV              7626   /* 标定电压，mV */
#define MOTOR_KV_UV                221   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               5092   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                 78   /* 动摩擦，mV */
#define MOTOR_FS_MV                162   /* 起转电压，mV */
#define MOTOR_DELAY_US            3673   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         10   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                  -1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        280   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      9000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      9000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         400U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3700U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */


#else
#error "App_Model.h：APP_MODEL 未选择有效型号"
#endif
