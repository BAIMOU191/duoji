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
#define APP_MODEL_T1_H10_V12_Q20    4   /* 电位器 12V 20kg */
#define APP_MODEL_T1_H10_V12_Q25    5   /* 电位器 12V 25kg */
#define APP_MODEL_T1_H10_V12_Q35    6   /* 电位器 12V 35kg */
#define APP_MODEL_T1_H10_V12_Q45    7   /* 电位器 12V 45kg */
#define APP_MODEL_T3_H10_V8_Q15     8   /* 电位器 8V 15kg */

#define APP_MODEL  APP_MODEL_T1_H10_V12_Q35


/* ==================== 2. 编码器相关常量(本工程共用) ====================
 * 电位器接ADC：带符号线性行程(不回绕)，抽头脱离碳膜为死区，悬空时下拉钳地，据此判定死区并开环脱困。 */
#define ENCODER_IS_CIRCULAR          0   /** 1=反馈坐标整圈回绕：支持360度位置模式与多圈运动 */
#define ENCODER_HAS_DEADZONE         1   /** 1=有测不到角度的区间(读失败时开环脱困)，0=读失败即故障 */
#define ENCODER_POT_MARGIN_CDEG   1000   /** 标定行程两端各外扩的可读角度(厘度)，只兜端点超调和脱困 */
#define ENCODER_POT_ADC_FLOAT     100U   /** 低于它=抽头悬空=在死区里(整数计数) */
/* 控制器整定里随编码器变化的部分(A_Config.h 第4节引用)：带宽越高电位器噪声放大越多 */
#define ENCODER_TUNE_OBS_BW_HZ      12   /** 观测器位置/速度极点(Hz) */
#define ENCODER_TUNE_VEL_BW_CAP     60U  /** 速度环带宽上限(rad/s)，另受纯延迟限制 */
/* 位置环带宽 ω_p(rad/s)：按负载惯量比约2的串级上限 (Kv+kv)/(3·ka)/4 取整；12V 35/45kg 反电势相对惯量大、速度环阻尼强，可取高些 */
#if APP_MODEL == APP_MODEL_T1_H10_V12_Q35
#define ENCODER_TUNE_POS_BW_RADS     7U
#elif APP_MODEL == APP_MODEL_T1_H10_V12_Q45
#define ENCODER_TUNE_POS_BW_RADS     6U
#else
#define ENCODER_TUNE_POS_BW_RADS     5U
#endif
#define ENCODER_DEF_RESOLUTION_CDEG 50U  /** 到位精度出厂值(厘度)：不小于编码器噪声与最小可靠位移决定的下限 */


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
#define MOTOR_CAL_MV              7638   /* 标定电压，mV */
#define MOTOR_KV_UV                249   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               7126   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                240   /* 动摩擦，mV */
#define MOTOR_FS_MV                556   /* 起转电压，mV */
#define MOTOR_DELAY_US            2259   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          9   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        230   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      7500   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      7500   /* 额定低速减速度，度/秒² */

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
#define MOTOR_CAL_MV              7695   /* 标定电压，mV */
#define MOTOR_KV_UV                339   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               9839   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                129   /* 动摩擦，mV */
#define MOTOR_FS_MV                486   /* 起转电压，mV */
#define MOTOR_DELAY_US            2733   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5  /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         10   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        181   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      5562   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      5751   /* 额定低速减速度，度/秒² */

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
#define MOTOR_CAL_MV              7697   /* 标定电压，mV */
#define MOTOR_KV_UV                400   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               8051   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                80   /* 动摩擦，mV */
#define MOTOR_FS_MV                457   /* 起转电压，mV */
#define MOTOR_DELAY_US            3000   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         18   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        148   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      7000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      7000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */

#elif APP_MODEL == APP_MODEL_T1_H10_V12_Q20
/* ==================== T1 电位器 12V 20kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    12U       /** 电压型号 12V */
#define FW_TORQUE     20U       /** 扭力型号 20kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV             12000   /* 标定电压，mV */
#define MOTOR_KV_UV                355   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               9486   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                950   /* 动摩擦，mV */
#define MOTOR_FS_MV               1300   /* 起转电压，mV */
#define MOTOR_DELAY_US            3000   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          8   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                  -1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        245   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      8000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      8000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      90U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV     130U  /** 用户可设的工作电压上限(也是出厂值) */


#elif APP_MODEL == APP_MODEL_T1_H10_V12_Q25
/* ==================== T1 电位器 12V 25kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    12U       /** 电压型号 12V */
#define FW_TORQUE     25U       /** 扭力型号 25kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV             11955   /* 标定电压，mV */
#define MOTOR_KV_UV                480   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV              13680   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                250   /* 动摩擦，mV */
#define MOTOR_FS_MV                600   /* 起转电压，mV */
#define MOTOR_DELAY_US            2899   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          8   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                  -1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        200   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      7000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      7000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      90U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV     130U  /** 用户可设的工作电压上限(也是出厂值) */


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
#define MOTOR_CAL_MV             12013   /* 标定电压，mV */
#define MOTOR_KV_UV                479   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               7142   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                328   /* 动摩擦，mV */
#define MOTOR_FS_MV                700   /* 起转电压，mV */
#define MOTOR_DELAY_US            2137   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             4   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         11   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                  -1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        205   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2     10000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2     10000   /* 额定低速减速度，度/秒² */

/* ---- 电位器标定（单台实测，整节替换） ---- */
#define ENCODER_POT_ADC_MIN         350U  /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX        3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG     27000U  /** 满行程角度(厘度)，270.00度 */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      90U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV     130U  /** 用户可设的工作电压上限(也是出厂值) */


#elif APP_MODEL == APP_MODEL_T1_H10_V12_Q45
/* ==================== T1 电位器 12V 35kg ==================== */
/* 固件适用硬件，写进固件信息块(System/iap.c)：升级时与舵机硬件信息页比对，0xFF=不限 */
#define FW_SERVO_TYPE 1U        /** 舵机类型：1=电位器ADC旧结构 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    12U       /** 电压型号 12V */
#define FW_TORQUE     45U       /** 扭力型号 45kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV             11920   /* 标定电压，mV */
#define MOTOR_KV_UV                466   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               7850   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                309   /* 动摩擦，mV */
#define MOTOR_FS_MV                720   /* 起转电压，mV */
#define MOTOR_DELAY_US            3005   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             5   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         12   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                  -1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        205   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2     10000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2     10000   /* 额定低速减速度，度/秒² */

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
#define MOTOR_KV_UV                230   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               6000   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                100   /* 动摩擦，mV */
#define MOTOR_FS_MV                240   /* 起转电压，mV */
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
