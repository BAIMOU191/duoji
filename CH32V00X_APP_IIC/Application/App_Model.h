#pragma once

/* App_Model.h —— 随工程(编码器类型)与型号变化的全部数据，A_Config.h 两个工程相同、只引用这里。
 *   1. 型号选择
 *   2. 编码器相关常量(本工程所有型号共用)
 *   3. 型号表：四参数(写入固件信息块，升级时与舵机硬件信息页比对，0xFF=不限)、电机标定、工作电压范围
 * 型号宏名 = 四个型号参数：T{舵机类型}_H{硬件版本}_V{电压型号}_Q{扭力型号}。
 * 电机标定由 A_Calib_Min 标定固件实测打印，换电机或减速箱后重新标定并按型号整节替换。 */

/* ==================== 1. 型号选择(只启用一个) ==================== */
#define APP_MODEL_T2_H10_V8_Q20     1   /* 磁编码 MT6701 8V 20kg */
#define APP_MODEL_T4_H10_V8_Q15     2   /* 磁编码 MT6701 8V 15kg */


#define APP_MODEL  APP_MODEL_T2_H10_V8_Q20


/* ==================== 2. 编码器相关常量(本工程共用) ====================
 * MT6701 14位磁编码器(I2C)：整圈绝对角度、整圈可测、无死区；读失败即器件故障，停机卸力。 */
#define ENCODER_IS_CIRCULAR          1   /** 1=反馈坐标整圈回绕：支持360度位置模式与多圈运动 */
#define ENCODER_HAS_DEADZONE         0   /** 1=有测不到角度的区间(读失败时开环脱困)，0=读失败即故障 */
/* 控制器整定里随编码器变化的部分(A_Config.h 第4节引用)：
 * 磁编码噪声不是瓶颈，限制带宽的是输出轴齿隙(轻载时越易在齿隙里抖) */
#define ENCODER_TUNE_OBS_BW_HZ      16   /** 观测器位置/速度极点(Hz) */
#define ENCODER_TUNE_VEL_BW_CAP     60U  /** 速度环带宽上限(rad/s)，另受纯延迟限制：再高时负载力矩过零段(齿轮不受力)速度环会在齿隙里自激 */
#define ENCODER_TUNE_POS_BW_RADS     5U  /** 位置环带宽 ω_p(rad/s)：按负载惯量比约2的串级上限 ω_v/(1+2)/4 取整 */
#define ENCODER_DEF_RESOLUTION_CDEG 50U  /** 到位精度出厂值(厘度)：不小于编码器噪声与最小可靠位移决定的下限 */


/* ==================== 3. 型号表 ==================== */

#if APP_MODEL == APP_MODEL_T2_H10_V8_Q20
/* ==================== T2 磁编码 8V 20kg ==================== */
#define FW_SERVO_TYPE 2U        /** 舵机类型：2=MT6701磁编码版 */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     20U       /** 扭力型号 20kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV              7700   /* 标定电压，mV */
#define MOTOR_KV_UV                361   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               9614   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                200   /* 动摩擦，mV */
#define MOTOR_FS_MV                523   /* 起转电压，mV */
#define MOTOR_DELAY_US             358   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             2   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG          9   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                   1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        183   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      6000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      6000   /* 额定低速减速度，度/秒² */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */

#elif APP_MODEL == APP_MODEL_T4_H10_V8_Q15
/* ==================== T4 磁编码 8V 15kg ==================== */
#define FW_SERVO_TYPE 4U        /** 舵机类型：4=MT6701磁编码版(飞特结构) */
#define FW_HW_VERSION 10U       /** 硬件版本 v1.0 */
#define FW_VOLTAGE    8U        /** 电压型号 8V */
#define FW_TORQUE     15U       /** 扭力型号 15kg；仅作型号标识，不参与扭矩换算 */

/* ---- 电机标定（标定固件打印，整节替换） ----
 * 电压域模型 U = kv·ω + ka·α + fc·sgn(ω)，静止且 |U| < fs 时粘住；参数与母线电压无关，
 * 运行时电压按实测母线换算成占空比。额定能力为标定电压下的值(已留10%余量)，只用于配置校验与出厂默认值。 */
#define MOTOR_CAL_MV              7700   /* 标定电压，mV */
#define MOTOR_KV_UV                326   /* 反电势斜率，µV/(厘度/s) */
#define MOTOR_KA_NV               9351   /* 加速系数，nV/(厘度/s²) */
#define MOTOR_FC_MV                200   /* 动摩擦，mV */
#define MOTOR_FS_MV                522   /* 起转电压，mV */
#define MOTOR_DELAY_US            1000   /* 输入到响应的纯延迟，µs */
#define MOTOR_NOISE_CDEG             1   /* 静止读数噪声幅值(±)，厘度 */
#define MOTOR_MIN_STEP_CDEG         10   /* 最小可靠位移，厘度 */
#define MOTOR_SIGN                  -1   /* 正电压使输出轴角度增大 = 1 */
#define MOTOR_RATED_VEL_DPS        200   /* 额定最高速度，度/秒 */
#define MOTOR_RATED_ACC_DPS2      7000   /* 额定零速加速度，度/秒² */
#define MOTOR_RATED_DEC_DPS2      7000   /* 额定低速减速度，度/秒² */

/* ---- 工作电压范围（0.1V） ---- */
#define PROT_VOLTAGE_CFG_MIN_DV      60U  /** 用户可设的工作电压下限(也是出厂值) */
#define PROT_VOLTAGE_CFG_MAX_DV      90U  /** 用户可设的工作电压上限(也是出厂值) */

#else
#error "App_Model.h：APP_MODEL 未选择有效型号"
#endif
