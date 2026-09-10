#pragma once

#include <stdint.h>

/* 控制设计参数：标定输入、调参面板、派生系数和编译期护栏。 */
#define CDEG_RANGE 36000  /** 位置环绕量程，整圈=36000厘度。 */
#define SERVO_PWM_MIN 500  /** 舵机协议最小脉宽，也是电机模式负向满速指令。 */
#define SERVO_PWM_MAX 2500 /** 舵机协议最大脉宽，也是电机模式正向满速指令。 */
#define SERVO_PWM_MID 1500 /** 电机模式零速指令；两侧分别线性映射到正负满PWM。 */
#define CFG_PWM_FULL 2400 /** PWM满量程，必须与驱动层实际限制保持一致。 */

/* 标定输入。A_Calib_Min 的输出替换此处的单机标定项。 */
#define CAL_SPEED_SLOPE_Q16 7434 /** 实测PWM-速度斜率，Q16；标定输出直接填入。 */
#define CAL_FRICTION_PWM 78      /** 实测等效动摩擦PWM，标定输出直接填入。 */
#define MODEL_TAU_MS 37          /** 实测机械时间常数(ms)；偏小会显著降低稳定裕度。 */
#define MODEL_DELAY_TICKS 3      /** 指令至轴响应的纯延迟(控制拍)；改变后需复核全部护栏。 */
#define CAL_MOTOR_SIGN 1         /** 装配方向，+1/-1；由标定程序自动判定。 */
#define MODEL_BRAKE_GAIN_Q8 256  /** 制动侧相对驱动侧的PWM增益，Q8；须通过实机制动测试确定。 */
#define MODEL_RESONANCE_HZ 0     /** 第一机械谐振频率(Hz)，0=未测；非0时自动收紧观测器带宽。 */

/* 调参面板。 */
#define TUNE_SMOOTH_ACC 0       /** 加速平滑度，0~100；增大可降低加速段 jerk，但会增加反向响应时间。 */
#define TUNE_SMOOTH_DEC 25      /** 减速平滑度，0~100；增大可降低到位时 PWM 跳变和冲击，但会延长减速过程。 */
#define TUNE_MOVE_MIN_MS 70     /** 小位移最短运动时间(ms)，0=关闭；增大使短距离动作更柔和、更慢。 */
#define TUNE_ACCEL_MIN_MS 30    /** 小位移加速段最短时间(ms)；增大可压低小位移 PWM 尖峰，且不得短于纯延迟。 */
#define TUNE_SPEED_PCT 91       /** 最大速度占物理稳态速度的百分比；增大更快但闭环可用 PWM 余量更少，最大100。 */
#define TUNE_ACCEL_PCT 95       /** 最大加速度占物理能力的百分比；增大更快但模型误差和饱和风险更高，最大100。 */
#define TUNE_STIFFNESS_RADS 19  /** 位置环带宽(rad/s)；增大跟随更快，但受速度环/观测器级联分离约束。 */
#define TUNE_DAMPING 24         /** 速度环带宽与位置环带宽之比的10倍；增大速度环更快，范围须满足护栏。 */
#define TUNE_INTEGRAL_RADS 15   /** 速度积分零点(rad/s)；增大可更快消除稳态误差，但不得高于位置环带宽。 */
#define TUNE_OBS_BW_HZ 22       /** 观测器带宽(Hz)；增大响应更快但受纯延迟和机械谐振上限约束。 */
#define TUNE_RESOLUTION_CDEG 10 /** 最小可靠位移(厘度)；增大将同步增大死区、到位窗和零速阈值。 */

/* 结构常数及由调参面板导出的设计量。 */
#define DSGC_HOLD_DB_X2 3                                                                  /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_IN_WIN_X2 5                                                                   /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_OUT_WIN_X 4                                                                   /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_IN_VEL_X 50                                                                   /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_WCMD_MIN_DIV 4                                                                /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_FRIC_BLEND_X 3                                                                /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_DEC_OF_ACC_Q8 189                                                             /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSG_SMOOTH_MAX 100                                                                 /** 平滑度取值上限。 */
#define DSG_IN_HOLD_MS 20                                                                  /** 进入到位状态前需连续满足条件的控制拍数。 */
#define DSG_WCORR_PCT 25                                                                   /** 位置环速度修正限幅占最大速度的百分比。 */
#define DSG_POS_BW_RADS TUNE_STIFFNESS_RADS                                                /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_VEL_BW_RADS (TUNE_STIFFNESS_RADS * TUNE_DAMPING / 10)                          /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_VEL_I_RADS TUNE_INTEGRAL_RADS                                                  /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_OBS_BW_HZ TUNE_OBS_BW_HZ                                                       /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_VMAX_PCT TUNE_SPEED_PCT                                                        /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_ACC_RATIO_Q8 (TUNE_ACCEL_PCT * 256 / 100)                                      /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_DEC_RATIO_Q8 ((DSG_ACC_RATIO_Q8 * DSGC_DEC_OF_ACC_Q8) >> 8)                    /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define TRAJ_ACCEL_MIN_MS TUNE_ACCEL_MIN_MS                                                /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define DSG_POS_DEADBAND_CDEG TUNE_RESOLUTION_CDEG                                         /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_HOLD_DEADBAND_CDEG (TUNE_RESOLUTION_CDEG * DSGC_HOLD_DB_X2 / 2)                /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_IN_WIN_CDEG (TUNE_RESOLUTION_CDEG * DSGC_IN_WIN_X2 / 2)                        /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_OUT_WIN_CDEG (TUNE_RESOLUTION_CDEG * DSGC_OUT_WIN_X)                           /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_IN_VEL_CDPS (TUNE_RESOLUTION_CDEG * DSGC_IN_VEL_X)                             /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_WCMD_MIN_CDPS (TUNE_STIFFNESS_RADS * TUNE_RESOLUTION_CDEG / DSGC_WCMD_MIN_DIV) /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_FRICTION_BLEND_CDPS (DSG_WCMD_MIN_CDPS * DSGC_FRIC_BLEND_X)                    /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */

/* 编译期派生系数。 */
#define CFG_TICK_HZ 1000                                                                                                                                      /** 控制任务频率(Hz)。 */
#define CFG_Q24_ONE 16777216L                                                                                                                                 /** Q24定点格式中的1.0。 */
#define CFG_Q24_MUL(a, b) ((int32_t)(((int64_t)(a) * (int64_t)(b)) >> 24))                                                                                    /** Q24定点乘法常量表达式。 */
#define PLANT_NET_PWM (((CFG_PWM_FULL - CAL_FRICTION_PWM) > 0) ? (CFG_PWM_FULL - CAL_FRICTION_PWM) : 1)                                                       /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_VSS_CDPS ((int32_t)(((int64_t)PLANT_NET_PWM << 16) / CAL_SPEED_SLOPE_Q16))                                                                      /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_A0_CDPSS ((int32_t)(((int64_t)PLANT_VSS_CDPS * 1000) / MODEL_TAU_MS))                                                                           /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_KA_Q8_RAW ((int32_t)(((int64_t)65536 * 1000 * 256) / ((int64_t)CAL_SPEED_SLOPE_Q16 * MODEL_TAU_MS)))                                            /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_KA_Q8 ((PLANT_KA_Q8_RAW > 0) ? PLANT_KA_Q8_RAW : 1)                                                                                             /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_KA_CDPSS (((PLANT_KA_Q8 >> 8) > 0) ? (PLANT_KA_Q8 >> 8) : 1)                                                                                    /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_SPEED_SLOPE_Q16 CAL_SPEED_SLOPE_Q16                                                                                                             /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_FRICTION_PWM CAL_FRICTION_PWM                                                                                                                   /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_TAU_MS MODEL_TAU_MS                                                                                                                             /** 由标定输入推导的被控对象参数，禁止手动修改。 */
#define PLANT_DEAD_TICKS MODEL_DELAY_TICKS                                                                                                                    /** 由标定输入推导的被控对象参数，禁止手动修改。 */

#define OBSG_X_Q24 ((int32_t)((105414357LL * DSG_OBS_BW_HZ) / CFG_TICK_HZ))                                                                                   /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_E1 OBSG_X_Q24                                                                                                                                    /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_E2 (CFG_Q24_MUL (OBSG_E1, OBSG_X_Q24) / 2)                                                                                                       /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_E3 (CFG_Q24_MUL (OBSG_E2, OBSG_X_Q24) / 3)                                                                                                       /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_E4 (CFG_Q24_MUL (OBSG_E3, OBSG_X_Q24) / 4)                                                                                                       /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_E5 (CFG_Q24_MUL (OBSG_E4, OBSG_X_Q24) / 5)                                                                                                       /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_E6 (CFG_Q24_MUL (OBSG_E5, OBSG_X_Q24) / 6)                                                                                                       /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_ALPHA (CFG_Q24_ONE - OBSG_E1 + OBSG_E2 - OBSG_E3 + OBSG_E4 - OBSG_E5 + OBSG_E6)                                                                  /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_BETA (CFG_Q24_ONE - OBSG_ALPHA)                                                                                                                  /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_A2 CFG_Q24_MUL (OBSG_ALPHA, OBSG_ALPHA)                                                                                                          /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_A3 CFG_Q24_MUL (OBSG_A2, OBSG_ALPHA)                                                                                                             /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_B2 CFG_Q24_MUL (OBSG_BETA, OBSG_BETA)                                                                                                            /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_B3 CFG_Q24_MUL (OBSG_B2, OBSG_BETA)                                                                                                              /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_A_Q24 ((MODEL_TAU_MS > 0) ? (CFG_Q24_ONE / MODEL_TAU_MS) : (CFG_Q24_ONE / 36))                                                                   /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_L1_NUM (((CFG_Q24_ONE - OBSG_A3 - OBSG_A_Q24) > 0) ? (CFG_Q24_ONE - OBSG_A3 - OBSG_A_Q24) : 0)                                                   /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_L1_DEN (((CFG_Q24_ONE - OBSG_A_Q24) > 1) ? (CFG_Q24_ONE - OBSG_A_Q24) : 1)                                                                       /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_L1_RAW ((int32_t)(((int64_t)OBSG_L1_NUM << 24) / OBSG_L1_DEN))                                                                                   /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBSG_L1_Q24 ((OBSG_L1_RAW > CFG_Q24_ONE) ? (int32_t)CFG_Q24_ONE : OBSG_L1_RAW)                                                                        /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBS_L1_Q15 (OBSG_L1_Q24 >> 9)                                                                                                                         /** 观测器增益、限幅或运行阈值，均由标定/调参派生。 */
#define OBSG_L2_T ((3 * OBSG_B2 - OBSG_B3 - CFG_Q24_MUL (OBSG_L1_Q24, OBSG_A_Q24)) > 0 ? (3 * OBSG_B2 - OBSG_B3 - CFG_Q24_MUL (OBSG_L1_Q24, OBSG_A_Q24)) : 0) /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBS_L2_Q15 ((int32_t)(((int64_t)OBSG_L2_T * CFG_TICK_HZ) >> 9))                                                                                       /** 观测器增益、限幅或运行阈值，均由标定/调参派生。 */
#define OBSG_L3_NUM ((((int64_t)OBSG_B3 * CFG_TICK_HZ) * CFG_TICK_HZ) << 15)                                                                                  /** 观测器离散极点及增益推导中间量，禁止手动修改。 */
#define OBS_L3_Q15 (-(int32_t)((((OBSG_L3_NUM / PLANT_KA_Q8) * 256) >> 24)))                                                                                  /** 观测器增益、限幅或运行阈值，均由标定/调参派生。 */
#define OBS_LOAD_MAX (CFG_PWM_FULL / 3)                                                                                                                       /** 观测器增益、限幅或运行阈值，均由标定/调参派生。 */
#define OBS_VMAX_CDPS (PLANT_VSS_CDPS + (PLANT_VSS_CDPS >> 3))                                                                                                /** 观测器增益、限幅或运行阈值，均由标定/调参派生。 */
#define OBS_FRIC_INV_Q15 ((int32_t)((1L << 23) / DSG_FRICTION_BLEND_CDPS))                                                                                    /** 观测器增益、限幅或运行阈值，均由标定/调参派生。 */

#define TRAJ_VMAX_CDPS ((int32_t)(((int64_t)PLANT_VSS_CDPS * DSG_VMAX_PCT) / 100))                                                                            /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define TRAJ_AMAX_ACC_RAW ((int32_t)(((int64_t)PLANT_A0_CDPSS * DSG_ACC_RATIO_Q8) >> 8))                                                                      /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define TRAJ_AMAX_DEC_RAW ((int32_t)(((int64_t)PLANT_A0_CDPSS * DSG_DEC_RATIO_Q8) >> 8))                                                                      /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define TRAJ_AMAX_ACC_CDPSS ((TRAJ_AMAX_ACC_RAW > 0) ? TRAJ_AMAX_ACC_RAW : 1)                                                                                 /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define TRAJ_AMAX_DEC_CDPSS ((TRAJ_AMAX_DEC_RAW > 0) ? TRAJ_AMAX_DEC_RAW : 1)                                                                                 /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define TRAJ_SMOOTH_MAX DSG_SMOOTH_MAX                                                                                                                        /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */
#define TRAJ_MOVE_MIN_MS TUNE_MOVE_MIN_MS                                                                                                                     /** 轨迹规划器的速度、加速度或时间限制，由设计量派生。 */

#define CTRL_KP_POS_Q8 (DSG_POS_BW_RADS * 256)                                                                                                                /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KP_VEL_RAW ((int32_t)(((int64_t)DSG_VEL_BW_RADS * 65536 * 256) / PLANT_KA_Q8))                                                                   /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KP_VEL_Q16 ((CTRL_KP_VEL_RAW > 0) ? CTRL_KP_VEL_RAW : 1)                                                                                         /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KP_VEL_BRAKE_Q16 ((int32_t)(((int64_t)CTRL_KP_VEL_Q16 * MODEL_BRAKE_GAIN_Q8) >> 8))                                                              /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KI_VEL_RAW ((int32_t)(((int64_t)DSG_VEL_I_RADS * CTRL_KP_VEL_Q16 * CFG_TICK_HZ) / 1000000))                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KI_VEL_Q16 ((CTRL_KI_VEL_RAW > 0) ? CTRL_KI_VEL_RAW : 1)                                                                                         /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_INTEGRAL_MAX (CFG_PWM_FULL / 3)                                                                                                                  /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_INTEGRAL_ACC_GATE (PLANT_A0_CDPSS / 19)                                                                                                          /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KVFF_Q16 CAL_SPEED_SLOPE_Q16                                                                                                                     /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KA_Q20 ((int32_t)((1048576LL * 256) / PLANT_KA_Q8))                                                                                              /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_KA_BRAKE_Q20 ((int32_t)(((int64_t)CTRL_KA_Q20 * MODEL_BRAKE_GAIN_Q8) >> 8))                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_FRIC_DYN CAL_FRICTION_PWM                                                                                                                        /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_WCMD_MIN_CDPS DSG_WCMD_MIN_CDPS                                                                                                                  /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_MOTOR_SIGN CAL_MOTOR_SIGN                                                                                                                        /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_VMAX_CDPS TRAJ_VMAX_CDPS                                                                                                                         /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_WCORR_MAX ((int32_t)(((int64_t)TRAJ_VMAX_CDPS * DSG_WCORR_PCT) / 100))                                                                           /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_PREVIEW_Q16 ((int32_t)(((int64_t)MODEL_DELAY_TICKS * 65536) / CFG_TICK_HZ))                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_POS_LEAD_Q16 (65536 / CFG_TICK_HZ)                                                                                                               /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_POS_DEADBAND DSG_POS_DEADBAND_CDEG                                                                                                               /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_HOLD_DEADBAND DSG_HOLD_DEADBAND_CDEG                                                                                                             /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_IN_WIN_CDEG DSG_IN_WIN_CDEG                                                                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_OUT_WIN_CDEG DSG_OUT_WIN_CDEG                                                                                                                    /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_IN_VEL_CDPS DSG_IN_VEL_CDPS                                                                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_IN_HOLD_MS DSG_IN_HOLD_MS                                                                                                                        /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_PWM_LIMIT CFG_PWM_FULL                                                                                                                           /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */

/* 编译期护栏。 */
#define GUARD_2L_TICKS (2 * MODEL_DELAY_TICKS + 1)                                                                                                       /** 编译期安全护栏的中间上限，禁止手动修改。 */
#define GUARD_OBS_BW_DELAY ((int)((CFG_TICK_HZ * 10000L) / (62832L * GUARD_2L_TICKS)))                                                                   /** 编译期安全护栏的中间上限，禁止手动修改。 */
#define GUARD_OBS_BW_MAX (((MODEL_RESONANCE_HZ > 0) && ((MODEL_RESONANCE_HZ / 3) < GUARD_OBS_BW_DELAY)) ? (MODEL_RESONANCE_HZ / 3) : GUARD_OBS_BW_DELAY) /** 编译期安全护栏的中间上限，禁止手动修改。 */
typedef char guard_obs_bw_exceeds_delay_margin[(TUNE_OBS_BW_HZ <= GUARD_OBS_BW_MAX) ? 1 : -1];
typedef char guard_observer_not_3x_faster_than_velocity_loop[(((long)DSG_VEL_BW_RADS * 30000L) <= (62832L * TUNE_OBS_BW_HZ)) ? 1 : -1];
typedef char guard_velocity_loop_not_2x_faster_than_position_loop[(TUNE_DAMPING >= 20) ? 1 : -1];
typedef char guard_integral_faster_than_position_loop[(TUNE_INTEGRAL_RADS <= TUNE_STIFFNESS_RADS) ? 1 : -1];
typedef char guard_hysteresis_window_inverted[(DSG_OUT_WIN_CDEG > DSG_IN_WIN_CDEG) ? 1 : -1];
typedef char guard_deadband_wider_than_in_window[(DSG_POS_DEADBAND_CDEG < DSG_IN_WIN_CDEG) ? 1 : -1];
typedef char guard_smooth_acc_out_of_range[((TUNE_SMOOTH_ACC >= 0) && (TUNE_SMOOTH_ACC <= DSG_SMOOTH_MAX)) ? 1 : -1];
typedef char guard_smooth_dec_out_of_range[((TUNE_SMOOTH_DEC >= 0) && (TUNE_SMOOTH_DEC <= DSG_SMOOTH_MAX)) ? 1 : -1];
typedef char guard_speed_pct_out_of_range[((TUNE_SPEED_PCT > 0) && (TUNE_SPEED_PCT <= 100)) ? 1 : -1];
typedef char guard_accel_pct_out_of_range[((TUNE_ACCEL_PCT > 0) && (TUNE_ACCEL_PCT <= 100)) ? 1 : -1];
typedef char guard_accel_min_ms_below_pure_delay[((TUNE_ACCEL_MIN_MS * 2) >= (2 * MODEL_DELAY_TICKS + 1)) ? 1 : -1];
typedef char guard_move_min_ms_out_of_range[((TUNE_MOVE_MIN_MS >= 0) && (TUNE_MOVE_MIN_MS <= 1000)) ? 1 : -1];
typedef char guard_resolution_too_small[(TUNE_RESOLUTION_CDEG >= 1) ? 1 : -1];
typedef char guard_speed_slope_invalid[(CAL_SPEED_SLOPE_Q16 > 0) ? 1 : -1];
typedef char guard_tau_invalid[(MODEL_TAU_MS > 0) ? 1 : -1];
typedef char guard_friction_exceeds_full[(CAL_FRICTION_PWM < CFG_PWM_FULL) ? 1 : -1];
typedef char guard_motor_sign_invalid[((CAL_MOTOR_SIGN == 1) || (CAL_MOTOR_SIGN == -1)) ? 1 : -1];
