#pragma once

#include <stdint.h>

/*
 * A_Parameter.h —— 控制设计参数：标定输入、调参面板、派生系数、编译期护栏
 *
 * 只允许改"标定输入"和"调参面板"两段。DSGC_/DSG_/PLANT_/OBS_/TRAJ_/CTRL_
 * 全是派生量，手改会与文件末尾的护栏冲突。
 *
 * ==================== 本机实测结论(2026-09-09，电位器编码器) ====================
 *
 * 连跑三次的散布：slope 4752/4716/4682(极差1.5%)、截距 45/57/59、tau 50/52/50、
 * 纯延迟 1.55/1.98/2.52ms、制动增益 261/274/252(极差8%)、噪声底峰峰 8.4/8.9/9.4
 * 厘度、起转 dir0 241/82/231 与 dir1 177/192/172、最小可靠步长 31/19/32 厘度。
 *
 * 1) 起转方向不对称约30%(dir0 236 / dir1 180)，三次一致，是系统性的。取两方向
 *    较大值：过补只是末端多窜一点，位置环兜得住；补不足则末端爬不动没人救。
 *
 * 2) 没有占空比耦合。加力/撤力的读数偏移几乎相同(0.5~3.5厘度)，是缓慢漂移不是
 *    跟着PWM走的串扰。运动中观测速度剩下的噪声是真实机械纹波，不是测量问题。
 *
 * 3) 噪声是电位器抽头在碳膜上滑动的接触噪声：运动中位置sigma 5~6厘度(静止1.5)，
 *    与|PWM|无关(r=-0.057)，驱动层修不掉。能量分布 0~11Hz占0.6%、11~50Hz占8.3%、
 *    50Hz以上占91%；而对象转折频率只有 1/(2*pi*52ms)=3.06Hz。速度噪声底同时正比于
 *    观测器带宽和位置分辨率，实测 obs22Hz/res32 时静止峰值±326，反推出结构常数
 *    DSGC_VEL_NOISE_NUM/DEN=3/8。所以环路带宽取的是
 *    "够用的最低"而不是"稳定允许的最高"：位置环1.43Hz、速度环3.5Hz(≈对象转折)、
 *    观测器11Hz(把91%的高频噪声挡在环外)、积分8rad/s。级联分离护栏余量只剩4.5%
 *    (22*3=66 <= 2*pi*11=69.1)，再降观测器带宽必须同时降 TUNE_STIFFNESS_RADS。
 *
 * 4) 动摩擦(速度直线截距，57)和静摩擦(缓升法起转，235)差3~4倍，必须分开补。末端
 *    修正正好工作在静摩擦工况：只补动摩擦时缺的那一块靠速度环积分以0.5计数/拍顶，
 *    实测要爬600ms。控制器按实测速度在两者间滞环切换(见 C_Pos_Ctrl.c 的 PosCtrl_Friction)。
 *
 * 5) 制动增益 261≈256，说明这台慢衰减H桥的制动侧与驱动侧增益基本对称。早先测不出
 *    来是工作点选错：制动亏损量只与窗口长度有关，窗口必须整个落在轴停住之前。
 *
 * 6) 前馈把输出占满则闭环不存在。TUNE_SPEED_PCT=91 时巡航前馈吃掉2189/2400，只剩
 *    211给反馈，输出常驻饱和(实测"规划29854/实测15500")。现按噪声峰值偏移(±200)
 *    留250余量，由 guard_feedforward_leaves_no_headroom 在编译期挡住。
 *
 * 7) v_ss=325.6度/秒，而MT6701时代是204.7度/秒，比值1.6。同一台机构物理速度不该变，
 *    所以角度标度仍有嫌疑：量一下 P0500->P2500 的实际转角A，若明显小于266度，按
 *    ENCODER_POT_SPAN_CDEG = round(101.5*A) 改标度并同比缩放 slope。控制环不受影响
 *    (反馈和目标同标度)，但协议的"270度行程"会跟着错。
 */

/* ============================== 基本量程 ============================== */
#define CDEG_RANGE 36000        /** 整圈角度，厘度 */
#define CFG_WRAP_RANGE_CDEG 0   /** 反馈坐标回绕量程；0=线性行程不回绕，须与ENCODER_MODE一致 */
#define SERVO_PWM_MIN 500       /** 协议最小脉宽，也是电机模式负向满速指令 */
#define SERVO_PWM_MAX 2500      /** 协议最大脉宽，也是电机模式正向满速指令 */
#define SERVO_PWM_MID 1500      /** 电机模式零速指令，两侧线性映射到正负满PWM */
#define CFG_PWM_FULL 2400       /** PWM满量程，必须与驱动层实际限制一致 */

/* ======================= 标定输入(A_Calib_Min 输出) ======================= */
#define CAL_SPEED_SLOPE_Q16 4716 /** PWM-速度斜率，Q16 */
#define CAL_FRICTION_PWM 57      /** 等效动摩擦PWM，速度直线的截距 */
#define CAL_BREAKAWAY_PWM 235    /** 起转(静摩擦)PWM；<=动摩擦时静摩擦前馈自动退化 */
#define MODEL_TAU_MS 52          /** 机械时间常数(ms)，偏小会显著降低稳定裕度 */
#define MODEL_DELAY_TICKS 2      /** 指令至轴响应的纯延迟(控制拍)，改后需复核全部护栏 */
#define CAL_MOTOR_SIGN 1         /** 装配方向，+1/-1，由标定程序自动判定 */
#define MODEL_BRAKE_GAIN_Q8 261  /** 制动侧相对驱动侧的PWM增益，Q8 */
#define MODEL_RESONANCE_HZ 0     /** 第一机械谐振频率(Hz)，0=未测，非0时自动收紧观测器带宽 */

/* ============================== 调参面板 ============================== */
#define TUNE_SMOOTH_ACC 0       /** 加速平滑度0~100，增大降低加速段jerk但延长反向响应 */
#define TUNE_SMOOTH_DEC 60      /** 减速平滑度0~100，增大减小末端制动力突变 */
#define TUNE_MOVE_MIN_MS 70     /** 小位移最短运动时间(ms)，0=关闭 */
#define TUNE_ACCEL_MIN_MS 30    /** 小位移加速段最短时间(ms)，不得短于纯延迟 */
#define TUNE_SPEED_PCT 84       /** 最大速度占物理稳态速度的百分比，受前馈余量护栏约束 */
#define TUNE_ACCEL_PCT 95       /** 加速度百分比，仅在 Traj_AutoAmax 拿不到对象常数时兜底 */
#define TUNE_STIFFNESS_RADS 9   /** 位置环带宽(rad/s)，受速度环/观测器级联分离约束 */
#define TUNE_DAMPING 25         /** 速度环带宽与位置环带宽之比的10倍 */
#define TUNE_INTEGRAL_RADS 8    /** 速度积分零点(rad/s)，不得高于位置环带宽 */
#define TUNE_OBS_BW_HZ 11       /** 观测器带宽(Hz)，受纯延迟和机械谐振上限约束 */
#define TUNE_RESOLUTION_CDEG 32 /** 最小可靠位移(厘度)，是全部位置阈值的公共标度 */

/* ========================= 结构常数与设计量 ========================= */
#define DSGC_HOLD_DB_X2 3       /** 静止捕获窗=1.5倍最小步长，运动死区仍为1倍 */
#define DSGC_IN_WIN_X2 5        /** 到位窗=2.5倍最小步长 */
#define DSGC_OUT_WIN_X 4        /** 退出窗=4倍最小步长，与到位窗构成滞环 */
#define DSGC_IN_VEL_X 50        /** 到位零速阈值=50倍最小步长(每秒) */
#define DSGC_VEL_NOISE_NUM 3    /** 速度噪声底 = 观测器带宽 * 分辨率 * 3/8 的分子 */
#define DSGC_VEL_NOISE_DEN 8    /** 同上的分母，位置噪声被观测器微分成速度噪声 */
#define DSGC_VEL_DB_X 2         /** 保持态速度死区取噪声底的倍数，1倍会漏过一半噪声 */
#define DSGC_FRIC_MOVE_X 4      /** 判"已挣脱静摩擦"的速度门限相对速度噪声底的倍数 */
#define DSGC_KICK_MAX_PCT 50    /** 顶起值最多在标定起转值之上再自适应抬这么多(%) */
#define DSGC_KICK_RISE_MS 50    /** 抬到上限所需时间(ms)，只在"要求动却不动"时计时 */
#define DSGC_STUCK_MS 20        /** 实测速度连续低于噪声底这么多拍才认定轴真的粘住了 */
#define DSGC_FF_MARGIN_PWM 250  /** 前馈之外必须留给反馈的PWM余量 */
#define DSGC_WCMD_MIN_DIV 4     /** 最小速度指令 = 位置环带宽*分辨率/4 */
#define DSGC_FRIC_BLEND_X 3     /** 观测器摩擦过渡速度相对最小速度指令的倍数 */
#define DSGC_DEC_OF_ACC_Q8 128  /** 减速加速度占加速侧的比例，Q8，留出对象误差裕量 */
#define DSG_SMOOTH_MAX 100      /** 平滑度取值上限 */
#define DSG_IN_HOLD_MS 20       /** 进入到位状态前需连续满足条件的控制拍数 */
#define DSG_WCORR_PCT 25        /** 位置环速度修正限幅占最大速度的百分比 */
#define DSG_POS_BW_RADS TUNE_STIFFNESS_RADS                                 /** 位置环带宽 */
#define DSG_VEL_BW_RADS (TUNE_STIFFNESS_RADS * TUNE_DAMPING / 10)           /** 速度环带宽 */
#define DSG_VEL_I_RADS TUNE_INTEGRAL_RADS                                   /** 速度积分零点 */
#define DSG_OBS_BW_HZ TUNE_OBS_BW_HZ                                        /** 观测器带宽 */
#define DSG_VMAX_PCT TUNE_SPEED_PCT                                         /** 最大速度百分比 */
#define DSG_ACC_RATIO_Q8 (TUNE_ACCEL_PCT * 256 / 100)                       /** 加速度比例，Q8 */
#define DSG_DEC_RATIO_Q8 ((DSG_ACC_RATIO_Q8 * DSGC_DEC_OF_ACC_Q8) >> 8)     /** 减速度比例，Q8 */
#define DSG_POS_DEADBAND_CDEG TUNE_RESOLUTION_CDEG                          /** 位置死区，厘度 */
#define DSG_HOLD_DEADBAND_CDEG (TUNE_RESOLUTION_CDEG * DSGC_HOLD_DB_X2 / 2) /** 静音捕获窗，厘度 */
#define DSG_IN_WIN_CDEG (TUNE_RESOLUTION_CDEG * DSGC_IN_WIN_X2 / 2)         /** 到位窗，厘度 */
#define DSG_OUT_WIN_CDEG (TUNE_RESOLUTION_CDEG * DSGC_OUT_WIN_X)            /** 退出窗，厘度 */
#define DSG_IN_VEL_CDPS (TUNE_RESOLUTION_CDEG * DSGC_IN_VEL_X)              /** 到位零速阈值，厘度/秒 */
#define DSG_VEL_NOISE_CDPS (TUNE_OBS_BW_HZ * TUNE_RESOLUTION_CDEG * DSGC_VEL_NOISE_NUM / DSGC_VEL_NOISE_DEN) /** 观测速度噪声底，厘度/秒 */
#define DSG_VEL_DB_CDPS (DSG_VEL_NOISE_CDPS * DSGC_VEL_DB_X)                /** 保持态速度死区，厘度/秒 */
#define DSG_FRIC_MOVE_CDPS (DSG_VEL_NOISE_CDPS * DSGC_FRIC_MOVE_X)          /** 判已挣脱静摩擦的速度门限，厘度/秒 */
#define DSG_WCMD_MIN_CDPS (TUNE_STIFFNESS_RADS * TUNE_RESOLUTION_CDEG / DSGC_WCMD_MIN_DIV) /** 最小速度指令，厘度/秒 */
#define DSG_FRICTION_BLEND_CDPS (DSG_WCMD_MIN_CDPS * DSGC_FRIC_BLEND_X)     /** 观测器摩擦过渡速度，厘度/秒 */

/* ============================ 编译期派生系数 ============================ */
#define CFG_TICK_HZ 1000                                                                              /** 控制任务频率(Hz) */
#define CFG_Q24_ONE 16777216L                                                                         /** Q24格式中的1.0 */
#define CFG_Q24_MUL(a, b) ((int32_t)(((int64_t)(a) * (int64_t)(b)) >> 24))                            /** Q24定点乘法 */
#define PLANT_NET_PWM (((CFG_PWM_FULL - CAL_FRICTION_PWM) > 0) ? (CFG_PWM_FULL - CAL_FRICTION_PWM) : 1)/** 扣除摩擦后的净PWM */
#define PLANT_VSS_CDPS ((int32_t)(((int64_t)PLANT_NET_PWM << 16) / CAL_SPEED_SLOPE_Q16))              /** 满PWM稳态速度，厘度/秒 */
#define PLANT_A0_CDPSS ((int32_t)(((int64_t)PLANT_VSS_CDPS * 1000) / MODEL_TAU_MS))                   /** 零速起步加速度，厘度/秒^2 */
#define PLANT_KA_Q8_RAW ((int32_t)(((int64_t)65536 * 1000 * 256) / ((int64_t)CAL_SPEED_SLOPE_Q16 * MODEL_TAU_MS))) /** 加速度增益原始值，Q8 */
#define PLANT_KA_Q8 ((PLANT_KA_Q8_RAW > 0) ? PLANT_KA_Q8_RAW : 1)                                     /** 加速度增益，Q8 */
#define PLANT_SPEED_SLOPE_Q16 CAL_SPEED_SLOPE_Q16                                                     /** PWM-速度斜率，Q16 */
#define PLANT_FRICTION_PWM CAL_FRICTION_PWM                                                           /** 动摩擦PWM */
#define PLANT_TAU_MS MODEL_TAU_MS                                                                     /** 机械时间常数(ms) */
#define PLANT_DEAD_TICKS MODEL_DELAY_TICKS                                                            /** 纯延迟(控制拍) */

/* 观测器增益：对(tau, Kv)做三重极点配置，令三个特征值均为 exp(-2*pi*f*T)，
 * T=1ms，f=TUNE_OBS_BW_HZ。OBSG_* 是推导中间量，禁止手动修改。 */
#define OBSG_X_Q24 ((int32_t)((105414357LL * DSG_OBS_BW_HZ) / CFG_TICK_HZ))                                                                                   /** 2*pi*f*T，Q24 */
#define OBSG_E1 OBSG_X_Q24                                                                                                                                    /** 指数展开第1项 */
#define OBSG_E2 (CFG_Q24_MUL (OBSG_E1, OBSG_X_Q24) / 2)                                                                                                       /** 指数展开第2项 */
#define OBSG_E3 (CFG_Q24_MUL (OBSG_E2, OBSG_X_Q24) / 3)                                                                                                       /** 指数展开第3项 */
#define OBSG_E4 (CFG_Q24_MUL (OBSG_E3, OBSG_X_Q24) / 4)                                                                                                       /** 指数展开第4项 */
#define OBSG_E5 (CFG_Q24_MUL (OBSG_E4, OBSG_X_Q24) / 5)                                                                                                       /** 指数展开第5项 */
#define OBSG_E6 (CFG_Q24_MUL (OBSG_E5, OBSG_X_Q24) / 6)                                                                                                       /** 指数展开第6项 */
#define OBSG_ALPHA (CFG_Q24_ONE - OBSG_E1 + OBSG_E2 - OBSG_E3 + OBSG_E4 - OBSG_E5 + OBSG_E6)                                                                  /** 离散极点 exp(-2*pi*f*T)，Q24 */
#define OBSG_BETA (CFG_Q24_ONE - OBSG_ALPHA)                                                                                                                  /** 1-极点，Q24 */
#define OBSG_A2 CFG_Q24_MUL (OBSG_ALPHA, OBSG_ALPHA)                                                                                                          /** 极点平方 */
#define OBSG_A3 CFG_Q24_MUL (OBSG_A2, OBSG_ALPHA)                                                                                                             /** 极点立方 */
#define OBSG_B2 CFG_Q24_MUL (OBSG_BETA, OBSG_BETA)                                                                                                            /** beta平方 */
#define OBSG_B3 CFG_Q24_MUL (OBSG_B2, OBSG_BETA)                                                                                                              /** beta立方 */
#define OBSG_A_Q24 ((MODEL_TAU_MS > 0) ? (CFG_Q24_ONE / MODEL_TAU_MS) : (CFG_Q24_ONE / 36))                                                                   /** T/tau，Q24 */
#define OBSG_L1_NUM (((CFG_Q24_ONE - OBSG_A3 - OBSG_A_Q24) > 0) ? (CFG_Q24_ONE - OBSG_A3 - OBSG_A_Q24) : 0)                                                   /** L1分子 */
#define OBSG_L1_DEN (((CFG_Q24_ONE - OBSG_A_Q24) > 1) ? (CFG_Q24_ONE - OBSG_A_Q24) : 1)                                                                       /** L1分母 */
#define OBSG_L1_RAW ((int32_t)(((int64_t)OBSG_L1_NUM << 24) / OBSG_L1_DEN))                                                                                   /** L1原始值，Q24 */
#define OBSG_L1_Q24 ((OBSG_L1_RAW > CFG_Q24_ONE) ? (int32_t)CFG_Q24_ONE : OBSG_L1_RAW)                                                                        /** L1限幅后，Q24 */
#define OBSG_L2_T ((3 * OBSG_B2 - OBSG_B3 - CFG_Q24_MUL (OBSG_L1_Q24, OBSG_A_Q24)) > 0 ? (3 * OBSG_B2 - OBSG_B3 - CFG_Q24_MUL (OBSG_L1_Q24, OBSG_A_Q24)) : 0) /** L2中间量，Q24 */
#define OBSG_L3_NUM ((((int64_t)OBSG_B3 * CFG_TICK_HZ) * CFG_TICK_HZ) << 15)                                                                                  /** L3分子 */
#define OBS_L1_Q15 (OBSG_L1_Q24 >> 9)                                                                                                                         /** 位置校正增益，Q15 */
#define OBS_L2_Q15 ((int32_t)(((int64_t)OBSG_L2_T * CFG_TICK_HZ) >> 9))                                                                                       /** 速度校正增益，Q15 */
#define OBS_L3_Q15 (-(int32_t)((((OBSG_L3_NUM / PLANT_KA_Q8) * 256) >> 24)))                                                                                  /** 负载校正增益，Q15，设计值为负 */
#define OBS_LOAD_MAX (CFG_PWM_FULL / 3)                                                                                                                       /** 负载估计限幅，PWM计数 */
#define OBS_VMAX_CDPS (PLANT_VSS_CDPS + (PLANT_VSS_CDPS >> 3))                                                                                                /** 速度估计限幅，厘度/秒 */
#define OBS_FRIC_INV_Q15 ((int32_t)((1L << 23) / DSG_FRICTION_BLEND_CDPS))                                                                                    /** 摩擦过渡斜率倒数，Q15 */

/* 轨迹规划器限制。 */
#define TRAJ_VMAX_CDPS ((int32_t)(((int64_t)PLANT_VSS_CDPS * DSG_VMAX_PCT) / 100))            /** 最大速度，厘度/秒 */
#define TRAJ_AMAX_ACC_RAW ((int32_t)(((int64_t)PLANT_A0_CDPSS * DSG_ACC_RATIO_Q8) >> 8))      /** 加速上限原始值 */
#define TRAJ_AMAX_DEC_RAW ((int32_t)(((int64_t)PLANT_A0_CDPSS * DSG_DEC_RATIO_Q8) >> 8))      /** 减速上限原始值 */
#define TRAJ_AMAX_ACC_CDPSS ((TRAJ_AMAX_ACC_RAW > 0) ? TRAJ_AMAX_ACC_RAW : 1)                 /** 加速上限，厘度/秒^2 */
#define TRAJ_AMAX_DEC_CDPSS ((TRAJ_AMAX_DEC_RAW > 0) ? TRAJ_AMAX_DEC_RAW : 1)                 /** 减速上限，厘度/秒^2 */
#define TRAJ_SMOOTH_MAX DSG_SMOOTH_MAX                                                        /** 平滑度上限 */
#define TRAJ_MOVE_MIN_MS TUNE_MOVE_MIN_MS                                                     /** 小位移最短运动时间(ms) */
#define TRAJ_ACCEL_MIN_MS TUNE_ACCEL_MIN_MS                                                   /** 小位移加速段最短时间(ms) */

/* 位置/速度控制器系数。 */
#define CTRL_KP_POS_Q8 (DSG_POS_BW_RADS * 256)                                                        /** 位置环P增益，Q8 */
#define CTRL_KP_VEL_RAW ((int32_t)(((int64_t)DSG_VEL_BW_RADS * 65536 * 256) / PLANT_KA_Q8))           /** 速度环P增益原始值 */
#define CTRL_KP_VEL_Q16 ((CTRL_KP_VEL_RAW > 0) ? CTRL_KP_VEL_RAW : 1)                                 /** 速度环P增益，Q16 */
#define CTRL_KP_VEL_BRAKE_Q16 ((int32_t)(((int64_t)CTRL_KP_VEL_Q16 * MODEL_BRAKE_GAIN_Q8) >> 8))      /** 制动侧速度环P增益，Q16 */
#define CTRL_KI_VEL_RAW ((int32_t)(((int64_t)DSG_VEL_I_RADS * CTRL_KP_VEL_Q16 * CFG_TICK_HZ) / 1000000))/** 速度环I增益原始值 */
#define CTRL_KI_VEL_Q16 ((CTRL_KI_VEL_RAW > 0) ? CTRL_KI_VEL_RAW : 1)                                 /** 速度环I增益，Q16 */
#define CTRL_INTEGRAL_MAX (CFG_PWM_FULL / 3)                                                          /** 积分限幅，PWM计数 */
#define CTRL_INTEGRAL_ACC_GATE (PLANT_A0_CDPSS / 19)                                                  /** 大加速度时冻结积分的阈值 */
#define CTRL_KVFF_Q16 CAL_SPEED_SLOPE_Q16                                                             /** 速度前馈增益，Q16 */
#define CTRL_KA_Q20 ((int32_t)((1048576LL * 256) / PLANT_KA_Q8))                                      /** 加速度前馈增益，Q20 */
#define CTRL_KA_BRAKE_Q20 ((int32_t)(((int64_t)CTRL_KA_Q20 * MODEL_BRAKE_GAIN_Q8) >> 8))              /** 制动侧加速度前馈增益，Q20 */
#define CTRL_FRIC_DYN CAL_FRICTION_PWM                                                                /** 动摩擦前馈，PWM计数 */
#define CTRL_FRIC_STATIC ((CAL_BREAKAWAY_PWM > CAL_FRICTION_PWM) ? CAL_BREAKAWAY_PWM : CAL_FRICTION_PWM) /** 静摩擦前馈，PWM计数 */
#define CTRL_FRIC_MOVE_CDPS DSG_FRIC_MOVE_CDPS                                                        /** 判已挣脱静摩擦的速度门限，厘度/秒 */
#define CTRL_FRIC_KICK_MAX ((int32_t)CTRL_FRIC_STATIC * DSGC_KICK_MAX_PCT / 100)                      /** 顶起值的自适应抬升上限，PWM计数 */
#define CTRL_FRIC_KICK_STEP (((CTRL_FRIC_KICK_MAX / DSGC_KICK_RISE_MS) > 0) ? (CTRL_FRIC_KICK_MAX / DSGC_KICK_RISE_MS) : 1) /** 每拍抬升步长，PWM计数 */
#define CTRL_STUCK_MS DSGC_STUCK_MS                                                                   /** 判定轴粘住所需的连续静止拍数 */
#define CTRL_NUDGE_PWM ((CTRL_FRIC_STATIC > CTRL_FRIC_DYN) ? CTRL_FRIC_STATIC : 0)                    /** 静止时值得驱动的最小PWM，0=未标定起转 */
#define CTRL_WCMD_MIN_CDPS DSG_WCMD_MIN_CDPS                                                          /** 最小速度指令，厘度/秒 */
#define CTRL_MOTOR_SIGN CAL_MOTOR_SIGN                                                                /** 算法坐标到H桥物理方向的符号 */
#define CTRL_VMAX_CDPS TRAJ_VMAX_CDPS                                                                 /** 最大速度，厘度/秒 */
#define CTRL_WCORR_MAX ((int32_t)(((int64_t)TRAJ_VMAX_CDPS * DSG_WCORR_PCT) / 100))                   /** 位置环速度修正限幅 */
#define CTRL_PREVIEW_Q16 ((int32_t)(((int64_t)MODEL_DELAY_TICKS * 65536) / CFG_TICK_HZ))              /** 速度指令的加速度预演时长，Q16秒 */
#define CTRL_POS_LEAD_Q16 ((MODEL_DELAY_TICKS + MODEL_TAU_MS / 8) * 65536 / CFG_TICK_HZ)              /** 相对速度预测时长(执行延迟+惯性裕量)，Q16秒 */
#define CTRL_TARGET_DB_CDEG DSG_POS_DEADBAND_CDEG                                                     /** 目标死区，小于它不重新规划，厘度 */
#define CTRL_POS_DEADBAND DSG_POS_DEADBAND_CDEG                                                       /** 位置死区，厘度 */
#define CTRL_HOLD_DEADBAND DSG_HOLD_DEADBAND_CDEG                                                     /** 静音捕获窗，厘度 */
#define CTRL_IN_WIN_CDEG DSG_IN_WIN_CDEG                                                              /** 到位窗，厘度 */
#define CTRL_OUT_WIN_CDEG DSG_OUT_WIN_CDEG                                                            /** 退出窗，厘度 */
#define CTRL_IN_VEL_CDPS DSG_IN_VEL_CDPS                                                              /** 到位零速阈值，厘度/秒 */
#define CTRL_VEL_DB_CDPS DSG_VEL_DB_CDPS                                                              /** 保持态速度反馈死区，厘度/秒 */
#define CTRL_IN_HOLD_MS DSG_IN_HOLD_MS                                                                /** 进入到位状态的连续拍数 */
#define CTRL_PWM_LIMIT CFG_PWM_FULL                                                                   /** 控制器输出限幅 */

/* ============================== 编译期护栏 ============================== */
#define GUARD_2L_TICKS (2 * MODEL_DELAY_TICKS + 1)                                                       /** 延迟裕度对应的拍数 */
#define GUARD_OBS_BW_DELAY ((int)((CFG_TICK_HZ * 10000L) / (62832L * GUARD_2L_TICKS)))                   /** 纯延迟允许的观测器带宽上限 */
#define GUARD_OBS_BW_MAX (((MODEL_RESONANCE_HZ > 0) && ((MODEL_RESONANCE_HZ / 3) < GUARD_OBS_BW_DELAY)) ? (MODEL_RESONANCE_HZ / 3) : GUARD_OBS_BW_DELAY) /** 观测器带宽上限 */
#define GUARD_FF_PWM (((long)TUNE_SPEED_PCT * PLANT_NET_PWM) / 100 + CAL_FRICTION_PWM)                   /** 巡航段前馈占用的PWM */
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
typedef char guard_breakaway_exceeds_full[(CAL_BREAKAWAY_PWM < CFG_PWM_FULL) ? 1 : -1];
typedef char guard_motor_sign_invalid[((CAL_MOTOR_SIGN == 1) || (CAL_MOTOR_SIGN == -1)) ? 1 : -1];
/* 前馈把输出占满，闭环就不存在了。见文件头第6条。 */
typedef char guard_feedforward_leaves_no_headroom[((GUARD_FF_PWM + DSGC_FF_MARGIN_PWM) <= CFG_PWM_FULL) ? 1 : -1];
/* 保持态死区至少等于最小可靠步长，否则修一个刚超死区的误差就会窜到对面死区之外。 */
typedef char guard_hold_deadband_below_min_step[(DSGC_HOLD_DB_X2 >= 2) ? 1 : -1];
