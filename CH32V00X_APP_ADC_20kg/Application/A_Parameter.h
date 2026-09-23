#pragma once

#include <stdint.h>

/*
 * A_Parameter.h —— 控制设计参数：标定输入、调参面板、派生系数、编译期护栏
 * 只改"标定输入"和"调参面板"两段，DSGC_/DSG_/PLANT_/OBS_/TRAJ_/CTRL_ 均为派生量。
 *
 * 本机：电位器20kg单轴版，2026-09-21两次完整标定(日志 tests/fixtures/calibration_20kg_single_20260921.log)。
 *   两次输出取均值：斜率4921/4901->4911、动摩擦26/35->30、制动317/343->330；tau两次均为45ms、
 *   延迟两次均为2拍、电机符号两次均为-1；起转取两次上界192；最小可靠步长26/27取27。
 * 2026-09-22 起本工程的轨迹面板改为标准三参数(最大速度/加减速度/加加速度，见调参面板)，
 * 规划器按J受限求过渡段，其余控制链路与老版本电位器版(20kg，2026-09-09整定)相同。老版本整定要点：噪声是抽头滑动接触噪声，环路
 * 带宽取"够用的最低"(位置环1.43Hz、速度环3.5Hz、观测器11Hz、积分8rad/s)，级联分离护栏余量仅
 * 4.5%，降观测器带宽须同时降 TUNE_STIFFNESS_RADS；前馈给反馈留250余量(有护栏)。
 * 角度标度假设 ADC 350..3750 对应270度，自动标定不验证该标度。
 * 2026-09-22 实机遥测(tests/fixtures/vofa_20kg_20260922.log，0<->27000全行程往返)：两个方向加速时轴都跑在参考前面、
 *   减速末段都比参考先刹停(停在目标前0.6~1.2度再挪100~250ms)，拟合刹车并不比驱动弱。据此 MODEL_TAU_MS 45->42
 *   (仿真对照停住量得出，遥测逐点拟合更偏小但噪声大)，并让制动前馈按"轴比参考慢多少就松多少"放行
 *   (DSGC_BRAKE_RELAX_X)；MODEL_BRAKE_GAIN_Q8 330->290(256空载最好但惯量大30%时过冲1.5度，330带载最好但空载末端仍停顿)。
 *   标定日志里两向 L+tau 60/32ms 的差异在实际运动中不存在，故未采用分方向模型。
 */

/* ============================== 基本量程 ============================== */
#define CDEG_RANGE 36000        /** 整圈角度，厘度 */
#define CFG_WRAP_RANGE_CDEG 0   /** 反馈坐标回绕量程；0=线性行程不回绕，须与ENCODER_MODE一致 */
#define SERVO_PWM_MIN 500       /** 协议最小脉宽，也是电机模式负向满速指令 */
#define SERVO_PWM_MAX 2500      /** 协议最大脉宽，也是电机模式正向满速指令 */
#define SERVO_PWM_MID 1500      /** 电机模式零速指令，两侧线性映射到正负满PWM */
#define CFG_PWM_FULL 2400       /** PWM满量程，必须与驱动层实际限制一致 */

/* 整机旋向反转开关，含义见磁编码版同名宏。沿用本机型当前整机旋向。 */
#define CFG_DIR_INVERT 1        /** 1=反转标称旋向 */

/* ======================= 标定输入(A_Calib_Min 输出) ======================= */
#define CAL_SPEED_SLOPE_Q16 4911 /** PWM-速度斜率，Q16 */
#define CAL_FRICTION_PWM 30      /** 等效动摩擦PWM，速度直线的截距 */
#define CAL_BREAKAWAY_PWM 192    /** 起转(静摩擦)PWM；<=动摩擦时静摩擦前馈自动退化 */
#define MODEL_TAU_MS 42          /** 机械时间常数(ms)，偏小会显著降低稳定裕度；标定45，按2026-09-22实机遥测改42，见文件头 */
#define MODEL_DELAY_TICKS 2      /** 指令至轴响应的纯延迟(控制拍)，改后需复核全部护栏 */
#define CAL_MOTOR_SIGN -1        /** 装配方向，+1/-1，由标定程序自动判定 */
#define MODEL_BRAKE_GAIN_Q8 290  /** 制动侧相对驱动侧的PWM增益，Q8。标定工具给330(型号经验值)，实测刹车不弱；它是空载不停顿与带载不过冲的取舍：
                                     空载/轻载取290；舵盘外挂重负载(惯量明显变大)往330调，或带着负载重新标定 */
#define MODEL_RESONANCE_HZ 0     /** 第一机械谐振频率(Hz)，0=未测，非0时自动收紧观测器带宽 */

/* ============================== 调参面板 ============================== */
/* 轨迹参数，单位是真实物理量(度/秒、度/秒^2、度/秒^3)，不是百分比或档位。它们只是上限，不参与任何闭环增益
 * 和阈值；实际生效值 = min(面板值, 本机硬件能力)。下面的"实测有效范围"是在本机标定值上用规划器+仿真量出来的
 * (2026-09-23)，换台舵机或重新标定后按同样方法重量：tests/vaj_grid.py 单组参数模式可逐位移打印。
 *
 *   TUNE_VMAX_DPS  1~282，全程有效。282 是给反馈留 DSGC_FF_MARGIN_PWM 余量后的上限(满PWM空载316)，超过编译报错。
 *   TUNE_AMAX_DPS2 50~100000 可设，但硬件可用加速度随速度下降(反电势吃PWM)：大位移约 4370、30度约 4340、
 *                  5度约 4620(零速起步理论值 7530)。所以设到约 4500 以上不再变快，这是硬件极限不是软件限制。
 *   TUNE_DMAX_DPS2 50~6000 可设(硬件制动上限 6031，见 TRAJ_DEC_HW_CDPSS)，0=跟随加速度。实测生效：设2000得1990、
 *                  4000得3953、6000得5896、9000被硬件截到6025。越大停得越狠：标称对象下到6000都稳(超调<30厘度)，
 *                  但惯量比模型大30%时超调 129->152 厘度、增益偏差30%时 41->131 厘度。默认2000偏保守，
 *                  空载想更快可用4000，要硬件极限用6000并确认负载与标定一致。
 *   TUNE_JMAX_DPS3 500~2000000。斜坡时长=A/J，J>=约500*A 时斜坡短于1拍，等于加速度阶跃(老版本行为)，再大无意义。
 *
 * 小位移(角度小到 V/A 根本用不上)另有两级整形，沿用老版本 20kg-single 的做法，大位移时自动失效：
 *   TUNE_MOVE_MIN_MS 40~120，默认60(=1.4倍tau)。第一级：巡航速度 = 行程/该值。位移越小走得越慢，
 *                  但曲线形状不变，仍是"快起步-匀速-柔和停"。它不是"越大越柔"的旋钮，两头都有物理边界：
 *                  - 大于约120：参考速度低于轴能稳定爬行的速度。本机起转192/动摩擦30，挣脱静摩擦那一下
 *                    必然窜到6~9度/秒；参考比它还慢，每个小位移就变成"顶住-窜一下-再顶"，手感就是冲击。
 *                    2026-09-23 实机验证：把整条曲线拉长到150ms 反而比老版本更冲，拉到500ms 最差。
 *                  - 小于约40：巡航速度接近满速，小位移变成急动。
 *                  要让小位移"看起来慢"，应该调小 TUNE_VMAX_DPS，而不是拉长这个时间。
 *   TUNE_ACCEL_MIN_MS 20~60，默认40(≈1倍tau)。第二级：加减速度 = 巡航速度/该值，加减速段至少持续这么久。
 *                  它决定"起步和停住有多硬"。本机1度位移实测(峰值加速度 度/秒^2 / PWM单拍跳变)：
 *                      30ms(老版本取值) -> 605/175    40ms -> 559/156    45ms -> 505/122
 *                  短于纯延迟没有意义，编译报错；它同样压减速段，略增过冲换到位瞬间不砸。
 *
 * 本机到位时间(仿真、按实测标定的对象、正反向平均)：
 *   现在(V265 A8000 D跟随A J2e6 MOVE60/ACCEL40)：1度92  5度99  10度102  30度170  90度396  180度736 ms
 *   对照老版本 20kg-single                     ：1度88  5度94  10度100  30度372  90度600  180度930 ms
 *   小位移与老版本同速(差4~5ms)，大位移快一倍；1度位移的PWM单拍跳变 193->156、峰值加速度 669->559。
 */
#define TUNE_VMAX_DPS 265       /** 最大速度(度/秒) */
#define TUNE_AMAX_DPS2 5000     /** 最大加速度(度/秒^2)，超过硬件可用加速度后不再变快 */
#define TUNE_DMAX_DPS2 4000     /** 最大减速度(度/秒^2)，0=跟随 TUNE_AMAX_DPS2 */
#define TUNE_JMAX_DPS3 2000000  /** 最大加加速度(度/秒^3)，越小起停越柔 */
#define TUNE_MOVE_MIN_MS 60     /** 小位移最短运动时间(ms)：限巡航速度=行程/该值，0=关闭 */
#define TUNE_ACCEL_MIN_MS 40    /** 小位移加速段最短时间(ms)：限加速度=巡航速度/该值，不得短于纯延迟 */
#define TUNE_REV_SLIDE_DIV 0    /** 带时间指令中途换向时，参考最多反向滑出 剩余行程/该值 来放缓刹车；0=关闭(按硬件极限急转) */
#define TUNE_HOLD_STIFF 12      /** 保持刚度(PWM/厘度)：到位后被外力推开时按偏差直接回推，期间冻结积分；0=关闭。电位器版环路带宽被噪声压得很低，靠它补保持刚度 */
#define TUNE_STIFFNESS_RADS 9   /** 位置环带宽(rad/s)，受速度环/观测器级联分离约束 */
#define TUNE_DAMPING 25         /** 速度环带宽与位置环带宽之比的10倍 */
#define TUNE_INTEGRAL_RADS 8    /** 速度积分零点(rad/s)，不得高于位置环带宽 */
#define TUNE_OBS_BW_HZ 11       /** 观测器带宽(Hz)，受纯延迟和机械谐振上限约束 */
#define TUNE_RESOLUTION_CDEG 27 /** 最小可靠位移(厘度)，是全部位置阈值的公共标度 */

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
/* 下面四个常数决定"顶开静摩擦"这件事做得有多平滑，取值依据见 C_Pos_Ctrl.c 的 PosCtrl_Friction 注释 */
#define DSGC_FRIC_SLEW_MS 12    /** 摩擦前馈每拍变化上限折算的时间(ms)：从0爬到起转值要这么久，挣脱瞬间就不会出现PWM阶跃 */
#define DSGC_FRIC_BLEND_MIN_X 1 /** 摩擦过渡终点的下限相对速度噪声底的倍数，低于它分辨不出在不在动 */
#define DSGC_FRIC_WIN_SHIFT 2   /** 判"轴到底动没动"的位移窗口=2^该值拍(4ms)：窗口越长越干净，但放开越晚、窜得越远 */
#define DSGC_FRIC_RELEASE_MS 2  /** 轴动起来后摩擦前馈退回动摩擦所需时间(ms)：越短窜动越小 */
#define DSGC_FRIC_REGRIP_MS 20  /** 轴停下后摩擦前馈顶回起转值所需时间(ms)：越长越不会与位移噪声打架，但停住后重新起步越慢 */
#define DSGC_FF_MARGIN_PWM 250  /** 前馈之外必须留给反馈的PWM余量 */
#define DSGC_WCMD_MIN_DIV 4     /** 最小速度指令 = 位置环带宽*分辨率/4 */
#define DSGC_FRIC_BLEND_X 3     /** 观测器摩擦过渡速度相对最小速度指令的倍数 */
#define DSGC_MOVE_MIN_TAU_X10 14 /** 建议的小位移最短运动时间 = 机械时间常数的该值/10 倍(本机59ms)，面板 TUNE_MOVE_MIN_MS 参考它 */
#define DSGC_ACCEL_MIN_TAU_X10 10 /** 建议的小位移加速段最短时间 = 机械时间常数的该值/10 倍(本机42ms)，短于tau电机跟不上加速度指令 */
#define DSGC_BRAKE_RELAX_X 2     /** 制动放行：轴比参考每慢1%，制动前馈少给该值%；1~4 扫描取2，4倍减速段PWM抖动明显变大 */
#define DSG_IN_HOLD_MS 20       /** 进入到位状态前需连续满足条件的控制拍数 */
#define DSG_WCORR_PCT 21        /** 位置环速度修正限幅占物理稳态速度的百分比(=旧面板84%x25%)，与轨迹速度上限无关 */
#define DSG_POS_BW_RADS TUNE_STIFFNESS_RADS                                 /** 位置环带宽 */
#define DSG_VEL_BW_RADS (TUNE_STIFFNESS_RADS * TUNE_DAMPING / 10)           /** 速度环带宽 */
#define DSG_VEL_I_RADS TUNE_INTEGRAL_RADS                                   /** 速度积分零点 */
#define DSG_OBS_BW_HZ TUNE_OBS_BW_HZ                                        /** 观测器带宽 */
/* 小位移整形的两个推荐时间(面板 TUNE_MOVE_MIN_MS / TUNE_ACCEL_MIN_MS 参考它们，理由见面板说明)：
 * 巡航速度按 行程/DSG_MOVE_MIN_MS 限，加减速度按 巡航速度/DSG_ACCEL_MIN_MS 限，都只在小位移时起作用。
 * 加速段短于一个机械时间常数时电机跟不上加速度指令，过冲变大；而把整条曲线拉长会让参考速度低于轴能稳定
 * 爬行的速度，小位移反而变成粘滑冲击。 */
#define DSG_MOVE_MIN_MS (MODEL_TAU_MS * DSGC_MOVE_MIN_TAU_X10 / 10)         /** 建议的小位移最短运动时间(ms) */
#define DSG_ACCEL_MIN_MS (MODEL_TAU_MS * DSGC_ACCEL_MIN_TAU_X10 / 10)       /** 建议的小位移加速段最短时间(ms) */
#define DSG_POS_DEADBAND_CDEG TUNE_RESOLUTION_CDEG                          /** 位置死区，厘度 */
#define DSG_HOLD_DEADBAND_CDEG (TUNE_RESOLUTION_CDEG * DSGC_HOLD_DB_X2 / 2) /** 静音捕获窗，厘度 */
#define DSG_IN_WIN_CDEG (TUNE_RESOLUTION_CDEG * DSGC_IN_WIN_X2 / 2)         /** 到位窗，厘度 */
#define DSG_QUIET_EXIT_CDEG ((TUNE_HOLD_STIFF > 0) ? (TUNE_RESOLUTION_CDEG * 2) : DSG_IN_WIN_CDEG) /** 静音退出窗，开保持弹簧时提前退出以便尽早回推，厘度 */
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

/* 轨迹规划器限制。用户值(TRAJ_VMAX/AMAX/JMAX)是上限，硬件值(*_HW)是任何用户值都不能越过的天花板。 */
#define TRAJ_VMAX_HW_CDPS ((int32_t)(((int64_t)PLANT_VSS_CDPS * (CFG_PWM_FULL - DSGC_FF_MARGIN_PWM - CAL_FRICTION_PWM)) / PLANT_NET_PWM)) /** 前馈留足反馈余量时的最大速度，厘度/秒 */
/* 硬件制动上限：末端(反电势帮不上忙时)满PWM扣掉反馈余量、再按制动侧增益折算出的减速度，本机约6030度/秒^2。
 * 高速段有反电势助力，实际能力更高，所以这是最保守的一档。再往上要吃掉给反馈的余量，减速跟不上就会过冲。 */
#define TRAJ_DEC_HW_RAW ((int32_t)(((int64_t)(CFG_PWM_FULL - DSGC_FF_MARGIN_PWM) * PLANT_KA_Q8) / MODEL_BRAKE_GAIN_Q8))
#define TRAJ_DEC_HW_CDPSS ((TRAJ_DEC_HW_RAW > 0) ? TRAJ_DEC_HW_RAW : 1)                       /** 硬件制动上限，厘度/秒^2 */
#define TRAJ_VMAX_CDPS ((int32_t)TUNE_VMAX_DPS * 100)                                         /** 最大速度，厘度/秒 */
#define TRAJ_AMAX_CDPSS ((int32_t)TUNE_AMAX_DPS2 * 100)                                       /** 最大加速度，厘度/秒^2 */
#define TRAJ_DMAX_CDPSS ((TUNE_DMAX_DPS2 > 0) ? ((int32_t)TUNE_DMAX_DPS2 * 100) : TRAJ_AMAX_CDPSS) /** 最大减速度，厘度/秒^2 */
#define TRAJ_JMAX_CDPSS3 ((int32_t)TUNE_JMAX_DPS3 * 100)                                      /** 最大加加速度，厘度/秒^3 */
#define TRAJ_AMAX_MIN_CDPSS 5000L                                                             /** 运行时可设加速度下限(50度/秒^2)，保证定点换算不溢出 */
#define TRAJ_AMAX_MAX_CDPSS 10000000L                                                         /** 运行时可设加速度上限 */
#define TRAJ_JMAX_MIN_CDPSS3 50000L                                                           /** 运行时可设加加速度下限(500度/秒^3) */
#define TRAJ_JMAX_MAX_CDPSS3 200000000L                                                       /** 运行时可设加加速度上限(2000000度/秒^3，此时斜坡短于1拍=加速度阶跃) */
#define TRAJ_MOVE_MIN_MS TUNE_MOVE_MIN_MS                                                     /** 小位移最短运动时间(ms)，限巡航速度 */
#define TRAJ_ACCEL_MIN_MS TUNE_ACCEL_MIN_MS                                                   /** 小位移加速段最短时间(ms)，限加速度 */

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
#define CTRL_FRIC_SLEW_PWM ((((int32_t)CTRL_FRIC_STATIC / DSGC_FRIC_SLEW_MS) > 0) ? ((int32_t)CTRL_FRIC_STATIC / DSGC_FRIC_SLEW_MS) : 1)  /** 摩擦前馈每拍允许的变化量，PWM计数 */
#define CTRL_FRIC_BLEND_MIN_CDPS (DSG_VEL_NOISE_CDPS * DSGC_FRIC_BLEND_MIN_X)                      /** 摩擦过渡区的最小宽度，厘度/秒 */
#define CTRL_FRIC_WIN_SHIFT DSGC_FRIC_WIN_SHIFT                                                                      /** 实测位移速度的窗口移位 */
#define CTRL_FRIC_RELEASE_Q15 ((((1L << 15) / DSGC_FRIC_RELEASE_MS) > 0) ? ((1L << 15) / DSGC_FRIC_RELEASE_MS) : 1)      /** 滑动程度每拍的上升量，Q15 */
#define CTRL_FRIC_REGRIP_Q15 ((((1L << 15) / DSGC_FRIC_REGRIP_MS) > 0) ? ((1L << 15) / DSGC_FRIC_REGRIP_MS) : 1)         /** 滑动程度每拍的回落量，Q15 */
#define CTRL_NUDGE_PWM ((CTRL_FRIC_STATIC > CTRL_FRIC_DYN) ? CTRL_FRIC_STATIC : 0)                    /** 静止时值得驱动的最小PWM，0=未标定起转 */
#define CTRL_WCMD_MIN_CDPS DSG_WCMD_MIN_CDPS                                                          /** 最小速度指令，厘度/秒 */
#define CTRL_MOTOR_SIGN CAL_MOTOR_SIGN                                                                /** 算法坐标到H桥物理方向的符号 */
#define CTRL_VMAX_CDPS TRAJ_VMAX_HW_CDPS                                                              /** 速度指令限幅，取硬件值，不随轨迹速度上限变 */
#define CTRL_WCORR_MAX ((int32_t)(((int64_t)PLANT_VSS_CDPS * DSG_WCORR_PCT) / 100))                   /** 位置环速度修正限幅，不随轨迹速度上限变 */
#define CTRL_PREVIEW_Q16 ((int32_t)(((int64_t)MODEL_DELAY_TICKS * 65536) / CFG_TICK_HZ))              /** 速度指令的加速度预演时长，Q16秒 */
#define CTRL_POS_LEAD_Q16 ((MODEL_DELAY_TICKS + MODEL_TAU_MS / 8) * 65536 / CFG_TICK_HZ)              /** 相对速度预测时长(执行延迟+惯性裕量)，Q16秒 */
#define CTRL_TARGET_DB_CDEG DSG_POS_DEADBAND_CDEG                                                     /** 目标死区，小于它不重新规划，厘度 */
#define CTRL_POS_DEADBAND DSG_POS_DEADBAND_CDEG                                                       /** 位置死区，厘度 */
#define CTRL_HOLD_DEADBAND DSG_HOLD_DEADBAND_CDEG                                                     /** 静音捕获窗，厘度 */
#define CTRL_IN_WIN_CDEG DSG_IN_WIN_CDEG                                                              /** 到位窗，厘度 */
#define CTRL_OUT_WIN_CDEG DSG_OUT_WIN_CDEG                                                            /** 退出窗，厘度 */
#define CTRL_QUIET_EXIT_CDEG DSG_QUIET_EXIT_CDEG                                                      /** 静音退出窗，厘度 */
#define CTRL_HOLD_SPRING_Q8 ((int32_t)TUNE_HOLD_STIFF * 256)                                          /** 保持弹簧刚度，Q8 PWM/厘度 */
#define CTRL_HOLD_SPRING_DB CTRL_POS_DEADBAND                                                        /** 保持弹簧死区，厘度 */
#define CTRL_HOLD_SPRING_MAX (CFG_PWM_FULL / 2)                                                      /** 保持弹簧出力上限，PWM */
#define CTRL_IN_VEL_CDPS DSG_IN_VEL_CDPS                                                              /** 到位零速阈值，厘度/秒 */
#define CTRL_VEL_DB_CDPS DSG_VEL_DB_CDPS                                                              /** 保持态速度反馈死区，厘度/秒 */
#define CTRL_IN_HOLD_MS DSG_IN_HOLD_MS                                                                /** 进入到位状态的连续拍数 */
#define CTRL_PWM_LIMIT CFG_PWM_FULL                                                                   /** 控制器输出限幅 */

/* ============================== 编译期护栏 ============================== */
#define GUARD_2L_TICKS (2 * MODEL_DELAY_TICKS + 1)                                                       /** 延迟裕度对应的拍数 */
#define GUARD_OBS_BW_DELAY ((int)((CFG_TICK_HZ * 10000L) / (62832L * GUARD_2L_TICKS)))                   /** 纯延迟允许的观测器带宽上限 */
#define GUARD_OBS_BW_MAX (((MODEL_RESONANCE_HZ > 0) && ((MODEL_RESONANCE_HZ / 3) < GUARD_OBS_BW_DELAY)) ? (MODEL_RESONANCE_HZ / 3) : GUARD_OBS_BW_DELAY) /** 观测器带宽上限 */
#define GUARD_FF_PWM ((long)(((int64_t)TRAJ_VMAX_CDPS * PLANT_NET_PWM) / PLANT_VSS_CDPS) + CAL_FRICTION_PWM) /** 巡航段前馈占用的PWM */
typedef char guard_obs_bw_exceeds_delay_margin[(TUNE_OBS_BW_HZ <= GUARD_OBS_BW_MAX) ? 1 : -1];
typedef char guard_observer_not_3x_faster_than_velocity_loop[(((long)DSG_VEL_BW_RADS * 30000L) <= (62832L * TUNE_OBS_BW_HZ)) ? 1 : -1];
typedef char guard_velocity_loop_not_2x_faster_than_position_loop[(TUNE_DAMPING >= 20) ? 1 : -1];
typedef char guard_integral_faster_than_position_loop[(TUNE_INTEGRAL_RADS <= TUNE_STIFFNESS_RADS) ? 1 : -1];
typedef char guard_hysteresis_window_inverted[(DSG_OUT_WIN_CDEG > DSG_IN_WIN_CDEG) ? 1 : -1];
typedef char guard_deadband_wider_than_in_window[(DSG_POS_DEADBAND_CDEG < DSG_IN_WIN_CDEG) ? 1 : -1];
/* 最大速度超过硬件能力时直接报错而不是悄悄截断：设了300却只跑282会误导调参 */
typedef char guard_vmax_out_of_range[((TUNE_VMAX_DPS >= 1) && (TRAJ_VMAX_CDPS <= TRAJ_VMAX_HW_CDPS)) ? 1 : -1];
typedef char guard_amax_out_of_range[((TRAJ_AMAX_CDPSS >= TRAJ_AMAX_MIN_CDPSS) && (TRAJ_AMAX_CDPSS <= TRAJ_AMAX_MAX_CDPSS)) ? 1 : -1];
typedef char guard_jmax_out_of_range[((TRAJ_JMAX_CDPSS3 >= TRAJ_JMAX_MIN_CDPSS3) && (TRAJ_JMAX_CDPSS3 <= TRAJ_JMAX_MAX_CDPSS3)) ? 1 : -1];
typedef char guard_rev_slide_div_out_of_range[((TUNE_REV_SLIDE_DIV >= 0) && (TUNE_REV_SLIDE_DIV <= 64)) ? 1 : -1];
typedef char guard_brake_relax_out_of_range[((DSGC_BRAKE_RELAX_X >= 1) && (DSGC_BRAKE_RELAX_X <= 8)) ? 1 : -1];
/* 上限按"参考速度不得低于轴能稳定爬行的速度"取，见面板说明；再大只会把小位移变成粘滑爬行 */
typedef char guard_move_min_ms_out_of_range[((TUNE_MOVE_MIN_MS >= 0) && (TUNE_MOVE_MIN_MS <= 200)) ? 1 : -1];
/* 加速段短于纯延迟时，加速度指令还没传到轴上这一段就结束了，等于加速度阶跃 */
typedef char guard_accel_min_ms_below_pure_delay[((TUNE_ACCEL_MIN_MS * 2) >= (2 * MODEL_DELAY_TICKS + 1)) ? 1 : -1];
typedef char guard_accel_min_ms_out_of_range[((TUNE_ACCEL_MIN_MS > 0) && (TUNE_ACCEL_MIN_MS <= 200)) ? 1 : -1];
typedef char guard_dmax_out_of_range[((TUNE_DMAX_DPS2 == 0) || ((TRAJ_DMAX_CDPSS >= TRAJ_AMAX_MIN_CDPSS) && (TRAJ_DMAX_CDPSS <= TRAJ_AMAX_MAX_CDPSS))) ? 1 : -1];
typedef char guard_resolution_too_small[(TUNE_RESOLUTION_CDEG >= 1) ? 1 : -1];
typedef char guard_speed_slope_invalid[(CAL_SPEED_SLOPE_Q16 > 0) ? 1 : -1];
typedef char guard_tau_invalid[(MODEL_TAU_MS > 0) ? 1 : -1];
typedef char guard_friction_exceeds_full[(CAL_FRICTION_PWM < CFG_PWM_FULL) ? 1 : -1];
typedef char guard_breakaway_exceeds_full[(CAL_BREAKAWAY_PWM < CFG_PWM_FULL) ? 1 : -1];
typedef char guard_motor_sign_invalid[((CAL_MOTOR_SIGN == 1) || (CAL_MOTOR_SIGN == -1)) ? 1 : -1];
/* 前馈把输出占满，闭环就不存在了 */
typedef char guard_feedforward_leaves_no_headroom[((GUARD_FF_PWM + DSGC_FF_MARGIN_PWM) <= CFG_PWM_FULL) ? 1 : -1];
/* 保持态死区至少等于最小可靠步长，否则修一个刚超死区的误差就会窜到对面死区之外。 */
typedef char guard_hold_stiff_out_of_range[((TUNE_HOLD_STIFF >= 0) && (TUNE_HOLD_STIFF <= 32)) ? 1 : -1];
/* 静音退出窗必须大于进入窗(捕获窗)，两者之间的回差挡住静止噪声，否则静音反复进出 */
typedef char guard_quiet_exit_no_hysteresis[(DSG_QUIET_EXIT_CDEG > DSG_HOLD_DEADBAND_CDEG) ? 1 : -1];
typedef char guard_hold_deadband_below_min_step[(DSGC_HOLD_DB_X2 >= 2) ? 1 : -1];
