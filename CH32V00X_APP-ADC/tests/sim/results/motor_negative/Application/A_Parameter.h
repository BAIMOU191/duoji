#pragma once

#include <stdint.h>

/* 控制设计参数：标定输入、调参面板、派生系数和编译期护栏。 */
#define CDEG_RANGE 36000  /** 位置环绕量程，整圈=36000厘度。 */
/* 反馈坐标是否回绕。
 *   磁编码器：整圈可测，坐标在 [0,36000) 上循环，跨 0 点必须折算最短路径。
 *   电位器  ：一段有限行程，而且**读数可以为负**——碳膜比标定行程两端各多
 *             出 10 度(见 A_Sensor.h 的外扩说明)。按圆周坐标处理会出人命：
 *             -10 度经无符号折算变成 65535，再投影到行程就成了最高端，
 *             控制器立刻朝反方向满舵。低端死区脱困后来回振荡就是这么来的。
 * 0 = 线性不回绕。必须与 A_Sensor.h 的 ENCODER_MODE 一致，A_Servo.c 有断言。 */
#define CFG_WRAP_RANGE_CDEG 0  /** 反馈坐标的回绕量程；0=线性行程不回绕。 */
#define SERVO_PWM_MIN 500  /** 舵机协议最小脉宽，也是电机模式负向满速指令。 */
#define SERVO_PWM_MAX 2500 /** 舵机协议最大脉宽，也是电机模式正向满速指令。 */
#define SERVO_PWM_MID 1500 /** 电机模式零速指令；两侧分别线性映射到正负满PWM。 */
#define CFG_PWM_FULL 2400 /** PWM满量程，必须与驱动层实际限制保持一致。 */

/* 标定输入。A_Calib_Min 的输出替换此处的单机标定项。
 *
 * ===== 2026-09-09 电位器编码器，加了 T9 之后连跑3次 =====
 *   slope    4752 4716 4682   极差 1.5%  -> 4716
 *   inter      45   57   59   极差  31%  -> 57
 *   tau        50   52   50              -> 52 (偏大方向保守)
 *   L(ms)    1.55 1.98 2.52              -> 均值2.0 -> 2拍
 *   brake_q8  261  274  252   极差 8%   -> 261  (改了T4窗口后终于稳定，见下)
 *   噪声底   8.43 8.93 9.42 厘度峰峰    -> 约9厘度(1.1个LSB)，重复性极好
 *   起转dir0  241   82  231   -> 236 (82是离群：那次step只有13厘度，是预载
 *                                     残余回弹越过阈值，同一次的T3也被判不可信)
 *   起转dir1  177  192  172   -> 180
 *   最小步长   31   19   32   -> 32
 *
 * 三条实测结论：
 *   a) **起转方向不对称30%**(dir0 236 / dir1 180)，三次一致，是系统性的不是
 *      噪声；T1 的截距也同向(dir0 67/81/70 vs dir1 21/32/47)。现在按两方向
 *      较大值取一个数，dir1 会被过补30%——过补只是末端多窜一点点，位置环兜
 *      得住；补不足则末端爬不动没人管，所以宁可偏大。真要抠再分方向标定。
 *   b) **没有占空比耦合**。T9 的三段均值检查(0->u->0)三次都显示加力/撤力的
 *      读数偏移几乎相同(0.5~3.5厘度)，是缓慢漂移不是跟着PWM走的串扰。原先
 *      怀疑"电位器与PWM同步采样导致占空比相关偏置"的假设**就此排除**：运动
 *      中观测速度剩下的噪声是真实机械纹波，不是测量问题。
 *   c) v_ss = 32559 厘度/秒 = 325.6 度/秒，而 MT6701 时代是 204.7 度/秒，
 *      比值 1.6。同一台机构物理速度不该变，所以**角度标度仍有嫌疑**：拿量
 *      角器量 P0500->P2500 的实际转角 A，若 A 明显小于 266 度，就要按
 *      ENCODER_POT_SPAN_CDEG = round(101.5*A) 改标度并同比缩放 slope。
 *      控制环不受影响(反馈和目标同标度)，但协议的"270度行程"会跟着错。 */
#define CAL_SPEED_SLOPE_Q16 4716 /** 实测PWM-速度斜率，Q16；标定输出直接填入。 */
#define CAL_FRICTION_PWM 57      /** 实测等效动摩擦PWM，标定输出直接填入。 */
#define MODEL_TAU_MS 52          /** 实测机械时间常数(ms)；偏小会显著降低稳定裕度。 */
#define MODEL_DELAY_TICKS 2      /** 指令至轴响应的纯延迟(控制拍)；改变后需复核全部护栏。 */
#define CAL_MOTOR_SIGN -1         /** 装配方向，+1/-1；由标定程序自动判定。 */
/* 起转(静摩擦)PWM，由 A_Calib_Min 的 T9 缓升法实测。
 *
 * 和 CAL_FRICTION_PWM 的区别：后者是 T1 速度直线的截距，是**动摩擦**——轴
 * 已经在转时维持转动要花的力；起转是轴**从静止**推动要花的力，实测大 3~4 倍。
 * 末端修正正好工作在后一种工况：舵机停在离目标几十厘度的地方，要动起来必须
 * 先越过静摩擦，只补动摩擦的话缺的那一块只能靠速度环积分以 0.5 计数/拍慢慢顶
 * (2026-09-09 实测要爬 600ms)。
 *
 * 取两方向的**较大**值(dir0 236 / dir1 180)：过补只是末端多窜一点，位置环
 * 兜得住；补不足则末端爬不动，没有任何机制会去救它。 */
#define CAL_BREAKAWAY_PWM 235    /** 实测起转PWM；<=动摩擦时静摩擦前馈自动退化。 */
/* 2026-09-09 终于测出来了：261/274/252，极差8%，三次都通过有效性判据。
 *
 * 原来测不出来是**工作点选错**，不是原理问题：制动亏损量只与窗口长度有关、
 * 与初速无关，而窗口必须整个落在轴停住之前(t_stop = v0/a_brake)。旧配置用
 * 1200 驱动，v0 只有约100度/秒，t_stop 仅13ms，8ms 窗的亏损量约19厘度=2.4个
 * LSB，直接淹在噪声里——于是5次里3次出数(555/422/912，差2.2倍)。改成满驱动
 * 之后 v0=v_ss，t_stop 涨到约46ms，16ms 窗的亏损量约72厘度=9个LSB，可测了。
 *
 * 261 ≈ 256，说明这台慢衰减H桥的制动侧和驱动侧增益基本对称，之前担心的
 * "制动比驱动弱一半"在这台机构上不成立。 */
#define MODEL_BRAKE_GAIN_Q8 261  /** 制动侧相对驱动侧的PWM增益，Q8；须通过实机制动测试确定。 */
#define MODEL_RESONANCE_HZ 0     /** 第一机械谐振频率(Hz)，0=未测；非0时自动收紧观测器带宽。 */

/* 调参面板。 */
#define TUNE_SMOOTH_ACC 0       /** 加速平滑度，0~100；增大可降低加速段 jerk，但会增加反向响应时间。 */
#define TUNE_SMOOTH_DEC 60      /** 减速平滑度，0~100；减小末端制动力突变。 */
#define TUNE_MOVE_MIN_MS 70     /** 小位移最短运动时间(ms)，0=关闭；增大使短距离动作更柔和、更慢。 */
#define TUNE_ACCEL_MIN_MS 30    /** 小位移加速段最短时间(ms)；增大可压低小位移 PWM 尖峰，且不得短于纯延迟。 */
/* 巡航段的 PWM 预算。这个数直接决定还剩多少输出留给反馈：
 *     前馈占用 = TUNE_SPEED_PCT% * PLANT_NET_PWM + CAL_FRICTION_PWM
 *     84% ->  0.84*2343 + 57 = 2025 / 2400，反馈余量 375
 *
 * 余量必须接得住巡航段 PWM 的噪声摆动(实测峰值偏移约 ±200，来源见
 * TUNE_OBS_BW_HZ 那一段)。余量不足的后果不是"慢一点"，而是**闭环彻底失去
 * 调节权限**：输出常驻饱和，反馈算多少都下不去。文件末尾的
 * guard_feedforward_leaves_no_headroom 在编译期挡这条线。 */
#define TUNE_SPEED_PCT 84       /** 最大速度占物理稳态速度的百分比；增大更快但闭环可用 PWM 余量更少，最大100。 */
/* 仅在 Traj_AutoAmax 拿不到对象常数时兜底。正常路径按 a(v)=a0*(1-v/v_ss)
 * 逐段解析求可用加速度，不走这个百分比。 */
#define TUNE_ACCEL_PCT 95       /** 最大加速度占物理能力的百分比；增大更快但模型误差和饱和风险更高，最大100。 */
/* ==================== 环路带宽：为什么是这一组数 ====================
 *
 * 四个数必须一起看，它们被一条级联分离链绑死：
 *     位置环 stiffness < 速度环 stiffness*damping/10 < 观测器 2*pi*obs_bw
 * 每一级至少比上一级快 2~3 倍，否则级联分离不成立、相位裕度塌掉。文件末尾
 * 三条护栏盯着这个关系。
 *
 * 但**真正的上限由传感器噪声定，不是由稳定性定**。2026-09-09 用 1ms 突发
 * 日志(400 拍)把巡航段拆开之后，结论是硬的：
 *
 *   a) 没有机械速度纹波。速度 sigma 从 1ms 一路按 1/sqrt(K) 降到 81ms，
 *      这是纯白噪声的特征；81ms 尺度上真实速度只波动 1.6%。
 *   b) 1ms 内 sigma=7543 厘度/秒，而 a0=626000 只允许 1ms 变 626——实测是
 *      物理极限的 12 倍。机构不可能这么动，只能是测量噪声。
 *   c) 噪声是电位器抽头在碳膜上滑动的接触噪声：运动中位置 sigma 5~6 厘度
 *      (静止时 1.5)，且与 |PWM| 无关(r=-0.057)，驱动层修不掉。
 *   d) 能量分布：0~11Hz 占 0.6%，11~50Hz 占 8.3%，**50Hz 以上占 91%**。
 *
 * 而对象转折频率只有 1/(2*pi*52ms) = 3.06Hz。3Hz 以上的反馈修正对真实运动
 * 毫无作用，只是把噪声灌回 PWM、吃掉本该留给真实扰动的余量。所以带宽取的是
 * "够用的最低"而不是"稳定允许的最高"：
 *
 *     位置环   9 rad/s = 1.43Hz  跟踪由前馈负责(实测跟随率 99.9%)，位置环
 *                                只修残差，1.43Hz 够用
 *     速度环  22 rad/s = 3.5Hz   约等于对象转折频率；再高就是在修对象
 *                                响应不了的频段
 *     观测器  11Hz               L2=9.8，把 91% 的高频噪声挡在环外
 *     积分     8 rad/s           <= 位置环，负责消除稳态负载误差
 *
 * 护栏余量只剩 4.5%(22*3=66 <= 2*pi*11=69.1)。**再降 obs_bw 必须同时降
 * stiffness**，否则编译期直接失败。 */
#define TUNE_STIFFNESS_RADS 9   /** 位置环带宽(rad/s)；增大跟随更快，但受速度环/观测器级联分离约束。 */
#define TUNE_DAMPING 25         /** 速度环带宽与位置环带宽之比的10倍；增大速度环更快，范围须满足护栏。 */
#define TUNE_INTEGRAL_RADS 8    /** 速度积分零点(rad/s)；增大可更快消除稳态误差，但不得高于位置环带宽。 */
#define TUNE_OBS_BW_HZ 11       /** 观测器带宽(Hz)；增大响应更快但受纯延迟和机械谐振上限约束。 */
/* 机构能可靠做出的最小一步。它是整套位置阈值的**公共标度**：死区、到位窗、
 * 迟滞窗、零速阈值、摩擦前馈的建立速度全部按它派生(见下面 DSG_* 段)，所以
 * 改这一个数就能整体收紧或放宽定位精度。
 *
 * 取值由 A_Calib_Min 实测决定，两条下界取大者：
 *     >= 2 x 噪声底(T9)      比它小 = 控制器在追传感器噪声
 *     >= 最小可靠步长(T9/T6) 比它小 = 为一个修不动的残差反复踹静摩擦，
 *                            窜过头再反向窜，必出极限环
 * 2026-09-09 连跑三次：噪声底峰峰 8.4/8.9/9.4 厘度，缓升法最小步长
 * 31/19/32 厘度 -> 取 32。
 *
 * 想再往下压，先确认 最小步长 < 2 x 保持态死区 仍成立(当前 32 < 64，一倍
 * 裕度)；不成立就会在死区边沿来回窜。 */
#define TUNE_RESOLUTION_CDEG 32 /** 最小可靠位移(厘度)；增大将同步增大死区、到位窗和零速阈值。 */

/* 结构常数及由调参面板导出的设计量。 */
/* 静音捕获窗为1.5倍最小步长(48厘度)，退出使用IN_WIN(80厘度)。
 * 运动和静止修正的P环死区均保持32厘度，避免停在捕获窗稍外仍持续通电。 */
#define DSGC_HOLD_DB_X2 3                                                                  /** 静止捕获窗口为1.5倍最小步长；运动死区保持32厘度。 */
#define DSGC_IN_WIN_X2 5                                                                   /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_OUT_WIN_X 4                                                                   /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_IN_VEL_X 50                                                                   /** 设计关系常数，用于将调参项折算为控制阈值。 */
/* 观测器速度噪声底 = k * 观测器带宽 * 位置分辨率。位置噪声被观测器微分成
 * 速度噪声，幅度**同时**正比于这两项——原来只挂在分辨率上，改了观测器带宽
 * 这个阈值就对不上了(obs 22Hz 时实测静止峰值 ±326，反推 k = 3/8；若仍按
 * 旧式只挂分辨率，obs 降到 11Hz 后阈值会大出实际噪声 2.5 倍)。 */
#define DSGC_VEL_NOISE_NUM 3                                                               /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_VEL_NOISE_DEN 8                                                               /** 设计关系常数，用于将调参项折算为控制阈值。 */
/* 保持态速度反馈死区取噪声底的倍数。死区要明显大于噪声峰值才 squelch 得
 * 干净；等于 1 倍的话噪声有一半时间会漏过去，取 2 倍。 */
#define DSGC_VEL_DB_X 2                                                                    /** 设计关系常数，用于将调参项折算为控制阈值。 */
/* Stribeck 过渡尺度相对速度噪声底的倍数。静摩擦按 |v| 线性退化到动摩擦，
 * 这个尺度必须显著大于噪声底，否则噪声会把摩擦前馈在静/动之间来回拉。 */
#define DSGC_FRIC_VS_X 12                                                                  /** 设计关系常数，用于将调参项折算为控制阈值。 */
/* 前馈之外必须留给反馈的 PWM 余量，见文件末尾 guard_feedforward_leaves_no_headroom。 */
#define DSGC_FF_MARGIN_PWM 250                                                             /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_WCMD_MIN_DIV 4                                                                /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_FRIC_BLEND_X 3                                                                /** 设计关系常数，用于将调参项折算为控制阈值。 */
#define DSGC_DEC_OF_ACC_Q8 128                                                             /** 减速加速度留出对象误差裕量，避免满速末端急刹。 */
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
#define DSG_VEL_NOISE_CDPS (TUNE_OBS_BW_HZ * TUNE_RESOLUTION_CDEG * DSGC_VEL_NOISE_NUM / DSGC_VEL_NOISE_DEN) /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_VEL_DB_CDPS (DSG_VEL_NOISE_CDPS * DSGC_VEL_DB_X)                               /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
#define DSG_FRIC_VS_CDPS (DSG_VEL_NOISE_CDPS * DSGC_FRIC_VS_X)                             /** 由调参面板和结构常数导出的设计量，禁止单独修改。 */
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
#define CTRL_FRIC_STATIC ((CAL_BREAKAWAY_PWM > CAL_FRICTION_PWM) ? CAL_BREAKAWAY_PWM : CAL_FRICTION_PWM)                                                       /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_FRIC_VS_CDPS DSG_FRIC_VS_CDPS                                                                                                                     /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_NUDGE_PWM ((CTRL_FRIC_STATIC > CTRL_FRIC_DYN) ? CTRL_FRIC_STATIC : 0) /** 静止补偿至少达到已标定起转值；3/4起转值仍可能只响不动。 */
#define CTRL_WCMD_MIN_CDPS DSG_WCMD_MIN_CDPS                                                                                                                  /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_MOTOR_SIGN CAL_MOTOR_SIGN                                                                                                                        /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_VMAX_CDPS TRAJ_VMAX_CDPS                                                                                                                         /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_WCORR_MAX ((int32_t)(((int64_t)TRAJ_VMAX_CDPS * DSG_WCORR_PCT) / 100))                                                                           /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_PREVIEW_Q16 ((int32_t)(((int64_t)MODEL_DELAY_TICKS * 65536) / CFG_TICK_HZ))                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_POS_LEAD_Q16 ((MODEL_DELAY_TICKS + MODEL_TAU_MS / 8) * 65536 / CFG_TICK_HZ) /** 相对速度预测：执行延迟加少量惯性裕量。 */
#define CTRL_TARGET_DB_CDEG DSG_POS_DEADBAND_CDEG                                                                                           /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_POS_DEADBAND DSG_POS_DEADBAND_CDEG                                                                                                               /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_HOLD_DEADBAND DSG_HOLD_DEADBAND_CDEG                                                                                                             /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_IN_WIN_CDEG DSG_IN_WIN_CDEG                                                                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_OUT_WIN_CDEG DSG_OUT_WIN_CDEG                                                                                                                    /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_IN_VEL_CDPS DSG_IN_VEL_CDPS                                                                                                                      /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
#define CTRL_VEL_DB_CDPS DSG_VEL_DB_CDPS                                                                                                                /** 位置/速度控制器的增益、前馈或限制，由设计量派生。 */
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
typedef char guard_breakaway_exceeds_full[(CAL_BREAKAWAY_PWM < CFG_PWM_FULL) ? 1 : -1];

/* 前馈把输出占满，闭环就不存在了。这条护栏守的是 2026-09-08 那次
 * "规划 29854 / 实测 15500" 的根因：当时 TUNE_SPEED_PCT=91，巡航前馈吃掉
 * 2189/2400，只剩 211 给反馈，输出常驻饱和，抗饱和回退又把积分顶到反向轨。
 * 那次是靠读日志才发现的，本该在编译期就拦住。
 * 余量 DSGC_FF_MARGIN_PWM 按实测噪声峰值偏移(±200)留 250。 */
#define GUARD_FF_PWM (((long)TUNE_SPEED_PCT * PLANT_NET_PWM) / 100 + CAL_FRICTION_PWM)  /** 编译期安全护栏的中间上限，禁止手动修改。 */
typedef char guard_feedforward_leaves_no_headroom[((GUARD_FF_PWM + DSGC_FF_MARGIN_PWM) <= CFG_PWM_FULL) ? 1 : -1];

/* 保持态死区必须至少等于分辨率(即最小可靠步长)，否则修一个刚超死区的误差时，
 * 哪怕只走一个最小步长也会窜到对面死区之外，形成极限环。 */
typedef char guard_hold_deadband_below_min_step[(DSGC_HOLD_DB_X2 >= 2) ? 1 : -1];
