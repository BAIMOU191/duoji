/*
 * A_Calib_Min.c —— 唯一的自动标定程序：3个单台参数+3个型号参数
 *
 * 【本版针对电位器编码器】原版是给 MT6701 磁编码器写的，前提是"输出轴可以
 * 自由连续转动"。电位器只有一段有限行程，转出碳膜就彻底没有反馈，所以整套
 * 激励策略加了行程保护和回中，扫频(T7)因为需要连续转好几圈而被关掉。
 * 三处结构性差异集中写在下面 CM_SWEEP_ENABLED 附近的说明里。
 *
 * ============================ 为什么只剩 6 个数 ============================
 * 已删除的旧版标定曾测 8 项、跑约 70 秒、输出大量互相依赖的对象常数。
 * 这 16 个数**不是互相独立的**，其中大部分可以由另外几个精确算出来：
 *
 *   v_ss  = (PWM_FULL - coulomb) / slope      单独测 20333，算出来 20443 (0.5%)
 *   a0    = v_ss / tau                        定义式
 *   Ka    = 1/(slope * tau)                   单独测 244，算出来 245.5 (0.6%)
 *   creep = Kv * coulomb * (brk_ratio - 1)    单独测 1126，算出来 1167 (3.6%)
 *   brk   = coulomb * brk_ratio               比值(2.16/2.27)比绝对值稳定
 *   L1/L2/L3 = 三重极点配置的闭式解(tau, Ka, 带宽)  编译期推导，见 A_Parameter.h
 *   _fwd/_rev 双向拆分：实测差 1%(动摩擦)、1.3%(斜率)、6%(起转)，
 *                       都小于同方向重复测量本身的散布(起转极差 22 计数)
 *
 * 把互相依赖的量分别测量不但浪费产线时间，还会因为各自的测量噪声互相矛盾
 * (比如测出来的 a0 和 v_ss/tau 对不上，到底信哪个)。所以量产只测独立信息：
 *
 *   1. slope     PWM-速度直线的斜率     -> 电机增益，**单台散差最大**
 *   2. coulomb   同一条直线的截距       -> 摩擦，**单台散差第二大**
 *   3. tau       机械时间常数           -> 惯量/阻尼，同型号基本一致
 *   4. dead      纯延迟                 -> 驱动电路+机构，同型号一致
 *   5. sign      电机接线方向           -> 装配检查，1 bit
 *   6. brake     主动制动/驱动增益比    -> 慢衰减H桥型号级常数
 *
 * 其中 3、4、6 是**型号级**的：同一套机构、同一块驱动板，单台之间的差异远小于
 * 测量重复性。量产可以只在金样上测一次写死，产线上只跑 T1(约 8 秒)测 1、2，
 * 再加 T0 的接线检查。想重测全部6项就在金样上跑全套，约17秒。
 *
 * ============================ 误差敏感度(仿真实测) ============================
 * 被控对象保持不变、只把标定值喂错，全场景总过冲(厘度)：
 *
 *     标定偏差        总过冲      结论
 *     准确             113
 *     slope 偏大15%    153       偏大=把电机估弱，较保守
 *     slope 偏小15%    174       偏小=把电机估强，开始恶化
 *     slope 偏小30%    266       长时间饱和
 *     coulomb 偏小50%  102       摩擦项不敏感(速度PI会补)
 *     tau 偏大40%       64       偏大=把机构估慢，保守
 *     tau 偏小20%      383
 *     tau 偏小40%     1469       最危险的一项
 *     dead 取 0/2倍   135/109    基本不敏感
 *
 * 两条可直接执行的结论，本程序已经照做：
 *   a) 误差是**单边**的：把对象估得"更弱、更慢"只损失一点速度，估得
 *      "更强、更快"会让规划器要求硬件做不到的加速度，反馈救不回来。
 *      因此 slope 和 tau 一律**向上取整**，宁可保守。
 *   b) tau 最敏感但它恰好是型号级常数——在金样上多测几次取偏大值即可，
 *      不需要每台都冒风险。
 *
 * ============================ 使用 ============================
 * main.c 定义 APP_MODE_CALIB 后编译下载，上电 1 秒自动开始，
 * 输出轴必须能自由转动(拆掉舵盘/负载)。结束后串口打印6行，
 * 直接替换 A_Parameter.h 中的6个标定 #define。
 */

#include "A_Calib_Min.h"
#include "A_Parameter.h"
#include "C_Task_Scheduler.h"
#include "D_motor.h"
#include "D_adc.h"      /* 电位器通道的ADC原始计数 */
#include "A_Sensor.h"   /* 电位器标定常数与死区门限 */
#include "D_iwdg.h"
#include <stdio.h>
#include <string.h>

/* ==================== 可调项(换更大扭力舵机时复核) ==================== */

/* ==================== 单一全量模式 ====================
 * 一次跑完 T0~T7，约45秒，输出全部 9 个宏。
 *
 * 原来分三档(产线10s / 金样25s / 扫频45s)已合并。分档省下的那点产线时间
 * 换来一个隐式耦合：模式0 印出来的 MODEL_TAU_MS 其实是金样的旧值，而 tau
 * 又是全参数里最敏感的一项——"印出来的数不是这台测的"这种坑不值得为 30 秒
 * 保留。要恢复快标，把 T2~T7 的 phase 跳过即可，状态机本身没有模式假设。
 *
 * 两个符号保留下来只是为了让下面既有的 #if 分支不用逐个改写。 */
#define CALIB_MODE 2
#define CALIB_CHARACTERIZE_MODEL 1

/* ==================== 电位器编码器：三处结构性差异 ====================
 *
 * 1) **没有回绕，只有一段有限行程**。原来"输出轴自由转动"的前提不成立了：
 *    所有开环激励都必须留在碳膜上，冲出去就落进死区(抽头脱离碳膜，彻底
 *    没有反馈，标定当场作废且轴自己转不回来)。对策是两道保护——每拍越界
 *    即停(CM_TRAVEL_LO/HI)，每项测试前回中(PH_CENTER)。
 *
 * 2) **分辨率粗 3.6 倍**。MT6701 是 2.197 厘度/LSB，电位器是 3400 计数摊
 *    到 27000 厘度 = 7.94 厘度/LSB。凡是按"数编码器计数"工作的测试(T3 纯
 *    延迟、T6 最小位移、T8 振铃)裕度都被吃掉，见各自的说明。
 *
 * 3) **扫频(T7)做不了**，见 CM_SWEEP_ENABLED。
 */

/* 扫频靠直流偏置让轴持续单向转动来 dither 编码器，12 个频点最长要转 20 多
 * 秒——好几圈，而电位器只有 270 度。关掉它只损失 MODEL_RESONANCE_HZ，而
 * "未发现"(0)本来就是合法结果；机构第一模态由 T8 振铃测，按 T8 自己的说明
 * 那本来就是更可靠的办法(振铃测自由响应，不被对象自身的滚降埋掉)。 */
#define CM_SWEEP_ENABLED 0

/* ---- 行程保护与预定位 ----
 *
 * 保护带按**碳膜可读范围**画，不是按标定行程画：要防的是抽头滑出碳膜从而
 * 彻底失去反馈，而碳膜比标定行程两端各多出 10 度(见 A_Sensor.h 的外扩说明)。
 * 按标定行程画会白白勒掉这 20 度，起跑点也就无处安放。 */
#define CM_GUARD_CDEG     500L  /* 距碳膜两端还有这么多厘度就判越界(5度) */
#define CM_TRAVEL_LO      (ENCODER_POT_ANGLE_MIN + CM_GUARD_CDEG)
#define CM_TRAVEL_HI      (ENCODER_POT_ANGLE_MAX - CM_GUARD_CDEG)
#define CM_CENTER_CDEG    ((int32_t)ENCODER_POT_SPAN_CDEG / 2)

/* 单向激励的起跑点：贴着行程低端，把整段行程都留给接下来的运动。
 * T2 满PWM要连跑 700ms(约143度)，从正中出发根本放不下。
 *
 * 离保护带留 25 度而不是 5 度：预定位是 bang-bang，松手之后还要滑行，
 * 900PWM 下接近速度约 7.2 厘度/ms，滑行量轻松超过 5 度。第一版留 5 度，
 * 结果预定位"完成"后轴继续滑出保护带，下一项一开跑就判越界(fail=4)。 */
#define CM_START_LO_CDEG  2000L
#define CM_CENTER_TOL     300L  /* 到位判据(3度)，标定不需要精确定位 */
#define CM_CENTER_PWM     900   /* 远距离接近幅值 */
/* 末段减速：接近目标时换小幅值，把滑行量压下来。滑多远直接决定起跑点能
 * 贴保护带多近，也决定"预定位完成"到底算不算数。 */
#define CM_CENTER_SLOW_CDEG 2500L /* 误差小于它就换慢挡 */
#define CM_CENTER_SLOW_PWM  600
#define CM_CENTER_CHECK_MS 150U /* 每隔这么久检查一次方向对不对 */
#define CM_CENTER_TIMEOUT 4000U /* 单趟接近超时(ms) */
#define CM_CENTER_TRIES      3U /* 滑过头了最多重新逼近几趟 */

/* T1 速度直线的档位。最低档必须高于起转PWM，否则该档转不起来、被自动剔除。
 * 覆盖范围越宽拟合越稳；5 档已经足够，再多只是拖长产线时间。 */
#define CM_LEVELS      5U
static const int16_t CM_PWM[CM_LEVELS] = { 700, 1120, 1540, 1960, 2380 };

#define CM_SETTLE_MS     250U   /* 施加PWM后等待进稳态 */
#define CM_MEAS_MS       300U   /* 测速窗口。长基线摊薄量化：1LSB=2.197厘度，
                                 * 300ms 下只带来 7.3 厘度/秒误差 */
#define CM_REST_MS       250U   /* 每次运动之间的静置 */
#define CM_STEP_MARK_MS  400U   /* T2 阶跃的取样时刻，必须已进稳态 */
#define CM_DEAD_PRE_MS   200U   /* T3 预运动，用于把齿隙顶死到测试方向 */
#define CM_DEAD_PRE_PWM  1200
#define CM_DEAD_REST_MS  500U   /* T3 预运动后必须完全停稳 */
#define CM_DEAD_TIMEOUT  300U
/* T3 记录前 N 个计数各自的跨越时刻，回归求截距。
 * 只看第 1 个计数时，测量分辨率(1ms)等于答案的整个动态范围(2~4拍)，
 * 实测同一台连跑会在 2/3/4 之间跳。取 N 个点后量化噪声被 sqrt(N) 摊薄。
 * N 不能太大：二次近似 x=a0*t^2/2 要求 t << tau，N=12 时 t/tau 约 0.29，
 * 已在下面用 (1-u/3) 一阶修正补掉。 */
/* 电位器 1 LSB = 7.94 厘度，是 MT6701 的 3.6 倍粗，跨过同样 N 个计数需要
 * 的时间也长 1.9 倍，t/tau 从 0.29 涨到 0.55，二次近似撑不住。取 6 折中：
 * t/tau 约 0.39，(1-u/3) 修正还站得住，同时保留 sqrt(6) 的量化摊薄。 */
#define CM_DEAD_NPTS      6U
#define CM_SIGN_MS       200U   /* T0 方向检测的施力时长 */
#define CM_SIGN_PWM      900
#define CM_SIGN_MIN_CDEG 50L    /* T0 认为"确实动了"的最小位移 */
/* 2026-09-09：驱动幅值 1200 -> 满量程。制动亏损量与初速无关(见下式)，只与
 * 窗口长度有关，而窗口必须整个落在轴停住之前：t_stop = v0/a_brake。1200 时
 * v0 只有约 100deg/s，t_stop 仅 13ms，8ms 窗测到的亏损量约 19 厘度 = 2.4 个
 * 电位器 LSB——直接淹在噪声里，这就是电位器版 T4 五次里三次出数、且三次差
 * 2.2 倍的原因。满驱动把 v0 抬到 v_ss，t_stop 涨到约 46ms，16ms 窗的亏损量
 * 约 72 厘度 = 9 个 LSB，才是可测的。 */
#define CM_BRAKE_DRIVE_PWM MOTOR_PWM_MAX /* T4 先建立满速稳态 */
#define CM_BRAKE_CMD_PWM    300  /* 随后施加反向PWM，测主动制动增益 */
#define CM_BRAKE_VEL_MS     100U
/* T4：制动**不是匀减速**，是一阶指数衰减，窗口平均值随窗口长度变
 * (实测同一台 10ms 窗给 666~759k，20ms 窗给 552~590k)。
 *
 * tau 已由 T2/T3 精确测出，所以不需要多窗口外推，直接解析反演即可：
 *     deficit(T) = a0*tau^2*[q-1+e^-q],  q = T/tau
 * 展开 e^-q 后化简成"匀减速结果 × 一个修正因子"：
 *     a0 = a_bar / (1 - q/3 + q^2/12),   a_bar = 2*deficit/T^2
 *
 * 窗口必须整个落在轴停住之前。t_stop = v0/a0 ≈ tau/g，g 是制动增益比；
 * 实测这台 g≈2 => t_stop 只有 13ms，所以主窗口取 8ms，另取 4ms 做
 * "是否还在动"的有效性判据。(第一版取 6/12ms 做 Richardson 外推，12ms
 * 那个点正好落在停机点上，两窗口比值 1.358 远大于理论的 1.057，外推出
 * a0=1038376 完全失真——这就是为什么现在改成单窗口 + 已知 tau。) */
#define CM_BRAKE_T1_MS        8U   /* 有效性检查窗 */
#define CM_BRAKE_T2_MS       16U   /* 主测量窗 */

/* ---- T5 齿隙：换向首次响应延迟 减去 T3 同向纯延迟 = 齿隙穿越时间 ---- */
#define CM_BL_PRE_MS     250U   /* 先朝一个方向顶死齿隙 */
#define CM_BL_PRE_PWM    1200
#define CM_BL_REST_MS    500U
#define CM_BL_TIMEOUT    400U
/* ---- T6 最小可靠位移：递增脉宽，找刚好能产生可重复位移的那一档 ---- */
#define CM_MS_PULSES     6U
#define CM_MS_PWM        1400
#define CM_MS_REST_MS    300U

/* ==================== T9 静摩擦(起转PWM) + 传感器噪声底 ====================
 *
 * 这一项回答的是"停位残差为什么下不去"。控制器的位置死区
 * (TUNE_RESOLUTION_CDEG)本质上是"多小的误差不值得去修"，它由两个物理量
 * 共同决定，而这两个数原来一个都没测过：
 *
 *   噪声底   —— 死区小于它，控制器就是在追噪声；
 *   最小步长 —— 死区小于它，控制器会为一个根本修不动的残差反复踹静摩擦，
 *               窜过头再反向窜，形成极限环。
 *
 * 起转PWM(CAL_BREAKAWAY_PWM)则是让最后那一小段位移**走得快**的前提：
 * T1 的截距只是动摩擦(实测59)，而从静止推动要克服的是静摩擦(实测约200)。
 * 不补这一块，末端只能靠速度环积分以 0.5 计数/拍慢慢顶，实测要爬 600ms。
 *
 * 三个量用同一次实验测出来：
 *   A) 电机短路制动，静置取样 -> 噪声底(峰峰值)
 *   B) 幅值从0缓慢爬升，直到读数变化超过阈值 -> 起转PWM
 *      **必须是缓慢爬升而不是固定脉冲**。固定脉冲(T6 的做法)要么推不动、
 *      要么一推就滑出去一大段，测到的是"脉冲响应"不是"最小步长"；缓慢
 *      爬升在恰好越过静摩擦的那一刻就撤力，走出来的才是机构能做的最小
 *      增量——这正是末端修正的工况。
 *   C) 撤力停稳后量净位移 -> 最小可靠步长
 *
 * 顺带做一个**占空比耦合检查**：施加一个明确推不动的PWM(起转值的一半)，
 * 看角度读数会不会跟着漂。电位器是与PWM同步采样的(见 D_adc.c 的扫描序列
 * 说明，转换会跨过驱动段末尾),如果读数随占空比变化，那就是电气串扰而不是
 * 真实位移——它会被观测器微分成假速度。判别靠三段均值：
 *     mean@0 -> mean@u -> mean@0
 *   两个 mean@0 相等而中间那个偏掉 = 电气耦合；
 *   第二个 mean@0 也跟着偏 = 轴真的动了(起转值估低了)，本次不作数。 */
#define CM_NZ_SAMPLES    500U   /* 噪声底/均值的取样拍数(=ms) */
#define CM_NZ_SETTLE_MS   50U   /* 换工况后先丢掉这么多拍再开始取样 */
#define CM_BA_REST_MS    300U   /* 斜坡前后的停稳时间 */
#define CM_BA_PRE_PWM   1000    /* 同向预载：把齿隙顶到测试方向那一侧 */
#define CM_BA_PRE_MS     150U
#define CM_BA_STEP_PWM      3   /* 每 CM_BA_STEP_MS 抬高的幅值 */
#define CM_BA_STEP_MS     10U   /* 合 300 计数/秒，从0爬到800约2.7秒 */
#define CM_BA_MAX_PWM     800   /* 爬到这里还不动就判本次失败 */
#define CM_BA_THR_MIN_Q4   24   /* "确实动了"的最小阈值，Q4计数(=1.5个LSB) */
#define CM_REP_BA          2U   /* 每方向跑几次(实际次数 = 值*2，正反交替) */
/* ---- T7 扫频：直流偏置(必须>摩擦，让轴一直转)+小幅正弦，单频点锁相 ---- */
#define CM_SW_POINTS    12U
#define CM_SW_BIAS      1200    /* 偏置。轴持续单向转动，编码器量化被自然dither */
#define CM_SW_AMP        400    /* 正弦幅值。偏置±幅值既不换向也不饱和 */
#define CM_SW_SETTLE_MS  250U
#define CM_SW_MIN_MS     400U
#define CM_SW_MAX_MS    2000U
#define CM_SW_CYCLES      20U
/* 信噪比有效带：幅值低于首点的 1/CM_SW_SNR_DIV 之后判谐振没有意义。
 * 实测同一台连跑两次扫频，35Hz 以下逐点偏差 <=11%，45Hz 以上偏差高达 90%
 * ——即幅值已滚降进噪声底。原判据"抬升 12.5% 即谐振"在带外必然误触发，
 * 实测就把一次 143->165 的噪声抬升报成了 45Hz 谐振(会把 w_q 上界压到 15Hz
 * 从而直接编译失败)。 */
#define CM_SW_SNR_DIV      8L
#define CM_SW_RISE_SHIFT   2U   /* 需抬升 > 1/4 才算候选(原为 >>3 即 1/8) */

/* ---- T8 振铃：短脉冲激发 + 自由衰减段锁相，测机构第一模态 ----
 * 为什么不用扫频测这个：对象是一阶低通(转折 4.3Hz)，扫频激励到达编码器时
 * 按 1/f 衰减——200Hz 处预期幅值只有 12.8，而 45~64Hz 实测噪声底已经 40~80，
 * 结构模态会被对象自身的滚降埋掉。振铃测的是**自由响应**，不经过前向通道，
 * 模态有多大就是多大。
 * 上限由采样率定：控制拍 1000Hz，实用到约 250Hz(4点/周期)。 */
#define CM_RING_PWM      2000   /* 激发脉冲幅值。要大到能激起来，又不至于跑远 */
#define CM_RING_MS          6U  /* 脉冲时长(ms)。越短频谱越平，激发的频带越宽 */
#define CM_RING_GAP_MS      4U  /* 脉冲后等几拍再采，躲开驱动本身的暂态 */
#define CM_RING_N         192U  /* 采样点数(=ms)。192ms 下频率分辨率约 5Hz */
#define CM_RING_FPTS       11U
/* 阈值按分辨率等比缩小(6 × 2.197/7.94 ≈ 1.7)。电位器粗 3.6 倍，振铃这种
 * 小幅值自由响应本来就在分辨率边缘，报不出模态(0)是很可能的正常结果。 */
#define CM_RING_MIN_MAG     2L  /* 峰值低于它就判"没测到"，避免把噪声报成模态 */

static const uint16_t CM_RING_HZ[CM_RING_FPTS] =
    { 30, 40, 52, 66, 84, 105, 130, 160, 195, 230, 250 };

#define CM_REP_SPD    1U        /* 每档正反各跑几次(实际次数 = 值*2) */
#define CM_REP_STEP   2U
#define CM_REP_DEAD   2U
#define CM_REP_BRAKE  3U

/* ==================== 状态机 ==================== */

enum { PH_BOOT = 0, PH_CENTER, PH_T0_SIGN, PH_T1_LINE, PH_T2_TAU, PH_T3_DEAD,
       PH_T4_BRAKE, PH_T5_BACKLASH, PH_T9_STICTION, PH_T6_MINSTEP, PH_T7_SWEEP,
       PH_T8_RING, PH_REPORT, PH_DONE };
/* ST_NOISE_A/ST_COUPLE/ST_NOISE_B 只有 T9 用；其余测试项不认识它们，
 * 各自的 switch 会落到 default，行为与原来完全一致。 */
enum { ST_REST = 0, ST_RUN, ST_MEAS, ST_STOP, ST_SAMPLE, ST_NEXT,
       ST_NOISE_A, ST_COUPLE_PRE, ST_COUPLE, ST_NOISE_B };

/* 正反交替，使净位移接近零：偶数次正向、奇数次反向 */
#define CM_DIR()  ((uint8_t)(s_cm.run & 1U))

static struct {
    uint8_t  phase, step, run, level;
    uint16_t timer;
    uint8_t  fail;          /* 1=满PWM不动 2=方向判不出 3=编码器无响应
                             * 4=冲出安全行程 5=回中超时 */
    uint8_t  enc_fail_cnt;

    int32_t  pos;           /* 本拍角度(厘度) */
    int32_t  raw;           /* 本拍ADC原始计数。电位器不回绕，天然连续 */
    int32_t  raw_q4;        /* 同一拍的Q4读数(计数×16)，只给 T9 用，见下 */
    uint8_t  raw_valid;

    uint8_t  next_phase;    /* 回中完成后要进入的测试项 */
    uint8_t  center_flip;   /* 1=回中方向取反(T0之前极性还没测出来) */
    uint8_t  center_level;  /* 到位后要恢复的档位，见 CM_GotoPos */
    uint8_t  center_try;    /* 本次预定位已经逼近了几趟 */
    int32_t  pos_target;    /* 本次预定位的目标角度(厘度) */
    int32_t  fail_pos;      /* 判越界时的角度，报告打出来便于定位问题 */
    uint8_t  fail_phase;    /* 判越界时正在跑哪一项 */
    uint16_t center_chk;    /* 回中方向检查计时 */
    int32_t  center_err0;   /* 上次检查时的回中误差 */

    int32_t  mark_pos;

    int8_t   sign;          /* 正PWM让角度增大=+1 */

    int32_t  spd_sum[CM_LEVELS][2];   /* T1 每档每方向的速度和 */
    uint8_t  spd_n[CM_LEVELS][2];

    int32_t  step_x[2], step_v[2];    /* T2 mark时刻位移 / 稳态速度 */
    uint8_t  step_n[2];

    int32_t  dead_ref;                /* T3 起始原始计数 */
    int32_t  dead_t_sum[CM_DEAD_NPTS];/* 第k个计数跨越时刻之和(ms)，k=0..N-1 */
    uint8_t  dead_t_n[CM_DEAD_NPTS];  /* 各点有效样本数 */
    uint8_t  dead_k;                  /* 本次已跨越几个计数 */
    int32_t  dead_L_x100;             /* T3 结束时算出的纯延迟(0.01ms)，T4 要用 */

    int32_t  brake_v0;                /* T4 本次切换前速度 */
    int32_t  brake_dx1[2], brake_dx2[2]; /* T4 两个窗口的位移(厘度) */
    int32_t  brake_v0_sum[2];
    uint8_t  brake_n[2];
    uint8_t  brake_got1;              /* T4 本次是否已取到第一个窗口 */

    int32_t  bl_sum[2];               /* T5 换向首次响应延迟(拍) */
    uint8_t  bl_n[2];
    int32_t  ms_disp[CM_MS_PULSES];   /* T6 各脉宽产生的位移(厘度) */
    uint8_t  ms_idx;
    uint16_t sw_idx;                  /* T7 当前频点 */
    uint16_t sw_ph;                   /* 相位累加器，Q16 一整圈 */
    uint16_t sw_inc;
    int32_t  sw_i, sw_q;              /* 锁相累加器 */
    int32_t  sw_prev_raw;
    uint32_t sw_n;
    int32_t  ring_ref;                /* T8 采样起点的原始计数 */
    uint16_t ring_idx;
    int16_t  ring_buf[CM_RING_N];     /* 相对起点的计数偏移，每拍一个 */
    int32_t  ring_mag[CM_RING_FPTS];
    int32_t  sw_mag[CM_SW_POINTS];
    int32_t  sw_phase[CM_SW_POINTS];

    /* ---- T9 静摩擦与噪声底 ---- */
    int32_t  ba_mark_q4;    /* 本次斜坡的起点读数，Q4         */
    int32_t  ba_thr_q4;     /* 判"确实动了"的阈值，Q4         */
    int16_t  ba_pwm;        /* 斜坡当前幅值                   */
    int32_t  ba_sum[2];     /* 起转PWM之和，按方向            */
    uint8_t  ba_n[2];
    int32_t  ba_step_sum[2];/* 撤力后净位移之和(厘度)，按方向 */
    uint8_t  ba_got;        /* 本次运行是否测到起转           */
    int32_t  nz_min, nz_max;/* 取样窗内的极值，Q4             */
    int64_t  nz_sum;        /* 取样窗内的和，Q4               */
    uint16_t nz_n;
    int32_t  nz_p2p;        /* 噪声峰峰值，Q4                 */
    int32_t  nz_mean0;      /* 耦合检查前一个 PWM=0 窗口的均值，Q4 */
    int32_t  nz_mean1;      /* PWM=起转/2 的均值，Q4          */
    int32_t  nz_mean2;      /* 再回到 PWM=0 的均值，Q4        */
    int16_t  nz_couple_pwm; /* 耦合检查实际用的幅值           */
    uint8_t  nz_done;       /* 噪声底已经量过                 */
} s_cm;

/* ==================== 工具 ==================== */

static int32_t CM_Abs(int32_t v) { return (v < 0) ? -v : v; }

/*
 * @fn      CM_Isqrt
 * @brief   整数平方根，标定全程不使用浮点
 */
static uint32_t CM_Isqrt(uint32_t v)
{
    uint32_t rem = v, root = 0, bit = 1UL << 30;

    while (bit > rem) bit >>= 2;
    while (bit != 0U)
    {
        if (rem >= root + bit) { rem -= root + bit; root = (root >> 1) + bit; }
        else                     root >>= 1;
        bit >>= 2;
    }
    return root;
}

/*
 * @fn      CM_Diff
 * @brief   两角度之差
 *
 * 电位器是一段有限行程，不存在转过0点这回事，所以**不能**再按最短路径折算：
 * 行程本身有 27000 厘度，比半圈(18000)还长，一折算就会把一个真实的大位移
 * 翻成反号的小位移。原来的折算是给整圈磁编码器写的。
 */
static int32_t CM_Diff(int32_t a, int32_t b)
{
    return a - b;
}

/* 把幅值按"本次运行方向 × 接线极性"变成实际下发的PWM。
 * 乘了 s_cm.sign 之后，dir=0 恒等于"角度增大"的方向，与接线无关。 */
static int16_t CM_Out(int16_t mag)
{
    int16_t v = (CM_DIR() == 0U) ? mag : (int16_t)-mag;

    return (s_cm.sign < 0) ? (int16_t)-v : v;
}

static int32_t CM_Speed(int32_t disp, uint16_t span_ms)
{
    return (int32_t)(((int64_t)disp * 1000) / (int32_t)span_ms);
}

static void CM_Enter(uint8_t step) { s_cm.step = step; s_cm.timer = 0U; }

static void CM_Phase(uint8_t phase)
{
    s_cm.phase = phase;
    s_cm.run   = 0U;
    s_cm.level = 0U;
    D_Motor_Set(0);
    CM_Enter(ST_REST);
}

/* @return 1=本项(或本档)的正反全部跑完 */
static uint8_t CM_Advance(uint8_t reps)
{
    if (++s_cm.run < (uint8_t)(reps * 2U)) return 0U;
    s_cm.run = 0U;
    return 1U;
}

/* ==================== 回中：每项测试的公共起点 ====================
 *
 * 开环激励在有限行程上跑，起点在哪里决定了还剩多少跑道。每项测试之前都先
 * 把轴预定位一次，这不是"锦上添花"而是必需的：
 *
 *   T2 满PWM连跑 700ms 约走 143 度，T1 最高档 550ms 约走 112 度。从正中
 *   (135度)出发，143 度会直接撞进保护带；贴着低端出发才有整段 260 度跑道。
 *
 * 默认定位到行程低端。这样安排是因为每项测试的第一次运行恒为 dir0(角度
 * 增大)，之后正反交替、每对往返回到原处——低端出发正好顺着这个节奏，dir1
 * 那次总是从 dir0 跑到的高处往回走。
 *
 * 两个例外，都得从正中出发让两边都有余量：
 *   T0 —— 接线极性还没测出来，不知道会往哪边转；
 *   T5 —— 第一个动作是反向预载(顶死齿隙)，第一下就是往下走的。
 *
 * 加新测试项之前先问一句：它第一个动作往哪边走？答案决定用哪个起跑点。
 */
static void CM_GotoPos(uint8_t next, int32_t target)
{
    s_cm.pos_target   = target;
    s_cm.next_phase   = next;
    s_cm.center_chk   = 0U;
    s_cm.center_err0  = target - s_cm.pos;
    s_cm.center_level = 0U;   /* 换测试项默认从头开始，T1换档会另行覆盖 */
    s_cm.center_try   = 0U;
    CM_Phase(PH_CENTER);
}

/* 绝大多数测试的起跑点。T0 是例外：那时还不知道往哪转，得从正中出发。 */
static void CM_GotoCenter(uint8_t next) { CM_GotoPos(next, CM_START_LO_CDEG); }

static void CM_Center(void)
{
    int32_t err = s_cm.pos_target - s_cm.pos;
    int16_t mag;

    switch (s_cm.step)
    {
    case ST_REST:
        if (CM_Abs(err) <= CM_CENTER_TOL)
        {
            D_Motor_Set(0);
            CM_Enter(ST_STOP);
            break;
        }
        if (s_cm.timer >= CM_CENTER_TIMEOUT)
        {
            D_Motor_Set(0);
            s_cm.fail = 5U;
            break;
        }

        /* 第一次回中发生在 T0 之前，那时接线极性还没测出来(sign 先当作+1)，
         * 方向可能一开始就是反的。每隔一段看看误差有没有真的变小，没有就
         * 掉头。这同时也兜住了 T0 把极性判错的情况。 */
        if (++s_cm.center_chk >= CM_CENTER_CHECK_MS)
        {
            s_cm.center_chk = 0U;
            if (CM_Abs(err) >= CM_Abs(s_cm.center_err0)) s_cm.center_flip ^= 1U;
            s_cm.center_err0 = err;
        }

        mag = (CM_Abs(err) > CM_CENTER_SLOW_CDEG) ? (int16_t)CM_CENTER_PWM
                                                  : (int16_t)CM_CENTER_SLOW_PWM;
        if (err < 0)          mag = (int16_t)-mag;
        if (s_cm.sign < 0)    mag = (int16_t)-mag;
        if (s_cm.center_flip) mag = (int16_t)-mag;
        D_Motor_Set(mag);
        break;

    default:                    /* 停稳了再交给下一项，避免带着速度开测 */
        D_Motor_Set(0);
        if (s_cm.timer < CM_REST_MS) break;

        /* 停稳后再确认一次：慢挡也还是有滑行量，滑过头就重新逼近。不确认的
         * 话，"到位"只代表松手那一刻到位，不代表下一项开测时还在位。 */
        if (CM_Abs(err) > CM_CENTER_TOL && ++s_cm.center_try < CM_CENTER_TRIES)
        {
            CM_Enter(ST_REST);
            break;
        }

        CM_Phase(s_cm.next_phase);      /* 注意它会把 level/run 清零 */
        s_cm.level = s_cm.center_level; /* T1 换档要把档位接回去 */
        break;
    }
}

/* ==================== T0 接线极性 ==================== */

/*
 * 施加一个明确高于起转值的正PWM，看编码器角度往哪边走。
 * **上机第一件事就是这个**：极性错了后面所有测量都是镜像的，而且闭环会正反馈。
 * 判出来之后，后续所有测试都乘上这个符号，于是 T1/T2 的"正向"恒等于
 * "角度增大方向"，报告里不再需要区分接线。
 */
static void CM_T0_Sign(void)
{
    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_REST_MS)
        {
            s_cm.mark_pos = s_cm.pos;
            CM_Enter(ST_RUN);
        }
        break;

    case ST_RUN:
        D_Motor_Set(CM_SIGN_PWM);          /* 此时 sign 还是 +1，直接下发 */
        if (s_cm.timer >= CM_SIGN_MS)
        {
            int32_t d = CM_Diff(s_cm.pos, s_cm.mark_pos);

            D_Motor_Set(0);
            if (CM_Abs(d) < CM_SIGN_MIN_CDEG) { s_cm.fail = 2U; return; }
            s_cm.sign = (d > 0) ? 1 : -1;
            CM_GotoCenter(PH_T1_LINE);
        }
        break;

    default:
        D_Motor_Set(0);
        CM_Enter(ST_REST);
        break;
    }
}

/* ==================== T1 PWM-速度直线(斜率+截距) ==================== */

/*
 * 整套标定里最重要的一项：**slope 和 coulomb 全靠它**，而这两个数又派生出
 * v_ss / a0 / Ka / creep / 起转 / 全部控制器增益。
 * 每档先跑 CM_SETTLE_MS 进稳态，再用 CM_MEAS_MS 的长基线位移测速。
 */
static void CM_T1_Line(void)
{
    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_REST_MS) CM_Enter(ST_RUN);
        break;

    case ST_RUN:
        D_Motor_Set(CM_Out(CM_PWM[s_cm.level]));
        if (s_cm.timer >= CM_SETTLE_MS)
        {
            s_cm.mark_pos = s_cm.pos;
            CM_Enter(ST_MEAS);
        }
        break;

    case ST_MEAS:
        D_Motor_Set(CM_Out(CM_PWM[s_cm.level]));
        if (s_cm.timer >= CM_MEAS_MS)
        {
            int32_t disp = CM_Abs(CM_Diff(s_cm.pos, s_cm.mark_pos));

            s_cm.spd_sum[s_cm.level][CM_DIR()] += CM_Speed(disp, CM_MEAS_MS);
            s_cm.spd_n[s_cm.level][CM_DIR()]++;
            D_Motor_Set(0);
            CM_Enter(ST_NEXT);
        }
        break;

    default:
        D_Motor_Set(0);
        if (CM_Advance(CM_REP_SPD))
        {
            if (++s_cm.level >= CM_LEVELS)
            {
#if CALIB_CHARACTERIZE_MODEL
                CM_GotoCenter(PH_T2_TAU);
#else
                CM_Phase(PH_REPORT);
#endif
            }
            /* 每档换档时回中：一档正反两次的净位移理论上抵消，但摩擦不
             * 对称会让它慢慢漂，五档下来足够漂出行程。
             * CM_Phase 会清 level，所以要把下一档存进 center_level 带过去，
             * 否则回中一次就退回第0档，T1 永远跑不完。 */
            else
            {
                uint8_t next_level = s_cm.level;

                CM_GotoCenter(PH_T1_LINE);
                s_cm.center_level = next_level;
            }
        }
        else CM_Enter(ST_REST);
        break;
    }
}

/* ==================== T2 机械时间常数 tau ==================== */

/*
 * 满PWM从静止起步，记 CM_STEP_MARK_MS 时刻的累计位移 X，再测稳态速度 v_ss。
 * 一阶系统进入稳态后 x(t) -> v_ss*(t - tau)，即真实位移永远比"一直以 v_ss
 * 匀速"少 v_ss*tau，于是
 *     tau_total = t_mark - X/v_ss
 * 用渐近线法而不是"首次到 63% v_ss"：后者的分辨率等于采样点间隔，
 * 在 tau 只有几十 ms 时会带来 50% 量级的误差。
 *
 * 这里得到的 tau_total 含纯延迟，报告里再扣掉 T3 测出的 L 得到机械时间常数。
 */
static void CM_T2_Tau(void)
{
    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_REST_MS)
        {
            s_cm.mark_pos = s_cm.pos;
            CM_Enter(ST_RUN);
        }
        break;

    case ST_RUN:
        D_Motor_Set(CM_Out(MOTOR_PWM_MAX));
        if (s_cm.timer >= CM_STEP_MARK_MS)
        {
            s_cm.step_x[CM_DIR()] += CM_Abs(CM_Diff(s_cm.pos, s_cm.mark_pos));
            s_cm.mark_pos = s_cm.pos;       /* 稳态测速窗口起点 */
            CM_Enter(ST_MEAS);
        }
        break;

    case ST_MEAS:
        D_Motor_Set(CM_Out(MOTOR_PWM_MAX));
        if (s_cm.timer >= CM_MEAS_MS)
        {
            int32_t disp = CM_Abs(CM_Diff(s_cm.pos, s_cm.mark_pos));

            s_cm.step_v[CM_DIR()] += CM_Speed(disp, CM_MEAS_MS);
            s_cm.step_n[CM_DIR()]++;
            D_Motor_Set(0);
            CM_Enter(ST_NEXT);
        }
        break;

    default:
        D_Motor_Set(0);
        if (CM_Advance(CM_REP_STEP)) CM_GotoCenter(PH_T3_DEAD);
        else                         CM_Enter(ST_REST);
        break;
    }
}

/* ==================== T2/T3 的共用解算 ==================== */

/* 一个编码器计数对应的厘度值 ×1000：36000/16384 = 2.197 厘度 */
/* 一个编码器计数值多少厘度，×1000。电位器：27000厘度/3400计数 = 7.941厘度。
 * (MT6701 是 2.197，粗了 3.6 倍——凡是靠数计数工作的测试都受这个影响。) */
#define CM_LSB_C1000  ((int32_t)(((int32_t)ENCODER_POT_SPAN_CDEG * 1000L) \
                               / (int32_t)ENCODER_POT_ADC_SPAN))

/*
 * @fn      CM_Step_Result
 * @brief   从 T2 累加器取出 v_ss(厘度/秒) 与 tau_tot=L+tau(0.01ms)
 *
 * 面积法：一阶+纯延迟对阶跃的响应在 T>>L+tau 时 x(T)=v_ss*(T-L-tau)，
 * 于是 L+tau = T - x/v_ss。用 400ms 长基线，x 与 v 都是四位数，
 * 结果精度远好于 1ms——**前提是不要像旧实现那样先整除到整毫秒**。
 */
static void CM_Step_Result(int32_t *vss, int32_t *tau_tot_x100)
{
    int32_t v = 0, t = 0;
    uint8_t d, n = 0U;

    for (d = 0U; d < 2U; d++)
    {
        if (s_cm.step_n[d] != 0U)
        {
            int32_t vd = s_cm.step_v[d] / s_cm.step_n[d];
            int32_t xd = s_cm.step_x[d] / s_cm.step_n[d];

            if (vd > 0)
            {
                v += vd;
                t += (int32_t)CM_STEP_MARK_MS * 100
                   - (int32_t)(((int64_t)xd * 100000) / vd);
                n++;
            }
        }
    }
    *vss          = (n != 0U) ? (v / (int32_t)n) : 0;
    *tau_tot_x100 = (n != 0U) ? (t / (int32_t)n) : 0;
}

/*
 * @fn      CM_Cross_T_x100
 * @brief   从静止施加满PWM后走过 k 个编码器计数所需的时间(0.01ms)
 *
 * 一阶对象在 t<<tau 段：x = v_ss*tau*u^2/2*(1-u/3)，u=t/tau。
 * 反解 u 用一次牛顿修正 u ≈ u0*(1+u0/6)，u0=sqrt(2k*LSB/(v_ss*tau))。
 * 不做这个 (1-u/3) 修正的话，k=12 处会低估 5%，直接变成 L 的系统偏差。
 */
static int32_t CM_Cross_T_x100(int32_t k, int32_t vss, int32_t tau_ms)
{
    int64_t a_q20;
    int32_t u0_q10, u_q10;

    if (vss <= 0 || tau_ms <= 0 || k <= 0) return 0;

    a_q20 = (((int64_t)2 * k * CM_LSB_C1000) << 20)
          / ((int64_t)vss * tau_ms);
    if (a_q20 <= 0) return 0;
    if (a_q20 > (1L << 20)) a_q20 = (1L << 20);   /* u>=1 时展开已失效，钳住 */

    u0_q10 = (int32_t)CM_Isqrt((uint32_t)a_q20);
    u_q10  = u0_q10 + (int32_t)(((int64_t)u0_q10 * u0_q10) / (6L * 1024L));
    return (int32_t)(((int64_t)u_q10 * tau_ms * 100) >> 10);
}

/*
 * @fn      CM_Dead_L_x100
 * @brief   由 T3 的 N 点跨越时刻解出纯延迟(0.01ms)，失败返回 -1
 * @param   resid  非NULL时返回残差极差(0.01ms)：模型成立时应远小于 100
 *
 * t_k = L + tau*u_k，斜率恒为 1(u_k 由已测的 v_ss、tau 算出，不是拟合量)，
 * 所以直接取 L = mean(t_k - tau*u_k)，方差比自由斜率拟合小。
 * tau = tau_tot - L 与 L 互相依赖，迭代两轮即收敛(L << tau)。
 */
static int32_t CM_Dead_L_x100(int32_t vss, int32_t tau_tot_ms, int32_t *resid)
{
    int32_t l_x100 = 0;
    uint8_t it, k;

    for (it = 0U; it < 2U; it++)
    {
        int32_t tau = tau_tot_ms - (l_x100 + 50) / 100;
        int64_t acc = 0;
        int32_t n = 0, lo = 0x7FFFFFFF, hi = -0x7FFFFFFF;

        if (tau < 1) tau = 1;
        for (k = 0U; k < CM_DEAD_NPTS; k++)
        {
            int32_t t_x100, d;

            if (s_cm.dead_t_n[k] == 0U) continue;
            t_x100 = (int32_t)(((int64_t)s_cm.dead_t_sum[k] * 100)
                               / (int32_t)s_cm.dead_t_n[k]);
            d = t_x100 - CM_Cross_T_x100((int32_t)k + 1, vss, tau);
            acc += d;
            n++;
            if (d < lo) lo = d;
            if (d > hi) hi = d;
        }
        if (n == 0) return -1;
        l_x100 = (int32_t)(acc / n);
        if (resid != NULL) *resid = hi - lo;
    }
    return (l_x100 < 0) ? 0 : l_x100;
}

/* ==================== T3 纯延迟 ==================== */

/*
 * 先朝**测试方向**预运动并停稳(把齿隙顶死在这一侧，测到的才是纯延迟而不是
 * 齿隙穿越时间)，再施加满PWM，记录**前 CM_DEAD_NPTS 个计数各自的跨越时刻**。
 *
 * 为什么不能只看第一个计数(旧实现的做法)：
 * 满PWM下走过 1 个计数(2.197厘度)本身就要 sqrt(2*LSB/a0) 约 2.7ms，与真实
 * 纯延迟(2~4ms)同量级；而跨越时刻只有 1ms 分辨率。于是"答案"和"分辨率"一样
 * 粗，同一台舵机连跑会在 2/3/4 拍之间跳——实测 11 次得到 3,4,3,2,2,2,2,2,4,3,3。
 *
 * 改法：第 k 个计数的跨越时刻满足
 *     t_k = L + tau*u_k,   其中 u_k 解 u^2*(1-u/3)/2 = k*LSB/(v_ss*tau)
 * 斜率恒为 1(u_k 由已测的 v_ss、tau 算出，不是拟合参数)，所以直接取
 *     L = mean(t_k - tau*u_k)
 * 量化噪声被 sqrt(N*reps) 摊薄：sigma 从 0.29ms 降到约 0.04ms。
 *
 * 主机上用合成响应验证过(见交接记录)：真实 L=2.6/3.0/3.4ms 时新方法给
 * 2.79/3.16/3.58ms，四种采样相位下**读数不变**；旧方法同样条件下在
 * 2.30~3.30ms 之间摆，正好对应实测到的 2/3/4 拍乱跳。
 * 残留 +0.2ms 的固定偏置来自"记录第一个不早于跨越时刻的整毫秒"这个采样
 * 约定，它是稳定的，不影响重复性；只有当真值恰好落在 x.3~x.5 之间时才可能
 * 把取整推高一拍，此时看上面打印的精确 L 自己定。
 */
static void CM_T3_Dead(void)
{
    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(CM_Out(CM_DEAD_PRE_PWM));
        if (s_cm.timer >= CM_DEAD_PRE_MS) CM_Enter(ST_STOP);
        break;

    case ST_STOP:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_DEAD_REST_MS)
        {
            s_cm.dead_ref = s_cm.raw;
            s_cm.dead_k   = 0U;
            CM_Enter(ST_RUN);
        }
        break;

    case ST_RUN:
        D_Motor_Set(CM_Out(MOTOR_PWM_MAX));
        {
            int32_t n = CM_Abs(s_cm.raw - s_cm.dead_ref);

            /* 一拍内可能跨过不止一个计数(越到后面速度越快)，用 while 把
             * 中间没有单独出现过的 k 一并登记到同一时刻。 */
            while (s_cm.dead_k < CM_DEAD_NPTS &&
                   n > (int32_t)s_cm.dead_k)
            {
                s_cm.dead_t_sum[s_cm.dead_k] += (int32_t)s_cm.timer;
                s_cm.dead_t_n[s_cm.dead_k]++;
                s_cm.dead_k++;
            }

            if (s_cm.dead_k >= CM_DEAD_NPTS)
            {
                D_Motor_Set(0);
                CM_Enter(ST_NEXT);
            }
            else if (s_cm.timer > CM_DEAD_TIMEOUT)
            {
                D_Motor_Set(0);
                CM_Enter(ST_NEXT);      /* 超时：已登记的点仍然有效 */
            }
        }
        break;

    default:
        D_Motor_Set(0);
        if (CM_Advance(CM_REP_DEAD))
        {
            /* T4 需要**纯延迟**来决定等多久再取 mark：旧实现等的是"第一个
             * 计数出现"的时刻(5~7ms)，比真实纯延迟(约3ms)长一倍，于是 mark
             * 时轴已经减速了约 3ms，而公式里仍拿切换前的 v0 当起点，
             * 减速度被系统性高估约 30%。这里先把 L 解出来备用。 */
            int32_t vss, tt_x100;

            CM_Step_Result(&vss, &tt_x100);
            s_cm.dead_L_x100 = CM_Dead_L_x100(vss, (tt_x100 + 50) / 100, NULL);
            if (s_cm.dead_L_x100 < 0) s_cm.dead_L_x100 = 0;
            CM_GotoCenter(PH_T4_BRAKE);
        }
        else CM_Enter(ST_REST);
        break;
    }
}

/* ==================== T4 主动制动增益（型号级） ==================== */

/* 取 T3 解出的**纯延迟**(向上取整，至少1拍)作为切换后等待时长。 */
static uint16_t CM_Measured_Dead(void)
{
    int32_t ms = (s_cm.dead_L_x100 + 99) / 100;

    return (uint16_t)((ms >= 1) ? ms : 1);
}

/* 在同一速度点比较“维持速度所需PWM”和“反向制动PWM”。命令差值本身就是
 * 制动净PWM，因此不需要额外知道反电动势或摩擦。
 *
 * 取 T1 与 2*T1 两个窗口的位移，各自反推"等效匀减速度"，再 Richardson
 * 外推回 t=0——因为制动是一阶指数衰减，单窗口测到的是窗口平均值，
 * 窗口一改读数就变(实测 10ms 窗 666~759k，20ms 窗 552~590k，同一台)。
 * 结果用于 MODEL_BRAKE_GAIN_Q8，同型号只需在金样上采用一次。 */
static void CM_T4_Brake(void)
{
    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_REST_MS) CM_Enter(ST_RUN);
        break;

    case ST_RUN:
        D_Motor_Set(CM_Out(CM_BRAKE_DRIVE_PWM));
        if (s_cm.timer >= CM_SETTLE_MS)
        {
            s_cm.mark_pos = s_cm.pos;
            CM_Enter(ST_MEAS);
        }
        break;

    case ST_MEAS:
        D_Motor_Set(CM_Out(CM_BRAKE_DRIVE_PWM));
        if (s_cm.timer >= CM_BRAKE_VEL_MS)
        {
            int32_t dx = CM_Abs(CM_Diff(s_cm.pos, s_cm.mark_pos));

            s_cm.brake_v0 = CM_Speed(dx, CM_BRAKE_VEL_MS);
            D_Motor_Set(CM_Out((int16_t)-CM_BRAKE_CMD_PWM));
            CM_Enter(ST_STOP);
        }
        break;

    case ST_STOP:
        D_Motor_Set(CM_Out((int16_t)-CM_BRAKE_CMD_PWM));
        if (s_cm.timer >= CM_Measured_Dead())
        {
            s_cm.mark_pos   = s_cm.pos;
            s_cm.brake_got1 = 0U;
            CM_Enter(ST_SAMPLE);
        }
        break;

    case ST_SAMPLE:
        D_Motor_Set(CM_Out((int16_t)-CM_BRAKE_CMD_PWM));
        /* 用标志而不是 timer==T1 判等，漏一拍也不会把这个点丢掉 */
        if (s_cm.brake_got1 == 0U && s_cm.timer >= CM_BRAKE_T1_MS)
        {
            s_cm.brake_dx1[CM_DIR()] += CM_Abs(CM_Diff(s_cm.pos, s_cm.mark_pos));
            s_cm.brake_got1 = 1U;
        }
        if (s_cm.timer >= CM_BRAKE_T2_MS)
        {
            if (s_cm.brake_got1 != 0U)
            {
                s_cm.brake_dx2[CM_DIR()] +=
                    CM_Abs(CM_Diff(s_cm.pos, s_cm.mark_pos));
                s_cm.brake_v0_sum[CM_DIR()] += s_cm.brake_v0;
                s_cm.brake_n[CM_DIR()]++;
            }
            D_Motor_Set(0);
            CM_Enter(ST_NEXT);
        }
        break;

    default:
        D_Motor_Set(0);
        /* T5 从正中出发，不能用低端：它第一下是反向预载，见下面的说明 */
        if (CM_Advance(CM_REP_BRAKE)) CM_GotoPos(PH_T5_BACKLASH, CM_CENTER_CDEG);
        else                          CM_Enter(ST_REST);
        break;
    }
}

/* ==================== 拟合与报告 ==================== */

/*
 * @fn      CM_Fit
 * @brief   对 (速度, PWM) 点集做最小二乘，拟合 PWM = intercept + slope*速度
 * @param   dir  0=正向 1=反向；2=两方向合并
 * @return  参与拟合的有效点数
 *
 * 全程 int64 整数，不引入软浮点。速度最大约 2e4、PWM 最大 2400，
 * 10 个点时分子最大约 1e14，不会溢出。
 */
static uint8_t CM_Fit(uint8_t dir, int32_t *slope_q16, int32_t *intercept)
{
    int64_t sv = 0, sp = 0, svv = 0, svp = 0, den, num;
    uint8_t i, d, d0, d1, n = 0U;

    d0 = (dir >= 2U) ? 0U : dir;
    d1 = (dir >= 2U) ? 1U : dir;

    for (i = 0U; i < CM_LEVELS; i++)
        for (d = d0; d <= d1; d++)
        {
            int32_t v;

            if (s_cm.spd_n[i][d] == 0U) continue;
            v = s_cm.spd_sum[i][d] / s_cm.spd_n[i][d];
            if (v <= 0) continue;                 /* 该档没转起来，剔除 */
            sv  += v;
            sp  += CM_PWM[i];
            svv += (int64_t)v * v;
            svp += (int64_t)v * CM_PWM[i];
            n++;
        }

    *slope_q16 = 0;
    *intercept = 0;
    if (n < 2U) return n;

    den = (int64_t)n * svv - sv * sv;
    if (den == 0) return n;

    num        = ((int64_t)n * svp - sv * sp) << 16;
    *slope_q16 = (int32_t)(num / den);
    *intercept = (int32_t)((sp - (((int64_t)*slope_q16 * sv) >> 16)) / n);
    return n;
}

/* ==================== T5 齿隙宽度（型号级） ====================
 *
 * 编码器装在输出轴上，所以电机侧到负载侧的齿隙**在稳态下完全不可见**——
 * 只能从换向时的首次响应延迟里分离出来：
 *
 *     T3 同向延迟  = 纯延迟 L                (齿隙已被预运动顶死在同一侧)
 *     T5 换向延迟  = 纯延迟 L + 齿隙穿越时间
 *     => 穿越时间 t = T5 - T3
 *
 * 穿越期间负载没被顶上，电机近似空载，从静止起步 x ~= a0*t^2/2，
 * 于是齿隙宽度 = a0 * t^2 / 2。
 *
 * 这个数用来判断传动间隙有多大。过齿隙时"电机转了负载
 * 没跟"，任何把估计出的扰动直接补偿到PWM的方案都会在这里冲击、啸叫。
 */
/* 起跑点必须是行程正中，不是低端(见 CM_GotoPos 的说明)：本项第一个动作是
 * 朝**测试方向的反面**顶死齿隙，1200PWM 跑 250ms 约 2473 厘度。dir0 那一次
 * 这一下是向下的，从低端 2000 出发会直接掉到 -473，加滑行就撞穿下沿。
 * 之后的满PWM反向起步最坏(超时400ms)还要再走约 8200 厘度，所以两边都得留。 */
static void CM_T5_Backlash(void)
{
#if (CALIB_MODE >= 1)
    switch (s_cm.step)
    {
    case ST_REST:
        /* 朝**测试方向的反面**驱动，把齿隙顶死到远端 */
        D_Motor_Set(CM_Out((int16_t)-CM_BL_PRE_PWM));
        if (s_cm.timer >= CM_BL_PRE_MS) CM_Enter(ST_STOP);
        break;

    case ST_STOP:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_BL_REST_MS)
        {
            s_cm.dead_ref = s_cm.raw;
            CM_Enter(ST_RUN);
        }
        break;

    case ST_RUN:
        /* 满PWM反向起步：等编码器出现第一个计数变化 */
        D_Motor_Set(CM_Out(MOTOR_PWM_MAX));
        if (s_cm.raw != s_cm.dead_ref)
        {
            s_cm.bl_sum[CM_DIR()] += (int32_t)s_cm.timer;
            s_cm.bl_n[CM_DIR()]++;
            D_Motor_Set(0);
            CM_Enter(ST_NEXT);
        }
        else if (s_cm.timer > CM_BL_TIMEOUT)
        {
            D_Motor_Set(0);
            CM_Enter(ST_NEXT);          /* 超时作废 */
        }
        break;

    default:
        D_Motor_Set(0);
        if (CM_Advance(CM_REP_DEAD)) CM_GotoCenter(PH_T9_STICTION);
        else                         CM_Enter(ST_REST);
        break;
    }
#else
    CM_Phase(PH_REPORT);
#endif
}

/* ==================== T9 静摩擦 + 噪声底（单台） ====================
 *
 * 见上面 CM_NZ_SAMPLES 附近的完整说明。一次实验出三个数：
 *     噪声底(峰峰)  -> TUNE_RESOLUTION_CDEG 的下界
 *     起转PWM       -> CAL_BREAKAWAY_PWM，末端静摩擦前馈用
 *     最小步长      -> TUNE_RESOLUTION_CDEG 的另一个下界
 * 外加一个占空比耦合检查。
 */

/* Q4 计数 -> 厘度×100。最大 65535*7941 < 2^31，不溢出。 */
static int32_t CM_Q4_C100(int32_t q4)
{
    return (int32_t)(((int64_t)q4 * CM_LSB_C1000) / 160);
}

/* 两个方向里较小的那个起转值：耦合检查要保证"确实推不动"。 */
static int32_t CM_Breakaway_Min(void)
{
    int32_t lo = 0;
    uint8_t d;

    for (d = 0U; d < 2U; d++)
    {
        int32_t v;

        if (s_cm.ba_n[d] == 0U) continue;
        v = s_cm.ba_sum[d] / s_cm.ba_n[d];
        if (lo == 0 || v < lo) lo = v;
    }
    return lo;
}

static void CM_Noise_Begin(void)
{
    s_cm.nz_min = 0x7FFFFFFF;
    s_cm.nz_max = -0x7FFFFFFF;
    s_cm.nz_sum = 0;
    s_cm.nz_n   = 0U;
}

static void CM_Noise_Add(void)
{
    if (!s_cm.raw_valid) return;
    if (s_cm.raw_q4 < s_cm.nz_min) s_cm.nz_min = s_cm.raw_q4;
    if (s_cm.raw_q4 > s_cm.nz_max) s_cm.nz_max = s_cm.raw_q4;
    s_cm.nz_sum += s_cm.raw_q4;
    s_cm.nz_n++;
}

static int32_t CM_Noise_Mean(void)
{
    return (s_cm.nz_n != 0U) ? (int32_t)(s_cm.nz_sum / s_cm.nz_n) : 0;
}

static void CM_T9_Stiction(void)
{
    switch (s_cm.step)
    {
    case ST_REST:                       /* 彻底停稳 */
        D_Motor_Set(0);
        if (s_cm.timer < CM_BA_REST_MS) break;
        if (s_cm.nz_done == 0U)
        {
            CM_Noise_Begin();           /* 第一次先量噪声底 */
            CM_Enter(ST_NOISE_A);
        }
        else CM_Enter(ST_MEAS);
        break;

    case ST_NOISE_A:                    /* 电机短路制动，静置取样 */
        D_Motor_Set(0);
        CM_Noise_Add();
        if (s_cm.nz_n >= CM_NZ_SAMPLES)
        {
            s_cm.nz_p2p    = s_cm.nz_max - s_cm.nz_min;
            /* 判"动了"的阈值挂在实测噪声上，而不是一个写死的数：换电位器、
             * 改分压、改平均点数之后它自动跟着走。 */
            /* 2 倍峰峰。峰峰本身约等于 6 sigma，所以这已经是 12 sigma，
             * 误触发概率可以忽略；再往大取只会让斜坡在"轴已经动了但位移还
             * 没到阈值"的那段时间里继续爬，把起转值系统性测高(主机仿真：
             * 注入真值 205，4 倍阈值测成 228，2 倍降到 222)。 */
            s_cm.ba_thr_q4 = 2 * s_cm.nz_p2p;
            if (s_cm.ba_thr_q4 < CM_BA_THR_MIN_Q4)
                s_cm.ba_thr_q4 = CM_BA_THR_MIN_Q4;
            s_cm.nz_done   = 1U;
            CM_Enter(ST_MEAS);
        }
        break;

    case ST_MEAS:                       /* 同向预载，把齿隙顶到测试方向 */
        D_Motor_Set(CM_Out(CM_BA_PRE_PWM));
        if (s_cm.timer >= CM_BA_PRE_MS) { D_Motor_Set(0); CM_Enter(ST_STOP); }
        break;

    case ST_STOP:                       /* 预载后停稳，锁定斜坡起点 */
        D_Motor_Set(0);
        if (s_cm.timer >= CM_BA_REST_MS)
        {
            s_cm.ba_mark_q4 = s_cm.raw_q4;
            s_cm.ba_pwm     = 0;
            s_cm.ba_got     = 0U;
            CM_Enter(ST_RUN);
        }
        break;

    case ST_RUN:                        /* 幅值缓慢爬升，直到轴动 */
    {
        int32_t want = ((int32_t)s_cm.timer / (int32_t)CM_BA_STEP_MS)
                     * CM_BA_STEP_PWM;

        if (want > CM_BA_MAX_PWM) want = CM_BA_MAX_PWM;
        s_cm.ba_pwm = (int16_t)want;
        D_Motor_Set(CM_Out(s_cm.ba_pwm));

        if (CM_Abs(s_cm.raw_q4 - s_cm.ba_mark_q4) >= s_cm.ba_thr_q4)
        {
            s_cm.ba_sum[CM_DIR()] += want;   /* 就是这一刻的幅值 */
            s_cm.ba_n[CM_DIR()]++;
            s_cm.ba_got = 1U;
            D_Motor_Set(0);                  /* 立刻撤力，别让它滑出去 */
            CM_Enter(ST_SAMPLE);
        }
        else if (want >= CM_BA_MAX_PWM)
        {
            D_Motor_Set(0);                  /* 爬到顶还不动，本次不计数 */
            CM_Enter(ST_SAMPLE);
        }
        break;
    }

    case ST_SAMPLE:                     /* 撤力停稳，量这一下到底走了多远 */
        D_Motor_Set(0);
        if (s_cm.timer >= CM_BA_REST_MS)
        {
            if (s_cm.ba_got)
                s_cm.ba_step_sum[CM_DIR()] +=
                    CM_Q4_C100(CM_Abs(s_cm.raw_q4 - s_cm.ba_mark_q4)) / 100;
            CM_Enter(ST_NEXT);
        }
        break;

    case ST_COUPLE_PRE:                 /* 耦合检查的 0 基线 */
        D_Motor_Set(0);
        if (s_cm.timer > CM_NZ_SETTLE_MS) CM_Noise_Add();
        if (s_cm.nz_n >= CM_NZ_SAMPLES)
        {
            s_cm.nz_mean0 = CM_Noise_Mean();
            CM_Noise_Begin();
            CM_Enter(ST_COUPLE);
        }
        break;

    case ST_COUPLE:                     /* 施加一个推不动的幅值，看读数漂不漂 */
        D_Motor_Set(CM_Out(s_cm.nz_couple_pwm));
        if (s_cm.timer > CM_NZ_SETTLE_MS) CM_Noise_Add();
        if (s_cm.nz_n >= CM_NZ_SAMPLES)
        {
            s_cm.nz_mean1 = CM_Noise_Mean();
            D_Motor_Set(0);
            CM_Noise_Begin();
            CM_Enter(ST_NOISE_B);
        }
        break;

    case ST_NOISE_B:                    /* 撤力后再量一次均值，用来分离"真动了" */
        D_Motor_Set(0);
        if (s_cm.timer > CM_NZ_SETTLE_MS) CM_Noise_Add();
        if (s_cm.nz_n >= CM_NZ_SAMPLES)
        {
            s_cm.nz_mean2 = CM_Noise_Mean();
            CM_GotoCenter(PH_T6_MINSTEP);
        }
        break;

    case ST_NEXT:
    default:
        D_Motor_Set(0);
        if (!CM_Advance(CM_REP_BA)) { CM_Enter(ST_REST); break; }

        /* 全部方向跑完，接着做占空比耦合检查。幅值取较小方向起转值的一半，
         * 保证在两个方向上都推不动。 */
        s_cm.nz_couple_pwm = (int16_t)(CM_Breakaway_Min() / 2);
        if (s_cm.nz_couple_pwm < 20) s_cm.nz_couple_pwm = 20;
        CM_Noise_Begin();
        /* **三个窗口必须首尾相连**：0 -> u -> 0。基线不能用本项开头量噪声底
         * 时的那个均值——中间四趟斜坡已经把轴挪走了(主机仿真里挪了约15厘度)，
         * 拿过期的基线去比，正常的位移会被误报成占空比耦合。 */
        CM_Enter(ST_COUPLE_PRE);
        break;
    }
}

/* ==================== T6 最小可靠位移（型号级） ====================
 *
 * 递增脉宽施加同一个PWM，量每个脉宽产生的净位移。第一个稳定超过 1 个
 * 编码器 LSB(2.197厘度) 的档次，就是这套机构"推得动的最小一步"。
 *
 * 它是 TUNE_RESOLUTION_CDEG 的物理依据：死区比它小的话，控制器会为一个
 * 根本修不动的残差反复踹静摩擦，必然窜过头再反向窜，形成极限环。
 */
static void CM_T6_MinStep(void)
{
#if (CALIB_MODE >= 1)
    static const uint16_t k_ms[CM_MS_PULSES] = { 2U, 4U, 6U, 10U, 15U, 25U };

    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_MS_REST_MS)
        {
            s_cm.mark_pos = s_cm.raw;
            CM_Enter(ST_RUN);
        }
        break;

    case ST_RUN:
        D_Motor_Set(CM_Out(CM_MS_PWM));
        if (s_cm.timer >= k_ms[s_cm.ms_idx])
        {
            D_Motor_Set(0);
            CM_Enter(ST_STOP);
        }
        break;

    case ST_STOP:                       /* 等完全停住再量净位移 */
        D_Motor_Set(0);
        if (s_cm.timer >= CM_MS_REST_MS)
        {
            int32_t d = CM_Abs(s_cm.raw - s_cm.mark_pos);
            /* 原始计数 -> 厘度 */
            s_cm.ms_disp[s_cm.ms_idx] = (int32_t)(((int64_t)d * CM_LSB_C1000) / 1000);
            CM_Enter(ST_NEXT);
        }
        break;

    default:
        D_Motor_Set(0);
        if (++s_cm.ms_idx >= CM_MS_PULSES)
        {
#if CM_SWEEP_ENABLED
            CM_GotoCenter(PH_T7_SWEEP);
#elif (CALIB_MODE >= 2)
            CM_GotoCenter(PH_T8_RING);   /* 扫频关掉了，直接去振铃 */
#else
            CM_Phase(PH_REPORT);
#endif
        }
        else
        {
            s_cm.run++;                 /* 下一档换个方向，净位移接近零 */
            CM_Enter(ST_REST);
        }
        break;
    }
#else
    CM_Phase(PH_REPORT);
#endif
}

/* ==================== T7 扫频辨识（型号级） ====================
 *
 * 这是全套标定里**唯一能给出 Delta(jw) 的实验**，也是任何基于逆模型的补偿器其带宽上界
 * ||Q*Delta||inf < 1 的唯一依据。没有它，w_q 只能按纯延迟保守估，
 * 未建模谐振完全裸奔。
 *
 * 三个必须做对的地方：
 *   1) **加直流偏置让轴一直转**。偏置必须显著大于库仑摩擦，否则量到的是
 *      stick-slip 非线性而不是线性对象。轴持续转动还有一个关键副作用：
 *      编码器量化被运动自然 dither 了，锁相平均才能恢复到亚 LSB。
 *   2) **锁相而不是FFT**。每个频点把序列分别乘 sin/cos 再平均(单频点DFT)，
 *      窄带增益极高，能从 1~2 个 LSB 的位置幅值里把信号捞出来。
 *   3) **锁相作用在位置差分上**。位置含匀速斜坡，斜坡对 sin 的积分不为零
 *      会泄漏进结果；差分把斜坡变成常值，常值在整数个周期上积分为零。
 *      代价是传递函数多乘一个 jw，比较形状时不影响。
 *
 * 一阶对象的 |Delta_pos| 随频率单调下降，所以**任何局部回升都是谐振**。
 * 这个判据不依赖对 Kv/tau 的假设，最稳。
 */
/* 四分之一周期正弦表，Q14。65 项，128 字节。
 * **不受 CM_SWEEP_ENABLED 控制**：T8 振铃的锁相分析同样要用它，扫频关掉了
 * 振铃还在跑。只有下面真正属于扫频的频点表和激励函数才进开关。 */
static const int16_t CM_SIN_Q14[65] = {
         0,   402,   804,  1205,  1606,  2006,  2404,  2801,
      3196,  3590,  3981,  4370,  4756,  5139,  5520,  5897,
      6270,  6639,  7005,  7366,  7723,  8076,  8423,  8765,
      9102,  9434,  9760, 10080, 10394, 10702, 11003, 11297,
     11585, 11866, 12140, 12406, 12665, 12916, 13160, 13395,
     13623, 13842, 14053, 14256, 14449, 14635, 14811, 14978,
     15137, 15286, 15426, 15557, 15679, 15791, 15893, 15986,
     16069, 16143, 16207, 16261, 16305, 16340, 16364, 16379,
     16384
};

/* ph: 0..65535 对应 0..2pi。象限对称，64 步/象限(1.4度)，
 * 截断带来的谐波在数千点平均后可忽略。 */
static int32_t CM_Sin_Q14(uint16_t ph)
{
    uint8_t  quad = (uint8_t)(ph >> 14);
    uint16_t idx  = (uint16_t)((ph >> 8) & 0x3FU);

    switch (quad)
    {
    case 0U:  return  CM_SIN_Q14[idx];
    case 1U:  return  CM_SIN_Q14[64U - idx];
    case 2U:  return -CM_SIN_Q14[idx];
    default:  return -CM_SIN_Q14[64U - idx];
    }
}
#define CM_Cos_Q14(ph)  CM_Sin_Q14((uint16_t)((uint16_t)(ph) + 16384U))

#if CM_SWEEP_ENABLED
static const uint16_t CM_SW_HZ[CM_SW_POINTS] =
    { 4U, 5U, 7U, 9U, 12U, 16U, 21U, 27U, 35U, 45U, 55U, 64U };

static uint16_t CM_SW_Dur(uint16_t hz)
{
    uint32_t ms = (uint32_t)CM_SW_CYCLES * 1000UL / hz;

    if (ms < CM_SW_MIN_MS) ms = CM_SW_MIN_MS;
    if (ms > CM_SW_MAX_MS) ms = CM_SW_MAX_MS;
    return (uint16_t)ms;
}

static void CM_T7_Sweep(void)
{
    uint16_t hz = CM_SW_HZ[s_cm.sw_idx];

    switch (s_cm.step)
    {
    case ST_REST:                       /* 只给偏置，等轴转到稳速 */
        D_Motor_Set(CM_Out(CM_SW_BIAS));
        if (s_cm.timer >= CM_SW_SETTLE_MS)
        {
            s_cm.sw_ph  = 0U;
            s_cm.sw_inc = (uint16_t)(((uint32_t)hz * 65536UL) / 1000UL);
            s_cm.sw_i   = 0;
            s_cm.sw_q   = 0;
            s_cm.sw_n   = 0U;
            s_cm.sw_prev_raw = s_cm.raw;
            CM_Enter(ST_MEAS);
        }
        break;

    case ST_MEAS:
        {
            int32_t s = CM_Sin_Q14(s_cm.sw_ph);
            int32_t d = s_cm.raw - s_cm.sw_prev_raw;   /* 差分掉匀速斜坡 */

            s_cm.sw_prev_raw = s_cm.raw;
            /* 激励 */
            D_Motor_Set(CM_Out((int16_t)(CM_SW_BIAS + ((CM_SW_AMP * s) >> 14))));
            /* 锁相累加(单频点DFT) */
            s_cm.sw_i += d * s;
            s_cm.sw_q += d * CM_Cos_Q14(s_cm.sw_ph);
            s_cm.sw_n++;
            s_cm.sw_ph = (uint16_t)(s_cm.sw_ph + s_cm.sw_inc);

            if (s_cm.timer >= CM_SW_Dur(hz)) CM_Enter(ST_NEXT);
        }
        break;

    default:
        {
            /* 归一化并降到 uint32 平方不溢出的量级 */
            int32_t ii = (s_cm.sw_n != 0U) ? (int32_t)(s_cm.sw_i / (int32_t)s_cm.sw_n) : 0;
            int32_t qq = (s_cm.sw_n != 0U) ? (int32_t)(s_cm.sw_q / (int32_t)s_cm.sw_n) : 0;
            int32_t a  = ii >> 4;
            int32_t b  = qq >> 4;

            s_cm.sw_mag[s_cm.sw_idx]   = (int32_t)CM_Isqrt((uint32_t)(a * a + b * b));
            s_cm.sw_phase[s_cm.sw_idx] = ii;   /* 同相分量，供上位机算相位 */
            s_cm.sw_q = qq;

            if (++s_cm.sw_idx >= CM_SW_POINTS)
            {
                D_Motor_Set(0);
                CM_Phase(PH_T8_RING);
            }
            else
            {
                CM_Enter(ST_REST);
            }
        }
        break;
    }
}
#else

static void CM_T7_Sweep(void) { CM_Phase(PH_T8_RING); }
#endif

/* ==================== T8 振铃：机构第一模态 ==================== */

/*
 * 打一个 CM_RING_MS 的短脉冲把机构"敲"一下，输出立刻归零，然后**在开环、
 * 零输出的情况下**采 CM_RING_N 拍的编码器。此时轴上只剩自由衰减的振荡，
 * 分析它的频率就得到机构第一模态——这正是输入整形(input shaping)需要的参数。
 *
 * 用零输出而不是闭环采样：闭环会主动镇定这个振荡，反而把要测的东西抹掉。
 */
static void CM_T8_Ring(void)
{
    switch (s_cm.step)
    {
    case ST_REST:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_REST_MS) CM_Enter(ST_RUN);
        break;

    case ST_RUN:
        D_Motor_Set(CM_Out(CM_RING_PWM));
        if (s_cm.timer >= CM_RING_MS)
        {
            D_Motor_Set(0);
            CM_Enter(ST_STOP);
        }
        break;

    case ST_STOP:
        D_Motor_Set(0);
        if (s_cm.timer >= CM_RING_GAP_MS)
        {
            s_cm.ring_ref = s_cm.raw;
            s_cm.ring_idx = 0U;
            CM_Enter(ST_MEAS);
        }
        break;

    case ST_MEAS:
        D_Motor_Set(0);
        if (s_cm.ring_idx < CM_RING_N)
        {
            int32_t dx = s_cm.raw - s_cm.ring_ref;

            if (dx >  32000) dx =  32000;
            if (dx < -32000) dx = -32000;
            s_cm.ring_buf[s_cm.ring_idx++] = (int16_t)dx;
        }
        if (s_cm.ring_idx >= CM_RING_N) CM_Enter(ST_NEXT);
        break;

    default:
        D_Motor_Set(0);
        CM_Phase(PH_REPORT);
        break;
    }
}

static void CM_Report(void)
{
    int32_t slope[3] = {0,0,0}, inter[3] = {0,0,0};
    int32_t vss = 0, tau_tot = 0, dead_ms = 0;
    int32_t a_full, tcross_x100, l_x100, tau_mech, ka_x100;
    int32_t brake_a = 0, brake_gain_q8 = 0;
    int32_t worst = 0;
    int32_t ba_out = 0;       /* T9 起转PWM(两方向取大) */
    int32_t nz_c100 = 0;      /* T9 噪声底峰峰，厘度×100 */
    int32_t step_ramp = 0;    /* T9 缓升法测到的最小步长(厘度) */
    int32_t res_reco = 0;     /* 推荐的 TUNE_RESOLUTION_CDEG */
    uint8_t i, d, n;

    printf("\r\n===== SERVO MINIMAL CALIBRATION =====\r\n");
    if (s_cm.fail != 0U)
    {
        printf("ABORTED fail=%d  (1=no motion at full PWM, 2=direction "
               "undetectable, 3=pot reads dead zone, 4=ran past safe travel, "
               "5=centering timeout)\r\n", (int)s_cm.fail);
        if (s_cm.fail == 3U)
            printf("The wiper is off the track. Turn the shaft back onto the "
                   "pot range by hand, then power-cycle.\r\n");
        else if (s_cm.fail == 4U)
            printf("Left the band at %ld cdeg in phase %u (allowed %ld..%ld)."
                   "\r\n", (long)s_cm.fail_pos, (unsigned)s_cm.fail_phase,
                   (long)CM_TRAVEL_LO, (long)CM_TRAVEL_HI);
        else if (s_cm.fail == 5U)
            printf("Could not reach the start position; check the shaft is "
                   "free and the pot constants in A_Sensor.h.\r\n");
        else
            printf("Check wiring, supply, and that the output shaft turns "
                   "freely.\r\n");
        return;
    }

    /* ---- T1 直线 ---- */
    printf("\r\n[T1] PWM-speed line   (dir0 = increasing angle)\r\n");
    printf("   PWM    dir0    dir1  (cdeg/s)\r\n");
    for (i = 0U; i < CM_LEVELS; i++)
        printf("  %4d  %6ld  %6ld\r\n", (int)CM_PWM[i],
               (long)(s_cm.spd_n[i][0] ? s_cm.spd_sum[i][0] / s_cm.spd_n[i][0] : 0),
               (long)(s_cm.spd_n[i][1] ? s_cm.spd_sum[i][1] / s_cm.spd_n[i][1] : 0));

    for (d = 0U; d < 2U; d++) (void)CM_Fit(d, &slope[d], &inter[d]);
    n = CM_Fit(2U, &slope[2], &inter[2]);

    if (n < 6U || slope[2] <= 0 || inter[2] < 0 || inter[2] >= MOTOR_PWM_MAX)
    {
        printf("  RESULT INVALID: speed-line fit failed; no macros emitted.\r\n");
        printf("  Check free shaft, supply stability and the lowest PWM level.\r\n");
        return;
    }

    /* 最大拟合残差：线性模型到底成不成立看这个数。远大于几个PWM计数说明
     * PWM-速度关系不是直线(供电压降/磁饱和)，此时单一斜率的速度前馈会在
     * 某个速度段系统性偏差，应改分段或查表。 */
    for (i = 0U; i < CM_LEVELS; i++)
        for (d = 0U; d < 2U; d++)
        {
            int32_t v, res;

            if (s_cm.spd_n[i][d] == 0U) continue;
            v = s_cm.spd_sum[i][d] / s_cm.spd_n[i][d];
            if (v <= 0) continue;
            res = CM_Abs((int32_t)CM_PWM[i]
                         - (inter[2] + (int32_t)(((int64_t)slope[2] * v) >> 16)));
            if (res > worst) worst = res;
        }

    printf("  dir0 slope=%ld inter=%ld | dir1 slope=%ld inter=%ld\r\n",
           (long)slope[0], (long)inter[0], (long)slope[1], (long)inter[1]);
    printf("  merged slope=%ld inter=%ld  (n=%u pts, max residual=%ld PWM)\r\n",
           (long)slope[2], (long)inter[2], (unsigned)n, (long)worst);
    if (slope[0] > 0 && slope[1] > 0)
    {
        int32_t sd = CM_Abs(slope[0] - slope[1]) * 100 / slope[2];
        int32_t id = CM_Abs(inter[0] - inter[1]) * 100
                   / ((inter[2] > 0) ? inter[2] : 1);

        printf("  direction spread: slope %ld%%  intercept %ld%%", (long)sd, (long)id);
        printf("%s\r\n", (sd > 10 || id > 25)
               ? "   <-- WARN: too large to merge, keep per-direction values"
               : "   (small enough to merge)");
    }
    if (worst > 40)
        printf("  WARN residual %ld PWM: PWM-speed is not a straight line\r\n",
               (long)worst);

    if (CALIB_CHARACTERIZE_MODEL)
    {
        /* ---- T2 tau ---- */
        if (s_cm.step_n[0] == 0U || s_cm.step_n[1] == 0U)
        {
            printf("\r\nRESULT INVALID: step response missing; no macros emitted.\r\n");
            return;
        }
    {
        int32_t tt_x100;

        for (d = 0U; d < 2U; d++)
            if (s_cm.step_n[d] != 0U)
            {
                int32_t v = s_cm.step_v[d] / s_cm.step_n[d];
                int32_t x = s_cm.step_x[d] / s_cm.step_n[d];
                int32_t t100 = (v > 0)
                             ? ((int32_t)CM_STEP_MARK_MS * 100
                                - (int32_t)(((int64_t)x * 100000) / v)) : 0;

                printf("\r\n[T2] dir%u  v_ss=%ld cdeg/s  x(%ums)=%ld  "
                       "L+tau=%ld.%02ldms\r\n",
                       (unsigned)d, (long)v, (unsigned)CM_STEP_MARK_MS, (long)x,
                       (long)(t100 / 100), (long)(t100 % 100));
            }
        CM_Step_Result(&vss, &tt_x100);
        tau_tot = (tt_x100 + 50) / 100;
        if (tau_tot < 1) tau_tot = 1;
    }

    /* ---- T3 纯延迟：N 点跨越时刻回归 ---- */
    {
        int32_t resid = 0;
        uint8_t k, have = 0U;

        for (k = 0U; k < CM_DEAD_NPTS; k++) if (s_cm.dead_t_n[k]) have++;
        if (have < 3U)
        {
            printf("\r\nRESULT INVALID: delay measurement missing; "
                   "no macros emitted.\r\n");
            return;
        }

        l_x100 = CM_Dead_L_x100(vss, tau_tot, &resid);
        if (l_x100 < 0)
        {
            printf("\r\nRESULT INVALID: delay regression failed; "
                   "no macros emitted.\r\n");
            return;
        }

        printf("\r\n[T3] pure delay L=%ld.%02ldms  (%u/%u counts used, "
               "residual spread %ld.%02ldms)\r\n",
               (long)(l_x100 / 100), (long)(l_x100 % 100),
               (unsigned)have, (unsigned)CM_DEAD_NPTS,
               (long)(resid / 100), (long)(resid % 100));
        printf("     k  t_meas(ms)  t_model(ms)   -> 两列之差应处处等于 L\r\n");
        for (k = 0U; k < CM_DEAD_NPTS; k++)
        {
            int32_t tm, tmod;

            if (s_cm.dead_t_n[k] == 0U) continue;
            tm   = (int32_t)(((int64_t)s_cm.dead_t_sum[k] * 100)
                             / (int32_t)s_cm.dead_t_n[k]);
            tmod = CM_Cross_T_x100((int32_t)k + 1, vss,
                                   tau_tot - (l_x100 + 50) / 100);
            printf("   %3u  %6ld.%02ld  %8ld.%02ld\r\n", (unsigned)(k + 1U),
                   (long)(tm / 100), (long)(tm % 100),
                   (long)(tmod / 100), (long)(tmod % 100));
        }
        /* 残差大不一定是噪声。分别算前半段和后半段的均值：
         *   两者接近 -> 随机散布，是测量噪声，L 仍可信；
         *   单调偏移 -> 系统性，一阶模型没描述住起步段(传动柔性/静摩擦
         *               脱离，输出轴早期落后于电机)，此时 L 依赖于你把权重
         *               放在曲线哪一段，打印的均值只是一个折中。
         * 实测这台是后者：残差从 k=1 的 3.78ms 单调降到 k=12 的 2.56ms。 */
        {
            int64_t s_lo = 0, s_hi = 0;
            int32_t n_lo = 0, n_hi = 0, trend;

            for (k = 0U; k < CM_DEAD_NPTS; k++)
            {
                int32_t t_x100, d2;

                if (s_cm.dead_t_n[k] == 0U) continue;
                t_x100 = (int32_t)(((int64_t)s_cm.dead_t_sum[k] * 100)
                                   / (int32_t)s_cm.dead_t_n[k]);
                d2 = t_x100 - CM_Cross_T_x100((int32_t)k + 1, vss,
                                              tau_tot - (l_x100 + 50) / 100);
                if (k < CM_DEAD_NPTS / 2U) { s_lo += d2; n_lo++; }
                else                       { s_hi += d2; n_hi++; }
            }
            trend = (n_lo && n_hi)
                  ? (int32_t)(s_lo / n_lo) - (int32_t)(s_hi / n_hi) : 0;

            if (resid > 150)
            {
                if (trend > 80 || trend < -80)
                    printf("     NOTE 残差%ld.%02ldms 呈单调趋势(前半-后半="
                           "%ld.%02ldms): 起步段非一阶(传动柔性/静摩擦)，\r\n"
                           "          这是对象的真实性质不是测量故障；L 取的是"
                           "全段均值，重复性仍好\r\n",
                           (long)(resid / 100), (long)(resid % 100),
                           (long)(trend / 100), (long)(trend % 100));
                else
                    printf("     WARN 残差%ld.%02ldms 且无趋势=随机散布: "
                           "L 不可信(检查齿隙是否顶死/供电是否跌落)\r\n",
                           (long)(resid / 100), (long)(resid % 100));
            }
        }
    }

    tau_mech = tau_tot - (l_x100 + 50) / 100;
    if (tau_mech < 1) tau_mech = 1;
    (void)dead_ms;
    (void)a_full;
    (void)tcross_x100;

    /* ---- 交叉校验：独立测出来的 v_ss 应该等于直线算出来的 ----
     * 两条完全不同的实验路径(阶跃 vs 稳态扫描)得到同一个物理量，
     * 差得多说明其中一项不可信——通常是供电压降或机构卡滞。 */
    if (slope[2] > 0)
    {
        int32_t vss_line = (int32_t)((((int64_t)(MOTOR_PWM_MAX - inter[2])) << 16)
                                     / slope[2]);
        int32_t err = (vss > 0) ? (CM_Abs(vss_line - vss) * 100 / vss) : 0;

        printf("\r\n[XCHK] v_ss: step=%ld  line=%ld  diff=%ld%%%s\r\n",
               (long)vss, (long)vss_line, (long)err,
               (err > 10) ? "   <-- WARN: inconsistent, re-run" : "");
    }

    ka_x100 = (int32_t)((65536LL * 1000 * 100)
                        / ((int64_t)slope[2] * tau_mech));

    if (s_cm.brake_n[0] == 0U || s_cm.brake_n[1] == 0U)
    {
        printf("\r\n[T4] SKIPPED: active-brake measurement missing;"
               " keep the existing MODEL_BRAKE_GAIN_Q8.\r\n");
        brake_gain_q8 = 0;
    }
    else
    {
        int32_t n_ok = 0;

        for (d = 0U; d < 2U; d++)
        {
            int32_t v0  = s_cm.brake_v0_sum[d] / s_cm.brake_n[d];
            int32_t dx1 = s_cm.brake_dx1[d] / s_cm.brake_n[d];
            int32_t dx2 = s_cm.brake_dx2[d] / s_cm.brake_n[d];
            /* dx = v0*T/1000 - a*T^2/2e6  =>  a_bar = 2000*(v0*T - dx*1000)/T^2 */
            int64_t df2 = (int64_t)v0 * CM_BRAKE_T2_MS - (int64_t)dx2 * 1000;
            int32_t a_bar = (int32_t)((df2 * 2000)
                                      / ((int32_t)CM_BRAKE_T2_MS * CM_BRAKE_T2_MS));
            int32_t q_q16, corr_q16, a0;

            printf("\r\n[T4] dir%u v0=%ld  dx(%ums)=%ld dx(%ums)=%ld  a_bar=%ld",
                   (unsigned)d, (long)v0, (unsigned)CM_BRAKE_T1_MS, (long)dx1,
                   (unsigned)CM_BRAKE_T2_MS, (long)dx2, (long)a_bar);

            /* 有效性：主窗内必须仍在明显运动。后半程位移不足前半程的 1/3
             * 就说明轴已接近停住(甚至被反向PWM拖着倒转)，此时 deficit 里
             * 混进了"停机"而不是"减速"，反演会把 a0 顶到天上——第一版就是
             * 栽在这里(12ms 窗外推出 a0=1038376)。 */
            if (a_bar <= 0 || dx1 <= 0 || (dx2 - dx1) * 3 < dx1)
            {
                printf("   <-- rejected (窗口内已停/反转)\r\n");
                continue;
            }

            /* a0 = a_bar / (1 - q/3 + q^2/12)，q = T/tau，Q16 */
            q_q16    = (int32_t)(((int64_t)CM_BRAKE_T2_MS << 16) / tau_mech);
            corr_q16 = 65536 - q_q16 / 3
                     + (int32_t)((((int64_t)q_q16 * q_q16) >> 16) / 12);
            if (corr_q16 < 32768) corr_q16 = 32768;   /* q 太大时展开失效，钳住 */
            a0 = (int32_t)(((int64_t)a_bar << 16) / corr_q16);
            printf("  -> a0=%ld (q=%ld.%03ld)\r\n", (long)a0,
                   (long)(q_q16 >> 16), (long)(((q_q16 & 0xFFFF) * 1000) >> 16));
            brake_a += a0;
            n_ok++;
        }

        if (n_ok == 0)
        {
            printf("[T4] SKIPPED: 两个方向都不可信;"
                   " keep the existing MODEL_BRAKE_GAIN_Q8.\r\n");
            brake_gain_q8 = 0;
        }
        else
        {
            int32_t opposing = CM_BRAKE_DRIVE_PWM + CM_BRAKE_CMD_PWM;

            brake_a /= n_ok;
            brake_gain_q8 = (int32_t)(((int64_t)opposing * 256 * 65536 * 1000)
                                      / ((int64_t)brake_a * slope[2] * tau_mech));
            /* 区间只用来提示，不再中止整份报告——一个子项失败不该让另外
             * 8 个宏一起拿不到。256 = 制动与驱动增益相同。 */
            if (brake_gain_q8 < 64 || brake_gain_q8 > 1024)
                printf("[T4] WARN: gain ratio %ld 超出 [64,1024]，先别采用\r\n",
                       (long)brake_gain_q8);
        }
    }
    }
    else
    {
        /* 量产快标只更新单台散差最大的斜率/摩擦/方向；动力学三项使用金样值。 */
        vss = (int32_t)((((int64_t)(MOTOR_PWM_MAX - inter[2])) << 16) / slope[2]);
        tau_mech = MODEL_TAU_MS;
        l_x100 = MODEL_DELAY_TICKS * 100;
        brake_gain_q8 = MODEL_BRAKE_GAIN_Q8;
        ka_x100 = (int32_t)((65536LL * 1000 * 100)
                            / ((int64_t)slope[2] * tau_mech));
        printf("\r\n[MODEL] reuse golden-sample tau=%ldms delay=%ld ticks brake_q8=%ld\r\n",
               (long)tau_mech, (long)MODEL_DELAY_TICKS, (long)brake_gain_q8);
    }

    /* ---- T9 噪声底 / 占空比耦合 / 起转PWM / 最小步长 ---- */
    {
        int32_t d0 = s_cm.nz_mean1 - s_cm.nz_mean0;   /* 加力后的读数偏移 */
        int32_t d1 = s_cm.nz_mean2 - s_cm.nz_mean0;   /* 撤力后回没回来 */

        nz_c100 = CM_Q4_C100(s_cm.nz_p2p);

        printf("\r\n\r\n[T9] sensor noise floor (motor braked, %u ticks)\r\n",
               (unsigned)CM_NZ_SAMPLES);
        printf("     peak-to-peak = %ld.%02ld cdeg  (%ld.%02ld raw LSB)\r\n",
               (long)(nz_c100 / 100), (long)(nz_c100 % 100),
               (long)(s_cm.nz_p2p / 16), (long)((s_cm.nz_p2p % 16) * 100 / 16));

        printf("\r\n[T9] duty-cycle coupling check (u=%d, below breakaway)\r\n",
               (int)s_cm.nz_couple_pwm);
        printf("     mean@0=%ld  mean@u=%ld  mean@0again=%ld  (Q4 counts)\r\n",
               (long)s_cm.nz_mean0, (long)s_cm.nz_mean1, (long)s_cm.nz_mean2);
        printf("     shift under drive = %ld.%02ld cdeg, after release = %ld.%02ld cdeg\r\n",
               (long)(CM_Q4_C100(d0) / 100), (long)(CM_Abs(CM_Q4_C100(d0)) % 100),
               (long)(CM_Q4_C100(d1) / 100), (long)(CM_Abs(CM_Q4_C100(d1)) % 100));
        if (CM_Abs(d1) > s_cm.nz_p2p)
            printf("     -> 轴真的动了(撤力后没回来)：起转值估低了，本项无效\r\n");
        else if (CM_Abs(d0) > 2 * s_cm.nz_p2p)
            printf("     -> **占空比耦合**：轴没动而读数跟着PWM漂，会被观测器\r\n"
                   "        微分成假速度。查 D_adc.c 的扫描序列(电位器的转换\r\n"
                   "        跨过了驱动段末尾)，或把电位器挪到规则组第1位\r\n");
        else
            printf("     -> 无明显耦合，读数与占空比无关(好)\r\n");

        printf("\r\n[T9] breakaway (slow ramp %d cnt / %u ms until motion)\r\n",
               CM_BA_STEP_PWM, (unsigned)CM_BA_STEP_MS);
        for (d = 0U; d < 2U; d++)
        {
            int32_t bv, sv;

            if (s_cm.ba_n[d] == 0U)
            {
                printf("     dir%u  NOT MEASURED (ramped to %d, never moved)\r\n",
                       (unsigned)d, CM_BA_MAX_PWM);
                continue;
            }
            bv = s_cm.ba_sum[d] / s_cm.ba_n[d];
            sv = s_cm.ba_step_sum[d] / s_cm.ba_n[d];
            printf("     dir%u  breakaway=%ld PWM   step-after-release=%ld cdeg"
                   "  (%u runs)\r\n", (unsigned)d, (long)bv, (long)sv,
                   (unsigned)s_cm.ba_n[d]);
            if (bv > ba_out)   ba_out    = bv;
            if (sv > step_ramp) step_ramp = sv;
        }
        if (ba_out > 0 && ba_out <= inter[2])
            printf("     WARN 起转(%ld) <= 动摩擦截距(%ld)：不合物理，重跑一次\r\n",
                   (long)ba_out, (long)inter[2]);
    }

    /* ================= 输出：直接替换 A_Parameter.h 标定区 =================
     * slope 与 tau 都**向上取整**。这不是随手取整——标定误差对闭环的影响
     * 是单边的：把电机估强/估快(slope 或 tau 偏小)会让规划器要求硬件做不到
     * 的加速度，仿真实测总过冲可从113涨到266~1469；估弱/估慢更保守。
     * 所以取整方向一律偏保守。 */
    printf("\r\n===== CAL VALUES -> A_Parameter.h =====\r\n");
    printf("#define CAL_SPEED_SLOPE_Q16 %5ld\r\n", (long)(slope[2] + 1));
    printf("#define CAL_FRICTION_PWM    %5ld\r\n", (long)inter[2]);
    /* 金样实测值向上留1ms裕量；量产快标复用已有型号值，不能每跑一次再+1。 */
    printf("#define MODEL_TAU_MS        %5ld\r\n",
           (long)(tau_mech + (CALIB_CHARACTERIZE_MODEL ? 1 : 0)));
    /* 四舍五入而不是向上取整：L 现在有 0.01ms 分辨率，向上取整会把 3.02ms
     * 判成 4 拍，白白把 w_obs 上界从 22Hz 压到 17Hz。上面已打印精确 L，
     * 落在 x.5 附近时按精确值自己决定。 */
    printf("#define MODEL_DELAY_TICKS   %5ld\r\n", (long)((l_x100 + 50) / 100));
    printf("#define CAL_MOTOR_SIGN  %5d\r\n", (int)s_cm.sign);
    /* 起转取两方向的**较大**值：补少了末端还是要爬，补多了会窜一点，但窜
     * 过头有位置环兜着，爬不动没人管。 */
    if (ba_out > 0)
        printf("#define CAL_BREAKAWAY_PWM   %5ld\r\n", (long)ba_out);
    else
        printf("/* CAL_BREAKAWAY_PWM 未测出(轴没转起来)，检查负载是否卡死 */\r\n");
    if (brake_gain_q8 <= 0)
        printf("/* MODEL_BRAKE_GAIN_Q8 本次未测出，保留 A_Parameter.h 里的现值 */\r\n");
    else
    printf("#define MODEL_BRAKE_GAIN_Q8 %3ld  /* model-level; use golden sample */\r\n",
           (long)brake_gain_q8);

#if (CALIB_MODE >= 1)
    /* ---- T5/T6 的型号级新增项 ---- */
    {
        int32_t bl_rev = 0, bl_fwd, bl_ticks, bl_cdeg, a0;
        uint8_t d, n = 0U;

        for (d = 0U; d < 2U; d++)
            if (s_cm.bl_n[d]) { bl_rev += s_cm.bl_sum[d] / s_cm.bl_n[d]; n++; }
        bl_rev  = (n != 0U) ? (bl_rev / n) : 0;
        bl_fwd  = (int32_t)CM_Measured_Dead();          /* T3 同向纯延迟 */
        bl_ticks = bl_rev - bl_fwd;
        if (bl_ticks < 0) bl_ticks = 0;
        /* 穿越期间负载未被顶上，电机近似空载从静止起步：x = a0*t^2/2 */
        a0 = (int32_t)(((int64_t)vss * 1000) / tau_mech);
        bl_cdeg = (int32_t)(((int64_t)a0 * bl_ticks * bl_ticks) / 2000000);

        /* 齿隙宽度当前控制律没有消费者(没做齿隙补偿)，所以不印成 #define
         * ——印了只会诱导人往参数表里粘一个死宏。它是机构特征量，用于判断
         * 换向工况的固有误差下限。 */
        printf("[T5] backlash = %ld cdeg  (换向延迟%ld拍 - 同向%ld拍 = %ld拍)\r\n"
               "     机构特征量，当前控制律未做齿隙补偿，不需要填进参数表\r\n",
               (long)bl_cdeg, (long)bl_rev, (long)bl_fwd, (long)bl_ticks);

        {
            static const uint16_t k_ms[CM_MS_PULSES] = { 2U, 4U, 6U, 10U, 15U, 25U };
            int32_t min_step = 0, min_pulse = 0, thr, nz_cdeg;
            uint8_t i;

            printf("\r\n[T6] min reliable step   pulse_ms:");
            for (i = 0U; i < CM_MS_PULSES; i++) printf(" %u", (unsigned)k_ms[i]);
            printf("\r\n                          disp_cdeg:");
            for (i = 0U; i < CM_MS_PULSES; i++) printf(" %ld", (long)s_cm.ms_disp[i]);
            printf("\r\n");
            /* 判据改成"3 倍实测噪声底"。原来那个"1 个编码器 LSB(约3厘度)"是
             * 给 MT6701 写的：电位器 1 LSB = 7.94 厘度，3 厘度正好落在噪声
             * 里，于是 2/4/6ms 那几档的读数(实测 7~23 厘度)会被当成真位移，
             * 报出一个根本没测到的"最小步长"。 */
            nz_cdeg = (nz_c100 + 99) / 100;
            thr = 3 * nz_cdeg;
            if (thr < 3) thr = 3;
            for (i = 0U; i < CM_MS_PULSES; i++)
                if (s_cm.ms_disp[i] >= thr) { min_pulse = s_cm.ms_disp[i]; break; }
            printf("     threshold=%ld cdeg (3x noise floor) -> pulse method gives %ld\r\n",
                   (long)thr, (long)min_pulse);

            /* **缓升法(T9)才是末端修正的真实工况**：固定脉冲要么推不动、要么
             * 一推就滑出去一大段，测到的是脉冲响应不是最小增量；缓升在刚越过
             * 静摩擦那一刻就撤力，走出来的才是机构能做的最小一步。两者取小。 */
            min_step = min_pulse;
            if (step_ramp > 0 && (min_step == 0 || step_ramp < min_step))
                min_step = step_ramp;
            if (min_step < 1) min_step = 1;
            /* 最小步长不单独进参数表：它的唯一用途是下面那条
             * TUNE_RESOLUTION_CDEG 的取值依据，直接给结论即可。 */
            printf("[T6/T9] 最小可靠步长 = %ld cdeg  (脉冲法%ld / 缓升法%ld，取小)\r\n",
                   (long)min_step, (long)min_pulse, (long)step_ramp);

            /* ---- 由上面两个实测量直接给出调参面板的分辨率 ---- */
            res_reco = 2 * nz_cdeg;
            if (min_step > res_reco) res_reco = min_step;
            if (res_reco < 5) res_reco = 5;
            printf("\r\n===== 建议的调参值 -> A_Parameter.h 调参面板 =====\r\n");
            printf("#define TUNE_RESOLUTION_CDEG %3ld\r\n", (long)res_reco);
            printf("/* = max(2 x 噪声底 %ld, 最小步长 %ld)。\r\n"
                   "   比噪声底小 = 控制器在追噪声；\r\n"
                   "   比最小步长小 = 为一个修不动的残差反复踹静摩擦，必出极限环。\r\n"
                   "   它会同步放大死区/到位窗/零速阈值，见 A_Parameter.h 的派生式。\r\n"
                   "   这是**保守起点**：填进去跑一轮，静止时听不到来回蹭就可以\r\n"
                   "   再往下压 20%%，压到出现蹭动为止再退回来。 */\r\n",
                   (long)(2 * nz_cdeg), (long)min_step);
        }
    }
#endif

#if CM_SWEEP_ENABLED
    /* ---- T7 扫频：原始锁相结果 + 谐振判定 ---- */
    {
        uint8_t i, res_i = 0U;
        int32_t res_hz = 0;
        int32_t band_hz = (int32_t)CM_SW_HZ[0];   /* SNR 有效带的上边界 */

        printf("\r\n[T7] frequency sweep (bias=%d amp=%d, lock-in on delta-position)\r\n",
               CM_SW_BIAS, CM_SW_AMP);
        printf("     f_Hz  magnitude  in_phase   (magnitude 用于形状比较，不是绝对标定)\r\n");
        for (i = 0U; i < CM_SW_POINTS; i++)
            printf("   %6u %10ld %9ld\r\n", (unsigned)CM_SW_HZ[i],
                   (long)s_cm.sw_mag[i], (long)s_cm.sw_phase[i]);

        /* 一阶对象的幅值随频率**单调下降**，带内任何局部回升都是谐振。
         *
         * "带内"两个字是后加的，也是关键：幅值滚降进噪声底之后判据必然误触发。
         * 实测同一台连跑两次扫频，35Hz 以下逐点偏差 <=11%，45Hz 以上偏差
         * 高达 90%，其中一次把 143->165 的噪声抬升报成了 45Hz 谐振——那会让
         * GUARD_OBS_BW_MAX 收到 15Hz，当前 22Hz 的配置直接编译失败。
         * 所以先划有效带(幅值 >= 首点/8)，只在带内判，且抬升门限从 1/8 提到 1/4。 */
        {
            int32_t floor_mag = s_cm.sw_mag[0] / CM_SW_SNR_DIV;

            for (i = 1U; i < CM_SW_POINTS; i++)
            {
                if (s_cm.sw_mag[i] < floor_mag) break;   /* 出了有效带，停止判定 */
                band_hz = (int32_t)CM_SW_HZ[i];
                if (s_cm.sw_mag[i] > s_cm.sw_mag[i - 1U]
                                   + (s_cm.sw_mag[i - 1U] >> CM_SW_RISE_SHIFT))
                { res_i = i; res_hz = (int32_t)CM_SW_HZ[i]; break; }
            }
            printf("\r\n     SNR 有效带: 4~%ld Hz (幅值 >= 首点/%ld = %ld)\r\n",
                   (long)band_hz, (long)CM_SW_SNR_DIV, (long)floor_mag);
        }

        if (res_hz > 0)
            printf("     ** resonance candidate at %ld Hz (magnitude rose %ld -> %ld) **\r\n"
                   "     建议再跑 2 次确认：真谐振每次都在同一频点，噪声不会。\r\n",
                   (long)res_hz, (long)s_cm.sw_mag[res_i - 1U], (long)s_cm.sw_mag[res_i]);
        else
            printf("     no magnitude rise within SNR band -> no resonance found\r\n");
        printf("#define MODEL_RESONANCE_HZ  %3ld  /* 0=未发现；非0时 w_q 上界再收到 /3 */\r\n",
               (long)res_hz);
    }
#endif

    /* ---- T8 振铃：机构第一模态，输入整形(input shaping)的唯一参数 ----
     * 对**一阶差分**(=速度)做单频点锁相：差分天然滤掉直流和脉冲留下的位移
     * 斜坡，所以不需要额外去趋势。幅值单位任意，只用来比较峰在哪。
     * 判据分三档：测不到 / 峰触边界 / 可用，宁可不给数也不给一个错的数——
     * 整形器用错频率会比不整形更糟(在错频点上反而放大)。 */
    {
        uint16_t k;
        uint8_t  fi, pk = 0U;
        int32_t  best = 0, second = 0;

        for (fi = 0U; fi < CM_RING_FPTS; fi++)
        {
            uint16_t ph  = 0U;
            uint16_t inc = (uint16_t)(((uint32_t)CM_RING_HZ[fi] * 65536UL) / 1000UL);
            int32_t  si = 0, sq = 0, a, b;

            for (k = 1U; k < CM_RING_N; k++)
            {
                int32_t dxx = (int32_t)s_cm.ring_buf[k]
                            - (int32_t)s_cm.ring_buf[k - 1U];

                si += dxx * CM_Sin_Q14(ph);
                sq += dxx * CM_Cos_Q14(ph);
                ph  = (uint16_t)(ph + inc);
            }
            a = si / (int32_t)CM_RING_N;
            b = sq / (int32_t)CM_RING_N;
            s_cm.ring_mag[fi] = (int32_t)CM_Isqrt((uint32_t)((a >> 2) * (a >> 2)
                                                           + (b >> 2) * (b >> 2)));
            if (s_cm.ring_mag[fi] > best)
            {
                second = best; best = s_cm.ring_mag[fi]; pk = fi;
            }
            else if (s_cm.ring_mag[fi] > second)
            {
                second = s_cm.ring_mag[fi];
            }
        }

        printf("\r\n[T8] ring-down  pulse %d cnt x %u ms, then open-loop %u ms\r\n",
               CM_RING_PWM, (unsigned)CM_RING_MS, (unsigned)CM_RING_N);
        printf("     f_Hz  magnitude\r\n");
        for (fi = 0U; fi < CM_RING_FPTS; fi++)
            printf("   %6u %10ld%s\r\n", (unsigned)CM_RING_HZ[fi],
                   (long)s_cm.ring_mag[fi], (fi == pk) ? "   <-- peak" : "");

        if (best < CM_RING_MIN_MAG)
        {
            printf("     peak %ld < floor %ld: no mode detected.\r\n",
                   (long)best, (long)CM_RING_MIN_MAG);
            printf("     振动可能不在输出轴上(编码器看不见)，或幅值小于 1 计数(2.197 厘度)。\r\n");
            printf("     这种情况下输入整形没有可用频率，**不要硬填**。\r\n");
        }
        else if (pk == 0U || pk == (CM_RING_FPTS - 1U))
        {
            printf("     peak at scan edge (%u Hz): 真实模态可能在扫描区间之外，先别用。\r\n",
                   (unsigned)CM_RING_HZ[pk]);
        }
        else
        {
            printf("     peak/second = %ld/%ld", (long)best, (long)second);
            printf("%s\r\n", (second != 0 && best < second * 2)
                   ? "   <-- 峰不够尖锐，重复几次确认是同一频点" : "");
            /* 参数表里能消费这个数的是 MODEL_RESONANCE_HZ(它会自动收紧观测器
             * 带宽上界)。但**必须先确认是真模态**：连跑三次落在同一频点才算，
             * 否则只是噪声底的随机峰，填进去会把 obs_bw 上界压到无法编译。 */
            printf("[T8] 第一模态候选 = %u Hz\r\n"
                   "     连跑三次若都落在这个频点，再填 MODEL_RESONANCE_HZ；\r\n"
                   "     每次都不一样就是噪声，保持 0\r\n",
                   (unsigned)CM_RING_HZ[pk]);
        }
    }

    /* ---- 角度标度自检：标定测不了，但能把该核对的数摆出来 ----
     * 控制环对标度不敏感(反馈和目标同标度)，但协议对它很敏感：标度错了，
     * "270度行程"就不是270度。判据是 slope——同一台机构换传感器不会改变
     * 物理速度，只会改变"一厘度有多大"。 */
    {
        int32_t dps  = (vss > 0) ? (vss / 100) : 0;      /* 厘度/秒 -> 度/秒 */
        int32_t dpsf = (vss > 0) ? ((vss / 10) % 10) : 0; /* 小数第一位 */

        printf("\r\n[SCALE] 角度标度自检(需要量角器，本程序测不了)\r\n");
        printf("   当前假设: ADC %u..%u = %u cdeg (%u.%02u deg), 1 LSB = %ld.%03ld cdeg\r\n",
               (unsigned)ENCODER_POT_ADC_MIN, (unsigned)ENCODER_POT_ADC_MAX,
               (unsigned)ENCODER_POT_SPAN_CDEG,
               (unsigned)(ENCODER_POT_SPAN_CDEG / 100),
               (unsigned)(ENCODER_POT_SPAN_CDEG % 100),
               (long)(CM_LSB_C1000 / 1000), (long)(CM_LSB_C1000 % 1000));
        printf("   按此标度实测 v_ss = %ld cdeg/s = %ld.%ld deg/s\r\n",
               (long)vss, (long)dps, (long)dpsf);
        printf("   核对办法: 装好舵盘，发 #000P0500T2000! 做记号，再发\r\n"
               "   #000P2500T2000!，用量角器量实际转过的角度 A(度)。则\r\n"
               "     ENCODER_POT_SPAN_CDEG 应为 round(101.5 * A)\r\n"
               "     CAL_SPEED_SLOPE_Q16   应乘以 %u / 新span\r\n",
               (unsigned)ENCODER_POT_SPAN_CDEG);
        printf("   (101.5 = 100 * ADC_SPAN / 实际走过的计数，行程两端各留了2度)\r\n");
    }

    printf("\r\n(derived: v_ss=%ld  a0=%ld  Ka=%ld.%02ld cdeg/s2 per count)\r\n",
           (long)vss, (long)(((int64_t)vss * 1000) / tau_mech),
           (long)(ka_x100 / 100), (long)(ka_x100 % 100));
    printf("===== END =====\r\n");
}

/* ==================== 任务入口 ==================== */

/*
 * @fn      CM_ReadEncoder
 * @brief   取一拍电位器读数，同时填 raw(ADC计数)和 pos(厘度)
 * @param   无
 * @return  1=读到有效值
 *
 * 只取一次ADC样本：pos 由 raw 线性换算而来，不去调 A_Encoder_Read——那会
 * 再取一次驱动层锁存的值，两个量可能落在相邻的两个1ms数据块上。T3 靠 1ms
 * 分辨率数计数跨越时刻，差一拍就是差整个动态范围的三分之一。
 *
 * 抽头掉出碳膜(死区)时下拉电阻把读数钳到地，和"没读到"同等对待：标定不做
 * 脱困，那是 A_Servo 的事；这里只要如实报告"轴不在可测范围里"。
 *
 * **标定一律用整数LSB，把驱动给的Q4小数位除掉。** 不是图省事：T3 数的是
 * "第几个编码器计数跨过去了"、T5 数的是"换向后第一次读数变化"，这两项测的
 * 就是量化台阶本身。喂给它们亚LSB的值，每一拍读数都在变，延迟会被测成0。
 * T1/T2 用的是300ms长基线差分，量化早就被摊薄了，也不需要这点精度。
 */
static uint8_t CM_ReadEncoder(void)
{
    uint16_t adc_q4 = 0;

    if (!D_ADC_Encoder_Read(&adc_q4)
        || adc_q4 < (uint16_t)(ENCODER_POT_ADC_FLOAT * D_ADC_ENCODER_SCALE))
        return 0U;

    s_cm.raw = (int32_t)(adc_q4 / D_ADC_ENCODER_SCALE);
    /* T9 量的就是"整数LSB以下还剩多少信息"，必须拿未取整的 Q4。其余测试项
     * 一律用 s_cm.raw(整数LSB)，理由见上面那段说明。 */
    s_cm.raw_q4 = (int32_t)adc_q4;
    s_cm.pos = ((s_cm.raw - (int32_t)ENCODER_POT_ADC_MIN) * CM_LSB_C1000) / 1000;
    s_cm.raw_valid = 1U;
    return 1U;
}

/*
 * @fn      A_Calib_Min_Tick
 * @brief   1ms 标定拍：读编码器 -> 查行程 -> 推进当前测试
 *
 * 单拍读失败保持上一拍——全程都是开环大幅运动，且测速用长基线差分，丢一两
 * 点不影响结果；连续失败才判死。
 */
static void A_Calib_Min_Tick(void)
{
    if (CM_ReadEncoder())
    {
        s_cm.enc_fail_cnt = 0U;
    }
    else if (s_cm.enc_fail_cnt < 0xFFU)
    {
        s_cm.enc_fail_cnt++;
        if (s_cm.enc_fail_cnt >= 10U) s_cm.fail = 3U;
    }

    /* 行程硬保护。开环激励没有闭环兜底，一旦冲出安全带就意味着下一步要滑进
     * 死区——那时轴自己转不回来，测出来的数也全废。回中过程本身是往里走的，
     * 不受这条限制，否则从带外根本没法回来。 */
    if (s_cm.raw_valid && s_cm.fail == 0U
        && s_cm.phase != PH_CENTER && s_cm.phase != PH_REPORT
        && s_cm.phase != PH_DONE
        && (s_cm.pos < CM_TRAVEL_LO || s_cm.pos > CM_TRAVEL_HI))
    {
        D_Motor_Set(0);
        s_cm.fail       = 4U;
        s_cm.fail_pos   = s_cm.pos;
        s_cm.fail_phase = s_cm.phase;
    }

    if (s_cm.fail != 0U && s_cm.phase != PH_REPORT && s_cm.phase != PH_DONE)
    {
        D_Motor_Set(0);
        s_cm.phase = PH_REPORT;
    }

    s_cm.timer++;

    switch (s_cm.phase)
    {
    case PH_CENTER: CM_Center(); break;
    case PH_BOOT:
        D_Motor_Set(0);
        if (s_cm.timer >= 1000U)
        {
            if (!s_cm.raw_valid) { s_cm.fail = 3U; break; }
            printf("\r\nCALIB start (pot encoder), T0~T6/T9+T8, keep the shaft "
                   "free.\r\nTravel %ld cdeg, guard band %ld cdeg each end.\r\n",
                   (long)ENCODER_POT_SPAN_CDEG, (long)CM_GUARD_CDEG);
            CM_GotoPos(PH_T0_SIGN, CM_CENTER_CDEG);
        }
        break;
    case PH_T0_SIGN: CM_T0_Sign(); break;
    case PH_T1_LINE: CM_T1_Line(); break;
    case PH_T2_TAU:  CM_T2_Tau();  break;
    case PH_T3_DEAD: CM_T3_Dead(); break;
    case PH_T4_BRAKE: CM_T4_Brake(); break;
    case PH_T5_BACKLASH: CM_T5_Backlash(); break;
    case PH_T9_STICTION: CM_T9_Stiction(); break;
    case PH_T6_MINSTEP:  CM_T6_MinStep();  break;
    case PH_T7_SWEEP:    CM_T7_Sweep();    break;
    case PH_T8_RING:     CM_T8_Ring();     break;
    case PH_REPORT:
        D_Motor_Set(0);
        CM_Report();                 /* 电机已停，此处允许阻塞式printf */
        s_cm.phase = PH_DONE;
        break;
    case PH_DONE:
    default:
        D_Motor_Set(0);
        break;
    }
}

void A_Calib_Min_Init(void)
{
    memset(&s_cm, 0, sizeof(s_cm));
    s_cm.sign = 1;                   /* T0 之前先当作正接线 */

    (void)CM_ReadEncoder();          /* 读不到就让 PH_BOOT 判 fail=3 */
    s_cm.phase = PH_BOOT;
    D_Motor_Set(0);
}

/* 标定期间只跑标定拍与喂狗：不注册舵机控制和串口命令任务，
 * 避免任何闭环动作干扰开环激励。 */
static void Task_CalibMinWatchdog(void) { D_IWDG_Feed(); }

static Task_t s_tasks[] = {
    TASK_DEF(A_Calib_Min_Tick,      1,  0),
    TASK_DEF(Task_CalibMinWatchdog, 50, 1),
};

void A_Calib_Min_Tasks_Init(void)
{
    C_Task_Init(s_tasks, (uint8_t)(sizeof(s_tasks) / sizeof(s_tasks[0])));
}
