/*
 * A_Calib_Min.c —— 唯一的自动标定程序：3个单台参数+3个型号参数
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
#include "D_mt6701.h"
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
#define CM_DEAD_NPTS     12U
#define CM_SIGN_MS       200U   /* T0 方向检测的施力时长 */
#define CM_SIGN_PWM      900
#define CM_SIGN_MIN_CDEG 50L    /* T0 认为"确实动了"的最小位移 */
#define CM_BRAKE_DRIVE_PWM 1200  /* T4 先建立约100deg/s稳态速度 */
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
#define CM_BRAKE_T1_MS        4U   /* 有效性检查窗 */
#define CM_BRAKE_T2_MS        8U   /* 主测量窗 */

/* ---- T5 齿隙：换向首次响应延迟 减去 T3 同向纯延迟 = 齿隙穿越时间 ---- */
#define CM_BL_PRE_MS     250U   /* 先朝一个方向顶死齿隙 */
#define CM_BL_PRE_PWM    1200
#define CM_BL_REST_MS    500U
#define CM_BL_TIMEOUT    400U
/* ---- T6 最小可靠位移：递增脉宽，找刚好能产生可重复位移的那一档 ---- */
#define CM_MS_PULSES     6U
#define CM_MS_PWM        1400
#define CM_MS_REST_MS    300U
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
#define CM_RING_MIN_MAG     6L  /* 峰值低于它就判"没测到"，避免把噪声报成模态 */

static const uint16_t CM_RING_HZ[CM_RING_FPTS] =
    { 30, 40, 52, 66, 84, 105, 130, 160, 195, 230, 250 };

#define CM_REP_SPD    1U        /* 每档正反各跑几次(实际次数 = 值*2) */
#define CM_REP_STEP   2U
#define CM_REP_DEAD   2U
#define CM_REP_BRAKE  2U

/* ==================== 状态机 ==================== */

enum { PH_BOOT = 0, PH_T0_SIGN, PH_T1_LINE, PH_T2_TAU, PH_T3_DEAD,
       PH_T4_BRAKE, PH_T5_BACKLASH, PH_T6_MINSTEP, PH_T7_SWEEP,
       PH_T8_RING, PH_REPORT, PH_DONE };
enum { ST_REST = 0, ST_RUN, ST_MEAS, ST_STOP, ST_SAMPLE, ST_NEXT };

/* 正反交替，使净位移接近零：偶数次正向、奇数次反向 */
#define CM_DIR()  ((uint8_t)(s_cm.run & 1U))

static struct {
    uint8_t  phase, step, run, level;
    uint16_t timer;
    uint8_t  fail;          /* 1=满PWM不动 2=方向判不出 3=编码器无响应 */
    uint8_t  enc_fail_cnt;

    int32_t  pos;           /* 本拍角度(厘度) */
    int32_t  raw;           /* 本拍原始计数，已连续化(不回绕) */
    int32_t  raw_hw_last;
    uint8_t  raw_valid;
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
 * @brief   两角度之差，按最短路径折算(标定不限行程，可能连续转过0点)
 */
static int32_t CM_Diff(int32_t a, int32_t b)
{
    int32_t d = a - b;

    if (d >  18000L) d -= 36000L;
    if (d < -18000L) d += 36000L;
    return d;
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
            CM_Phase(PH_T1_LINE);
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
                CM_Phase(PH_T2_TAU);
#else
                CM_Phase(PH_REPORT);
#endif
            }
            else                           CM_Enter(ST_REST);
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
        if (CM_Advance(CM_REP_STEP)) CM_Phase(PH_T3_DEAD);
        else                         CM_Enter(ST_REST);
        break;
    }
}

/* ==================== T2/T3 的共用解算 ==================== */

/* 一个编码器计数对应的厘度值 ×1000：36000/16384 = 2.197 厘度 */
#define CM_LSB_C1000  ((int32_t)((36000L * 1000L) / (int32_t)MT6701_RAW_MAX))

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
            CM_Phase(PH_T4_BRAKE);
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
        if (CM_Advance(CM_REP_BRAKE)) CM_Phase(PH_T5_BACKLASH);
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
        if (CM_Advance(CM_REP_DEAD)) CM_Phase(PH_T6_MINSTEP);
        else                         CM_Enter(ST_REST);
        break;
    }
#else
    CM_Phase(PH_REPORT);
#endif
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
            s_cm.ms_disp[s_cm.ms_idx] = (int32_t)(((int64_t)d * 36000) / MT6701_RAW_MAX);
            CM_Enter(ST_NEXT);
        }
        break;

    default:
        D_Motor_Set(0);
        if (++s_cm.ms_idx >= CM_MS_PULSES)
        {
#if (CALIB_MODE >= 2)
            CM_Phase(PH_T7_SWEEP);
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
#if (CALIB_MODE >= 2)
/* 四分之一周期正弦表，Q14。65 项，128 字节。 */
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
    uint8_t i, d, n;

    printf("\r\n===== SERVO MINIMAL CALIBRATION =====\r\n");
    if (s_cm.fail != 0U)
    {
        printf("ABORTED fail=%d  (1=no motion at full PWM, 2=direction "
               "undetectable, 3=encoder dead)\r\n", (int)s_cm.fail);
        printf("Check wiring, supply, and that the output shaft turns freely.\r\n");
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

        printf("#define MODEL_BACKLASH_CDEG %3ld  /* 换向延迟%ld - 同向%ld = %ld拍 */\r\n",
               (long)bl_cdeg, (long)bl_rev, (long)bl_fwd, (long)bl_ticks);

        {
            static const uint16_t k_ms[CM_MS_PULSES] = { 2U, 4U, 6U, 10U, 15U, 25U };
            int32_t min_step = 0;
            uint8_t i;

            printf("\r\n[T6] min reliable step   pulse_ms:");
            for (i = 0U; i < CM_MS_PULSES; i++) printf(" %u", (unsigned)k_ms[i]);
            printf("\r\n                          disp_cdeg:");
            for (i = 0U; i < CM_MS_PULSES; i++) printf(" %ld", (long)s_cm.ms_disp[i]);
            printf("\r\n");
            /* 第一个稳定超过 1 个编码器 LSB(约3厘度)的档 */
            for (i = 0U; i < CM_MS_PULSES; i++)
                if (s_cm.ms_disp[i] >= 3) { min_step = s_cm.ms_disp[i]; break; }
            if (min_step < 3) min_step = 3;
            printf("#define MODEL_MIN_STEP_CDEG %3ld  /* TUNE_RESOLUTION_CDEG 的物理依据 */\r\n",
                   (long)min_step);
        }
    }
#endif

#if (CALIB_MODE >= 2)
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
            printf("#define MODEL_RING_HZ  %3u  /* 机构第一模态(Hz)，输入整形用 */\r\n",
                   (unsigned)CM_RING_HZ[pk]);
        }
    }

    printf("\r\n(derived: v_ss=%ld  a0=%ld  Ka=%ld.%02ld cdeg/s2 per count)\r\n",
           (long)vss, (long)(((int64_t)vss * 1000) / tau_mech),
           (long)(ka_x100 / 100), (long)(ka_x100 % 100));
    printf("===== END =====\r\n");
}

/* ==================== 任务入口 ==================== */

/*
 * @fn      A_Calib_Min_Tick
 * @brief   1ms 标定拍：读编码器 -> 推进当前测试
 *
 * 只读一次 I2C：原始计数是唯一数据源，厘度值由它换算，避免一拍里读两次
 * 造成两个量对应不同时刻。单拍读失败保持上一拍——全程都是开环大幅运动，
 * 且测速用长基线差分，丢一两点不影响结果。
 */
static void A_Calib_Min_Tick(void)
{
    uint16_t raw_hw = D_MT6701_Read_Raw();

    if (raw_hw != MT6701_ANGLE_ERROR)
    {
        s_cm.enc_fail_cnt = 0U;
        if (s_cm.raw_valid)
        {
            int32_t d = (int32_t)raw_hw - s_cm.raw_hw_last;

            if (d >  8192) d -= 16384;      /* 连续化：跨0点按最短路径累加 */
            if (d < -8192) d += 16384;
            s_cm.raw += d;
        }
        else
        {
            s_cm.raw       = (int32_t)raw_hw;
            s_cm.raw_valid = 1U;
        }
        s_cm.raw_hw_last = (int32_t)raw_hw;
        s_cm.pos = (int32_t)(((uint32_t)raw_hw * 36000U) / MT6701_RAW_MAX);
    }
    else if (s_cm.enc_fail_cnt < 0xFFU)
    {
        s_cm.enc_fail_cnt++;
        if (s_cm.enc_fail_cnt >= 10U) s_cm.fail = 3U;
    }

    if (s_cm.fail != 0U && s_cm.phase != PH_REPORT && s_cm.phase != PH_DONE)
    {
        D_Motor_Set(0);
        s_cm.phase = PH_REPORT;
    }

    s_cm.timer++;

    switch (s_cm.phase)
    {
    case PH_BOOT:
        D_Motor_Set(0);
        if (s_cm.timer >= 1000U)
        {
            if (!s_cm.raw_valid) { s_cm.fail = 3U; break; }
            printf("\r\nCALIB start, full T0~T7 ~45s, keep the shaft free.\r\n");
            CM_Phase(PH_T0_SIGN);
        }
        break;
    case PH_T0_SIGN: CM_T0_Sign(); break;
    case PH_T1_LINE: CM_T1_Line(); break;
    case PH_T2_TAU:  CM_T2_Tau();  break;
    case PH_T3_DEAD: CM_T3_Dead(); break;
    case PH_T4_BRAKE: CM_T4_Brake(); break;
    case PH_T5_BACKLASH: CM_T5_Backlash(); break;
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
    uint16_t raw_hw;

    memset(&s_cm, 0, sizeof(s_cm));
    s_cm.sign = 1;                   /* T0 之前先当作正接线 */

    raw_hw = D_MT6701_Read_Raw();
    if (raw_hw != MT6701_ANGLE_ERROR)
    {
        s_cm.raw         = (int32_t)raw_hw;
        s_cm.raw_hw_last = (int32_t)raw_hw;
        s_cm.raw_valid   = 1U;
        s_cm.pos = (int32_t)(((uint32_t)raw_hw * 36000U) / MT6701_RAW_MAX);
    }
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
