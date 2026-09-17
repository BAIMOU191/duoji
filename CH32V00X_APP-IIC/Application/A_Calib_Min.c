/*
 * A_Calib_Min.c —— 自动标定程序，两种编码器共用；差异收在"编码器抽象"一段，测试项只认连续计数 raw 与厘度 pos。
 * 只测互相独立的量(其余由 A_Parameter.h 派生，分别测会因测量噪声互相矛盾)：
 *   T0 接线方向、T1 slope/coulomb(单台散差最大)、T2 tau、T3 纯延迟、T4 制动比、T9 起转与噪声底；
 *   tau/纯延迟/制动比为型号级常数。
 * 标定误差是单边的：把对象估强/估快(slope 或 tau 偏小)会让规划器要求做不到的加速度，故两者一律向上取整。
 */

#include "A_Calib_Min.h"
#include "A_Parameter.h"
#include "C_Task_Scheduler.h"
#include "D_motor.h"
#include "D_adc.h"      /* 电位器通道的ADC原始计数 */
#include "D_mt6701.h"   /* 磁编码器的原始计数 */
#include "A_Sensor.h"   /* 编码器模式、标定常数与死区门限 */
#include "D_iwdg.h"
#include <stdio.h>
#include <string.h>

/* ==================== 可调项(换更大扭力舵机时复核) ==================== */

/* 单一全量模式：一次跑完全部测试项并输出全部宏(分档会让快标印出金样的旧 tau) */
#define CALIB_MODE 2
#define CALIB_CHARACTERIZE_MODEL 1

/* 电位器与磁编码器的结构性差异：行程有限(每拍越界即停 + 每项测试前预定位)；
 * 分辨率粗3.6倍(7.94 vs 2.197 厘度/LSB，T3/T6/T8 裕度被吃掉)；扫频(T7)做不了。 */

/* 编码器抽象：CM_HAS_TRAVEL=行程是否有限；CM_SWEEP_ENABLED=能否扫频(需持续单向转好几圈)；
 * CM_SUB_SCALE=驱动发布的亚LSB倍数(电位器Q4；磁编码器静止无dither取1)，只有T9用。 */
#if ENCODER_MODE == 0
#define CM_HAS_TRAVEL     1
#define CM_SWEEP_ENABLED  0
#define CM_SUB_SCALE      ((int32_t)D_ADC_ENCODER_SCALE)
/* 一个编码器计数值多少厘度，×1000。电位器：27000厘度/3400计数 = 7.941厘度。 */
#define CM_LSB_C1000      ((int32_t)(((int32_t)ENCODER_POT_SPAN_CDEG * 1000L) \
                                   / (int32_t)ENCODER_POT_ADC_SPAN))
/* 保护带按可读范围画(碳膜比标定行程两端各多10度)，要防的是抽头滑出碳膜 */
#define CM_GUARD_CDEG     500L  /* 距可读两端还有这么多厘度就判越界(5度) */
#define CM_TRAVEL_LO      (ENCODER_ANGLE_LO + CM_GUARD_CDEG)
#define CM_TRAVEL_HI      (ENCODER_ANGLE_HI - CM_GUARD_CDEG)
#define CM_CENTER_CDEG    ((int32_t)ENCODER_POT_SPAN_CDEG / 2)
#else
#define CM_HAS_TRAVEL     0
#define CM_SWEEP_ENABLED  1
#define CM_SUB_SCALE      1
/* 一个编码器计数值多少厘度，×1000。MT6701：36000厘度/16384计数 = 2.197厘度。 */
#define CM_LSB_C1000      ((int32_t)((36000L * 1000L) / (int32_t)MT6701_RAW_MAX))
#define CM_GUARD_CDEG     0L
#define CM_TRAVEL_LO      0L
#define CM_TRAVEL_HI      0L
#define CM_CENTER_CDEG    0L
#endif

/* 单向激励起跑点贴行程低端(T2满PWM要走约143度)；离保护带留25度给预定位松手后的滑行 */
#define CM_START_LO_CDEG  2000L
#define CM_CENTER_TOL     300L  /* 到位判据(3度)，标定不需要精确定位 */
#define CM_CENTER_PWM     900   /* 远距离接近幅值 */
/* 接近目标时换小幅值，压低滑行量 */
#define CM_CENTER_SLOW_CDEG 2500L /* 误差小于它就换慢挡 */
#define CM_CENTER_SLOW_PWM  600
#define CM_CENTER_CHECK_MS 150U /* 每隔这么久检查一次方向对不对 */
#define CM_CENTER_TIMEOUT 4000U /* 单趟接近超时(ms) */
#define CM_CENTER_TRIES      3U /* 滑过头了最多重新逼近几趟 */

/* T1 速度直线档位，最低档须高于起转PWM(转不起来的档自动剔除) */
#define CM_LEVELS      5U
static const int16_t CM_PWM[CM_LEVELS] = { 700, 1120, 1540, 1960, 2380 };

#define CM_SETTLE_MS     250U   /* 施加PWM后等待进稳态 */
#define CM_MEAS_MS       300U   /* 测速窗口，长基线摊薄量化 */
#define CM_REST_MS       250U   /* 每次运动之间的静置 */
#define CM_STEP_MARK_MS  400U   /* T2 阶跃的取样时刻，必须已进稳态 */
#define CM_DEAD_PRE_MS   200U   /* T3 预运动，用于把齿隙顶死到测试方向 */
#define CM_DEAD_PRE_PWM  1200
#define CM_DEAD_REST_MS  500U   /* T3 预运动后必须完全停稳 */
#define CM_DEAD_TIMEOUT  300U
/* T3 记录前N个计数各自的跨越时刻回归求纯延迟，量化噪声按 sqrt(N) 摊薄；N 过大二次近似失效 */
/* 电位器LSB粗3.6倍，跨越同样计数耗时更长，N取6折中 */
#define CM_DEAD_NPTS      6U
#define CM_SIGN_MS       200U   /* T0 方向检测的施力时长 */
#define CM_SIGN_PWM      900
#define CM_SIGN_MIN_CDEG 50L    /* T0 认为"确实动了"的最小位移 */
/* 满驱动：制动亏损量只与窗口长度有关，满速才能让测量窗整个落在轴停住之前且亏损量够几个LSB */
#define CM_BRAKE_DRIVE_PWM MOTOR_PWM_MAX /* T4 先建立满速稳态 */
#define CM_BRAKE_CMD_PWM    300  /* 随后施加反向PWM，测主动制动增益 */
#define CM_BRAKE_VEL_MS     100U
/* T4：制动为一阶指数衰减，已知 tau 后单窗口解析反演
 *     a0 = a_bar / (1 - q/3 + q^2/12),  a_bar = 2*deficit/T^2,  q = T/tau
 * 窗口须整个落在轴停住之前，另一窗口用作"是否还在动"的有效性判据。 */
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
 *   A) 短路制动静置取样 -> 噪声底(峰峰)
 *   B) 幅值从0缓慢爬升到读数变化超阈值 -> 起转PWM(缓升而非固定脉冲，才是末端修正的真实工况)
 *   C) 撤力停稳量净位移 -> 最小可靠步长
 * 附带占空比耦合检查：施加推不动的PWM，三段均值 0 -> u -> 0；两个0段相等而中间偏移=电气耦合，
 * 第二个0段也偏=轴真动了，本次不作数。 */
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
/* 有效带：幅值低于首点 1/CM_SW_SNR_DIV 后已滚降进噪声底，不再判谐振(否则会误报) */
#define CM_SW_SNR_DIV      8L
#define CM_SW_RISE_SHIFT   2U   /* 需抬升 > 1/4 才算候选(原为 >>3 即 1/8) */

/* ---- T8 振铃：短脉冲激发后零输出采自由衰减，锁相求机构第一模态(扫频会被对象滚降埋掉)；上限约250Hz ---- */
#define CM_RING_PWM      2000   /* 激发脉冲幅值。要大到能激起来，又不至于跑远 */
#define CM_RING_MS          6U  /* 脉冲时长(ms)。越短频谱越平，激发的频带越宽 */
#define CM_RING_GAP_MS      4U  /* 脉冲后等几拍再采，躲开驱动本身的暂态 */
#define CM_RING_N         192U  /* 采样点数(=ms)。192ms 下频率分辨率约 5Hz */
#define CM_RING_FPTS       11U
/* 阈值按分辨率等比缩小；电位器报不出模态(0)是正常结果 */
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
/* ST_NOISE_A/ST_COUPLE/ST_NOISE_B 只有 T9 用 */
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
    int32_t  raw_sub;       /* 同一拍的亚LSB读数(计数×CM_SUB_SCALE)，只给 T9 用 */
    int32_t  raw_hw_last;   /* 上一拍的硬件读数，圆周编码器靠它连续化 */
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
    int32_t  ba_mark_sub;   /* 本次斜坡的起点读数，亚LSB */
    int32_t  ba_thr_sub;    /* 判"确实动了"的阈值，亚LSB */
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

/* 整数平方根，标定全程不使用浮点 */
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

/* 两角度之差；电位器是有限行程，不做最短路径折算 */
static int32_t CM_Diff(int32_t a, int32_t b)
{
    return a - b;
}

/* 幅值按"运行方向 × 接线极性"变成实际PWM，dir=0 恒为角度增大方向 */
static int16_t CM_Out(int16_t mag)
{
    int16_t v = (CM_DIR() == 0U) ? mag : (int16_t)-mag;

    return (s_cm.sign < 0) ? (int16_t)-v : v;
}

/* 位移(厘度)与窗口长度(ms)换算成速度，厘度/秒 */
static int32_t CM_Speed(int32_t disp, uint16_t span_ms)
{
    return (int32_t)(((int64_t)disp * 1000) / (int32_t)span_ms);
}

static void CM_Enter(uint8_t step) { s_cm.step = step; s_cm.timer = 0U; }

/* 切换到下一个测试项，顺带清掉本项用过的计时与步进状态 */
static void CM_Phase(uint8_t phase)
{
    s_cm.phase = phase;
    s_cm.run   = 0U;
    s_cm.level = 0U;
    D_Motor_Set(0);
    CM_Enter(ST_REST);
}

/* 推进一次运行计数，返回1=本项(或本档)的正反两个方向全部跑完 */
static uint8_t CM_Advance(uint8_t reps)
{
    if (++s_cm.run < (uint8_t)(reps * 2U)) return 0U;
    s_cm.run = 0U;
    return 1U;
}

/* ==================== 预定位：每项测试的公共起点 ====================
 * 有限行程上起点决定剩多少跑道，默认定位到行程低端(每项第一次运行为 dir0，之后正反交替)。
 * 例外从正中出发：T0(极性未知)、T5(第一个动作是反向预载)。新增测试项先确认第一个动作往哪边走。 */
static void CM_GotoPos(uint8_t next, int32_t target)
{
#if !CM_HAS_TRAVEL
    /* 整圈可测，任何位置都是合法起跑点，不需要预定位 */
    (void)target;
    CM_Phase(next);
    return;
#else
    s_cm.pos_target   = target;
    s_cm.next_phase   = next;
    s_cm.center_chk   = 0U;
    s_cm.center_err0  = target - s_cm.pos;
    s_cm.center_level = 0U;   /* 换测试项默认从头开始，T1换档会另行覆盖 */
    s_cm.center_try   = 0U;
    CM_Phase(PH_CENTER);
#endif
}

/* 多数测试的起跑点 */
static void CM_GotoCenter(uint8_t next) { CM_GotoPos(next, CM_START_LO_CDEG); }

/* 预定位的一拍：bang-bang 往起跑点靠，接近时换小幅值压滑行量 */
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

        /* T0 之前极性未知，定期检查误差是否变小，不是就掉头(也兜住T0判错极性) */
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

        /* 停稳后再确认，滑过头就重新逼近 */
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

/* 施加高于起转值的正PWM看角度往哪走；极性错了后续测量全部镜像、闭环正反馈。之后所有测试乘上该符号 */
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

/* 最重要的一项：slope 与 coulomb 派生出 v_ss/a0/Ka 与全部控制器增益。每档先进稳态再长基线测速 */
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
            /* 每档换档时回中，防止正反不对称漂出行程；CM_Phase 会清 level，下一档存入 center_level 带过去 */
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

/* 满PWM起步，记 CM_STEP_MARK_MS 时刻位移X与稳态速度 v_ss，渐近线法 tau_total = t_mark - X/v_ss
 * (比"首次到63%"精确)；tau_total 含纯延迟，报告里再扣 T3 的 L。 */
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


/* 从 T2 累加器取 v_ss(厘度/秒) 与 tau_tot=L+tau(0.01ms)：面积法 L+tau = T - x/v_ss，不要先整除到整毫秒 */
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

/* 从静止满PWM走过k个计数所需时间(0.01ms)：x = v_ss*tau*u^2/2*(1-u/3)，一次牛顿修正反解 u */
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

/* 由 T3 的N点跨越时刻解纯延迟(0.01ms)，失败返回-1：L = mean(t_k - tau*u_k)，与 tau 迭代两轮收敛 */
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

/* 先朝测试方向预运动并停稳(顶死齿隙)，再满PWM记录前 CM_DEAD_NPTS 个计数的跨越时刻：
 *     t_k = L + tau*u_k,  L = mean(t_k - tau*u_k)
 * 只看第一个计数时分辨率与答案同量级(连跑在2/3/4拍间跳)，N点回归后 sigma 约0.04ms。 */
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

            /* 一拍可能跨过多个计数，一并登记到同一时刻 */
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
            /* 先解出纯延迟，供 T4 决定等多久再取 mark */
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

/* ==================== T4 主动制动增益(型号级) ==================== */

/* T3 解出的纯延迟向上取整(至少1拍)，作为切换后等待时长 */
static uint16_t CM_Measured_Dead(void)
{
    int32_t ms = (s_cm.dead_L_x100 + 99) / 100;

    return (uint16_t)((ms >= 1) ? ms : 1);
}

/* 同一速度点比较维持速度所需PWM与反向制动PWM，结果用于 MODEL_BRAKE_GAIN_Q8(金样测一次即可) */
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

/* 对 (速度, PWM) 最小二乘拟合 PWM = intercept + slope*速度，全程 int64 */
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

/* ==================== T5 齿隙宽度(型号级) ====================
 * 编码器在输出轴，齿隙稳态不可见：换向延迟 - 同向纯延迟 = 穿越时间 t，齿隙 ≈ a0*t^2/2。 */
/* 起跑点须在正中：第一个动作是反向预载(约25度)，之后满PWM反向起步最坏还要再走约82度 */
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

/* ==================== T9 静摩擦 + 噪声底(单台)，说明见 CM_NZ_SAMPLES ==================== */

/* 亚LSB计数 -> 厘度×100。最大 65535*7941 < 2^31，不溢出。 */
static int32_t CM_Sub_C100(int32_t sub)
{
    return (int32_t)(((int64_t)sub * CM_LSB_C1000) / (CM_SUB_SCALE * 10));
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

/* T9 噪声统计：开一段新的采样窗 */
static void CM_Noise_Begin(void)
{
    s_cm.nz_min = 0x7FFFFFFF;
    s_cm.nz_max = -0x7FFFFFFF;
    s_cm.nz_sum = 0;
    s_cm.nz_n   = 0U;
}

/* T9 噪声统计：把本拍读数并进当前窗口 */
static void CM_Noise_Add(void)
{
    if (!s_cm.raw_valid) return;
    if (s_cm.raw_sub < s_cm.nz_min) s_cm.nz_min = s_cm.raw_sub;
    if (s_cm.raw_sub > s_cm.nz_max) s_cm.nz_max = s_cm.raw_sub;
    s_cm.nz_sum += s_cm.raw_sub;
    s_cm.nz_n++;
}

/* T9 噪声统计：当前窗口的均值(亚LSB) */
static int32_t CM_Noise_Mean(void)
{
    return (s_cm.nz_n != 0U) ? (int32_t)(s_cm.nz_sum / s_cm.nz_n) : 0;
}

/* T9 的一拍：静止测噪声底 -> 缓升测起转PWM -> 三段均值查占空比耦合 */
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
            /* 判"动了"的阈值挂在实测噪声上，换硬件自动跟随 */
            /* 取2倍峰峰(约12 sigma)：再大会让斜坡多爬，把起转值测高 */
            s_cm.ba_thr_sub = 2 * s_cm.nz_p2p;
            if (s_cm.ba_thr_sub < CM_BA_THR_MIN_Q4)
                s_cm.ba_thr_sub = CM_BA_THR_MIN_Q4;
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
            s_cm.ba_mark_sub = s_cm.raw_sub;
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

        if (CM_Abs(s_cm.raw_sub - s_cm.ba_mark_sub) >= s_cm.ba_thr_sub)
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
                    CM_Sub_C100(CM_Abs(s_cm.raw_sub - s_cm.ba_mark_sub)) / 100;
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

        /* 全部方向跑完做耦合检查，幅值取较小方向起转值的一半，保证推不动 */
        s_cm.nz_couple_pwm = (int16_t)(CM_Breakaway_Min() / 2);
        if (s_cm.nz_couple_pwm < 20) s_cm.nz_couple_pwm = 20;
        CM_Noise_Begin();
        /* 三个窗口首尾相连(0 -> u -> 0)，不能复用开头的噪声底均值(中间斜坡已把轴挪走) */
        CM_Enter(ST_COUPLE_PRE);
        break;
    }
}

/* ==================== T6 最小可靠位移(型号级)：递增脉宽，找稳定超过1个LSB的最小一档 ==================== */
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

/* ==================== T7 扫频辨识(型号级) ====================
 * 直流偏置让轴持续转动(避开 stick-slip 并 dither 量化)，叠加小幅正弦；对位置差分做单频点锁相(差分去掉匀速斜坡)。
 * 一阶对象幅值随频率单调下降，带内任何局部回升都是谐振。 */
/* 四分之一周期正弦表，Q14；T8 也要用，不受 CM_SWEEP_ENABLED 控制 */
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

/* ph: 0..65535 对应 0..2pi，象限对称查表 */
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

/* T7 每个频点要驻留多久(ms)：低频点需要更长时间凑够整周期 */
static uint16_t CM_SW_Dur(uint16_t hz)
{
    uint32_t ms = (uint32_t)CM_SW_CYCLES * 1000UL / hz;

    if (ms < CM_SW_MIN_MS) ms = CM_SW_MIN_MS;
    if (ms > CM_SW_MAX_MS) ms = CM_SW_MAX_MS;
    return (uint16_t)ms;
}

/* T7 的一拍：直流偏置上叠小幅正弦，单频点锁相求幅频 */
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

/* 短脉冲敲一下后零输出开环采样自由衰减(闭环会把振荡镇定掉)，分析频率得机构第一模态 */
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

/* 打印全部结果和可直接粘贴的 #define，电机已停，允许阻塞式printf */
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
               "undetectable, 3=encoder unreadable, 4=ran past safe travel, "
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

    /* 最大拟合残差：远大于几个PWM计数说明PWM-速度关系不是直线 */
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
        /* 残差前后半段均值接近=随机噪声，L可信；单调偏移=一阶模型没描述住起步段，L只是折中 */
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

    /* 交叉校验：阶跃测的 v_ss 应等于直线算出的，差得多说明有一项不可信 */
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

            /* 有效性：后半窗位移不足前半窗1/3说明轴已近停住，反演会失真 */
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
            /* 区间只提示不中止整份报告 */
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

        nz_c100 = CM_Sub_C100(s_cm.nz_p2p);

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
               (long)(CM_Sub_C100(d0) / 100), (long)(CM_Abs(CM_Sub_C100(d0)) % 100),
               (long)(CM_Sub_C100(d1) / 100), (long)(CM_Abs(CM_Sub_C100(d1)) % 100));
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

    /* ===== 输出：直接替换 A_Parameter.h 标定区；slope 与 tau 向上取整(误差单边，偏保守) ===== */
    printf("\r\n===== CAL VALUES -> A_Parameter.h =====\r\n");
    printf("#define CAL_SPEED_SLOPE_Q16 %5ld\r\n", (long)(slope[2] + 1));
    printf("#define CAL_FRICTION_PWM    %5ld\r\n", (long)inter[2]);
    /* 金样实测值向上留1ms裕量；量产快标复用已有型号值，不能每跑一次再+1。 */
    printf("#define MODEL_TAU_MS        %5ld\r\n",
           (long)(tau_mech + (CALIB_CHARACTERIZE_MODEL ? 1 : 0)));
    /* 四舍五入：L已有0.01ms分辨率，向上取整会白白压低观测器带宽上界 */
    printf("#define MODEL_DELAY_TICKS   %5ld\r\n", (long)((l_x100 + 50) / 100));
    printf("#define CAL_MOTOR_SIGN  %5d\r\n", (int)s_cm.sign);
    /* 起转取两方向较大值：补多了窜一点有位置环兜，补少了爬不动 */
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

        /* 齿隙当前无消费者(无齿隙补偿)，只打印不输出 #define */
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
            /* 判据取3倍实测噪声底(电位器1LSB已达7.94厘度，按1LSB判会把噪声当位移) */
            nz_cdeg = (nz_c100 + 99) / 100;
            thr = 3 * nz_cdeg;
            if (thr < 3) thr = 3;
            for (i = 0U; i < CM_MS_PULSES; i++)
                if (s_cm.ms_disp[i] >= thr) { min_pulse = s_cm.ms_disp[i]; break; }
            printf("     threshold=%ld cdeg (3x noise floor) -> pulse method gives %ld\r\n",
                   (long)thr, (long)min_pulse);

            /* 缓升法才是末端修正的真实工况，与脉冲法取小 */
            min_step = min_pulse;
            if (step_ramp > 0 && (min_step == 0 || step_ramp < min_step))
                min_step = step_ramp;
            if (min_step < 1) min_step = 1;
            /* 最小步长只作为 TUNE_RESOLUTION_CDEG 的依据，不单独输出 */
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

        /* 只在有效带内(幅值>=首点/8)判谐振，抬升门限1/4；带外噪声会误报并导致观测器带宽护栏编译失败 */
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

    /* ---- T8 振铃：对一阶差分做单频点锁相找峰；测不到/峰触边界/可用三档，宁可不给数也不给错的数 ---- */
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
            /* 须连跑三次落在同一频点才算真模态，才可填 MODEL_RESONANCE_HZ */
            printf("[T8] 第一模态候选 = %u Hz\r\n"
                   "     连跑三次若都落在这个频点，再填 MODEL_RESONANCE_HZ；\r\n"
                   "     每次都不一样就是噪声，保持 0\r\n",
                   (unsigned)CM_RING_HZ[pk]);
        }
    }

#if ENCODER_MODE == 0
    /* ---- 角度标度自检(仅电位器)：标度错了协议的"270度"就不是270度；同一机构换传感器物理速度不变 ---- */
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
#endif

    printf("\r\n(derived: v_ss=%ld  a0=%ld  Ka=%ld.%02ld cdeg/s2 per count)\r\n",
           (long)vss, (long)(((int64_t)vss * 1000) / tau_mech),
           (long)(ka_x100 / 100), (long)(ka_x100 % 100));
    printf("===== END =====\r\n");
}

/* ==================== 任务入口 ==================== */

/* 取一拍编码器读数，同时填 raw(连续计数)和 pos(厘度)，只取一次样本避免两者落在相邻数据块。
 * 死区读数按读不到处理(标定不做脱困)。标定一律用整数LSB：T3/T5 测的就是量化台阶本身，亚LSB会把延迟测成0。 */
static uint8_t CM_ReadEncoder(void)
{
#if ENCODER_MODE == 0
    uint16_t adc_sub = 0;

    if (!D_ADC_Encoder_Read(&adc_sub)
        || adc_sub < (uint16_t)(ENCODER_POT_ADC_FLOAT * CM_SUB_SCALE))
        return 0U;

    s_cm.raw     = (int32_t)adc_sub / CM_SUB_SCALE;
    s_cm.raw_sub = (int32_t)adc_sub;
    s_cm.pos     = ((s_cm.raw - (int32_t)ENCODER_POT_ADC_MIN) * CM_LSB_C1000) / 1000;
    s_cm.raw_valid = 1U;
    return 1U;
#else
    uint16_t raw_hw = D_MT6701_Read_Raw();

    if (raw_hw == MT6701_ANGLE_ERROR) return 0U;

    /* 整圈编码器跨0点时连续化成不回绕计数 */
    if (s_cm.raw_valid)
    {
        int32_t d = (int32_t)raw_hw - s_cm.raw_hw_last;

        if (d >  (int32_t)MT6701_RAW_MAX / 2) d -= (int32_t)MT6701_RAW_MAX;
        if (d < -(int32_t)MT6701_RAW_MAX / 2) d += (int32_t)MT6701_RAW_MAX;
        s_cm.raw += d;
    }
    else
    {
        s_cm.raw       = (int32_t)raw_hw;
        s_cm.raw_valid = 1U;
    }
    s_cm.raw_hw_last = (int32_t)raw_hw;
    s_cm.raw_sub = s_cm.raw * CM_SUB_SCALE;
    s_cm.pos     = (s_cm.raw * CM_LSB_C1000) / 1000;
    return 1U;
#endif
}

/* 1ms 标定拍：读编码器 -> 查行程 -> 推进当前测试；单拍读失败保持上一拍，连续失败才判死 */
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

    /* 行程硬保护(仅有限行程)：开环激励无闭环兜底，冲出安全带即中止；预定位本身往里走不受限 */
    if (CM_HAS_TRAVEL && s_cm.raw_valid && s_cm.fail == 0U
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
#if CM_HAS_TRAVEL
            printf("\r\nCALIB start (pot encoder), T0~T6/T9+T8, keep the shaft "
                   "free.\r\nTravel %ld cdeg, guard band %ld cdeg each end.\r\n",
                   (long)ENCODER_POT_SPAN_CDEG, (long)CM_GUARD_CDEG);
#else
            printf("\r\nCALIB start (magnetic encoder), T0~T9, keep the shaft "
                   "free to rotate continuously.\r\n");
#endif
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

/* 初始化标定状态机；上电读不到编码器会在 PH_BOOT 判 fail=3 */
void A_Calib_Min_Init(void)
{
    memset(&s_cm, 0, sizeof(s_cm));
    s_cm.sign = 1;                   /* T0 之前先当作正接线 */

    (void)CM_ReadEncoder();          /* 读不到就让 PH_BOOT 判 fail=3 */
    s_cm.phase = PH_BOOT;
    D_Motor_Set(0);
}

static void Task_CalibMinWatchdog(void) { D_IWDG_Feed(); }

static Task_t s_tasks[] = {
    TASK_DEF(A_Calib_Min_Tick,      1,  0),
    TASK_DEF(Task_CalibMinWatchdog, 50, 1),
};

/* 标定期间只跑标定拍与喂狗，不注册舵机控制和串口命令，避免闭环干扰开环激励 */
void A_Calib_Min_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
}
