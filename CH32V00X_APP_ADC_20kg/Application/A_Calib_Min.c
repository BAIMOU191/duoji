/*
 * A_Calib_Min.c —— 自动标定程序，两种编码器共用；差异收在"编码器抽象"一段，测试项只认连续计数 raw 与厘度 pos。
 * 只测互相独立的量(其余由 A_Config.h 派生，分别测会因测量噪声互相矛盾)：
 *   T0 接线方向、T1 slope/coulomb(单台散差最大)、T2 tau、T3 纯延迟、T4 制动比、T9 起转与噪声底；
 *   tau/纯延迟/制动比为型号级常数。
 * 标定误差是单边的：把对象估强/估快(slope 或 tau 偏小)会让规划器要求做不到的加速度，故两者一律向上取整。
 */

#include "A_Calib_Min.h"
#include "A_Config.h"
#include "C_Task_Scheduler.h"
#include "D_motor.h"
#include "D_adc.h"      /* 电位器通道的ADC原始计数 */
#include "A_Sensor.h"   /* 编码器模式、标定常数与死区门限 */
#include "D_iwdg.h"
#include "D_uart.h"
#include "iap.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>

/* ==================== 可调项(换更大扭力舵机时复核) ==================== */

/* 单一全量模式：一次跑完全部测试项并输出全部宏(分档会让快标印出金样的旧 tau) */
#define CALIB_MODE 2
#define CALIB_CHARACTERIZE_MODEL 1

/* 电位器：行程有限(每拍越界即停 + 每项测试前预定位)；分辨率约 7.94 厘度/LSB；不能持续单向转，不做扫频。
 * CM_SUB_SCALE=驱动发布的亚LSB倍数(电位器均值Q4)，只有T9用。 */
#define CM_SUB_SCALE      ((int32_t)D_ADC_ENCODER_SCALE)
/* 一个编码器计数值多少厘度，×1000。电位器：27000厘度/3400计数 = 7.941厘度。 */
#define CM_LSB_C1000      ((int32_t)(((int32_t)ENCODER_POT_SPAN_CDEG * 1000L) \
                                   / (int32_t)ENCODER_POT_ADC_SPAN))
/* 保护带按可读范围画(碳膜比标定行程两端各多10度)，要防的是抽头滑出碳膜 */
#define CM_GUARD_CDEG     500L  /* 距可读两端还有这么多厘度就判越界(5度) */
#define CM_TRAVEL_LO      (ENCODER_ANGLE_LO + CM_GUARD_CDEG)
#define CM_TRAVEL_HI      (ENCODER_ANGLE_HI - CM_GUARD_CDEG)
#define CM_CENTER_CDEG    ((int32_t)ENCODER_POT_SPAN_CDEG / 2)

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
#define CM_REP_SPD    1U        /* 每档正反各跑几次(实际次数 = 值*2) */
#define CM_REP_STEP   2U
#define CM_REP_DEAD   2U
#define CM_REP_BRAKE  3U

/* 整套测试连跑几遍，最后逐项取中位数。一遍的散差来自接触噪声/齿隙停位/温度，
 * 三遍取中位数能把任意一遍的离群值整个丢掉；改成1即恢复单遍(约省2/3时间) */
#define CM_ROUNDS      3U

/* ==================== 状态机 ==================== */

enum { PH_BOOT = 0, PH_CENTER, PH_T0_SIGN, PH_T1_LINE, PH_T2_TAU, PH_T3_DEAD,
       PH_T4_BRAKE, PH_T9_STICTION, PH_T6_MINSTEP,
       PH_REPORT, PH_DONE };
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

    int32_t  ms_disp[CM_MS_PULSES];   /* T6 各脉宽产生的位移(厘度) */
    uint8_t  ms_idx;

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

static uint8_t s_iap_stopped; /* 进入升级后锁住标定，直到复位，不再施力或打印 */

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
    s_cm.pos_target   = target;
    s_cm.next_phase   = next;
    s_cm.center_chk   = 0U;
    s_cm.center_err0  = target - s_cm.pos;
    s_cm.center_level = 0U;   /* 换测试项默认从头开始，T1换档会另行覆盖 */
    s_cm.center_try   = 0U;
    CM_Phase(PH_CENTER);
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
        if (CM_Advance(CM_REP_BRAKE)) CM_GotoCenter(PH_T9_STICTION);
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
            CM_Phase(PH_REPORT);
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

/* ==================== 结果汇总与输出 ====================
 * 整套测试连跑 CM_ROUNDS 遍，每遍的结果存进 s_res[]，最后取中位数再打印。
 * 一遍不准的来源是随机的：抽头接触噪声、齿隙停在哪一侧、温度、当时轴停的位置。
 * 三遍取中位数能把其中任意一遍的离群值整个丢掉(取均值做不到这一点)。
 */

typedef struct {
    uint8_t valid;        /* 本轮 T1/T2/T3 是否都拿到了数 */
    int8_t  sign;
    int32_t slope_q16;
    int32_t friction_pwm;
    int32_t tau_ms;       /* 机械时间常数(已扣掉纯延迟) */
    int32_t delay_x100;   /* 纯延迟，0.01ms */
    int32_t vss;          /* 满PWM稳态速度，厘度/秒 */
    int32_t res_cdeg;     /* 建议的 TUNE_RESOLUTION_CDEG */
    int32_t breakaway;    /* 起转PWM，0=本轮没测出 */
    int32_t brake_q8;     /* 制动增益，0=本轮没测出 */
} CalibRound_t;

static CalibRound_t s_res[CM_ROUNDS];
static uint8_t      s_round;   /* 已经跑完几轮 */

/* n 个样本里挑一个代表值：n=3 取中位数(离群的那一遍直接被丢掉)，n=2 取均值，n=1 原样返回。
 * 就地排序，调用方传的是临时数组。 */
static int32_t CM_Pick(int32_t *v, uint8_t n)
{
    uint8_t i, j;

    for (i = 1U; i < n; i++)                       /* 插入排序，n<=3 */
        for (j = i; j > 0U && v[j - 1U] > v[j]; j--)
        {
            int32_t t = v[j]; v[j] = v[j - 1U]; v[j - 1U] = t;
        }
    if (n == 0U) return 0;
    if (n == 2U) return (v[0] + v[1] + 1) / 2;
    return v[n / 2U];
}

/* 把某一项在各轮的有效样本收集起来并挑出代表值；zero_means_missing=1 时 0 视为"本轮没测出" */
static int32_t CM_PickField(uint32_t offset, uint8_t zero_means_missing)
{
    int32_t v[CM_ROUNDS];
    uint8_t i, n = 0U;

    for (i = 0U; i < s_round; i++)
    {
        int32_t x;

        if (!s_res[i].valid) continue;
        x = *(const int32_t *)((const uint8_t *)&s_res[i] + offset);
        if (zero_means_missing && x == 0) continue;
        v[n++] = x;
    }
    return CM_Pick(v, n);
}

#define CM_PICK(field)          CM_PickField(offsetof(CalibRound_t, field), 0U)
#define CM_PICK_OPT(field)      CM_PickField(offsetof(CalibRound_t, field), 1U)

/*
 * 跑完一轮后把这一轮的结果算出来存进 r。只在有问题时打印告警(带轮次号)，
 * 不打印中间量：原始曲线看不出问题，真出问题的地方告警会点名。
 */
static void CM_Collect(CalibRound_t *r)
{
    int32_t slope[3] = {0,0,0}, inter[3] = {0,0,0};
    int32_t vss = 0, tau_tot = 0, tau_mech, l_x100 = 0;
    int32_t brake_gain_q8 = 0;
    int32_t ba_out = 0;        /* T9 起转PWM，两方向取大 */
    int32_t nz_cdeg;           /* T9 噪声底峰峰，厘度(向上取整) */
    int32_t step_ramp = 0;     /* T9 缓升法测到的最小步长 */
    int32_t res_reco;
    unsigned rd = (unsigned)(s_round + 1U);
    uint8_t d, n;

    r->valid = 0U;

    if (s_cm.fail != 0U)
    {
        printf("\r\n[%u] ABORTED fail=%d (1=满PWM不动 2=方向判不出 3=编码器读不到"
               " 4=冲出安全行程 5=预定位超时)\r\n", rd, (int)s_cm.fail);
        if (s_cm.fail == 4U)
            printf("    在相位%u 跑到 %ld 厘度(允许 %ld..%ld)\r\n",
                   (unsigned)s_cm.fail_phase, (long)s_cm.fail_pos,
                   (long)CM_TRAVEL_LO, (long)CM_TRAVEL_HI);
        else
            printf("    检查接线、供电，以及输出轴能不能自由转动\r\n");
        return;
    }

    /* ---- T1 PWM-速度直线 ---- */
    for (d = 0U; d < 2U; d++) (void)CM_Fit(d, &slope[d], &inter[d]);
    n = CM_Fit(2U, &slope[2], &inter[2]);
    if (n < 6U || slope[2] <= 0 || inter[2] < 0 || inter[2] >= MOTOR_PWM_MAX)
    {
        printf("[%u] INVALID: 速度直线拟合失败，本轮作废。检查轴是否自由、供电是否稳。\r\n", rd);
        return;
    }
    if (slope[0] > 0 && slope[1] > 0
        && (CM_Abs(slope[0] - slope[1]) * 100 / slope[2] > 10
            || CM_Abs(inter[0] - inter[1]) * 100 / ((inter[2] > 0) ? inter[2] : 1) > 25))
        printf("[%u] WARN 两个方向差异过大(slope %ld/%ld inter %ld/%ld)，合并值只是折中\r\n",
               rd, (long)slope[0], (long)slope[1], (long)inter[0], (long)inter[1]);

#if CALIB_CHARACTERIZE_MODEL
    /* ---- T2 时间常数 / T3 纯延迟 ---- */
    {
        int32_t tt_x100, resid = 0;
        uint8_t k, have = 0U;

        if (s_cm.step_n[0] == 0U || s_cm.step_n[1] == 0U)
        {
            printf("[%u] INVALID: 阶跃响应缺失，本轮作废。\r\n", rd);
            return;
        }
        CM_Step_Result(&vss, &tt_x100);
        tau_tot = (tt_x100 + 50) / 100;
        if (tau_tot < 1) tau_tot = 1;

        for (k = 0U; k < CM_DEAD_NPTS; k++) if (s_cm.dead_t_n[k]) have++;
        if (have < 3U)
        {
            printf("[%u] INVALID: 纯延迟测点不足，本轮作废。\r\n", rd);
            return;
        }
        l_x100 = CM_Dead_L_x100(vss, tau_tot, &resid);
        if (l_x100 < 0)
        {
            printf("[%u] INVALID: 纯延迟回归失败，本轮作废。\r\n", rd);
            return;
        }
        if (resid > 150)
            printf("[%u] WARN 纯延迟残差 %ld.%02ldms 偏大：起步段非一阶(传动柔性/静摩擦)，"
                   "L 取的是全段均值\r\n", rd, (long)(resid / 100), (long)(resid % 100));
    }
    tau_mech = tau_tot - (l_x100 + 50) / 100;
    if (tau_mech < 1) tau_mech = 1;

    /* 交叉校验：阶跃测的 v_ss 应等于直线算出的 */
    {
        int32_t vss_line = (int32_t)((((int64_t)(MOTOR_PWM_MAX - inter[2])) << 16) / slope[2]);

        if (vss > 0 && CM_Abs(vss_line - vss) * 100 / vss > 10)
            printf("[%u] WARN v_ss 自相矛盾(阶跃%ld vs 直线%ld)\r\n",
                   rd, (long)vss, (long)vss_line);
    }

    /* ---- T4 主动制动增益 ---- */
    if (s_cm.brake_n[0] != 0U && s_cm.brake_n[1] != 0U)
    {
        int32_t brake_a = 0, n_ok = 0;

        for (d = 0U; d < 2U; d++)
        {
            int32_t v0  = s_cm.brake_v0_sum[d] / s_cm.brake_n[d];
            int32_t dx1 = s_cm.brake_dx1[d] / s_cm.brake_n[d];
            int32_t dx2 = s_cm.brake_dx2[d] / s_cm.brake_n[d];
            /* 由位移公式求平均加速度：dx = v0*T/1000 - a*T^2/2e6，得a_bar = 2000*(v0*T - dx*1000)/T^2 */
            int64_t df2 = (int64_t)v0 * CM_BRAKE_T2_MS - (int64_t)dx2 * 1000;
            int32_t a_bar = (int32_t)((df2 * 2000)
                                      / ((int32_t)CM_BRAKE_T2_MS * CM_BRAKE_T2_MS));
            int32_t q_q16, corr_q16;

            /* 后半窗位移不足前半窗1/3说明轴已近停住，反演会失真 */
            if (a_bar <= 0 || dx1 <= 0 || (dx2 - dx1) * 3 < dx1) continue;

            /* 初始加速度修正：a0 = a_bar / (1 - q/3 + q^2/12)，q = T/tau，采用Q16定点数 */
            q_q16    = (int32_t)(((int64_t)CM_BRAKE_T2_MS << 16) / tau_mech);
            corr_q16 = 65536 - q_q16 / 3 + (int32_t)((((int64_t)q_q16 * q_q16) >> 16) / 12);
            if (corr_q16 < 32768) corr_q16 = 32768;   /* q 太大时展开失效，钳住 */
            brake_a += (int32_t)(((int64_t)a_bar << 16) / corr_q16);
            n_ok++;
        }
        if (n_ok > 0)
        {
            int32_t opposing = CM_BRAKE_DRIVE_PWM + CM_BRAKE_CMD_PWM;

            brake_a /= n_ok;
            brake_gain_q8 = (int32_t)(((int64_t)opposing * 256 * 65536 * 1000)
                                      / ((int64_t)brake_a * slope[2] * tau_mech));
            if (brake_gain_q8 < 64 || brake_gain_q8 > 1024)
            {
                printf("[%u] WARN 制动增益 %ld 超出 [64,1024]，本轮不采用\r\n",
                       rd, (long)brake_gain_q8);
                brake_gain_q8 = 0;
            }
        }
    }
    if (brake_gain_q8 == 0) printf("[%u] WARN 制动增益本轮没测出来\r\n", rd);
#else
    /* 量产快标只更新单台散差最大的斜率/摩擦/方向，动力学三项沿用金样值 */
    vss           = (int32_t)((((int64_t)(MOTOR_PWM_MAX - inter[2])) << 16) / slope[2]);
    tau_mech      = MODEL_TAU_MS;
    l_x100        = MODEL_DELAY_TICKS * 100;
    brake_gain_q8 = MODEL_BRAKE_GAIN_Q8;
#endif

    /* ---- T9 噪声底 / 占空比耦合 / 起转PWM ---- */
    {
        int32_t d0 = s_cm.nz_mean1 - s_cm.nz_mean0;   /* 加力后的读数偏移 */
        int32_t d1 = s_cm.nz_mean2 - s_cm.nz_mean0;   /* 撤力后回没回来 */

        nz_cdeg = (CM_Sub_C100(s_cm.nz_p2p) + 99) / 100;

        if (CM_Abs(d1) <= s_cm.nz_p2p && CM_Abs(d0) > 2 * s_cm.nz_p2p)
            printf("[%u] WARN **占空比耦合**：轴没动而读数跟着PWM漂，会被观测器微分成假速度。\r\n"
                   "    查 D_adc.c 的扫描序列，或把电位器挪到规则组第1位\r\n", rd);

        for (d = 0U; d < 2U; d++)
        {
            int32_t bv, sv;

            if (s_cm.ba_n[d] == 0U)
            {
                printf("[%u] WARN dir%u 起转没测出来(爬到%d还不动)\r\n",
                       rd, (unsigned)d, CM_BA_MAX_PWM);
                continue;
            }
            bv = s_cm.ba_sum[d] / s_cm.ba_n[d];
            sv = s_cm.ba_step_sum[d] / s_cm.ba_n[d];
            if (bv > ba_out)    ba_out    = bv;
            if (sv > step_ramp) step_ramp = sv;
        }
        if (ba_out > 0 && ba_out <= inter[2])
        {
            printf("[%u] WARN 起转(%ld) <= 动摩擦(%ld)：不合物理，本轮不采用\r\n",
                   rd, (long)ba_out, (long)inter[2]);
            ba_out = 0;
        }
    }

    /* ---- T6 最小可靠步长 -> 分辨率 ---- */
    {
        int32_t min_step = 0, thr;
        uint8_t i;

        /* 判据取3倍实测噪声底(电位器1LSB已达7.94厘度，按1LSB判会把噪声当位移) */
        thr = 3 * nz_cdeg;
        if (thr < 3) thr = 3;
        for (i = 0U; i < CM_MS_PULSES; i++)
            if (s_cm.ms_disp[i] >= thr) { min_step = s_cm.ms_disp[i]; break; }
        /* 缓升法才是末端修正的真实工况，与脉冲法取小 */
        if (step_ramp > 0 && (min_step == 0 || step_ramp < min_step)) min_step = step_ramp;

        /* 比噪声底小 = 控制器在追噪声；比最小步长小 = 为一个修不动的残差反复踹静摩擦 */
        res_reco = 2 * nz_cdeg;
        if (min_step > res_reco) res_reco = min_step;
        if (res_reco < 5) res_reco = 5;
    }

    r->valid        = 1U;
    r->sign         = s_cm.sign;
    r->slope_q16    = slope[2];
    r->friction_pwm = inter[2];
    r->tau_ms       = tau_mech;
    r->delay_x100   = l_x100;
    r->vss          = vss;
    r->res_cdeg     = res_reco;
    r->breakaway    = ba_out;
    r->brake_q8     = brake_gain_q8;

    printf("[%u/%u] 完成  slope=%ld fric=%ld tau=%ld L=%ld.%02ldms 起转=%ld 制动=%ld 分辨率=%ld\r\n",
           rd, (unsigned)CM_ROUNDS, (long)slope[2], (long)inter[2], (long)tau_mech,
           (long)(l_x100 / 100), (long)(l_x100 % 100), (long)ba_out,
           (long)brake_gain_q8, (long)res_reco);
}

/* 全部轮次跑完：逐项取中位数，再算出轨迹默认值并打印可粘贴的 #define */
static void CM_Print(void)
{
    int32_t slope, fric, tau_mech, l_x100, vss, res_reco, ba_out, brake_q8;
    int32_t vhw, a0, abig, dhw, vmax_reco, amax_reco, dmax_reco, jmax_reco;
    int32_t sign_sum = 0;
    uint8_t i, nv = 0U;

    for (i = 0U; i < s_round; i++)
        if (s_res[i].valid) { nv++; sign_sum += s_res[i].sign; }

    printf("\r\n===== SERVO CALIBRATION (%u/%u 轮有效) =====\r\n",
           (unsigned)nv, (unsigned)CM_ROUNDS);
    if (nv == 0U)
    {
        printf("没有一轮跑通，不输出参数。\r\n===== END =====\r\n");
        return;
    }
    if (nv < CM_ROUNDS)
        printf("WARN 只有 %u 轮有效，代表性不如三轮；建议排除故障后重跑\r\n", (unsigned)nv);

    slope    = CM_PICK(slope_q16);
    fric     = CM_PICK(friction_pwm);
    tau_mech = CM_PICK(tau_ms);
    l_x100   = CM_PICK(delay_x100);
    vss      = CM_PICK(vss);
    res_reco = CM_PICK(res_cdeg);
    ba_out   = CM_PICK_OPT(breakaway);   /* 某轮没测出就不参与 */
    brake_q8 = CM_PICK_OPT(brake_q8);

    /* ---- 由实测量算出轨迹出厂默认值：一律取硬件能力的 90%，余下一成留给负载与个体差异 ---- */
    {
        int32_t net  = MOTOR_PWM_MAX - fric;                        /* 扣掉摩擦的净PWM */
        int32_t bq8  = (brake_q8 > 0) ? brake_q8 : MODEL_BRAKE_GAIN_Q8;
        int32_t fq15;

        vhw  = (net > 0) ? (int32_t)(((int64_t)vss * (MOTOR_PWM_MAX - PLANT_FF_MARGIN_PWM
                                                     - fric)) / net) : 0;
        a0   = (int32_t)(((int64_t)vss * 1000) / tau_mech);         /* 零速起步加速度 */
        fq15 = (vss > 0) ? (int32_t)(((int64_t)vhw << 15) / vss) : 0;
        abig = (int32_t)(((int64_t)a0 * (32768 - fq15 / 2)) >> 15); /* 走满速时还剩多少 */
        /* 制动上限：末端反电势帮不上忙时，满PWM扣掉反馈余量、再按制动比折算 */
        dhw  = (int32_t)(((int64_t)(MOTOR_PWM_MAX - PLANT_FF_MARGIN_PWM) * 65536 * 1000 * 256)
                         / ((int64_t)slope * tau_mech * bq8));

        vmax_reco = vhw  / 100 * 90 / 100;                          /* 度/秒 */
        amax_reco = abig / 100 * 90 / 100;                          /* 度/秒^2 */
        dmax_reco = dhw  / 100 * 90 / 100;
        jmax_reco = amax_reco * 500;                                /* 再大斜坡短于1拍 */
        if (jmax_reco > 2000000 || jmax_reco < 0) jmax_reco = 2000000;
    }

    /* ================= 以下整段抄进 A_Config.h ================= */
    printf("\r\n----- A_Config.h 第2节：本机标定 -----\r\n");
    /* slope 与 tau 向上取整：误差单边，偏保守 */
    printf("#define CAL_SPEED_SLOPE_Q16 %5ld\r\n", (long)(slope + 1));
    printf("#define CAL_FRICTION_PWM    %5ld\r\n", (long)fric);
    if (ba_out > 0)  /* 起转取两方向较大值：补多了窜一点有位置环兜，补少了爬不动 */
        printf("#define CAL_BREAKAWAY_PWM   %5ld\r\n", (long)ba_out);
    else
        printf("/* CAL_BREAKAWAY_PWM 三轮都没测出，保留现值 */\r\n");
    printf("#define CAL_MOTOR_SIGN      %5d\r\n", (sign_sum >= 0) ? 1 : -1);
    printf("#define MODEL_TAU_MS        %5ld\r\n",
           (long)(tau_mech + (CALIB_CHARACTERIZE_MODEL ? 1 : 0)));  /* 金样留1ms裕量 */
    /* L 已有0.01ms分辨率，四舍五入；向上取整会白白压低观测器带宽上界 */
    printf("#define MODEL_DELAY_TICKS   %5ld\r\n", (long)((l_x100 + 50) / 100));
    if (brake_q8 > 0)
        printf("#define MODEL_BRAKE_GAIN_Q8 %5ld\r\n", (long)brake_q8);
    else
        printf("/* MODEL_BRAKE_GAIN_Q8 三轮都没测出，保留现值 */\r\n");
    printf("#define CAL_MIN_STEP_CDEG   %5ld\r\n", (long)res_reco);

    printf("\r\n----- A_Config.h 第3节：调参(只改这两个，带宽四项不动；分辨率=到位精度，可按需另调) -----\r\n");
    printf("#define TUNE_RESOLUTION_CDEG %4ld\r\n", (long)res_reco);
    printf("#define TUNE_ACCEL_MIN_MS    %4ld\r\n", (long)tau_mech);

    printf("\r\n----- A_Config.h 第5节：轨迹出厂默认值(硬件能力的90%%) -----\r\n");
    printf("#define CFG_DEF_VMAX_DPS     %7ld   /* 硬件 %ld */\r\n",
           (long)vmax_reco, (long)(vhw / 100));
    printf("#define CFG_DEF_AMAX_DPS2    %7ld   /* 硬件 %ld */\r\n",
           (long)amax_reco, (long)(abig / 100));
    printf("#define CFG_DEF_DMAX_DPS2    %7ld   /* 硬件 %ld */\r\n",
           (long)dmax_reco, (long)(dhw / 100));
    printf("#define CFG_DEF_JMAX_DPS3    %7ld   /* = 500 x 加速度 */\r\n", (long)jmax_reco);
    printf("#define CFG_DEF_MOVE_MIN_MS  %7ld   /* = 2.2 倍机械时间常数 */\r\n",
           (long)(tau_mech * 22 / 10));
    printf("===== END =====\r\n");
}

/* ==================== 任务入口 ==================== */

/* 开始新的一轮：测量累加器全部清零，从预热(PH_BOOT)重新走一遍。
 * s_res[]/s_round 在 s_cm 之外，不会被清掉。 */
static void CM_RoundRestart(void)
{
    memset(&s_cm, 0, sizeof(s_cm));
    s_cm.sign  = 1;                  /* T0 之前先当作正接线 */
    s_cm.phase = PH_BOOT;
    D_Motor_Set(0);
}


/* 取一拍编码器读数，同时填 raw(连续计数)和 pos(厘度)，只取一次样本避免两者落在相邻数据块。
 * 死区读数按读不到处理(标定不做脱困)。标定一律用整数LSB：T3/T5 测的就是量化台阶本身，亚LSB会把延迟测成0。 */
static uint8_t CM_ReadEncoder(void)
{
    uint16_t adc_sub = 0;

    if (!D_ADC_Encoder_Read(&adc_sub)
        || adc_sub < (uint16_t)(ENCODER_POT_ADC_FLOAT * CM_SUB_SCALE))
        return 0U;

    s_cm.raw     = (int32_t)adc_sub / CM_SUB_SCALE;
    s_cm.raw_sub = (int32_t)adc_sub;
    s_cm.pos     = ((s_cm.raw - (int32_t)ENCODER_POT_ADC_MIN) * CM_LSB_C1000) / 1000;
    s_cm.raw_valid = 1U;
    return 1U;
}

/* 1ms 标定拍：读编码器 -> 查行程 -> 推进当前测试；单拍读失败保持上一拍，连续失败才判死 */
static void A_Calib_Min_Tick(void)
{
    if (s_iap_stopped) return;
    /* 标定日志直接使用 USART，打印前先让 IAP 应答完整发完。 */
    if (s_cm.phase == PH_BOOT && !D_UART_Tx_Idle()) return;

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
            if (s_round == 0U)
            {
                printf("\r\nCALIB start: 整套测试连跑 %u 遍取中位数，"
                       "中途请勿触碰输出轴。\r\n", (unsigned)CM_ROUNDS);
                printf("可用行程 %ld 厘度，两端各留 %ld 厘度保护带。\r\n",
                       (long)ENCODER_POT_SPAN_CDEG, (long)CM_GUARD_CDEG);
            }
            printf("\r\n--- 第 %u/%u 轮 ---\r\n",
                   (unsigned)(s_round + 1U), (unsigned)CM_ROUNDS);
            CM_GotoPos(PH_T0_SIGN, CM_CENTER_CDEG);
        }
        break;
    case PH_T0_SIGN: CM_T0_Sign(); break;
    case PH_T1_LINE: CM_T1_Line(); break;
    case PH_T2_TAU:  CM_T2_Tau();  break;
    case PH_T3_DEAD: CM_T3_Dead(); break;
    case PH_T4_BRAKE: CM_T4_Brake(); break;
    case PH_T9_STICTION: CM_T9_Stiction(); break;
    case PH_T6_MINSTEP:  CM_T6_MinStep();  break;
    case PH_REPORT:
        D_Motor_Set(0);
        if (!D_UART_Tx_Idle()) break; /* 本拍刚发生故障时也要避开正在发送的 IAP 应答 */
        if (s_round < CM_ROUNDS) CM_Collect(&s_res[s_round++]); /* 电机已停，允许阻塞式printf */
        /* 还没跑满且没出硬件故障：清空本轮累加器，从头再跑一遍 */
        if (s_cm.fail == 0U && s_round < CM_ROUNDS) { CM_RoundRestart(); break; }
        CM_Print();
        s_cm.phase = PH_DONE;
        break;
    case PH_DONE:
    default:
        D_Motor_Set(0);
        break;
    }
}

/* 通过身份校验后先停标定、低阻卸力，等待应答发送结束期间也不能重新施力。 */
static void CM_StopForIAP(void)
{
    s_iap_stopped = 1U;
    D_Motor_Release(0U);
}

/* 初始化标定状态机；上电读不到编码器会在 PH_BOOT 判 fail=3 */
void A_Calib_Min_Init(void)
{
    memset(&s_cm, 0, sizeof(s_cm));
    memset(s_res, 0, sizeof(s_res));
    s_round = 0U;
    s_iap_stopped = 0U;
    IAP_SetStopHandler(CM_StopForIAP);
    s_cm.sign = 1;                   /* T0 之前先当作正接线 */

    (void)CM_ReadEncoder();          /* 读不到就让 PH_BOOT 判 fail=3 */
    s_cm.phase = PH_BOOT;
    D_Motor_Set(0);
}

/* 只处理 IAP，普通舵机运动/参数指令不能干扰开环标定。 */
static void Task_CalibMinIAP(void)
{
    uint8_t byte;

    while (D_UART1_Rx_Get(&byte)) IAP_Rx_Deal(byte);
    D_UART_Service();
    IAP_Service();
}

static void Task_CalibMinWatchdog(void) { D_IWDG_Feed(); }

static Task_t s_tasks[] = {
    TASK_DEF(A_Calib_Min_Tick,      1,  0),
    TASK_DEF(Task_CalibMinIAP,      1,  1),
    TASK_DEF(Task_CalibMinWatchdog, 50, 2),
};

/* 标定、IAP、喂狗同时运行；不注册闭环控制和普通舵机串口命令。 */
void A_Calib_Min_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
}
