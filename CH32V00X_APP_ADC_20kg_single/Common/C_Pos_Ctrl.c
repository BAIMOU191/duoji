/* C_Pos_Ctrl.c 位置控制器：状态 -> 律参数 -> 前馈与反馈 -> 起转补偿(运动中，及收尾/保持时窗外卡住) -> 限幅与积分
 * 增益在初始化/SetGains/SetHold中换算；控制拍使用定点运算，交接期间混合已有两种控制律。 */
#include "C_Pos_Ctrl.h"
#include "C_Fixed.h"
#include <string.h>

#define CAPTURE_MS     20U    /* 捕获条件连续满足这么久才算到位，随即开始切换到保持律 */
#define HOLD_BLEND_MS  50U    /* 收尾/保持律渐进交接，共用一个积分器 */
#define HOLD_GAIN_X10  15     /* 保持 P/I 的实际效果 = 参数 × 1.5(参数范围不变) */
#define VEL_FILTER_MS   1    /* 运动律速度残差一阶滤波 α=1/(1+它)，群延迟=它(ms)；不计入速度环带宽上限：
                              * 三份标定仿真里计入它会让延迟大的电机 ω_v 从103降到79、抗扰变差，不计没有出现振荡 */
#define STICK_MOVE_MS  5U     /* 这么久没挪过最小可靠位移且估计速度低于噪声底，才算卡住 */
#define STICK_ESC_MS   25U    /* 顶到起转电压后仍卡这么久，开始逐拍加码 */
#define STICK_ESC_DIV  50     /* 加码每拍 fs/50；运动/保持中最多再加一个 fs，收尾中加到推动为止(上限半个可用电压) */
/* 运动中积分增量按参考的快慢分账：参考速度/加速度越大，这一拍积分补的越是模型误差(kv、ka、负载惯量与标定不符，
 * 误差 ∝ 速度、加速度)，停下后这股力就不该再顶着；慢速时补的才是静载(重力、摩擦偏差)。
 * "动态"份额 = min(1, |v_r|/ROLL_VEL + |a_r|/ROLL_ACC)，停下后按 2^ROLL_SHIFT ms 时间常数从积分里撤回 */
#define ROLL_VEL_CDPS   10000  /* 100°/s：再低会把重力臂匀速流式运动中学到的重力也当成动态撤掉 */
#define ROLL_ACC_CDPSS  30000  /* 300°/s² */
#define ROLL_SHIFT          4
#define ROLL_KV_Q8 ((int32_t)((65536UL * 256U + ROLL_VEL_CDPS / 2U) / ROLL_VEL_CDPS))
#define ROLL_KA_Q8 ((int32_t)((65536UL * 256U + ROLL_ACC_CDPSS / 2U) / ROLL_ACC_CDPSS))
#define HOLD_BLEED_DIV  4      /* 保持中托举力把轴推走一个最小可靠位移，积分退 fs/4 */
#define HOLD_BLEED_MS 300      /* 推走要持续这么久且慢才算：力臂余振半周期就回来，不能当成托举力过大 */
#define SETTLE_GIVEUP_MS 300   /* 收尾中在到位窗内静止这么久仍没进半窗，按窗内到位，不再来回找 */
#define DIST_FF_MS      60     /* 运动中扰动前馈的低通时间常数：滤掉电位器滑动噪声，重力随角度的变化跟得上 */
#define DIST_FF_SETTLE_MS 400  /* 收尾中扰动前馈的低通时间常数：见 C_PosCtrl_Update */
#define HOLD_LEAK_MS  1000     /* 保持中窗内静止时积分退向0的速度：每这么多ms退一个 fs；轴一沉就全部补回 */
#define SETTLE_LEARN_X  6      /* 收尾中学托举力的速度 = 运动积分零点 × 它(只在窗外且没朝目标走时学)；
                                * 3 时重力臂冲过头回来后要多爬几下(停稳慢约0.5s)，6 与原来相当 */

static struct {
    int32_t kv_q16;          /* 速度反馈 Kv，mV/(厘度/s)，Q16 */
    int32_t kv_pos_q16;      /* 模型kv：位置修正速度对应的电压，Q16 */
    int32_t kp_q8;           /* 位置增益 ω_p，1/s，Q8 */
    int32_t ki_q20;          /* 积分每拍增益，mV/(厘度/s)，Q20 */
    int32_t kh_q16;          /* 保持刚度 mV/厘度，Q16 */
    int32_t kih_q20;         /* 保持积分每拍增益 mV/厘度，Q20 */
    int32_t delay_q16;       /* 前馈速度超前时长，秒，Q16 */
    int32_t fric_inv_q16;    /* 库仑前馈过渡速度的倒数 */
    int32_t vel_noise, w_in; /* w_in：到位窗，也是保持窗 */
    int32_t ka_nv, kv_uv;
    int32_t ka_load_nv;      /* 负载惯量对应的加速系数 = ka×负载惯量比：加速度前馈要推动电机+负载 */
    uint16_t precision, vel_bw_max;
    uint16_t hold_kp, hold_ki;   /* 保持 P(‰/°)、I(‰/(°·s))，‰ 指标定电压的千分之一 */
    uint16_t cal_mv;
    int16_t fc_mv, fs_mv;
    uint8_t noise, min_step;
} s_gain;

static struct {
    int32_t  vel_error_q8;   /* 速度跟踪残差的一阶降噪状态，Q8 */
    int32_t  integral_q16;   /* mV，Q16 */
    int32_t  anchor;         /* 上次挪动够最小步长时的位置 */
    int32_t  boost;          /* 起转加码，mV */
    uint16_t still_ms;       /* 自上次挪动起的拍数 */
    uint16_t cap_ms;
    uint16_t hold_blend_ms;
    uint8_t  qualified;      /* 本条运动的到位资格，与当前使用的控制律分开 */
    uint8_t  state;
    int32_t  hold_p;         /* 保持中 P 项的输出，mV：转入运动时并入积分 */
    int32_t  i_dyn_q16;      /* 本段运动里按"动态"份额攒进积分的量，mV Q16：停下后撤回 */
    int32_t  hold_anchor;    /* 保持中上次退托举力(或进保持)时的位置 */
    uint8_t  hold_anchor_ok;
    uint8_t  leak_off;       /* 本次保持中退托举力把轴退沉了：已补回，不再退 */
    int32_t  leak_anchor;    /* 进保持时的位置 */
    int32_t  leak_q16;       /* 本次保持中已退掉的托举力(带原积分符号)，mV Q16 */
    int32_t  dist_q8;        /* 扰动前馈低通状态，mV Q8 */
    uint16_t bleed_ms;       /* 保持中托举力持续推走轴的时长 */
    uint16_t inwin_ms;       /* 收尾中在到位窗内静止的时长 */
    uint8_t  braking;        /* 保持中处于短路制动(带回差)*/
    uint8_t  stall_hold;     /* 堵转保持：清托举力后不再使用/学习积分，直到下一段运动 */
} s_ctrl;

void C_PosCtrl_Init(const MotorCal_t *cal, int32_t vel_noise, uint16_t obs_bw_hz)
{
    memset(&s_gain, 0, sizeof(s_gain));
    s_gain.ka_nv     = cal->ka_nv;
    s_gain.kv_uv     = cal->kv_uv;
    s_gain.kv_pos_q16 = (int32_t)(((int64_t)cal->kv_uv << 16) / 1000);
    s_gain.fc_mv     = cal->fc_mv;
    s_gain.fs_mv     = cal->fs_mv;
    s_gain.cal_mv    = cal->cal_mv;
    s_gain.noise     = cal->noise_cdeg;
    s_gain.min_step  = cal->min_step_cdeg;
    s_gain.vel_noise = vel_noise;
    s_gain.fric_inv_q16 = 65536 / vel_noise;   /* 参考速度没有噪声：过一个噪声底就给满动摩擦，慢速跟随不欠补偿 */
    s_gain.delay_q16 = (int32_t)(((int64_t)cal->delay_us << 16) / 1000000);
    /* 速度环带宽上限：纯延迟的相位 ω·延迟 ≤ 1/3(与 A_Config.h 的 CTRL_VEL_BW_DELAY_MAX 同一公式)，另不超过观测器带宽 */
    s_gain.vel_bw_max = (uint16_t)((cal->delay_us > 333U) ? 1000000UL / (3UL * cal->delay_us) : 1000UL);
    if (s_gain.vel_bw_max > (uint32_t)obs_bw_hz * 6283U / 1000U) s_gain.vel_bw_max = (uint16_t)((uint32_t)obs_bw_hz * 6283U / 1000U);
    C_PosCtrl_Reset();
}

/* 保持两项都是相对扭矩增益，以标定电压为满量程(电压驱动下出力 ∝ 电压)，与电机参数无关，实际效果再乘 1.5：
 *   K_h = 1.5·P‰·标定电压/°(mV/厘度 = 1.5·P·cal/100000)；积分每秒增加 1.5·I‰·标定电压/°·ex */
static void PosCtrl_HoldGains(void)
{
    s_gain.kh_q16  = (int32_t)(((int64_t)s_gain.cal_mv * s_gain.hold_kp * 65536 * HOLD_GAIN_X10) / 1000000);
    s_gain.kih_q20 = (int32_t)((((int64_t)s_gain.cal_mv * s_gain.hold_ki << 20) * HOLD_GAIN_X10) / 1000000000);   /* 1ms 一拍 */
}


uint8_t C_PosCtrl_SetGains(PosCtrlGains_t *g)
{
    PosCtrlGains_t in = *g;
    int64_t kv_tot, kv_place;

    g->vel_bw = (uint16_t)Fx_ClampRange(g->vel_bw, 10, s_gain.vel_bw_max);
    g->pos_bw = (uint16_t)Fx_ClampRange(g->pos_bw, 1, g->vel_bw / 4);
    g->int_bw = (uint16_t)Fx_ClampRange(g->int_bw, 0, g->pos_bw);
    if (g->load_ratio_x10 > POS_CTRL_LOAD_RATIO_MAX_X10) g->load_ratio_x10 = POS_CTRL_LOAD_RATIO_MAX_X10;
    if (g->bw_load_ratio_x10 > POS_CTRL_LOAD_RATIO_MAX_X10) g->bw_load_ratio_x10 = POS_CTRL_LOAD_RATIO_MAX_X10;
    s_gain.ka_load_nv = (int32_t)((int64_t)s_gain.ka_nv * g->load_ratio_x10 / 10);

    /* 速度环：对象 1/(ka·s + kv)，闭环极点落在 ω_v：ka·ω_v = kv + Kv。电机自身带宽 kv/ka 已够时 Kv 取 ka·ω_v/4 */
    kv_tot   = ((int64_t)s_gain.ka_nv * g->vel_bw * 65536) / 1000000;      /* ka·ω_v，Q16 */
    kv_place = kv_tot - ((int64_t)s_gain.kv_uv * 65536) / 1000;
    s_gain.kv_q16 = (int32_t)((kv_place > kv_tot / 4) ? kv_place : kv_tot / 4);
    /* 带宽和模型用两个惯量：稳定裕度要按可能的最大惯量收敛外环；扰动观测器回路却要求模型惯量不超过实际(偏大时
     * 回路增益>1，空载会振)，一个数两头顾不了 */
    if (g->bw_load_ratio_x10 != 0U)
    {
        int64_t damping = s_gain.kv_q16 + s_gain.kv_pos_q16;
        int64_t inertia = (int64_t)s_gain.ka_nv * (10U + g->bw_load_ratio_x10) * 65536;
        int32_t max_pos_bw = (int32_t)(damping * 10000000 / inertia) / 4;

        if (max_pos_bw < 1) max_pos_bw = 1;
        if (g->pos_bw > max_pos_bw) g->pos_bw = (uint16_t)max_pos_bw;
        if (g->int_bw > g->pos_bw) g->int_bw = g->pos_bw;
    }
    s_gain.kp_q8  = g->pos_bw * 256;
    /* 积分零点按 ka·ω_v 放：Kv 取下限时积分不跟着变弱 */
    s_gain.ki_q20 = (int32_t)((kv_tot * g->int_bw * 16) / 1000);
    PosCtrl_HoldGains();
    return (uint8_t)(memcmp(&in, g, sizeof(in)) != 0);
}

void C_PosCtrl_SetHold(uint16_t precision, uint16_t hold_kp, uint16_t hold_ki)
{
    if (precision == s_gain.precision && hold_kp == s_gain.hold_kp && hold_ki == s_gain.hold_ki) return;
    s_gain.precision = precision;
    s_gain.w_in      = precision;   /* 唯一的窗口：到位窗 = 保持窗，下限由配置层保证(1.5×最小可靠位移) */
    s_gain.hold_kp   = hold_kp;
    s_gain.hold_ki   = hold_ki;
    PosCtrl_HoldGains();
}

void C_PosCtrl_Reset(void)
{
    memset(&s_ctrl, 0, sizeof(s_ctrl));
    s_ctrl.state    = PC_SETTLE;
    s_ctrl.still_ms = 0xFFFFU;
}

/* 堵转触发的保持：目标即当前位置，直接进 HOLD(跳过捕获/交接)；去掉积分托举力，
 * 只留窗外弹簧。结束状态与普通到位保持一致，直到下一段运动才恢复正常积分。 */
void C_PosCtrl_StallHold(void)
{
    s_ctrl.integral_q16 = 0;
    s_ctrl.i_dyn_q16    = 0;
    s_ctrl.hold_p       = 0;
    s_ctrl.dist_q8      = 0;   /* 并入积分的扰动前馈随积分一起丢掉；不清的话恢复运动时会按冻结值再扣一次，积分永久偏 */
    s_ctrl.stall_hold   = 1U;
    s_ctrl.state        = PC_HOLD;
    s_ctrl.qualified    = 1U;
    s_ctrl.cap_ms       = 0U;
    s_ctrl.hold_blend_ms = 0U;
    s_ctrl.hold_anchor_ok = 0U;
}

uint8_t C_PosCtrl_Reached(void) { return (uint8_t)(s_ctrl.state == PC_HOLD); }
uint8_t C_PosCtrl_ArrivalQualified(void) { return s_ctrl.qualified; }

void C_PosCtrl_BeginMotion(void)
{
    s_ctrl.cap_ms = 0U;
    s_ctrl.qualified = 0U;
    s_ctrl.stall_hold = 0U;
}

/* 起转补偿：要求动却卡住时把输出补到 积分±fs，仍不动再逐拍加码(至多 boost_max)；挪动够最小步长后立即撤掉。
 * "要求动" = 运动中参考在起步/匀速，或收尾/保持时误差超出到位窗(往窗内推)。
 * 与加大积分不同：只在卡住时加，加码快慢与误差大小无关，一动就清零——不会把攒下的力带过静摩擦，冲到另一侧。
 * 以积分为基准：带载时积分托着负载，以0为基准会把向下的顶起变成整段卸载(力臂掉落)，向上的顶起则不起作用。
 * 卡住计时每拍都更新，所以不在运动中时也要调用 */
static int32_t PosCtrl_Stiction(const PosCtrlFeedback_t *fb, int32_t demand, int32_t u, int32_t load, int32_t boost_max)
{
    int32_t need;

    if (Fx_Abs32(fb->pos - s_ctrl.anchor) >= s_gain.min_step) { s_ctrl.anchor = fb->pos; s_ctrl.still_ms = 0U; }
    else if (s_ctrl.still_ms < 0xFFFFU) s_ctrl.still_ms++;

    if (demand == 0 || s_ctrl.still_ms < STICK_MOVE_MS || Fx_Abs32(fb->vel) >= s_gain.vel_noise)
    {
        s_ctrl.boost -= s_gain.fs_mv / STICK_ESC_DIV;
        if (s_ctrl.boost < 0) s_ctrl.boost = 0;
        return 0;
    }
    if (s_ctrl.boost > boost_max) s_ctrl.boost = boost_max;   /* 收尾里攒的加码不带进运动/保持 */
    if (s_ctrl.still_ms >= STICK_ESC_MS && s_ctrl.boost < boost_max) s_ctrl.boost += s_gain.fs_mv / STICK_ESC_DIV;
    need = Fx_MulShift(s_ctrl.integral_q16, 1, 16) + load + demand * s_gain.fs_mv;   /* 托举力=积分+扰动前馈 */
    return (((demand > 0) ? (u < need) : (u > need)) ? need - u : 0) + demand * s_ctrl.boost;
}

/* 本拍积分增量里"动态"的份额，Q16：参考越快、加减速越急越接近1 */
static int32_t PosCtrl_DynShare(const TrajRef_t *ref)
{
    int32_t v = Fx_Abs32(ref->vel), a = Fx_Abs32(ref->acc), share;

    if (v >= (int32_t)ROLL_VEL_CDPS || a >= (int32_t)ROLL_ACC_CDPSS) return 65536;
    share = ((v * ROLL_KV_Q8) >> 8) + ((a * ROLL_KA_Q8) >> 8);
    return (share > 65536) ? 65536 : share;
}

/* 收尾/保持中误差超过它就做起转补偿，收尾中误差进它才捕获。
 * 收尾瞄准半窗(不小于最小可靠位移)：只瞄到位窗的话窗内 P 推不动静摩擦，到位总贴着窗边，静止读数噪声再让它
 * 在窗边进出；半窗进不去(静摩擦/齿隙卡住)时窗内静止 SETTLE_GIVEUP_MS 后放宽到整窗，不来回找。
 * 保持中出窗再多一个最小可靠位移才补(回差)，否则停在窗边时每次噪声出窗都抽动一下 */
static int32_t PosCtrl_PushWindow(void)
{
    int32_t aim = (s_gain.w_in / 2 > (int32_t)s_gain.min_step) ? s_gain.w_in / 2 : (int32_t)s_gain.min_step;

    if (s_ctrl.state == PC_HOLD) return s_gain.w_in + (int32_t)s_gain.min_step;
    return (s_ctrl.inwin_ms >= SETTLE_GIVEUP_MS && s_gain.w_in > aim) ? s_gain.w_in : aim;
}

/* 参考在动用MOVE，参考结束进SETTLE；到位资格和进入保持律分别判断。
 * 条件撤销时交接权重逐拍退回，不搬运/清零积分。HOLD保持到下一段运动。 */
static void PosCtrl_State(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t e)
{
    if (!ref->done)
    {
        s_ctrl.qualified = 0U;
        s_ctrl.cap_ms = s_ctrl.hold_blend_ms = 0U;
        s_ctrl.state = PC_MOVE;
        return;
    }
    if (!fb->valid) return;
    if (s_ctrl.state == PC_MOVE)
    {
        s_ctrl.state = PC_SETTLE;
        s_ctrl.cap_ms = 0U;
        return;
    }
    if (s_ctrl.state == PC_HOLD)
    {
        /* 新提交的零位移目标可能已经在保持，不能因清到位资格而永远不完成。 */
        if (Fx_Abs32(e) <= s_gain.w_in && Fx_Abs32(fb->vel) <= s_gain.vel_noise) s_ctrl.qualified = 1U;
        /* 保持到下一段运动，不因出窗回收尾：保持区是刚度由保持 P 决定的弹簧(保持 I 默认0，不攒力)。
         * 出窗回收尾的话，外力掰住时收尾的积分、起转补偿、扰动前馈会对着手攒力，松手冲过中心。要回中心就设保持 I */
        return;
    }
    if (s_ctrl.state != PC_SETTLE) return;
    /* 捕获计数：满足加1、不满足减2(要求约2/3的拍满足)。一拍不满足就清零的话，观测速度噪声偶尔越过噪声底
     * 会让它长期进不了保持。捕获后即上报到位并开始交接，不再另等位置静止(力臂在齿隙里轻晃也能进保持)。
     * 捕获范围见 PosCtrl_PushWindow */
    if (Fx_Abs32(fb->vel) <= s_gain.vel_noise && Fx_Abs32(e) <= s_gain.w_in) { if (s_ctrl.inwin_ms < 0xFFFFU) s_ctrl.inwin_ms++; }
    else s_ctrl.inwin_ms = 0U;
    if (Fx_Abs32(fb->vel) <= s_gain.vel_noise && Fx_Abs32(e) <= PosCtrl_PushWindow())
    {
        if (s_ctrl.cap_ms < CAPTURE_MS) s_ctrl.cap_ms++;
        if (s_ctrl.cap_ms >= CAPTURE_MS)
        {
            s_ctrl.qualified = 1U;
            if (s_ctrl.hold_blend_ms < HOLD_BLEND_MS) s_ctrl.hold_blend_ms++;
            else s_ctrl.state = PC_HOLD;
        }
    }
    else
    {
        s_ctrl.cap_ms = (s_ctrl.cap_ms > 2U) ? (uint16_t)(s_ctrl.cap_ms - 2U) : 0U;
        if (s_ctrl.hold_blend_ms != 0U) s_ctrl.hold_blend_ms--;
    }
}

/* 保持律(HOLD 与收尾→保持的交接共用)：只对超出到位窗的量 ex 做 PI，返回弹簧项 K_h·ex，积分增量写到 *di。
 * 窗内只剩积分(到位时继承的托举力)；窗外是从窗口边沿连续长出来的弹簧，积分每秒加 K_ih·ex。
 * 不另加阻尼：电压驱动下反电势本身就是阻尼；大保持 P 松手时的少量越过来自齿隙和力臂弹性，阻尼管不到 */
static int32_t PosCtrl_HoldLaw(int32_t e, int64_t *di)
{
    int32_t ex = Fx_Deadband(e, s_gain.w_in);

    *di = ((int64_t)s_gain.kih_q20 * ex) >> 4;
    return Fx_MulShift(s_gain.kh_q16, ex, 16);
}

/* 保持中退掉"把轴推着走"的托举力：轴朝积分推的方向挪了一个最小可靠位移，积分就退 fs/4，直到推不动为止。
 * 保持中积分不动(保持 I 默认0)，继承的力若大于动摩擦，轴就爬出到位窗、被保持 P/起转补偿顶回、补偿一撤
 * 再被推出去，在窗边来回摆。收尾积分已不再攒反向力，这里兜其它来源。
 * 只在继承的力本身推着轴走时才退；重力压着轴往反方向走时不动它 */
static void PosCtrl_HoldBleed(const PosCtrlFeedback_t *fb)
{
    int32_t i_mv = Fx_MulShift(s_ctrl.integral_q16, 1, 16), moved, bleed;

    if (!fb->valid) return;
    if (!s_ctrl.hold_anchor_ok) { s_ctrl.hold_anchor = s_ctrl.leak_anchor = fb->pos; s_ctrl.hold_anchor_ok = 1U; }
    moved = (fb->pos - s_ctrl.hold_anchor) * Fx_Sign32(i_mv);
    /* 推走要持续 HOLD_BLEED_MS 且在推走处已慢下来：进保持时力臂常还在余振，往托举力方向晃一下又回来，
     * 不持续地判会连退几次，托举力不够、轴沉出窗 */
    if (moved >= s_gain.min_step && s_ctrl.bleed_ms < 0xFFFFU) s_ctrl.bleed_ms++;
    else if (moved < s_gain.min_step) s_ctrl.bleed_ms = 0U;
    if (s_ctrl.bleed_ms >= HOLD_BLEED_MS && Fx_Abs32(fb->vel) <= s_gain.vel_noise)
    {
        s_ctrl.bleed_ms = 0U;
        bleed = s_gain.fs_mv / HOLD_BLEED_DIV;
        if (bleed > Fx_Abs32(i_mv)) bleed = Fx_Abs32(i_mv);
        s_ctrl.integral_q16 -= Fx_Sign32(i_mv) * (bleed << 16);
        s_ctrl.hold_anchor = fb->pos;
    }
    else if (moved <= -s_gain.min_step) s_ctrl.hold_anchor = fb->pos;
}

/* 保持中退掉"推不动轴"的托举力：HoldBleed 只管推着轴走的力，小于静摩擦的力(比如上电第一次收尾在静摩擦上
 * 攒的)推不动轴、永远退不掉，动态回撤每次运动后又把它原样还回来，空载静止时一直出几十mV。
 * 只退进保持时不超过 fs/2 的积分，窗内静止时按 fs/HOLD_LEAK_MS 每拍退向0。更大的是重力托举力：收尾只在
 * 卡住时学，学到的正好在静摩擦边缘，退几十mV就沉、沉出窗后补回也推不回来(保持中起转补偿封顶)，实测带载
 * 卡在窗外 0.36° 持续出力，所以这次保持整段不退。小于 fs/2 的力若是负载要的(重力略大于静摩擦)：轴朝它的
 * 反方向沉了一个最小可靠位移或沉出到位窗，全部补回，本次保持不再退。窗外不退，交给保持 PI */
static void PosCtrl_HoldLeak(const PosCtrlFeedback_t *fb, int32_t e)
{
    int32_t s, step, sag;

    if (!fb->valid || s_ctrl.stall_hold || s_ctrl.leak_off || !s_ctrl.hold_anchor_ok) return;
    if (s_ctrl.leak_q16 == 0 && Fx_Abs32(s_ctrl.integral_q16) > ((int32_t)s_gain.fs_mv << 15))
    {
        s_ctrl.leak_off = 1U;
        return;
    }
    s = Fx_Sign32(s_ctrl.leak_q16);
    sag = (fb->pos - s_ctrl.leak_anchor) * s;
    if (s != 0 && (sag <= -s_gain.min_step || (sag < 0 && Fx_Abs32(e) > s_gain.w_in)))
    {
        s_ctrl.integral_q16 += s_ctrl.leak_q16;
        s_ctrl.leak_q16 = 0;
        s_ctrl.leak_off = 1U;
        return;
    }
    if (Fx_Abs32(e) > s_gain.w_in) return;
    s = Fx_Sign32(s_ctrl.integral_q16);
    step = ((int32_t)s_gain.fs_mv << 16) / HOLD_LEAK_MS;
    if (step > Fx_Abs32(s_ctrl.integral_q16)) step = Fx_Abs32(s_ctrl.integral_q16);
    s_ctrl.integral_q16 -= s * step;
    s_ctrl.leak_q16 += s * step;
}

/* 扰动前馈量(已低通)，mV；堵转保持不托负载 */
static int32_t PosCtrl_DistFF(int32_t limit_mv)
{
    return s_ctrl.stall_hold ? 0 : Fx_Clamp32(s_ctrl.dist_q8 / 256, limit_mv / 2);
}

/* 积分加 di(mV Q16)，幅值不超过半个可用电压；返回实际加上的量(限幅时比 di 少) */
static int32_t PosCtrl_IntegralAdd(int64_t di, int32_t limit_mv)
{
    int64_t i_lim = (int64_t)limit_mv << 15, next = s_ctrl.integral_q16 + di;
    int32_t prev = s_ctrl.integral_q16;

    s_ctrl.integral_q16 = (int32_t)((next > i_lim) ? i_lim : ((next < -i_lim) ? -i_lim : next));
    return s_ctrl.integral_q16 - prev;
}

int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit_mv, PosCtrlDbg_t *dbg)
{
    int32_t e = ref->pos - fb->pos;
    int32_t u_ff = 0, u_fb, u_i, u_d, u_stick, desired, output, demand = 0, i_in, dyn_share = 0;
    int64_t di;   /* 本拍积分增量，mV Q16 */
    uint8_t prev_state = s_ctrl.state;

    /* 扰动前馈的带宽随状态：运动中要跟上随角度快变的重力；收尾中负载已不变、扰动又估不准——竖直处负载在齿隙里
     * 悬空，力臂自身有几Hz弹性模态，刚体观测器把臂的晃动当扰动估出来，原样前馈回去等于一根带滞后、与位置反馈
     * 同相的弹簧，竖直处来回摆。收尾用慢低通，减速段的动态成分也慢慢退到静态值。保持中冻结，见下面的交接。
     * 滤波状态连续，换带宽不跳 */
    if (s_ctrl.state != PC_HOLD)
        s_ctrl.dist_q8 += (fb->dist * 256 - s_ctrl.dist_q8) / (((s_ctrl.state == PC_MOVE) ? DIST_FF_MS : DIST_FF_SETTLE_MS) + 1);
    if (!ref->done && s_ctrl.state != PC_MOVE) s_ctrl.i_dyn_q16 = 0;   /* 新一段运动重新分账 */
    PosCtrl_State(ref, fb, e);
    /* 进出保持的无扰交接，本拍输出都不变：
     * 进保持把扰动前馈并入积分——轴被静摩擦粘住时扰动不可观测(±fs 内任何值都解释得了静止)，估计被读数噪声
     * 推着走，留在输出上会绕着短路制动门槛来回跳；保持中交给继承托举力/退残留/泄放那套。
     * 出保持把窗外托着负载的保持 P 并入积分(新运动从实测位置起步、误差≈0，不并入这份力一拍消失，负载先掉再追)，
     * 并按进保持时冻结的同一个扰动前馈值扣回 */
    if (s_ctrl.state == PC_HOLD && prev_state != PC_HOLD)
        PosCtrl_IntegralAdd((int64_t)PosCtrl_DistFF(limit_mv) << 16, limit_mv);
    else if (s_ctrl.state != PC_HOLD && prev_state == PC_HOLD)
        PosCtrl_IntegralAdd((int64_t)(s_ctrl.hold_p - PosCtrl_DistFF(limit_mv)) << 16, limit_mv);
    if (s_ctrl.state != PC_HOLD)
    {
        s_ctrl.hold_anchor_ok = s_ctrl.leak_off = s_ctrl.braking = 0U;
        s_ctrl.leak_q16 = 0; s_ctrl.bleed_ms = 0U;
    }
    if (s_ctrl.state != PC_SETTLE) s_ctrl.inwin_ms = 0U;
    /* 仅运动律的比例速度反馈使用滤波残差；积分、状态判定和收尾仍使用原观测量。 */
    s_ctrl.vel_error_q8 += ((ref->vel - fb->vel) * 256 - s_ctrl.vel_error_q8) / (VEL_FILTER_MS + 1);

    switch (s_ctrl.state)
    {
    case PC_MOVE:
    {
        int32_t v_pos = Fx_MulShift(s_gain.kp_q8, Fx_Deadband(e, s_gain.noise), 8);
        int32_t v_cmd = ref->vel + v_pos;
        int32_t frac = Fx_Clamp32(ref->vel * s_gain.fric_inv_q16, 65536);   /* 平滑符号，Q16 */

        u_ff = C_MotorModel_Feedforward(ref->vel + Fx_MulShift(ref->acc, s_gain.delay_q16, 16), ref->acc)
             + Fx_MulShift(s_gain.fc_mv, frac, 16)
             + (int32_t)(((int64_t)s_gain.ka_load_nv * ref->acc) / 1000000);   /* 负载惯量：不补的话减速段只给一半制动力，冲过头 */
        i_in = v_cmd - fb->vel;
        u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + s_ctrl.vel_error_q8 / 256, 16)
             + (int32_t)(((int64_t)s_gain.kv_pos_q16 * v_pos) / 65536);
        di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
        dyn_share = PosCtrl_DynShare(ref);
        /* 参考在起步/匀速时按速度指令方向要求动；参考减速段(含反转前的过零)轴本来就该慢下来，
         * 低速不是卡住，顶起会和减速前馈对着干；已超前参考一个到位窗以上也不顶 */
        if (Fx_Abs32(v_cmd) > s_gain.vel_noise && (int64_t)ref->acc * ref->vel >= 0
            && (int64_t)Fx_Sign32(v_cmd) * e >= -s_gain.w_in) demand = Fx_Sign32(v_cmd);
        break;
    }
    case PC_SETTLE:
    {
        /* 收尾：无记忆的 P + 阻尼把轴拉到目标，积分只当静载托举力。
         * 刚度 = 位置环 P + 弹簧 K_s·e。原来积分里带 −∫v，等于一根锚在"进入收尾时轴所在位置"(常是冲过头处)的
         * 弹簧 K_s·(x_进入 − x)，平衡点落在目标与冲过头处之间、常在窗外，只能靠起转补偿一下下爬回(快速来回拖动
         * 后的到位蠕动)。现在同一刚度锚在目标：带载动态不变，平衡点就是目标。K_s = 积分增益(mV/厘度)，
         * P 与弹簧都留最小可靠位移的死区；速度项用半个噪声底的死区，静止时只剩噪声不出力。
         * 积分(托举力)只在窗外、且轴没在朝目标走时学窗外误差：卡住或被推走说明力不够；正往回走说明力够了，
         * 再学就会攒过头。速度项不进积分。窗内不学，卡在窗内就算到了，不会推破静摩擦滑到另一侧来回找 */
        int32_t e_db = Fx_Deadband(e, s_gain.min_step);
        int32_t v_pos = Fx_MulShift(s_gain.kp_q8, e_db, 8);
        int32_t v_damp = Fx_Deadband(fb->vel, s_gain.vel_noise / 2);
        int32_t ex = Fx_Deadband(e, s_gain.w_in);

        i_in = v_pos - v_damp;
        u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)
             + (int32_t)(((int64_t)s_gain.kv_pos_q16 * v_pos) / 65536)
             + Fx_MulShift(s_gain.ki_q20, e_db * 1000, 20);   /* K_s·e：积分每拍增益 × 1000拍/s = mV/厘度 */
        di = (!s_ctrl.stall_hold && ex != 0 && (Fx_Abs32(fb->vel) < s_gain.vel_noise || (int64_t)fb->vel * e <= 0))
           ? (((int64_t)s_gain.ki_q20 * Fx_MulShift(s_gain.kp_q8, ex, 8) * SETTLE_LEARN_X) >> 4) : 0;
        if (s_ctrl.hold_blend_ms != 0U)
        {
            int64_t hold_di;
            int32_t hold_fb = PosCtrl_HoldLaw(e, &hold_di);
            int32_t integral = Fx_MulShift(s_ctrl.integral_q16, 1, 16);

            /* 交接终点也包含HOLD的小电压短路制动规则，避免最后一拍再跳一次。 */
            if (Fx_Abs32(hold_fb + integral) < s_gain.fc_mv / 2) hold_fb = -integral;
            u_fb += (int32_t)(((int64_t)hold_fb - u_fb) * s_ctrl.hold_blend_ms / HOLD_BLEND_MS);
            di += (hold_di - di) * s_ctrl.hold_blend_ms / HOLD_BLEND_MS;
        }
        break;
    }
    default:   /* PC_HOLD：保持到下一段运动，见 PosCtrl_HoldLaw */
        PosCtrl_HoldBleed(fb);
        PosCtrl_HoldLeak(fb, e);
        s_ctrl.hold_p = PosCtrl_HoldLaw(e, &di);
        u_fb = s_ctrl.hold_p;
        break;
    }

    /* 堵转保持：积分托举力清零且不再积累，出力只靠窗外弹簧/起转补偿 */
    if (s_ctrl.stall_hold) { di = 0; u_i = 0; }
    else                   u_i = Fx_MulShift(s_ctrl.integral_q16, 1, 16);
    /* 扰动前馈：观测器把电机模型解释不了的那部分电压(重力随角度变、负载摩擦、惯量/标定误差)按扰动极点估出来，
     * 直接抵消。负载不再全靠积分慢慢攒：力臂过竖直时重力在半秒内反号，积分(零点 ω_i)跟不上。
     * 积分只剩补观测器的静差。保持中已并入积分(见上面的交接)；堵转保持不托负载 */
    u_d = (s_ctrl.state == PC_HOLD) ? 0 : PosCtrl_DistFF(limit_mv);
    /* 收尾/保持时卡在窗外：P 和慢积分都推不动静摩擦，同样做起转补偿 */
    if (s_ctrl.state != PC_MOVE && fb->valid && Fx_Abs32(e) > PosCtrl_PushWindow()) demand = Fx_Sign32(e);
    /* 收尾时卡在窗外：重力+静摩擦可能比 积分+2fs 还大，窗外那一点误差让积分攒得极慢(实测要等几秒才自己动)，
     * 所以收尾中加码不封顶到一个 fs，加到推动为止 */
    u_stick = PosCtrl_Stiction(fb, demand, u_ff + u_fb + u_i + u_d, u_d, (s_ctrl.state == PC_SETTLE) ? limit_mv / 2 : s_gain.fs_mv);
    desired = u_ff + u_fb + u_i + u_d + u_stick;
    /* 不值得驱动就短路制动；带回差(低于 fc/2 进、高于 fc 出)：窗边输出被噪声推着过门槛会 0↔几十PWM 来回翻 */
    if (s_ctrl.state == PC_HOLD)
    {
        s_ctrl.braking = (uint8_t)(Fx_Abs32(desired) < (s_ctrl.braking ? s_gain.fc_mv : s_gain.fc_mv / 2));
        if (s_ctrl.braking) desired = 0;
    }
    output = Fx_Clamp32(desired, limit_mv);

    /* 积分：无效采样时冻结；输出限幅时只允许往回退 */
    if (fb->valid && di != 0 && !((di > 0 && desired > output) || (di < 0 && desired < output)))
    {
        /* 只记实际进了积分的那份：积分顶在限幅上时不多记，撤回时才不会把积分撤成反号 */
        s_ctrl.i_dyn_q16 += (int32_t)(((int64_t)PosCtrl_IntegralAdd(di, limit_mv) * dyn_share) >> 16);
    }
    /* 停下后撤回运动中按速度/加速度攒下的那份积分(模型误差)，按 2^ROLL_SHIFT ms 渐撤，输出不跳 */
    if (s_ctrl.state != PC_MOVE && s_ctrl.i_dyn_q16 != 0)
    {
        int32_t step = s_ctrl.i_dyn_q16 >> ROLL_SHIFT;

        if (step == 0) step = s_ctrl.i_dyn_q16;
        s_ctrl.i_dyn_q16 -= step;
        PosCtrl_IntegralAdd(-(int64_t)step, limit_mv);
    }

    if (dbg)
    {
        dbg->e_pos = e;
        dbg->u_ff = (int16_t)u_ff; dbg->u_fb = (int16_t)u_fb; dbg->u_i = (int16_t)u_i;
        dbg->u_stick = (int16_t)u_stick; dbg->u_out = (int16_t)output;
        dbg->sat = (uint8_t)(desired != output); dbg->state = s_ctrl.state;
    }
    return output;
}
