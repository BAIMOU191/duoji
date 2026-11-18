/* C_Pos_Ctrl.c 位置控制器(纯反馈)：状态→律参数→电机模型前馈与反馈→起转补偿→限幅与积分。
 * 负载(重力、负载惯量、摩擦变化)全部由积分与保持律承担：不用扰动前馈、位置负载学习、负载惯量加速度前馈。
 * 积分抗饱和：限幅时不往加深饱和的方向积；运动/收尾中回算只把积分往 0 退、不推成反号。
 * 增益在 Init/SetGains/SetHold 换算；收尾→保持交接期间混合两种控制律。 */
#include "C_Pos_Ctrl.h"
#include "C_Fixed.h"
#include <string.h>

#define CAPTURE_MS     20U    /* 捕获条件连续满足这么久才开始切到保持律 */
#define HOLD_BLEND_MS  50U    /* 收尾→保持渐进交接，共用一个积分器 */
#define HOLD_GAIN_X10  15     /* 保持 P/I 实际效果 = 参数×1.5(参数范围不变) */
#define VEL_FILTER_MS   1    /* 运动律速度残差一阶滤波 α=1/(1+它)，群延迟=它(ms)；不计入 ω_v 上限(计入会压低带宽、变差抗扰) */
#define STICK_MOVE_MS  5U     /* 这么久没挪过最小可靠位移且估计速度低于噪声底，才算卡住 */
#define STICK_ESC_MS   25U    /* 顶到起转电压后仍卡这么久，开始逐拍加码 */
#define STICK_ESC_DIV  50     /* 加码每拍 fs/50；运动/保持中最多再加一个 fs，收尾中加到推动为止(上限半个可用电压) */
#define HOLD_BLEED_DIV  4      /* 保持中托举力把轴推走一个最小可靠位移，积分退 fs/4 */
#define HOLD_BLEED_MS 300      /* 推走要持续这么久且慢才算：余振半周期不能当成力过大 */
#define HOLD_LEAK_MS  1000     /* 保持中窗内静止时积分退向0的速度：每这么多ms退一个 fs；轴一沉就补回 */
#define SETTLE_LEARN_X  6      /* 收尾中学托举力的速度 = 运动积分零点×它(只在窗外且没朝目标走时学) */
#define MOVE_POS_BW_X   2      /* 运动段位置带宽最多 = ω_p×它(≤ω_v/2)：过冲主要在大角度运动段末形成；收尾仍用 ω_p */
#define MOVE_SPAN_MIN_CDEG  500    /* 本段运动距离 ≤5°：运动段增益 ×1(小角度加大增益显得冲) */
#define MOVE_SPAN_FULL_CDEG 3000   /* ≥30°：加满到 MOVE_POS_BW_X 倍；中间按距离线性过渡 */
#define MOVE_MIX_SLEW_Q8    5      /* 运动中改目标时增益每拍最多变 5/256 份(约50ms走完×1↔×2)，不阶跃 */
/* 每拍用到的常数除法改乘倒数(见 C_Fixed.h)：本芯片无除法指令，Update 里不出现运行时除法 */
#define VEL_FILTER_RECIP     FX_RECIP(VEL_FILTER_MS + 1)
#define HOLD_BLEND_RECIP     FX_RECIP(HOLD_BLEND_MS)

static struct {
    int32_t kv_q16;          /* 速度反馈 Kv，mV/(厘度/s)，Q16 */
    int32_t kv_pos_q16;      /* 模型kv：位置修正速度对应的电压，Q16 */
    int32_t kp_q8;           /* 位置增益 ω_p，1/s，Q8 */
    int32_t kp_move_q8;      /* 运动段位置增益上限，Q8：有速度前馈，位置环只修跟踪误差，可比收尾快 */
    int32_t ki_q20;          /* 积分每拍增益，mV/(厘度/s)，Q20 */
    int32_t kh_q16;          /* 保持刚度 mV/厘度，Q16 */
    int32_t kih_q20;         /* 保持积分每拍增益 mV/厘度，Q20 */
    int32_t delay_q16;       /* 前馈速度超前时长，秒，Q16 */
    int32_t fric_inv_q16;    /* 库仑前馈过渡速度的倒数 */
    int32_t vel_noise, w_in; /* w_in：到位窗，也是保持窗 */
    int32_t ka_nv, kv_uv;
    int32_t aw_q16;          /* 回算抗饱和每拍增益 ω_v·T，Q16 */
    int32_t leak_step_q16;   /* 保持中托举力每拍退的量 fs/HOLD_LEAK_MS，mV Q16 */
    int32_t esc_step_mv;     /* 起转加码每拍步长 fs/STICK_ESC_DIV，mV */
    uint16_t vel_bw_max;
    uint16_t hold_kp, hold_ki;   /* 保持 P(‰/°)、I(‰/(°·s))，‰=标定电压千分之一 */
    uint16_t cal_mv;
    int16_t fc_mv, fs_mv;
    uint8_t noise, min_step;
} s_gain;

static struct {
    int32_t  vel_error_q8;   /* 速度残差一阶降噪状态，Q8 */
    int32_t  integral_q16;   /* mV，Q16 */
    int32_t  anchor;         /* 上次挪够最小步长时的位置 */
    int32_t  boost;          /* 起转加码，mV */
    uint16_t still_ms;       /* 自上次挪动起的拍数 */
    uint16_t cap_ms;
    uint16_t hold_blend_ms;
    uint8_t  qualified;      /* 本条运动的到位资格(与当前控制律分开) */
    uint8_t  state;
    int32_t  hold_p;         /* 保持中 P 项输出 mV：转入运动时并入积分 */
    int32_t  hold_anchor;    /* 保持中上次退托举力(或进保持)时的位置 */
    uint8_t  hold_anchor_ok;
    uint8_t  leak_off;       /* 本次保持退托举力把轴退沉了：已补回，不再退 */
    int32_t  leak_anchor;    /* 进保持时的位置 */
    int32_t  leak_q16;       /* 本次保持已退掉的托举力(带原积分符号)，mV Q16 */
    uint16_t bleed_ms;       /* 保持中托举力持续推走轴的时长 */
    uint8_t  braking;        /* 保持中处于短路制动(带回差) */
    uint8_t  stall_hold;     /* 堵转保持：清托举力后不再用/学积分，直到下一段运动 */
    int32_t  move_mix_q8;    /* 运动段增益在 ω_p 与上限之间的份额(0~256)，跟随 move_mix_tgt 平滑变化 */
    int32_t  move_mix_tgt;   /* 按本段运动距离算出的目标份额 */
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
    s_gain.fric_inv_q16 = 65536 / vel_noise;   /* 参考无噪声：过噪声底就给满动摩擦，慢速不欠补偿 */
    s_gain.leak_step_q16 = ((int32_t)cal->fs_mv << 16) / HOLD_LEAK_MS;   /* 每拍用到的常数除法在这里预算 */
    s_gain.esc_step_mv   = cal->fs_mv / STICK_ESC_DIV;
    s_gain.delay_q16 = (int32_t)(((int64_t)cal->delay_us << 16) / 1000000);
    /* ω_v 上限：纯延迟相位 ω·延迟 ≤ 1/3，且不超过观测器带宽 */
    s_gain.vel_bw_max = (uint16_t)((cal->delay_us > 333U) ? 1000000UL / (3UL * cal->delay_us) : 1000UL);
    if (s_gain.vel_bw_max > (uint32_t)obs_bw_hz * 6283U / 1000U) s_gain.vel_bw_max = (uint16_t)((uint32_t)obs_bw_hz * 6283U / 1000U);
    C_PosCtrl_Reset();
}

/* 保持两项相对扭矩增益，以标定电压为满量程：K_h=1.5·P‰·cal(mV/厘度)，I 每秒 +1.5·I‰·cal·ex */
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
    /* 速度环：对象 1/(ka·s+kv)，极点落 ω_v ⇒ ka·ω_v=kv+Kv；Kv 至少 ka·ω_v/4 */
    kv_tot   = ((int64_t)s_gain.ka_nv * g->vel_bw * 65536) / 1000000;      /* ka·ω_v，Q16 */
    kv_place = kv_tot - ((int64_t)s_gain.kv_uv * 65536) / 1000;
    s_gain.kv_q16 = (int32_t)((kv_place > kv_tot / 4) ? kv_place : kv_tot / 4);
    s_gain.kp_q8  = g->pos_bw * 256;
    s_gain.kp_move_q8 = ((g->pos_bw * MOVE_POS_BW_X < g->vel_bw / 2) ? g->pos_bw * MOVE_POS_BW_X : g->vel_bw / 2) * 256;
    /* 积分零点按 ka·ω_v 放：Kv 取下限时积分不跟着变弱 */
    s_gain.ki_q20 = (int32_t)((kv_tot * g->int_bw * 16) / 1000);
    s_gain.aw_q16 = (int32_t)(((int32_t)g->vel_bw << 16) / 1000);   /* 回算抗饱和每拍增益 ω_v·T */
    PosCtrl_HoldGains();
    return (uint8_t)(memcmp(&in, g, sizeof(in)) != 0);
}

void C_PosCtrl_SetHold(uint16_t precision, uint16_t hold_kp, uint16_t hold_ki)
{
    if ((int32_t)precision == s_gain.w_in && hold_kp == s_gain.hold_kp && hold_ki == s_gain.hold_ki) return;
    s_gain.w_in      = precision;   /* 唯一窗口：到位窗=保持窗，下限由配置层保证 */
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

/* 堵转触发的保持：目标即当前位置，直接进 HOLD；去掉积分托举力，只留窗外弹簧。下一段运动才恢复正常积分 */
void C_PosCtrl_StallHold(void)
{
    s_ctrl.integral_q16 = 0;
    s_ctrl.hold_p       = 0;
    s_ctrl.stall_hold   = 1U;
    s_ctrl.state        = PC_HOLD;
    s_ctrl.qualified    = 1U;
    s_ctrl.cap_ms       = 0U;
    s_ctrl.hold_blend_ms = 0U;
    s_ctrl.hold_anchor_ok = 0U;
}

uint8_t C_PosCtrl_Reached(void) { return (uint8_t)(s_ctrl.state == PC_HOLD); }
uint8_t C_PosCtrl_ArrivalQualified(void) { return s_ctrl.qualified; }

/* 新指令的运动距离(厘度)：≤MIN 用 ω_p，≥FULL 用上限，中间线性 */
void C_PosCtrl_SetMoveSpan(int32_t span_cdeg)
{
    s_ctrl.move_mix_tgt = Fx_ClampRange((Fx_Abs32(span_cdeg) - MOVE_SPAN_MIN_CDEG) * 256 / (MOVE_SPAN_FULL_CDEG - MOVE_SPAN_MIN_CDEG), 0, 256);
}

void C_PosCtrl_BeginMotion(void)
{
    s_ctrl.cap_ms = 0U;
    s_ctrl.qualified = 0U;
    s_ctrl.stall_hold = 0U;
}

/* 起转补偿：要求动却卡住时补到 积分±fs，仍不动再逐拍加码(至多 boost_max)；一动就撤。
 * "要求动"=运动中参考起步/匀速，或收尾/保持时误差超窗(往窗内推)。
 * 与加大积分不同：只卡住时加、与误差大小无关、一动清零——不会攒力冲过静摩擦。以积分为基准(带载时托着负载)。
 * 卡住计时每拍都更新，不在运动中也要调用 */
static int32_t PosCtrl_Stiction(const PosCtrlFeedback_t *fb, int32_t demand, int32_t u, int32_t boost_max)
{
    int32_t need;

    if (Fx_Abs32(fb->pos - s_ctrl.anchor) >= s_gain.min_step) { s_ctrl.anchor = fb->pos; s_ctrl.still_ms = 0U; }
    else if (s_ctrl.still_ms < 0xFFFFU) s_ctrl.still_ms++;

    if (demand == 0 || s_ctrl.still_ms < STICK_MOVE_MS || Fx_Abs32(fb->vel) >= s_gain.vel_noise)
    {
        s_ctrl.boost -= s_gain.esc_step_mv;
        if (s_ctrl.boost < 0) s_ctrl.boost = 0;
        return 0;
    }
    if (s_ctrl.boost > boost_max) s_ctrl.boost = boost_max;   /* 收尾里攒的加码不带进运动/保持 */
    if (s_ctrl.still_ms >= STICK_ESC_MS && s_ctrl.boost < boost_max) s_ctrl.boost += s_gain.esc_step_mv;
    need = Fx_MulShift(s_ctrl.integral_q16, 1, 16) + demand * s_gain.fs_mv;   /* 托举力=积分 */
    return (((demand > 0) ? (u < need) : (u > need)) ? need - u : 0) + demand * s_ctrl.boost;
}

/* 参考在动用 MOVE，参考结束进 SETTLE；到位资格与进入保持律分别判断。
 * 条件撤销时交接权重逐拍退回，不搬运/清零积分。HOLD 保持到下一段运动 */
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
        /* 新提交的零位移目标可能已在保持，不能因清到位资格而永远不完成 */
        if (Fx_Abs32(e) <= s_gain.w_in && Fx_Abs32(fb->vel) <= s_gain.vel_noise) s_ctrl.qualified = 1U;
        /* 保持到下一段运动，不因出窗回收尾：保持区是刚度由保持 P 决定的弹簧(I 默认0不攒力)。
         * 出窗回收尾的话，外力掰住时积分/起转补偿会对着手攒力，松手冲过中心 */
        return;
    }
    if (s_ctrl.state != PC_SETTLE) return;
    /* 捕获计数：满足+1、不满足-2(约2/3拍满足即可)；一拍不满足就清零会让噪声让它长期进不了保持 */
    if (Fx_Abs32(fb->vel) <= s_gain.vel_noise && Fx_Abs32(e) <= s_gain.w_in)
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

/* 保持律(HOLD 与收尾→保持交接共用)：只对超出到位窗的 ex 做 PI，返回弹簧 K_h·ex，积分增量写 *di。
 * 窗内只剩积分(继承的托举力)；窗外从窗边连续长出弹簧。不另加阻尼：电压驱动下反电势本身就是阻尼 */
static int32_t PosCtrl_HoldLaw(int32_t e, int64_t *di)
{
    int32_t ex = Fx_Deadband(e, s_gain.w_in);

    *di = ((int64_t)s_gain.kih_q20 * ex) >> 4;
    return Fx_MulShift(s_gain.kh_q16, ex, 16);
}

/* 保持中退掉"把轴推着走"的托举力：轴朝积分方向挪过一个最小可靠位移就退 fs/4，直到推不动。
 * 继承力大于动摩擦时轴会爬出窗、被顶回、补偿一撤再被推出(窗边摆)。只退推着走的力；重力压反方向时不动 */
static void PosCtrl_HoldBleed(const PosCtrlFeedback_t *fb)
{
    int32_t i_mv = Fx_MulShift(s_ctrl.integral_q16, 1, 16), moved, bleed;

    if (!fb->valid) return;
    if (!s_ctrl.hold_anchor_ok) { s_ctrl.hold_anchor = s_ctrl.leak_anchor = fb->pos; s_ctrl.hold_anchor_ok = 1U; }
    moved = (fb->pos - s_ctrl.hold_anchor) * Fx_Sign32(i_mv);
    /* 推走须持续 HOLD_BLEED_MS 且已慢下来：进保持时力臂常还在余振，晃一下又回来不能退 */
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

/* 保持中退掉"推不动轴"的托举力：小于静摩擦的力(如上电第一次收尾攒的)HoldBleed 退不掉，空载会一直出力。
 * 只退进保持时 ≤ fs/2 的积分；更大是重力托举(收尾学的正好在静摩擦边缘，退一点就沉)，整段不退。
 * 若负载确实要这个力：轴朝反方向沉或沉出窗就全部补回。窗外不退，交给保持 PI */
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
    step = s_gain.leak_step_q16;
    if (step > Fx_Abs32(s_ctrl.integral_q16)) step = Fx_Abs32(s_ctrl.integral_q16);
    s_ctrl.integral_q16 -= s * step;
    s_ctrl.leak_q16 += s * step;
}

/* 积分加 di(mV Q16)，幅值≤半个可用电压 */
static void PosCtrl_IntegralAdd(int64_t di, int32_t limit_mv)
{
    int64_t i_lim = (int64_t)limit_mv << 15, next = s_ctrl.integral_q16 + di;

    s_ctrl.integral_q16 = (int32_t)((next > i_lim) ? i_lim : ((next < -i_lim) ? -i_lim : next));
}

int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit_mv, PosCtrlDbg_t *dbg)
{
    int32_t e = ref->pos - fb->pos;
    int32_t u_ff = 0, u_fb, u_i, u_stick, desired, output, demand = 0, i_in;
    int64_t di;   /* 本拍积分增量，mV Q16 */
    uint8_t prev_state = s_ctrl.state;

    PosCtrl_State(ref, fb, e);
    /* 无扰切换：出保持把窗外托着负载的保持 P 并入积分(新运动从实测位置起步、误差≈0，不并入则这份力一拍消失，负载先掉再追) */
    if (s_ctrl.state != PC_HOLD && prev_state == PC_HOLD)
        PosCtrl_IntegralAdd((int64_t)s_ctrl.hold_p << 16, limit_mv);
    if (s_ctrl.state != PC_HOLD)
    {
        s_ctrl.hold_anchor_ok = s_ctrl.leak_off = s_ctrl.braking = 0U;
        s_ctrl.leak_q16 = 0; s_ctrl.bleed_ms = 0U;
    }
    /* 仅运动律比例速度反馈用滤波残差；积分、状态判定和收尾用原观测量 */
    s_ctrl.vel_error_q8 += Fx_DivR((ref->vel - fb->vel) * 256 - s_ctrl.vel_error_q8, VEL_FILTER_RECIP);

    switch (s_ctrl.state)
    {
    case PC_MOVE:
    {
        int32_t kp, v_pos;

        /* 从静止起步直接取本段份额(误差≈0，不跳)；运动中改目标按斜率过渡 */
        if (prev_state != PC_MOVE) s_ctrl.move_mix_q8 = s_ctrl.move_mix_tgt;
        else s_ctrl.move_mix_q8 += Fx_Clamp32(s_ctrl.move_mix_tgt - s_ctrl.move_mix_q8, MOVE_MIX_SLEW_Q8);
        kp = s_gain.kp_q8 + (((s_gain.kp_move_q8 - s_gain.kp_q8) * s_ctrl.move_mix_q8) >> 8);
        v_pos = Fx_MulShift(kp, Fx_Deadband(e, s_gain.noise), 8);
        int32_t v_cmd = ref->vel + v_pos;
        int32_t frac = Fx_Clamp32(ref->vel * s_gain.fric_inv_q16, 65536);   /* 平滑符号，Q16 */

        u_ff = C_MotorModel_Feedforward(ref->vel + Fx_MulShift(ref->acc, s_gain.delay_q16, 16), ref->acc)
             + Fx_MulShift(s_gain.fc_mv, frac, 16);
        i_in = v_cmd - fb->vel;
        u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + s_ctrl.vel_error_q8 / 256, 16)
             + (int32_t)(((int64_t)s_gain.kv_pos_q16 * v_pos) / 65536);
        di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
        /* 只在参考起步/匀速(acc·v≥0)且指令速度超噪声底、未超前一个到位窗时要求动；减速段低速不是卡住 */
        if (Fx_Abs32(v_cmd) > s_gain.vel_noise && (int64_t)ref->acc * ref->vel >= 0
            && (int64_t)Fx_Sign32(v_cmd) * e >= -s_gain.w_in) demand = Fx_Sign32(v_cmd);
        break;
    }
    case PC_SETTLE:
    {
        /* 收尾：无记忆 P+阻尼拉到目标，积分只当静载托举力。刚度=位置环P+弹簧 K_s·e(锚在目标；
         * 若弹簧锚在进入点则平衡点落在窗外、到位蠕动)。P 与弹簧留最小步长死区，速度项用半个噪声底死区。
         * 托举力只在窗外且轴没朝目标走时学：卡住说明力不够，正往回走说明够了再学会攒过头；窗内不学 */
        int32_t e_db = Fx_Deadband(e, s_gain.min_step);
        int32_t v_pos = Fx_MulShift(s_gain.kp_q8, e_db, 8);
        int32_t v_damp = Fx_Deadband(fb->vel, s_gain.vel_noise / 2);
        int32_t ex = Fx_Deadband(e, s_gain.w_in);

        i_in = v_pos - v_damp;
        u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)
             + (int32_t)(((int64_t)s_gain.kv_pos_q16 * v_pos) / 65536)
             + Fx_MulShift(s_gain.ki_q20, e_db * 1000, 20);   /* K_s·e：积分每拍增益×1000拍/s = mV/厘度 */
        di = (!s_ctrl.stall_hold && ex != 0 && (Fx_Abs32(fb->vel) < s_gain.vel_noise || (int64_t)fb->vel * e <= 0))
           ? (((int64_t)s_gain.ki_q20 * Fx_MulShift(s_gain.kp_q8, ex, 8) * SETTLE_LEARN_X) >> 4) : 0;
        if (s_ctrl.hold_blend_ms != 0U)
        {
            int64_t hold_di;
            int32_t hold_fb = PosCtrl_HoldLaw(e, &hold_di);
            int32_t integral = Fx_MulShift(s_ctrl.integral_q16, 1, 16);
            int32_t w_q16 = (int32_t)Fx_UDivR((uint32_t)s_ctrl.hold_blend_ms << 16, HOLD_BLEND_RECIP);   /* 交接权重，满=65536 */

            /* 交接终点也包含 HOLD 的小电压短路制动规则，避免最后一拍再跳一次 */
            if (Fx_Abs32(hold_fb + integral) < s_gain.fc_mv / 2) hold_fb = -integral;
            u_fb += Fx_MulShift(hold_fb - u_fb, w_q16, 16);
            di += ((hold_di - di) * w_q16) / 65536;
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
    /* 收尾/保持时卡在窗外：P 和慢积分都推不动静摩擦，同样做起转补偿。保持中出窗再多一个最小可靠位移才补(回差)：
     * 停在窗边时读数噪声让它进进出出，每次都补一下会零星抽动 */
    if (s_ctrl.state != PC_MOVE && fb->valid
        && Fx_Abs32(e) > s_gain.w_in + ((s_ctrl.state == PC_HOLD) ? (int32_t)s_gain.min_step : 0)) demand = Fx_Sign32(e);
    /* 收尾卡在窗外时重力+静摩擦可能 > 积分+2fs，加码不封顶到一个 fs，加到推动为止 */
    u_stick = PosCtrl_Stiction(fb, demand, u_ff + u_fb + u_i, (s_ctrl.state == PC_SETTLE) ? limit_mv / 2 : s_gain.fs_mv);
    desired = u_ff + u_fb + u_i + u_stick;
    /* 不值得驱动就短路制动；带回差(fc/2 进、fc 出)：噪声推着过门槛会 0↔几十PWM 来回翻 */
    if (s_ctrl.state == PC_HOLD)
    {
        s_ctrl.braking = (uint8_t)(Fx_Abs32(desired) < (s_ctrl.braking ? s_gain.fc_mv : s_gain.fc_mv / 2));
        if (s_ctrl.braking) desired = 0;
    }
    output = Fx_Clamp32(desired, limit_mv);

    /* 积分：无效采样冻结。限幅时不往加深饱和的方向积；运动/收尾中再回算 (限幅后−限幅前)·ω_v·T，
     * 只把积分往 0 退、最多退到 0：饱和多由前馈和速度反馈造成，普通回算会把积分推成反向力，停下后负载回落或冲过。
     * 保持中只做前一条 */
    if (fb->valid)
    {
        int64_t corr = 0;

        if ((di > 0 && desired > output) || (di < 0 && desired < output)) di = 0;
        if (s_ctrl.state != PC_HOLD && !s_ctrl.stall_hold)
        {
            corr = (int64_t)(output - desired) * s_gain.aw_q16;
            if (corr != 0 && (int64_t)s_ctrl.integral_q16 * corr < 0)
            {
                if ((corr < 0 ? -corr : corr) > Fx_Abs32(s_ctrl.integral_q16)) corr = -(int64_t)s_ctrl.integral_q16;
            }
            else corr = 0;
        }
        if (di + corr != 0) PosCtrl_IntegralAdd(di + corr, limit_mv);
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
