/* C_Pos_Ctrl.c 位置控制器：状态 -> 律参数 -> 前馈与反馈 -> 起转补偿(只在运动中) -> 限幅与积分
 * 增益在初始化/SetGains/SetHold中换算；控制拍使用定点运算，交接期间混合已有两种控制律。 */
#include "C_Pos_Ctrl.h"
#include "C_Fixed.h"
#include <string.h>

#define CAPTURE_MS     20U    /* 捕获条件连续满足这么久才算到位 */
#define HOLD_STILL_MS  200U   /* 位置锚点稳定后才开始切换到保持律 */
#define HOLD_BLEND_MS  50U    /* 收尾/保持律渐进交接，共用一个积分器 */
#define STICK_SLEW_MS   5U     /* 补偿分量每拍最多改变fs/5，主输出不受限 */
#define STICK_MOVE_MS  5U     /* 运动中这么久没挪过最小可靠位移且估计速度低于噪声底，才算卡住 */
#define STICK_ESC_MS   25U    /* 顶到起转电压后仍卡这么久，开始逐拍加码 */
#define STICK_ESC_DIV  50     /* 加码每拍 fs/50，最多再加一个 fs */

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
    uint16_t precision, vel_bw_max;
    uint16_t hold_kp, hold_ki;   /* 保持 P(‰/°)、I(‰/(°·s))，‰ 指标定电压的千分之一 */
    uint16_t cal_mv;
    int16_t fc_mv, fs_mv;
    uint8_t noise, min_step;
} s_gain;

static struct {
    int32_t  vel_error_prev; /* 上一拍速度残差 */
    int32_t  integral_q16;   /* mV，Q16 */
    int32_t  anchor;         /* 上次挪动够最小步长时的位置 */
    int32_t  stick;          /* 起转补偿的连续输出，mV */
    int32_t  boost;          /* 起转加码，mV */
    uint16_t still_ms;       /* 自上次挪动起的拍数 */
    uint16_t cap_ms;
    uint16_t hold_blend_ms;
    uint8_t  qualified;      /* 本条运动的到位资格，与当前使用的控制律分开 */
    uint8_t  state;
    int32_t  hold_p;         /* 保持中 P 项的输出，mV：转入运动时并入积分 */
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
    /* 速度环带宽上限：纯延迟的相位(ω·延迟 ≤ 1/3)与观测器带宽 */
    s_gain.vel_bw_max = (uint16_t)(((cal->delay_us + 500U) > 333U) ? 1000000UL / (3UL * (cal->delay_us + 500U)) : 1000UL);
    if (s_gain.vel_bw_max > (uint32_t)obs_bw_hz * 6283U / 1000U) s_gain.vel_bw_max = (uint16_t)((uint32_t)obs_bw_hz * 6283U / 1000U);
    C_PosCtrl_Reset();
}

/* 保持两项都是相对扭矩增益，以标定电压为满量程(电压驱动下出力 ∝ 电压)，与电机参数无关：
 *   K_h = P‰·标定电压/°(mV/厘度 = P·cal/100000)；积分每秒增加 I‰·标定电压/°·ex */
static void PosCtrl_HoldGains(void)
{
    s_gain.kh_q16  = (int32_t)(((int64_t)s_gain.cal_mv * s_gain.hold_kp * 65536) / 100000);
    s_gain.kih_q20 = (int32_t)(((int64_t)s_gain.cal_mv * s_gain.hold_ki << 20) / 100000000);   /* 1ms 一拍 */
}


uint8_t C_PosCtrl_SetGains(PosCtrlGains_t *g)
{
    PosCtrlGains_t in = *g;
    int64_t kv_tot, kv_place;

    g->vel_bw = (uint16_t)Fx_ClampRange(g->vel_bw, 10, s_gain.vel_bw_max);
    g->pos_bw = (uint16_t)Fx_ClampRange(g->pos_bw, 1, g->vel_bw / 4);
    g->int_bw = (uint16_t)Fx_ClampRange(g->int_bw, 0, g->pos_bw);
    if (g->load_ratio_x10 > POS_CTRL_LOAD_RATIO_MAX_X10) g->load_ratio_x10 = POS_CTRL_LOAD_RATIO_MAX_X10;

    /* 速度环：对象 1/(ka·s + kv)，闭环极点落在 ω_v：ka·ω_v = kv + Kv。电机自身带宽 kv/ka 已够时 Kv 取 ka·ω_v/4 */
    kv_tot   = ((int64_t)s_gain.ka_nv * g->vel_bw * 65536) / 1000000;      /* ka·ω_v，Q16 */
    kv_place = kv_tot - ((int64_t)s_gain.kv_uv * 65536) / 1000;
    s_gain.kv_q16 = (int32_t)((kv_place > kv_tot / 4) ? kv_place : kv_tot / 4);
    if (g->load_ratio_x10 != 0U)
    {
        /* 固定惯量只收敛外环带宽；电压前馈仍使用原始标定模型。 */
        int64_t damping = s_gain.kv_q16 + s_gain.kv_pos_q16;
        int64_t inertia = (int64_t)s_gain.ka_nv * (10U + g->load_ratio_x10) * 65536;
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

uint8_t C_PosCtrl_Reached(void) { return (uint8_t)(s_ctrl.state == PC_HOLD); }
uint8_t C_PosCtrl_ArrivalQualified(void) { return s_ctrl.qualified; }

void C_PosCtrl_BeginMotion(void)
{
    s_ctrl.cap_ms = 0U;
    s_ctrl.qualified = 0U;
}

/* 起转补偿以积分托载量为基准；只对补偿分量限变化率，主闭环和制动输出不滤波。
 * 最多5ms升/降一个fs，跨过速度/位移判据时逐步交回闭环，反向时也经过零。 */
static int32_t PosCtrl_Stiction(const PosCtrlFeedback_t *fb, int32_t demand, int32_t u)
{
    int32_t target = 0, need;
    int32_t step = (s_gain.fs_mv + STICK_SLEW_MS - 1) / STICK_SLEW_MS;

    if (Fx_Abs32(fb->pos - s_ctrl.anchor) >= s_gain.min_step) { s_ctrl.anchor = fb->pos; s_ctrl.still_ms = 0U; }
    else if (s_ctrl.still_ms < 0xFFFFU) s_ctrl.still_ms++;

    if (demand != 0 && s_ctrl.still_ms >= STICK_MOVE_MS && Fx_Abs32(fb->vel) < s_gain.vel_noise)
    {
        if (s_ctrl.still_ms >= STICK_ESC_MS) s_ctrl.boost = Fx_ClampRange(s_ctrl.boost + s_gain.fs_mv / STICK_ESC_DIV, 0, s_gain.fs_mv);
        need = Fx_MulShift(s_ctrl.integral_q16, 1, 16) + demand * s_gain.fs_mv;
        target = (((demand > 0) ? (u < need) : (u > need)) ? need - u : 0) + demand * s_ctrl.boost;
    }
    else
    {
        s_ctrl.boost -= s_gain.fs_mv / STICK_ESC_DIV;
        if (s_ctrl.boost < 0) s_ctrl.boost = 0;
    }
    s_ctrl.stick += Fx_Clamp32(target - s_ctrl.stick, step);
    return s_ctrl.stick;
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
        return;
    }
    if (s_ctrl.state != PC_SETTLE) return;
    /* 捕获计数：满足加1、不满足减2(要求约2/3的拍满足)。一拍不满足就清零的话，观测速度噪声偶尔越过噪声底
     * 会让它长期进不了保持 */
    if (Fx_Abs32(fb->vel) <= s_gain.vel_noise && Fx_Abs32(e) <= s_gain.w_in)
    {
        if (s_ctrl.cap_ms < CAPTURE_MS) s_ctrl.cap_ms++;
        if (s_ctrl.cap_ms >= CAPTURE_MS)
        {
            s_ctrl.qualified = 1U;
            if (s_ctrl.still_ms >= HOLD_STILL_MS)
            {
                if (s_ctrl.hold_blend_ms < HOLD_BLEND_MS) s_ctrl.hold_blend_ms++;
                else s_ctrl.state = PC_HOLD;
            }
            else if (s_ctrl.hold_blend_ms != 0U) s_ctrl.hold_blend_ms--;
        }
    }
    else
    {
        s_ctrl.cap_ms = (s_ctrl.cap_ms > 2U) ? (uint16_t)(s_ctrl.cap_ms - 2U) : 0U;
        if (s_ctrl.hold_blend_ms != 0U) s_ctrl.hold_blend_ms--;
    }
}

/* 积分加 di(mV Q16)，幅值不超过半个可用电压 */
static void PosCtrl_IntegralAdd(int64_t di, int32_t limit_mv)
{
    int64_t i_lim = (int64_t)limit_mv << 15, next = s_ctrl.integral_q16 + di;

    s_ctrl.integral_q16 = (int32_t)((next > i_lim) ? i_lim : ((next < -i_lim) ? -i_lim : next));
}

int32_t C_PosCtrl_Update(const TrajRef_t *ref, const PosCtrlFeedback_t *fb, int32_t limit_mv, PosCtrlDbg_t *dbg)
{
    int32_t e = ref->pos - fb->pos;
    int32_t u_ff = 0, u_fb, u_i, u_stick, desired, output, demand = 0, i_in;
    int32_t vel_error = ref->vel - fb->vel;
    int32_t vel_filtered = (vel_error + s_ctrl.vel_error_prev) / 2;
    int64_t di;   /* 本拍积分增量，mV Q16 */
    s_ctrl.vel_error_prev = vel_error;

    /* 无扰切换：保持中窗外托着负载的 P 并入积分。新运动从实测位置起步、误差≈0，不并入的话这份力一拍消失，
     * 负载先掉再追 */
    if (!ref->done && s_ctrl.state == PC_HOLD) PosCtrl_IntegralAdd((int64_t)s_ctrl.hold_p << 16, limit_mv);
    PosCtrl_State(ref, fb, e);

    switch (s_ctrl.state)
    {
    case PC_MOVE:
    {
        int32_t v_pos = Fx_MulShift(s_gain.kp_q8, Fx_Deadband(e, s_gain.noise), 8);
        int32_t v_cmd = ref->vel + v_pos;
        int32_t frac = Fx_Clamp32(ref->vel * s_gain.fric_inv_q16, 65536);   /* 平滑符号，Q16 */

        u_ff = C_MotorModel_Feedforward(ref->vel + Fx_MulShift(ref->acc, s_gain.delay_q16, 16), ref->acc)
             + Fx_MulShift(s_gain.fc_mv, frac, 16);
        i_in = v_cmd - fb->vel;
        u_fb = Fx_MulShift(s_gain.kv_q16, v_pos + vel_filtered, 16)
             + (int32_t)(((int64_t)s_gain.kv_pos_q16 * v_pos) / 65536);
        di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
        /* 参考在起步/匀速时按速度指令方向要求动；参考减速段(含反转前的过零)轴本来就该慢下来，
         * 低速不是卡住，顶起会和减速前馈对着干；已超前参考一个到位窗以上也不顶 */
        if (Fx_Abs32(v_cmd) > s_gain.vel_noise && (int64_t)ref->acc * ref->vel >= 0
            && (int64_t)Fx_Sign32(v_cmd) * e >= -s_gain.w_in) demand = Fx_Sign32(v_cmd);
        break;
    }
    case PC_SETTLE:
    {
        /* 收尾：对最小可靠位移以外的误差做 PI，不顶起(积分自己会把轴推出静摩擦)。速度项用半个噪声底的死区：
         * 静止时只剩噪声、不积(否则积分随机游走)；轴真在挪动时照常阻尼、回收积分 */
        int32_t v_pos = Fx_MulShift(s_gain.kp_q8, Fx_Deadband(e, s_gain.min_step), 8);

        i_in = v_pos - Fx_Deadband(fb->vel, s_gain.vel_noise / 2);
        u_fb = Fx_MulShift(s_gain.kv_q16, i_in, 16)
             + (int32_t)(((int64_t)s_gain.kv_pos_q16 * v_pos) / 65536);
        di = ((int64_t)s_gain.ki_q20 * i_in) >> 4;
        if (s_ctrl.hold_blend_ms != 0U)
        {
            int32_t ex = Fx_Deadband(e, s_gain.w_in);
            int32_t hold_fb = Fx_MulShift(s_gain.kh_q16, ex, 16);
            int32_t integral = Fx_MulShift(s_ctrl.integral_q16, 1, 16);
            int64_t hold_di = ((int64_t)s_gain.kih_q20 * ex) >> 4;

            /* 交接终点也包含HOLD的小电压短路制动规则，避免最后一拍再跳一次。 */
            if (Fx_Abs32(hold_fb + integral) < s_gain.fc_mv / 2) hold_fb = -integral;
            u_fb += (int32_t)(((int64_t)hold_fb - u_fb) * s_ctrl.hold_blend_ms / HOLD_BLEND_MS);
            di += (hold_di - di) * s_ctrl.hold_blend_ms / HOLD_BLEND_MS;
        }
        break;
    }
    default:   /* PC_HOLD */
    {
        /* 保持：只对超出到位窗的量 ex 做 PI。窗内不出回正力，只输出积分(到位时继承的托举力)；
         * 窗外 P = K_h·ex(从窗口边沿连续长出来的弹簧)，积分每秒加 K_ih·ex。不另加阻尼：
         * 电压驱动下反电势本身就是阻尼，额外的速度项只会在出窗时变成一下反推 */
        int32_t ex = Fx_Deadband(e, s_gain.w_in);

        s_ctrl.hold_p = Fx_MulShift(s_gain.kh_q16, ex, 16);
        u_fb = s_ctrl.hold_p;
        di = ((int64_t)s_gain.kih_q20 * ex) >> 4;
        break;
    }
    }

    u_i = Fx_MulShift(s_ctrl.integral_q16, 1, 16);
    u_stick = PosCtrl_Stiction(fb, demand, u_ff + u_fb + u_i);
    desired = u_ff + u_fb + u_i + u_stick;
    if (s_ctrl.state == PC_HOLD && Fx_Abs32(desired) < s_gain.fc_mv / 2) desired = 0;   /* 不值得驱动就短路制动 */
    output = Fx_Clamp32(desired, limit_mv);

    /* 积分：无效采样时冻结；输出限幅时只允许往回退 */
    if (fb->valid && di != 0 && !((di > 0 && desired > output) || (di < 0 && desired < output))) PosCtrl_IntegralAdd(di, limit_mv);

    if (dbg)
    {
        dbg->e_pos = e;
        dbg->u_ff = (int16_t)u_ff; dbg->u_fb = (int16_t)u_fb; dbg->u_i = (int16_t)u_i;
        dbg->u_stick = (int16_t)u_stick; dbg->u_out = (int16_t)output;
        dbg->sat = (uint8_t)(desired != output); dbg->state = s_ctrl.state;
    }
    return output;
}
