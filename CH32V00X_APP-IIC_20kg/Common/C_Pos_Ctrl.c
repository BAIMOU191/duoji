/* C_Pos_Ctrl.c 舵机位置闭环控制器，纯算法，无硬件依赖 */

#include "C_Pos_Ctrl.h"
#include "A_Parameter.h"

#define POSCTRL_Q15        15U
#define POSCTRL_Q16        16U
#define POSCTRL_Q20        20U
typedef char guard_brake_gate_numerator_fits_int32[
    (CTRL_FRIC_MOVE_CDPS > 0 && CTRL_FRIC_MOVE_CDPS <= 65536) ? 1 : -1];

static struct {
    /* 速度PI积分项，Q16 PWM。跨指令保持，只在 Reset 时清零 */
    int32_t vel_i_q16;

    /* 到位保持状态机 */
    uint8_t  state;
    uint16_t hold_cnt;
    uint8_t  quiet;          /* 真正静止的锁存，进入与退出使用不同位置窗口 */
    uint8_t  sliding;        /* 1=轴已挣脱静摩擦正在滑动，摩擦前馈改用动摩擦 */
    int16_t  kick;           /* 顶起值在标定起转值之上的自适应抬升量，PWM计数 */
    uint16_t stop_cnt;       /* 实测速度连续低于噪声底的拍数，用于判"真的粘住了" */
    uint8_t  held;           /* 1=本段轨迹走完后已进入过保持，此后被外力推开才启用保持弹簧 */

    /* 本拍允许的输出上限，由限功率逻辑下发 */
    int16_t  out_limit;

    /* 由编译期参数换算的内部系数，Init 时算一次 */
    int32_t  fric_dir_k;     /* Q15/(厘度/秒)：摩擦方向在零速附近连续建立 */
} s_ctl;

/* 对称限幅 */
static int32_t PosCtrl_Clamp32(int32_t v, int32_t lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

static int32_t PosCtrl_Abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* 位置差折算：圆周坐标折到最短路径(否则过0点时位置环反向满舵)；
 * 线性坐标禁止折算(捷径要穿过测不到角度的死区) */
static int32_t PosCtrl_WrapDiff(int32_t d)
{
#if CFG_WRAP_RANGE_CDEG > 0
    if (d >  (CFG_WRAP_RANGE_CDEG / 2)) d -= CFG_WRAP_RANGE_CDEG;
    if (d < -(CFG_WRAP_RANGE_CDEG / 2)) d += CFG_WRAP_RANGE_CDEG;
#endif
    return d;
}

/* 清空积分与到位状态，参数不动 */
void C_PosCtrl_Reset(void)
{
    s_ctl.vel_i_q16 = 0;
    s_ctl.state    = (uint8_t)POSCTRL_RUN;
    s_ctl.hold_cnt = 0;
    s_ctl.quiet    = 0;
    s_ctl.sliding  = 0;   /* 复位后按"粘住"起步，第一次修正才有起转力 */
    s_ctl.kick     = 0;
    s_ctl.stop_cnt = 0;
    s_ctl.held     = 0;
}

/* 按编译期参数换算内部系数并复位；改 CTRL_WCMD_MIN_CDPS 后须重新调用 */
void C_PosCtrl_Init(void)
{
    int32_t vs = CTRL_WCMD_MIN_CDPS;
    if (vs < 1) vs = 1;
    s_ctl.fric_dir_k = (1 << POSCTRL_Q15) / vs;

    s_ctl.out_limit = CTRL_PWM_LIMIT;

    C_PosCtrl_Reset();
}

void C_PosCtrl_Set_Output_Limit(int16_t limit)
{
    if (limit <= 0 || limit > CTRL_PWM_LIMIT)
    {
        limit = CTRL_PWM_LIMIT;
    }
    s_ctl.out_limit = limit;
}

uint8_t C_PosCtrl_In_Position(void)
{
    return (uint8_t)(s_ctl.state == (uint8_t)POSCTRL_HOLD);
}

/* 摩擦前馈，返回带符号PWM。
 * 方向取速度指令 w_cmd(取误差或实测速度的符号会在零速附近来回翻、形成极限环)，
 * 从0到 CTRL_WCMD_MIN_CDPS 线性建立。幅值按实测速度二值锁存：
 *   |v| >= CTRL_FRIC_MOVE_CDPS                   -> 已滑动，动摩擦直接加上
 *   |v| <= CTRL_VEL_DB_CDPS 连续 CTRL_STUCK_MS 拍  -> 粘住，只把总输出补足到起转值
 * 粘住且要求动却没动时起转值逐拍抬升(上限 CTRL_FRIC_KICK_MAX)，补偿标定偏小。
 * CAL_BREAKAWAY_PWM 未标定时退化为纯动摩擦。 */
static int32_t PosCtrl_Friction(int32_t w_cmd, int32_t av_obs, int32_t u_rest)
{
    int32_t dir_q15, want, have;

    /* 正反向共用一组摩擦值：两向差异在标定重复性以内 */
    dir_q15 = w_cmd * s_ctl.fric_dir_k;
    dir_q15 = PosCtrl_Clamp32(dir_q15, 1 << POSCTRL_Q15);

    if (av_obs >= CTRL_FRIC_MOVE_CDPS)
    {
        s_ctl.sliding  = 1U;
        s_ctl.stop_cnt = 0U;
    }
    else if (av_obs <= CTRL_VEL_DB_CDPS)
    {
        if (s_ctl.stop_cnt < CTRL_STUCK_MS) s_ctl.stop_cnt++;
        else s_ctl.sliding = 0U;
    }
    else s_ctl.stop_cnt = 0U;

    if (s_ctl.sliding)  /* 已滑动：动摩擦是持续阻力，直接加上 */
    {
        s_ctl.kick = 0;
        return (int32_t)(((int64_t)dir_q15 * CTRL_FRIC_DYN) >> POSCTRL_Q15);
    }

    /* 要求动却没动：逐拍抬高顶起值，直到轴真的挣脱。指令归零就清零。 */
    if (PosCtrl_Abs32(w_cmd) > CTRL_WCMD_MIN_CDPS)
    {
        if (s_ctl.kick < CTRL_FRIC_KICK_MAX)
            s_ctl.kick = (int16_t)(s_ctl.kick + CTRL_FRIC_KICK_STEP);
    }
    else s_ctl.kick = 0;

    /* 粘住：把总输出顶到起转值，方向由 w_cmd 给，已经够大就不再补 */
    want = (int32_t)(((int64_t)dir_q15 * (CTRL_FRIC_STATIC + s_ctl.kick)) >> POSCTRL_Q15);
    have = u_rest;
    if (want > 0) return (have < want) ? (want - have) : 0;
    if (want < 0) return (have > want) ? (want - have) : 0;
    return 0;
}

/* 到位保持状态机：进入窗小于退出窗形成滞环；退出判据用未加死区的原始误差 */
static void PosCtrl_Hold_Update(int32_t e_pos_raw, int32_t av_obs, uint8_t traj_done)
{
    int32_t ae = PosCtrl_Abs32(e_pos_raw);

    if (s_ctl.state == (uint8_t)POSCTRL_RUN)
    {
        if (traj_done && ae <= CTRL_IN_WIN_CDEG &&
            av_obs <= CTRL_IN_VEL_CDPS)
        {
            if (s_ctl.hold_cnt < 0xFFFFU) s_ctl.hold_cnt++;
            if (s_ctl.hold_cnt >= CTRL_IN_HOLD_MS)
            {
                s_ctl.state = (uint8_t)POSCTRL_HOLD;
            }
        }
        else
        {
            s_ctl.hold_cnt = 0;
        }
    }
    else
    {
        if (!traj_done || ae >= CTRL_OUT_WIN_CDEG)
        {
            s_ctl.state    = (uint8_t)POSCTRL_RUN;
            s_ctl.hold_cnt = 0;
        }
    }
}

/* 推进一拍，返回下发给H桥的PWM；无除法、无开方 */
int16_t C_PosCtrl_Update(const TrajRef_t *ref, const SpeedObsOut_t *obs,
                         uint8_t traj_done, PosCtrlDbg_t *dbg)
{
    int32_t pos_lead, e_pos, e_pos_ctl, e_vel;
    int32_t w_corr, w_ff, w_cmd;
    int32_t av_obs, dead, e_measured, ae_measured;
    uint8_t decelerating;
    uint8_t holding_load;
    int32_t u_fric, u_vel, u_acc, u_fb, u_i, u_spring;
    int32_t u_unsat, u_sat, u_out;
    int32_t lim = s_ctl.out_limit;

    if (ref == 0 || obs == 0) return 0;

    av_obs = PosCtrl_Abs32(obs->vel);
    /* 用扰动估计识别持续负载(只做判断不前馈)：带载时保留闭环和积分，防止静音后滑落 */
    holding_load = (uint8_t)(traj_done && CTRL_NUDGE_PWM > 0
        && PosCtrl_Abs32(obs->load) >= CTRL_FRIC_STATIC / 2);

    /* 1. 位置误差：对 obs-ref 相对位移做预测，匀速吻合时附加项为0，超速时提前加制动 */
    pos_lead = obs->pos
             + (int32_t)(((int64_t)(obs->vel - ref->vel)
                          * CTRL_POS_LEAD_Q16) >> POSCTRL_Q16);
    e_pos = PosCtrl_WrapDiff(ref->pos - pos_lead);

    /* 2. 到位保持：先用原始误差更新状态机，再加死区 */
    /* 到位判断用实际残差；预测项仅供运动控制，不能让速度噪声改变静音窗口。 */
    e_measured = PosCtrl_WrapDiff(ref->pos - obs->pos);
    ae_measured = PosCtrl_Abs32(e_measured);
    PosCtrl_Hold_Update(e_measured, av_obs, traj_done);
    if (!traj_done) s_ctl.held = 0U;
    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD) s_ctl.held = 1U;
    if (!traj_done || holding_load
        || ae_measured >= CTRL_QUIET_EXIT_CDEG
        || av_obs > CTRL_IN_VEL_CDPS)
        s_ctl.quiet = 0U;
    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD
             && ae_measured <= CTRL_HOLD_DEADBAND
             && av_obs <= CTRL_VEL_DB_CDPS)
        s_ctl.quiet = 1U;

    /* 位置死区任何状态都生效：比最小可靠位移还小的残差去踹静摩擦必然窜过头，形成极限环。
     * 修正阶段仍用小死区，让轴能走进 CTRL_HOLD_DEADBAND 捕获窗。 */
    dead = CTRL_POS_DEADBAND;
    e_pos_ctl = e_pos;
    if (e_pos_ctl >  dead) e_pos_ctl -= dead;
    else if (e_pos_ctl < -dead) e_pos_ctl += dead;
    else e_pos_ctl = 0;

    /* 3. 位置P -> 速度修正，单独限幅，跟踪误差再大也不把速度指令推满 */
    w_corr = (int32_t)(((int64_t)CTRL_KP_POS_Q8 * e_pos_ctl) >> 8);
    w_corr = PosCtrl_Clamp32(w_corr, CTRL_WCORR_MAX);

    /* 4. 预览：用参考加速度把参考速度提前 Tact 补执行延迟，只作用于前馈 */
    w_ff  = ref->vel
          + (int32_t)(((int64_t)ref->acc * CTRL_PREVIEW_Q16) >> POSCTRL_Q16);
    w_cmd = PosCtrl_Clamp32(w_ff + w_corr, CTRL_VMAX_CDPS);

    /* 5. 速度误差：观测器速度无滞后，直接与参考比较 */
    e_vel = (ref->vel + w_corr) - obs->vel;

    /* 6. 减速状态：减速段及轨迹结束后仍有残余速度时用独立增益 */
    decelerating = 0U;
    if (av_obs > CTRL_WCMD_MIN_CDPS &&
        (((int64_t)ref->acc * obs->vel < 0) || traj_done))
    {
        decelerating = 1U;
    }

    /* 7. 前馈组。摩擦前馈放到第9节(粘住时需先知道其余各项之和)，且不按HOLD关闭：末端挪动正需要它 */
    u_vel  = (int32_t)(((int64_t)CTRL_KVFF_Q16 * w_cmd) >> POSCTRL_Q16);

    /* 减速段 acc 与 vel 反号即主动制动；慢衰减H桥制动侧增益较弱，单独用 Ka_brake。判据用参考量，无噪声 */
    u_acc  = (int32_t)(((int64_t)(((int64_t)ref->acc * ref->vel < 0)
                                  ? CTRL_KA_BRAKE_Q20
                                  : CTRL_KA_Q20)
                        * ref->acc) >> POSCTRL_Q20);

    /* 制动前馈按行进方向上的实测剩余速度放行：轴已先停时，残余制动前馈会把轴往回推(到位回摆)。
     * 尺度取 CTRL_FRIC_MOVE_CDPS，与判"轴在动"是同一门限。 */
    if ((int64_t)ref->acc * ref->vel < 0)
    {
        int32_t v_along = (ref->vel >= 0) ? obs->vel : -obs->vel;
        int32_t g_q15;

        if (v_along <= 0)                       g_q15 = 0;
        else if (v_along >= CTRL_FRIC_MOVE_CDPS) g_q15 = 1 << POSCTRL_Q15;
        else g_q15 = (v_along * (1L << POSCTRL_Q15)) / CTRL_FRIC_MOVE_CDPS;

        u_acc = (int32_t)(((int64_t)u_acc * g_q15) >> POSCTRL_Q15);
    }

    /* 8. 速度PI的P项：减速段与残余运动用独立增益 */
    /* 保持态给 e_vel 加噪声底死区(轴停住后 obs.vel 只剩量化噪声)；只在保持态生效，积分仍用原始 e_vel */
    {
        int32_t e_vel_fb = e_vel;   /* 供P反馈用的速度误差 */

        if (s_ctl.state == (uint8_t)POSCTRL_HOLD)
        {
            if (e_vel_fb >  CTRL_VEL_DB_CDPS)      e_vel_fb -= CTRL_VEL_DB_CDPS;
            else if (e_vel_fb < -CTRL_VEL_DB_CDPS) e_vel_fb += CTRL_VEL_DB_CDPS;
            else                                      e_vel_fb  = 0;
        }

        u_fb = (int32_t)(((int64_t)(decelerating ? CTRL_KP_VEL_BRAKE_Q16
                                                : CTRL_KP_VEL_Q16) * e_vel_fb)
                         >> POSCTRL_Q16);
    }

    /* 8.5 保持弹簧：到位后被外力推开时按目标误差直接回推。级联环为压传感器噪声调得很软，
     * 走位和到位过程不受影响(只在进入过保持之后生效)，静止时仍由静音锁存保证不出力。 */
    u_spring = 0;
    if (CTRL_HOLD_SPRING_Q8 > 0 && s_ctl.held && !s_ctl.quiet)
    {
        int32_t e_sp = e_measured;
        if (e_sp >  CTRL_HOLD_SPRING_DB)      e_sp -= CTRL_HOLD_SPRING_DB;
        else if (e_sp < -CTRL_HOLD_SPRING_DB) e_sp += CTRL_HOLD_SPRING_DB;
        else                                   e_sp  = 0;
        u_spring = PosCtrl_Clamp32((int32_t)(((int64_t)CTRL_HOLD_SPRING_Q8 * e_sp) >> 8),
                                   CTRL_HOLD_SPRING_MAX);
    }

    /* 9. 摩擦前馈与合成限幅 */
    u_i     = s_ctl.vel_i_q16 >> POSCTRL_Q16;
    u_fric  = PosCtrl_Friction(w_cmd, av_obs, u_vel + u_acc + u_fb + u_i + u_spring);
    u_unsat = u_fric + u_vel + u_acc + u_fb + u_i + u_spring;
    u_sat   = PosCtrl_Clamp32(u_unsat, lim);

    /* 10. 速度PI积分项，全系统唯一的积分器(观测器 d 不外送)。三重保护：
     *   死区泄放 —— 位置误差进死区只泄放不积分，消除极限环；
     *   瞬态冻结 —— |acc_ref| 大时不积分，动态模型残差不是负载；
     *   条件积分 —— 饱和时只冻结加深饱和的方向。不用回退式：本结构饱和主体是前馈，
     *               回退会把积分器钉在反向钳位上。
     * 不随新指令清零，换姿态后仍带着负载补偿。 */
    if (s_ctl.quiet)
    {
        s_ctl.vel_i_q16 = 0; /* 已选择短路制动，不能保留会在下一次释放的历史力矩 */
    }
    else if (PosCtrl_Abs32(e_pos) <= dead)
    {
        if (!holding_load) s_ctl.vel_i_q16 -= s_ctl.vel_i_q16 >> 12;
    }
    else if (u_spring != 0)
    {
        /* 被推开期间冻结积分：推力由弹簧顶，松手后积分仍是推之前的负载补偿，
         * 不会把轴弹过原位，也不会剩一截积分在死区里带电顶着 */
    }
    else if (PosCtrl_Abs32(ref->acc) <= CTRL_INTEGRAL_ACC_GATE)
    {
        int32_t di = CTRL_KI_VEL_Q16 * e_vel;   /* 本拍积分增量，Q16 PWM */

        /* 轨迹结束、轴已低速且仍在静音窗外：加快建立缺少的静摩擦力 */
        if (traj_done && av_obs <= CTRL_VEL_DB_CDPS
            && PosCtrl_Abs32(e_pos) > CTRL_HOLD_DEADBAND)
            di *= 4;

        /* 饱和且增量还往饱和侧推 -> 冻结 */
        if (!((u_unsat > u_sat && di > 0) || (u_unsat < u_sat && di < 0)))
        {
            s_ctl.vel_i_q16 += di;
            s_ctl.vel_i_q16 = PosCtrl_Clamp32(s_ctl.vel_i_q16,
                                              CTRL_INTEGRAL_MAX << POSCTRL_Q16);
        }
    }

    /* 10.5 保持态输出：无负载、进捕获窗且速度降到噪声底时锁存短路制动，误差超出 CTRL_IN_WIN_CDEG、
     * 明显运动或新轨迹时退出。静止轴净输出不足 CTRL_NUDGE_PWM 时先不驱动，让积分建立；带负载保留保持力。 */
    u_out = u_sat;
    if (s_ctl.quiet)
    {
        u_out = 0;
    }
    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD && !holding_load)
    {
        if (PosCtrl_Abs32(e_pos) <= dead && av_obs <= CTRL_VEL_DB_CDPS)
        {
            u_out = 0;
        }
        else if (av_obs <= CTRL_VEL_DB_CDPS && PosCtrl_Abs32(u_sat) < CTRL_NUDGE_PWM && u_spring == 0)
        {
            u_out = 0;              /* 还不够起转，先积累 */
        }
    }

    /* 11. 遥测 */
    if (dbg != 0)
    {
        dbg->e_pos  = e_pos;
        dbg->e_vel  = e_vel;
        dbg->w_corr = w_corr;
        dbg->w_cmd  = w_cmd;
        dbg->u_fric = (int16_t)u_fric;
        dbg->u_vel  = (int16_t)u_vel;
        dbg->u_acc  = (int16_t)u_acc;
        dbg->u_fb   = (int16_t)u_fb;
        dbg->u_i    = (int16_t)u_i;
        dbg->u_out  = (int16_t)u_out;
        dbg->state  = s_ctl.state;
        dbg->sat    = (uint8_t)((u_unsat != u_sat) ? 1U : 0U);
        dbg->braking = decelerating;
    }

    return (int16_t)(u_out * CTRL_MOTOR_SIGN);
}
