/* C_Pos_Ctrl.c 舵机位置闭环控制器，纯算法，无硬件依赖 */

#include "C_Pos_Ctrl.h"
#include "A_Parameter.h"

#define POSCTRL_Q15        15U
#define POSCTRL_Q16        16U
#define POSCTRL_Q20        20U

static struct {
    /* 速度PI积分项，Q16 PWM。跨指令保持，只在 Reset 时清零 */
    int32_t vel_i_q16;

    /* 到位保持状态机 */
    uint8_t  state;
    uint16_t hold_cnt;

    /* 本拍允许的输出上限，由限功率逻辑下发 */
    int16_t  out_limit;

    /* 由编译期参数换算的内部系数，Init 时算一次 */
    int32_t  fric_dir_k;     /* Q15/(厘度/秒)：摩擦方向在零速附近连续建立 */
} s_ctl;

/*
 * @fn      PosCtrl_Clamp32
 * @brief   对称限幅
 */
static int32_t PosCtrl_Clamp32(int32_t v, int32_t lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/*
 * @fn      PosCtrl_Abs32
 */
static int32_t PosCtrl_Abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/*
 * @fn      PosCtrl_WrapDiff
 * @brief   把位置差折算到最短路径(-量程/2, 量程/2]
 *
 * **必须有**。规划器的 ref.pos 是不回绕的行程坐标，而编码器/观测器的 obs.pos
 * 被折在 [0, 36000) 里。目标定在 0° 时，实际只要越过 0 一点点，obs.pos 就跳到
 * 35999，两者直接相减会得到 -35999 而不是 +1，位置环立刻朝反方向满舵，
 * 舵机就一直倒转停不下来。实测日志里失控后转速恒定在 -5000 厘度/s，
 * 正好等于 wcorr_max，就是这个原因。
 */
static int32_t PosCtrl_WrapDiff(int32_t d)
{
    if (d >  (CDEG_RANGE / 2)) d -= CDEG_RANGE;
    if (d < -(CDEG_RANGE / 2)) d += CDEG_RANGE;
    return d;
}

/*
 * @fn      C_PosCtrl_Reset
 * @brief   清空积分、延迟缓冲与到位状态；参数不动
 */
void C_PosCtrl_Reset(void)
{
    s_ctl.vel_i_q16 = 0;
    s_ctl.state    = (uint8_t)POSCTRL_RUN;
    s_ctl.hold_cnt = 0;
}

/*
 * @fn      C_PosCtrl_Init
 * @brief   按编译期参数换算内部系数并复位状态
 *
 * 唯一的初始化除法在这里：把摩擦前馈完整建立的速度阈值换成Q15斜率，
 * 之后每拍只有一次乘法。改了ctrl_wcmd_min_cdps必须重新调用本函数。
 */
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

/*
 * @fn      PosCtrl_Friction
 * @brief   连续动摩擦前馈
 * @param   w_cmd   速度指令(厘度/秒)，决定补偿方向
 * @return  摩擦前馈 PWM(带符号)
 *
 * 三个要点：
 *   1) 方向取**指令**方向。若取速度误差或实测速度的符号，零速附近符号会来回翻，
 *      产生 ±1200 计数量级的跳变和极限环——这是摩擦补偿最常见的翻车方式。
 *   2) 只补偿速度直线拟合出的动摩擦截距，不再引入需额外标定的起转PWM。
 *   3) 从w_cmd=0到阈值线性建立，避免阈值处突然跳入整块摩擦PWM。
 */
static int32_t PosCtrl_Friction(int32_t w_cmd)
{
    int32_t dir_q15;

    /* 正反向不再分开标定：实测两个方向的动摩擦差 1%(87/88)、起转差 6%
     * (188/200)，都在标定重复性(正向起转散布 22 计数)以内。用一组值换来
     * 少两项标定、少两个分支，代价小于测量噪声本身。 */
    /* 原实现到 |w_cmd|=阈值时从0直接跳到完整起转PWM，是一个人为力矩阶跃。
     * 现在用饱和直线建立方向：w_cmd=0严格为0，到阈值时平滑达到完整补偿。 */
    dir_q15 = w_cmd * s_ctl.fric_dir_k;
    dir_q15 = PosCtrl_Clamp32(dir_q15, 1 << POSCTRL_Q15);
    return (int32_t)(((int64_t)dir_q15 * CTRL_FRIC_DYN) >> POSCTRL_Q15);
}

/*
 * @fn      PosCtrl_Hold_Update
 * @brief   到位保持状态机(施密特窗口)
 * @param   e_pos_raw 未加死区的位置误差
 * @param   av_obs    实测速度绝对值
 * @param   traj_done 轨迹是否走完
 *
 * 进入窗口比退出窗口小，中间是滞环，避免在最小可靠位移(0.1~0.2°)附近来回猎振。
 * 退出判据必须用**未加死区**的原始误差，否则死区会把误差吃掉，永远退不出保持态。
 */
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

/*
 * @fn      C_PosCtrl_Update
 * @brief   推进一拍，返回下发给H桥的PWM
 *
 * 计算量：约14次乘法(其中4次32x32->64)、无除法、无开方。
 */
int16_t C_PosCtrl_Update(const TrajRef_t *ref, const SpeedObsOut_t *obs,
                         uint8_t traj_done, PosCtrlDbg_t *dbg)
{
    int32_t pos_lead, e_pos, e_pos_ctl, e_vel;
    int32_t w_corr, w_ff, w_cmd;
    int32_t av_obs, dead;
    uint8_t decelerating;
    int32_t u_fric, u_vel, u_acc, u_fb, u_i;
    int32_t u_unsat, u_sat;
    int32_t lim = s_ctl.out_limit;

    if (ref == 0 || obs == 0) return 0;

    av_obs = PosCtrl_Abs32(obs->vel);

    /* ---- 1. 位置误差：用相对参考速度做预测阻尼 ----
     * 只预测实测位置会在匀速跟踪时人为制造 vel*T 的位置误差，T 一旦为了
     * 提前制动而调大，就会让整段轨迹故意落后。这里预测的是 obs-ref 的相对
     * 位移：匀速且速度吻合时附加项为0；实测速度高于参考(尤其减速段)时，
     * 提前看到即将发生的越位并增加制动。它等价于只对跟踪误差加相位超前，
     * 不改变理想参考轨迹本身。 */
    pos_lead = obs->pos
             + (int32_t)(((int64_t)(obs->vel - ref->vel)
                          * CTRL_POS_LEAD_Q16) >> POSCTRL_Q16);
    e_pos = PosCtrl_WrapDiff(ref->pos - pos_lead);

    /* ---- 2. 到位保持：先用原始误差更新状态机，再决定要不要加死区 ---- */
    PosCtrl_Hold_Update(e_pos, av_obs, traj_done);

    /* ---- 位置死区：**任何状态下都生效**，这是防极限环的关键 ----
     * 机构最小可靠位移实测 0.1~0.2°。为一个 0.13° 的残差去踹 1270 计数的
     * 静摩擦，物理上必然窜过头，然后反向再窜，形成极限环；而极限环又让
     * |vel_obs| 一直在 ±10°/s 摆，到位判据永远不满足，于是死区永远进不来
     * ——这是个自锁。所以死区必须无条件生效，不能只在 HOLD 里加。
     *
     * 死区与 ctrl_wcmd_min_cdps 共同决定"多大的误差才值得去修"：
     *     可分辨误差 = 死区 + wcmd_min/Kp
     * 当前值 = 10 + 50/20 ≈ 13 厘度 = 0.13°，正好落在机构分辨率上。
     * 运动过程中 w_cmd 由前馈主导，死区只削掉那一点残差，不影响跟踪。 */
    dead = (s_ctl.state == (uint8_t)POSCTRL_HOLD)
         ? CTRL_HOLD_DEADBAND      /* 保持态可以再放宽一点 */
         : CTRL_POS_DEADBAND;
    e_pos_ctl = e_pos;
    if (e_pos_ctl >  dead) e_pos_ctl -= dead;
    else if (e_pos_ctl < -dead) e_pos_ctl += dead;
    else e_pos_ctl = 0;

    /* ---- 3. 位置P -> 速度修正，单独限幅 ----
     * 限幅是"不过冲"的关键之一：跟踪误差再大也不能把速度指令一把推满。 */
    w_corr = (int32_t)(((int64_t)CTRL_KP_POS_Q8 * e_pos_ctl) >> 8);
    w_corr = PosCtrl_Clamp32(w_corr, CTRL_WCORR_MAX);

    /* ---- 4. 预览：用解析加速度把参考速度提前 Tact，补执行延迟 ----
     * 只作用在前馈上，不进反馈环，所以不会影响稳定性。 */
    w_ff  = ref->vel
          + (int32_t)(((int64_t)ref->acc * CTRL_PREVIEW_Q16) >> POSCTRL_Q16);
    w_cmd = PosCtrl_Clamp32(w_ff + w_corr, CTRL_VMAX_CDPS);

    /* ---- 5. 速度误差 ----
     * 模型观测器的速度估计**没有滞后**(它靠模型补掉了微分器的滞后)，
     * 所以参考速度直接拿来比即可。旧版这里有一个 8 深的环形缓冲，用来
     * 对齐 SG 微分器的 (N-1)/2 拍滞后；换成观测器之后 delay 已经配成 0，
     * 那段代码退化成"写进去再原样读出来"，只剩 32 字节 RAM 和两次取模的
     * 开销，已删除。 */
    e_vel = (ref->vel + w_corr) - obs->vel;

    /* ---- 6. 减速状态 ----
     * 减速段以及轨迹结束后仍有残余速度时，使用独立的减速反馈增益。 */
    decelerating = 0U;
    if (av_obs > CTRL_WCMD_MIN_CDPS &&
        (((int64_t)ref->acc * obs->vel < 0) || traj_done))
    {
        decelerating = 1U;
    }

    /* ---- 7. 前馈组 ---- */
    u_fric = (s_ctl.state == (uint8_t)POSCTRL_HOLD)
           ? 0
           : PosCtrl_Friction(w_cmd);
    u_vel  = (int32_t)(((int64_t)CTRL_KVFF_Q16 * w_cmd) >> POSCTRL_Q16);

    /* 减速段 acc 与 vel 反号，本项自然变成反向驱动，也就是主动制动。
     * 但制动方向的对象增益比驱动方向弱(慢衰减H桥实测约 1/2)，用同一个 Ka
     * 会让制动前馈系统性给少，缺的那部分只能等反馈慢慢补——过去是靠把
     * preview 和 kv_brake 一起往上调硬凑回来的，两个不相干的旋钮补同一个
     * 未建模的不对称。这里直接给制动方向一个自己的 Ka，物理含义单一。
     * 判据用**参考**量而不是观测量：前馈路径必须无噪声、无滞后。 */
    u_acc  = (int32_t)(((int64_t)(((int64_t)ref->acc * ref->vel < 0)
                                  ? CTRL_KA_BRAKE_Q20
                                  : CTRL_KA_Q20)
                        * ref->acc) >> POSCTRL_Q20);

    /* ---- 8. 速度PI的P项 ----
     * 减速段和轨迹结束后的残余运动使用独立增益。实测慢衰减制动在小制动力区
     * 明显比加速侧“软”，提高这一段的反馈可以更早消掉超速，又不会把高增益
     * 带来的编码器量化噪声扩散到整段匀速运动。 */
    u_fb = (int32_t)(((int64_t)(decelerating ? CTRL_KP_VEL_BRAKE_Q16
                                            : CTRL_KP_VEL_Q16) * e_vel)
                     >> POSCTRL_Q16);

    /* ---- 9. 合成与限幅 ---- */
    u_i     = s_ctl.vel_i_q16 >> POSCTRL_Q16;
    u_unsat = u_fric + u_vel + u_acc + u_fb + u_i;
    u_sat   = PosCtrl_Clamp32(u_unsat, lim);

    /* ---- 10. 速度PI积分项 ----
     * 纯P在600计数负载下留下约76厘度(0.76°)的稳态位置误差，必须有积分项。
     * 本项就是那个积分器，**全系统只允许有它一个**：观测器的 d 状态估计的
     * 是同一个物理量，两者同时前馈到PWM会互相打架，所以 d 只在观测器内部
     * 参与预测，绝不外送(见 C_Speed_Observer.h 第3点)。
     *
     * 三重保护，缺任何一个都会出问题：
     *
     *   死区泄放 —— 位置误差进入死区后只泄放不积分。这是**消灭极限环的关键**：
     *               不加它，静摩擦一变大(实测按1.5倍)就出现7~22厘度的持续
     *               摆动；加了以后同样条件下峰峰恒为0。死区外照常积分，
     *               所以不影响运动中的抗扰能力。
     *   瞬态冻结 —— |acc_ref|大时误差主要来自动态模型残差而不是负载，学进去
     *               就是错的，而且正是这一项会在终点变成过冲。
     *   抗饱和回退 —— 输出被限幅时把超出的部分从积分器里退掉。比单纯"冻结"
     *               收敛快，且不会在长时间饱和后留下一大坨积分量。
     *
     * 钳位到物理上可能的保持力矩范围；**不随新指令清零**，这样换姿态后
     * 下一条指令一开始就带着正确的负载补偿。 */
    if (PosCtrl_Abs32(e_pos) < dead)
    {
        s_ctl.vel_i_q16 -= s_ctl.vel_i_q16 >> 12;    /* 慢泄放，时间常数约4秒 */
    }
    else if (PosCtrl_Abs32(ref->acc) <= CTRL_INTEGRAL_ACC_GATE)
    {
        s_ctl.vel_i_q16 += CTRL_KI_VEL_Q16 * e_vel;
        s_ctl.vel_i_q16 = PosCtrl_Clamp32(s_ctl.vel_i_q16,
                                          CTRL_INTEGRAL_MAX << POSCTRL_Q16);
    }

    /* 回退**必须和上面的积分受同一个加速度闸门约束**，否则是一个单向棘轮：
     * 加速段 u_acc 本身就有 1800 计数量级，叠上 u_vel 后 u_unsat 远超限幅，
     * 回退把这一整块超出量记到积分器头上——可积分那一路却被闸门冻着，涨不
     * 回来。2026-09-03 实测：一个打印间隔内 u_i 从 -30 掉到 -787 直接撞负钳位，
     * 之后 1.5 秒巡航都没恢复，等于从 PWM 里白扣掉 800 计数，实速只有指令的
     * 85%(17057 vs 20051)。超出量来自前馈，不是积分器攒的，不该由它买单。 */
    if (u_unsat != u_sat &&
        PosCtrl_Abs32(ref->acc) <= CTRL_INTEGRAL_ACC_GATE)
    {
        s_ctl.vel_i_q16 -= (int32_t)((int64_t)(u_unsat - u_sat)
                                     * (1L << POSCTRL_Q16));
        s_ctl.vel_i_q16 = PosCtrl_Clamp32(s_ctl.vel_i_q16,
                                          CTRL_INTEGRAL_MAX << POSCTRL_Q16);
    }

    /* ---- 11. 遥测 ---- */
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
        dbg->u_out  = (int16_t)u_sat;
        dbg->state  = s_ctl.state;
        dbg->sat    = (uint8_t)((u_unsat != u_sat) ? 1U : 0U);
        dbg->braking = decelerating;
    }

    return (int16_t)(u_sat * CTRL_MOTOR_SIGN);
}
