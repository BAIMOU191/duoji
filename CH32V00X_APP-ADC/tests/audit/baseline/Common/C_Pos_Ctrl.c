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
    uint8_t  quiet;          /* 真正静止的锁存，进入与退出使用不同位置窗口 */

    /* 本拍允许的输出上限，由限功率逻辑下发 */
    int16_t  out_limit;

    /* 由编译期参数换算的内部系数，Init 时算一次 */
    int32_t  fric_dir_k;     /* Q15/(厘度/秒)：摩擦方向在零速附近连续建立 */
    int32_t  fric_v_k;       /* Q15/(厘度/秒)：静摩擦->动摩擦的 Stribeck 过渡 */
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

/* 仅磁编码器折算圆周差值；电位器使用带符号线性误差，禁止走跨死区的捷径。 */
static int32_t PosCtrl_WrapDiff(int32_t d)
{
#if CFG_WRAP_RANGE_CDEG > 0
    if (d >  (CFG_WRAP_RANGE_CDEG / 2)) d -= CFG_WRAP_RANGE_CDEG;
    if (d < -(CFG_WRAP_RANGE_CDEG / 2)) d += CFG_WRAP_RANGE_CDEG;
#endif
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
    s_ctl.quiet    = 0;
}

/*
 * @fn      C_PosCtrl_Init
 * @brief   按编译期参数换算内部系数并复位状态
 *
 * 唯一的初始化除法在这里：把摩擦前馈完整建立的速度阈值换成Q15斜率，
 * 之后每拍只有一次乘法。改了 CTRL_WCMD_MIN_CDPS 或 CTRL_FRIC_VS_CDPS
 * 必须重新调用本函数。
 */
void C_PosCtrl_Init(void)
{
    int32_t vs = CTRL_WCMD_MIN_CDPS;
    if (vs < 1) vs = 1;
    s_ctl.fric_dir_k = (1 << POSCTRL_Q15) / vs;

    vs = CTRL_FRIC_VS_CDPS;
    if (vs < 1) vs = 1;
    s_ctl.fric_v_k = (1 << POSCTRL_Q15) / vs;

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
 *   2) **幅值随实测速度在静摩擦和动摩擦之间连续过渡**(Stribeck)，见下。
 *   3) 从w_cmd=0到阈值线性建立，避免阈值处突然跳入整块摩擦PWM。
 *
 * ==================== 为什么要分静/动摩擦 ====================
 * CAL_FRICTION_PWM 是 T1 速度直线的截距，测的是**动摩擦**——轴已经在转时维持
 * 转动要花的力(实测57)。而末端修正的工况恰恰相反：舵机停在离目标几十厘度的
 * 地方，要动起来必须先越过**静摩擦**(T9 缓升法实测 dir0 236 / dir1 180)。
 * 只补动摩擦的话，缺的那 180 计数只能靠速度环积分以 0.5 计数/拍慢慢顶——
 * 2026-09-09 实测要爬 600ms 才走完最后 200 厘度。
 *
 * 过渡尺度取 CTRL_FRIC_VS_CDPS(=到位零速阈值，当前1600厘度/秒)，而不是更小的
 * CTRL_WCMD_MIN_CDPS：观测器在静止时的速度噪声峰值就有±200厘度/秒，尺度取小了
 * 噪声会把 f 在静/动之间来回拉。1600 下静止时 f 只被噪声拉低约12%，够稳。
 * 物理上这也正是 Stribeck 曲线的形状：低速段摩擦本来就比高速段大。
 *
 * CAL_BREAKAWAY_PWM 未标定(0)时 CTRL_FRIC_STATIC 退化为动摩擦，整个过渡项恒为
 * 零，行为与加这段之前完全一致。
 */
static int32_t PosCtrl_Friction(int32_t w_cmd, int32_t av_obs)
{
    int32_t dir_q15, moving_q15, f;

    /* 正反向不再分开标定：实测两个方向的动摩擦差 1%(87/88)、起转差 6%
     * (188/200)，都在标定重复性(正向起转散布 22 计数)以内。用一组值换来
     * 少两项标定、少两个分支，代价小于测量噪声本身。 */
    /* 原实现到 |w_cmd|=阈值时从0直接跳到完整起转PWM，是一个人为力矩阶跃。
     * 现在用饱和直线建立方向：w_cmd=0严格为0，到阈值时平滑达到完整补偿。 */
    dir_q15 = w_cmd * s_ctl.fric_dir_k;
    dir_q15 = PosCtrl_Clamp32(dir_q15, 1 << POSCTRL_Q15);

    /* f = 静摩擦 - (静-动) * min(|v|/Vs, 1)：
     *     |v|=0  -> 静摩擦(推得动)
     *     |v|>Vs -> 动摩擦(别多推) */
    moving_q15 = PosCtrl_Clamp32(av_obs * s_ctl.fric_v_k, 1 << POSCTRL_Q15);
    f = CTRL_FRIC_STATIC
      - (int32_t)(((int64_t)(CTRL_FRIC_STATIC - CTRL_FRIC_DYN) * moving_q15)
                  >> POSCTRL_Q15);

    return (int32_t)(((int64_t)dir_q15 * f) >> POSCTRL_Q15);
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
    uint8_t holding_load;
    int32_t u_fric, u_vel, u_acc, u_fb, u_i;
    int32_t u_unsat, u_sat, u_out;
    int32_t lim = s_ctl.out_limit;

    if (ref == 0 || obs == 0) return 0;

    av_obs = PosCtrl_Abs32(obs->vel);
    /* 这里只用扰动估计识别需要持续出力的负载，不把估计量前馈到电机。
     * 带载时保留闭环和已学到的积分，防止静音切掉保持力后反复滑落。 */
    holding_load = (uint8_t)(traj_done && CTRL_NUDGE_PWM > 0
        && PosCtrl_Abs32(obs->load) >= CTRL_FRIC_STATIC / 2);

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
    /* 到位判断用实际残差；预测项仅供运动控制，不能让速度噪声改变静音窗口。 */
    PosCtrl_Hold_Update(PosCtrl_WrapDiff(ref->pos - obs->pos), av_obs, traj_done);
    if (!traj_done || holding_load
        || PosCtrl_Abs32(PosCtrl_WrapDiff(ref->pos - obs->pos)) >= CTRL_IN_WIN_CDEG
        || av_obs > CTRL_IN_VEL_CDPS)
        s_ctl.quiet = 0U;
    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD
             && PosCtrl_Abs32(PosCtrl_WrapDiff(ref->pos - obs->pos)) <= CTRL_HOLD_DEADBAND
             && av_obs <= CTRL_VEL_DB_CDPS)
        s_ctl.quiet = 1U;

    /* ---- 位置死区：**任何状态下都生效**，这是防极限环的关键 ----
     * 机构最小可靠位移实测 32 厘度(A_Calib_Min 的 T9 缓升法)。为一个比它还小
     * 的残差去踹 235 计数的静摩擦，物理上必然窜过头，然后反向再窜，形成极限
     * 环；而极限环又让 |vel_obs| 一直摆着，到位判据永远不满足，于是死区永远
     * 进不来——这是个自锁。所以死区必须无条件生效，不能只在 HOLD 里加。
     *
     * 死区与 CTRL_WCMD_MIN_CDPS 共同决定"多大的误差才值得去修"：
     *     可分辨误差 = 死区 + WCMD_MIN/Kp_pos = 32 + 72/9 = 40 厘度 = 0.40°
     * 两项都由 TUNE_RESOLUTION_CDEG 派生，所以改那一个数就能整体收放定位精度。
     * 运动过程中 w_cmd 由前馈主导，死区只削掉那一点残差，不影响跟踪。 */
    /* 修正阶段仍用小死区，使轴能进入48厘度静音捕获窗。
     * 若HOLD一进入就把修正死区也扩大到48，轴会停在48稍外，永远捕获不到。 */
    dead = CTRL_POS_DEADBAND;
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

    /* ---- 7. 前馈组 ----
     * 摩擦前馈**不再按 HOLD 状态关掉**。原来那条"到位就关摩擦前馈"是给
     * 还没有位置死区的老版本写的防蜂鸣措施，现在多余而且有害：
     *
     *   多余 —— u_fric 正比于 w_cmd，而 w_cmd 由死区后的 e_pos_ctl 驱动，
     *           误差进死区时它自然就是 0，不需要再用状态关一遍。
     *   有害 —— "停在离目标几十厘度处需要挪一下"这件事**永远发生在 HOLD 态**，
     *           正是最需要静摩擦补偿的时刻。2026-09-09 实测：停在欠 50 厘度
     *           处，u_fric 恒为 0，只能靠积分以 0.5 计数/30ms 往上爬，爬了
     *           2.2 秒才越过起转值动起来，期间一直在响。
     *
     * 现在误差超过死区 8 厘度(= WCMD_MIN/Kp_pos)时 u_fric 就达到满值 235，
     * 轴一两拍内就挣脱静摩擦；挪进死区后 w_cmd 归零、输出被静音。 */
    u_fric = PosCtrl_Friction(w_cmd, av_obs);
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

    /* ---- 制动前馈的实测速度闸门：**这是"到位回摆"的修复** ----
     * 制动前馈的物理作用只有一个：抵消轴上还剩的动量。轴要是已经停了，就没有
     * 动量可抵消，继续按轨迹输出的那一块就地变成**往回推的力**。
     *
     * 2026-09-09 实测(P0500 到位瞬间，26800->200 的大角度移动)：
     *     ref.vel=-647  obs.vel=+2139  pwm=+702
     *     分项 u_vel-10 u_fb-313 u_i+97 u_fric-57 合计 -283
     *     -> 差出来的 +985 就是 u_acc
     * 机构被静摩擦在轨迹之前刹停了，而 985 计数远超起转值 235，于是把轴往回
     * 推了 26 厘度——就是肉眼看到的那下回摆。只在大角度(有满速减速段)出现，
     * 而且"有时候"，因为取决于轴是否比轨迹先停。
     *
     * 闸门量取**行进方向上还剩多少实测速度**，而不是 |obs.vel|：到位瞬间
     * 观测器会因为模型预测继续运动、实测已停而甩出一个反向速度尖峰(上面那个
     * +2139 就是)，用绝对值反而会判成"还在高速运动"，闸门就白加了。
     * 尺度取 CTRL_IN_VEL_CDPS(判"已停住"的零速阈值)，正是同一个物理含义。 */
    if ((int64_t)ref->acc * ref->vel < 0)
    {
        int32_t v_along = (ref->vel >= 0) ? obs->vel : -obs->vel;
        int32_t g_q15;

        if (v_along <= 0)                     g_q15 = 0;
        else if (v_along >= CTRL_IN_VEL_CDPS) g_q15 = 1 << POSCTRL_Q15;
        else g_q15 = (int32_t)(((int64_t)v_along << POSCTRL_Q15) / CTRL_IN_VEL_CDPS);

        u_acc = (int32_t)(((int64_t)u_acc * g_q15) >> POSCTRL_Q15);
    }

    /* ---- 8. 速度PI的P项 ----
     * 减速段和轨迹结束后的残余运动使用独立增益。实测慢衰减制动在小制动力区
     * 明显比加速侧“软”，提高这一段的反馈可以更早消掉超速，又不会把高增益
     * 带来的编码器量化噪声扩散到整段匀速运动。 */
    /* 保持态额外给 e_vel 套一个**噪声底死区**。轴停住之后 obs.vel 里已经
     * 没有真速度了，剩下的全是观测器把位置量化噪声微分出来的东西——2026-09-09
     * 实测静止段 obs.vel 峰值 ±326、典型 ±150，乘 Kp_vel 就是 ±30 计数的
     * PWM 抖动，一直灌进堵转电机。死区取 TUNE_RESOLUTION_CDEG*10(=400)，
     * 正好盖住噪声底又不吃掉真实扰动：外力真把轴推动时速度远大于 400。
     *
     * **只在保持态生效**。运动中这一项是唯一的阻尼来源，加死区会直接吃掉
     * 减速段的制动能力。
     *
     * 积分那一路仍用未加死区的 e_vel：积分本来就在做平均，噪声会自己抵消，
     * 而加了死区反而会让稳态偏差永远学不进去。 */
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
     *   条件积分 —— 输出已经饱和时，只冻结"会让饱和更深"的那个方向；反向
     *               增量永远放行，于是饱和一解除，积分器已经在正确的一侧。
     *
     * ============ 为什么是条件积分而不是抗饱和回退 ============
     * 原实现用回退式(back-calculation)：饱和时把超出量 (u_unsat-u_sat) 从
     * 积分器里退掉。那个方案隐含一个前提——**饱和是积分器攒出来的**。在这个
     * "前馈为主、反馈为辅"的结构里这个前提从来不成立：饱和的主体是 u_vel
     * (巡航段单项就有 2128 计数)和 u_acc，积分器占比不到一成。于是回退变成
     * 一个单向棘轮：
     *
     *     回退量  -(u_unsat - u_sat)  ≈ 420 PWM/拍
     *     积分量   KI * e_vel         ≈  10 PWM/拍      比值 42:1
     *
     * 巡航段 acc_ref=0、闸门是开的，于是**40 拍里只要有 1 拍饱和，积分器就
     * 被钉死在钳位上，而且符号与运动方向相反**。2026-09-09 电位器版实测
     * (遥测 30ms/帧)：
     *     13.5度小位移 —— u_i 在一个打印间隔内从 +704 掉到 -800(负钳位)，
     *                     整个减速段舵机在反着推，之后 600ms 爬行才停稳；
     *     全行程移动   —— u_i 全程在 -800~-411，等于从驱动里白扣 400~800 计数。
     * 2026-09-03 给回退加加速度闸门只堵住了加速段，巡航段这个洞更大：那里
     * acc_ref 恒为 0，闸门永远是开的。
     *
     * 条件积分没有这个失效模式：饱和期间积分器**保持不动**(而不是被推向
     * 反向钳位)。收敛比回退慢一点，但它不会制造一个方向错误的力矩。
     *
     * 钳位到物理上可能的保持力矩范围；**不随新指令清零**，这样换姿态后
     * 下一条指令一开始就带着正确的负载补偿。 */
    if (s_ctl.quiet)
    {
        s_ctl.vel_i_q16 = 0; /* 已选择短路制动，不能保留会在下一次释放的历史力矩 */
    }
    else if (PosCtrl_Abs32(e_pos) <= dead)
    {
        if (!holding_load) s_ctl.vel_i_q16 -= s_ctl.vel_i_q16 >> 12;
    }
    else if (PosCtrl_Abs32(ref->acc) <= CTRL_INTEGRAL_ACC_GATE)
    {
        int32_t di = CTRL_KI_VEL_Q16 * e_vel;   /* 本拍积分增量，Q16 PWM */

        /* 轨迹结束、轴已低速且仍在静音窗外时，加快建立缺少的静摩擦力。
         * 否则起转力偏高的样机要等待几秒才补最后一步。运动/制动阶段不加速积分。 */
        if (traj_done && av_obs <= CTRL_VEL_DB_CDPS
            && PosCtrl_Abs32(e_pos) > CTRL_HOLD_DEADBAND)
            di *= 4;

        /* 输出已饱和(u_unsat != u_sat)，且本次增量还要往饱和的那一侧推
         * -> 冻结。未饱和时两个判据同时为假，照常积分。 */
        if (!((u_unsat > u_sat && di > 0) || (u_unsat < u_sat && di < 0)))
        {
            s_ctl.vel_i_q16 += di;
            s_ctl.vel_i_q16 = PosCtrl_Clamp32(s_ctl.vel_i_q16,
                                              CTRL_INTEGRAL_MAX << POSCTRL_Q16);
        }
    }

    /* ---- 10.5 保持态输出 ----
     * 无负载时，位置进入48厘度窗口且速度降到噪声底，锁存短路制动。
     * 直到真实误差达到80厘度、出现明显运动或收到新轨迹才退出。
     * 仍需修正的静止轴，净输出不足已标定起转PWM时先不驱动，积分继续建立；
     * 一旦已经运动，保留小PWM制动，避免静音切掉残余速度阻尼。
     * 已识别持续负载时保留保持力，不能为了安静而让轴滑落。 */
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
        else if (av_obs <= CTRL_VEL_DB_CDPS && PosCtrl_Abs32(u_sat) < CTRL_NUDGE_PWM)
        {
            u_out = 0;              /* 还不够起转，先积累 */
        }
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
        dbg->u_out  = (int16_t)u_out;
        dbg->state  = s_ctl.state;
        dbg->sat    = (uint8_t)((u_unsat != u_sat) ? 1U : 0U);
        dbg->braking = decelerating;
    }

    return (int16_t)(u_out * CTRL_MOTOR_SIGN);
}
