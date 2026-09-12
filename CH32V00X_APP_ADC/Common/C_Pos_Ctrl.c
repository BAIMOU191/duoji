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
 * @brief   位置差的坐标折算，由 CFG_WRAP_RANGE_CDEG 决定折算方式
 *
 * 圆周坐标(整圈可测的编码器)**必须折算**：规划器的 ref.pos 是不回绕的行程坐标，
 * 而反馈被折在 [0, 量程) 里。目标定在 0 度时实际只要越过 0 一点点，反馈就跳到
 * 35999，直接相减得到 -35999 而不是 +1，位置环立刻朝反方向满舵停不下来。
 *
 * 线性坐标(有死区、读数可为负的传感器)**禁止折算**：那条"捷径"要穿过测不到角度
 * 的死区，折算等于让控制器朝着一个它看不见的方向满舵。 */
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
    s_ctl.sliding  = 0;   /* 复位后按"粘住"起步，第一次修正才有起转力 */
    s_ctl.kick     = 0;
    s_ctl.stop_cnt = 0;
}

/*
 * @fn      C_PosCtrl_Init
 * @brief   按编译期参数换算内部系数并复位状态
 *
 * 唯一的初始化除法在这里：把摩擦前馈完整建立的速度阈值换成Q15斜率，
 * 之后每拍只有一次乘法。改了 CTRL_WCMD_MIN_CDPS 必须重新调用本函数。
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

/* 摩擦前馈：方向由速度指令给，幅值在静摩擦和动摩擦之间二选一(带滞环锁存)。
 * 返回带符号的摩擦前馈 PWM。
 *
 * 方向取**指令**方向。若取速度误差或实测速度的符号，零速附近符号会来回翻，产生
 * ±1200 计数量级的跳变和极限环——这是摩擦补偿最常见的翻车方式。从 w_cmd=0 到
 * CTRL_WCMD_MIN_CDPS 线性建立，避免阈值处突然跳进整块摩擦PWM。
 *
 * ==================== 为什么分静/动摩擦，以及为什么是锁存 ====================
 * CTRL_FRIC_DYN(=CAL_FRICTION_PWM) 是速度直线的截距，测的是**动摩擦**——轴已经在
 * 转时维持转动要花的力。而末端修正的工况恰恰相反：舵机停在离目标几十厘度的地方，
 * 要动起来必须先越过**静摩擦** CTRL_FRIC_STATIC(=CAL_BREAKAWAY_PWM，实测是动摩擦
 * 的2~4倍)。只补动摩擦的话，缺的那一块只能靠速度环积分以不到1计数/拍慢慢顶，实测
 * 要爬数百毫秒才走完最后200厘度。两台机器的实测值见 A_Parameter.h。
 *
 * 原实现按 |v| 在静/动之间线性插值(Stribeck)，过渡尺度挂在速度噪声底上。**那个
 * 结构在小位移上是错的**：过渡尺度必然远大于噪声底，而小位移的巡航速度由
 * TUNE_MOVE_MIN_MS 限到很低，往往还不到过渡尺度——于是整段运动里 f 从来降不到动
 * 摩擦，一路过驱。主机仿真实测(5us 台阶，约67厘度)：摩擦前馈全程 82~234 而真实动
 * 摩擦只有 57，峰值速度是指令的 1.4 倍，减速段前馈还在正向推、把制动前馈抵消掉，
 * 结果每一级都冲过目标 55 厘度再倒回来——就是肉眼可见的那一下抖动。
 *
 * 现在改成**二值 + 滞环锁存**，因为物理上本来就是二值的：轴要么被静摩擦粘住，要么
 * 已经滑动。判据用实测速度，不用指令速度——"粘住"是机构的状态，不是指令的状态。
 *     |v| >= CTRL_FRIC_MOVE_CDPS               -> 已挣脱，用动摩擦
 *     |v| <= CTRL_VEL_DB_CDPS 连续 CTRL_STUCK_MS 拍 -> 真粘住了，重新武装静摩擦
 * 两个门限拉开一倍以上，噪声峰值远低于下门限，所以噪声拉不动它。
 *
 * 重新武装用的是**连续停住的时长**，不是"指令已归零"。早先那版按指令判，结果轴
 * 停在死区外(误差还在、指令没归零)时锁存永远解不开，摩擦前馈一直按动摩擦给，再也
 * 顶不起来——仿真里把真实起转抬30%，台阶就会卡住、尾部持续通电几百拍。按时长判则
 * 两种情况都对：慢速运动中速度偶尔掉到下门限只是零星几拍，攒不够时长；真停住了
 * 几十毫秒就一定攒得够。
 *
 * ==================== 粘住时是"顶到"而不是"加上" ====================
 * 两种工况要的东西不一样，必须分开写：
 *   已滑动 —— 动摩擦是一份持续的阻力，必须**加在**其它项之上，少一分就走不动。
 *   粘住   —— 要的只是"总输出达到起转值"这一个条件。此时把整块静摩擦再**加到**
 *             加速度前馈上，总输出就远超起转值，多出来的部分在挣脱的那一瞬间
 *             全部变成多余的加速度。
 * 主机仿真实测(5us 台阶)：起转只需 235，而 静摩擦235 + 加速度前馈116 = 358，
 * 多出的 123 在头 12ms 把轴推得比参考还快，最后留下 +35 厘度的系统性超前——
 * 这是把 Stribeck 换成锁存之后剩下的全部误差。
 * 所以粘住时本函数返回的是**补足量** max(0, 顶起值 - 其余各项)，推到刚好挣脱
 * 为止；其余各项本身已经够大时补0，轴照样会动。
 *
 * ==================== 顶起值为什么要自适应抬升 ====================
 * 标定出来的起转值会偏小：实测同一台机构两个方向就差 30%，再叠上温度、磨损和
 * 个体差异。顶到一个偏小的值就是顶不动——仿真里把真实起转抬 30%，固定顶起值会
 * 让台阶卡住、尾部持续通电几百拍，最后靠积分爬起来时又猛冲一下。
 * 而固定加一个裕量是纯粹的取舍：裕量够大能盖住失配，标定准确时又白白多推(仿真
 * 实测裕量从 0 加到 50%，标称工况的平均过冲从 18 涨到 30 厘度)。
 *
 * 抬升只在"**要求动却确实没动**"时逐拍发生，轴一滑动或指令一归零就清零，于是
 * 标定准确时抬升量恒为 0(拿到全部精度)，标定偏小时几十毫秒内自动补齐(拿到全部
 * 可靠性)。上限 CTRL_FRIC_KICK_MAX 防止顶着障碍物无限加力，真堵转由保护模块管。
 *
 * CAL_BREAKAWAY_PWM 未标定(<=动摩擦)时两条支路给出同一个值，行为退化成纯动摩擦。 */
static int32_t PosCtrl_Friction(int32_t w_cmd, int32_t av_obs, int32_t u_rest)
{
    int32_t dir_q15, want, have;

    /* 正反向不再分开标定：实测两个方向的动摩擦差 1%、起转差 6%，都在标定重复性
     * 以内。用一组值换来少两项标定、少两个分支，代价小于测量噪声本身。 */
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
    int32_t av_obs, dead, e_measured, ae_measured;
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
    e_measured = PosCtrl_WrapDiff(ref->pos - obs->pos);
    ae_measured = PosCtrl_Abs32(e_measured);
    PosCtrl_Hold_Update(e_measured, av_obs, traj_done);
    if (!traj_done || holding_load
        || ae_measured >= CTRL_IN_WIN_CDEG
        || av_obs > CTRL_IN_VEL_CDPS)
        s_ctl.quiet = 0U;
    else if (s_ctl.state == (uint8_t)POSCTRL_HOLD
             && ae_measured <= CTRL_HOLD_DEADBAND
             && av_obs <= CTRL_VEL_DB_CDPS)
        s_ctl.quiet = 1U;

    /* ---- 位置死区：**任何状态下都生效**，这是防极限环的关键 ----
     * 死区取 CTRL_POS_DEADBAND(=TUNE_RESOLUTION_CDEG，机构最小可靠位移)。为一个
     * 比它还小的残差去踹整块静摩擦，物理上必然窜过头，然后反向再窜，形成极限
     * 环；而极限环又让 |vel_obs| 一直摆着，到位判据永远不满足，于是死区永远
     * 进不来——这是个自锁。所以死区必须无条件生效，不能只在 HOLD 里加。
     *
     * 死区与 CTRL_WCMD_MIN_CDPS 共同决定"多大的误差才值得去修"：
     *     可分辨误差 = CTRL_POS_DEADBAND + CTRL_WCMD_MIN_CDPS/Kp_pos
     * 两项都由 TUNE_RESOLUTION_CDEG 派生，所以改那一个数就能整体收放定位精度。
     * 运动过程中 w_cmd 由前馈主导，死区只削掉那一点残差，不影响跟踪。
     *
     * 修正阶段**仍用小死区**，让轴能一路走进 CTRL_HOLD_DEADBAND 静音捕获窗。
     * 若一进 HOLD 就把修正死区也放宽到捕获窗，轴会停在捕获窗稍外，永远捕获不到。 */
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
     * 摩擦前馈**不按 HOLD 状态关掉**。原来那条"到位就关摩擦前馈"是给还没有位置
     * 死区的老版本写的防蜂鸣措施，现在多余而且有害：
     *
     *   多余 —— u_fric 正比于 w_cmd，而 w_cmd 由死区后的 e_pos_ctl 驱动，
     *           误差进死区时它自然就是 0，不需要再用状态关一遍。
     *   有害 —— "停在离目标几十厘度处需要挪一下"这件事**永远发生在 HOLD 态**，
     *           正是最需要静摩擦补偿的时刻。实测关掉它以后，末端残差只能靠积分
     *           以不到1计数/拍往上爬，爬了两秒多才越过起转值动起来，期间一直在响。
     *
     * 它的计算挪到第9节：粘住时要"把总输出顶到起转值"，得先知道其余各项有多少。 */
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
     * 实测(大角度移动的到位瞬间)：机构被静摩擦在轨迹之前刹停，而残余 u_acc 远
     * 超起转值，于是把轴往回推了几十厘度——就是肉眼看到的那下回摆。只在大角度
     * (有满速减速段)出现，而且"有时候"，因为取决于轴是否比轨迹先停。
     *
     * 闸门量取**行进方向上还剩多少实测速度**，而不是 |obs.vel|：到位瞬间观测器
     * 会因为模型预测继续运动、实测已停而甩出一个反向速度尖峰，用绝对值反而会
     * 判成"还在高速运动"，闸门就白加了。
     *
     * 尺度取 CTRL_FRIC_MOVE_CDPS，也就是判"轴确实在动"的那同一个门限——闸门要
     * 回答的本来就是"轴还在不在动"。早先取的是 CTRL_IN_VEL_CDPS(判"已到位"的零速
     * 阈值，是它的几倍)，**对小位移是错的**：小位移的巡航速度由 TUNE_MOVE_MIN_MS
     * 限得很低，整段减速都落在闸门的线性段里，制动前馈只放行 35~60%。主机仿真
     * 实测(5us 台阶)每一级因此比参考多走约 7 厘度，逐级累积到静音窗边缘。 */
    if ((int64_t)ref->acc * ref->vel < 0)
    {
        int32_t v_along = (ref->vel >= 0) ? obs->vel : -obs->vel;
        int32_t g_q15;

        if (v_along <= 0)                       g_q15 = 0;
        else if (v_along >= CTRL_FRIC_MOVE_CDPS) g_q15 = 1 << POSCTRL_Q15;
        else g_q15 = (v_along * (1L << POSCTRL_Q15)) / CTRL_FRIC_MOVE_CDPS;

        u_acc = (int32_t)(((int64_t)u_acc * g_q15) >> POSCTRL_Q15);
    }

    /* ---- 8. 速度PI的P项 ----
     * 减速段和轨迹结束后的残余运动使用独立增益。实测慢衰减制动在小制动力区
     * 明显比加速侧“软”，提高这一段的反馈可以更早消掉超速，又不会把高增益
     * 带来的编码器量化噪声扩散到整段匀速运动。 */
    /* 保持态额外给 e_vel 套一个**噪声底死区** CTRL_VEL_DB_CDPS。轴停住之后
     * obs.vel 里已经没有真速度了，剩下的全是观测器把位置量化噪声微分出来的
     * 东西，乘 Kp_vel 就是几十计数的 PWM 抖动，一直灌进堵转电机。死区按
     * DSG_VEL_NOISE_CDPS(正比于观测器带宽和位置分辨率)的 DSGC_VEL_DB_X 倍派生，
     * 正好盖住噪声底又不吃掉真实扰动：外力真把轴推动时速度远大于它。
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

    /* ---- 9. 摩擦前馈与合成限幅 ----
     * 摩擦项最后算：粘住时它给的是"离起转值还差多少"，必须先知道其余各项之和。 */
    u_i     = s_ctl.vel_i_q16 >> POSCTRL_Q16;
    u_fric  = PosCtrl_Friction(w_cmd, av_obs, u_vel + u_acc + u_fb + u_i);
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
     * 巡航段 acc_ref=0、闸门是开的，于是**几十拍里只要有 1 拍饱和，积分器就
     * 被钉死在钳位上，而且符号与运动方向相反**。实测小位移时 u_i 在一个遥测
     * 间隔内就从正钳位掉到负钳位，整个减速段舵机在反着推；全行程移动时 u_i
     * 全程贴着负钳位，等于从驱动里白扣掉几百计数。
     * 给回退加加速度闸门只堵得住加速段，巡航段这个洞更大：那里 acc_ref 恒为 0，
     * 闸门永远是开的。
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
     * 无负载时，位置进入 CTRL_HOLD_DEADBAND 捕获窗且速度降到 CTRL_VEL_DB_CDPS
     * 噪声底，锁存短路制动；直到真实误差达到 CTRL_IN_WIN_CDEG、出现明显运动或
     * 收到新轨迹才退出。仍需修正的静止轴，净输出不足 CTRL_NUDGE_PWM(已标定的
     * 起转值)时先不驱动，让积分继续建立；一旦已经运动，保留小PWM制动，避免
     * 静音切掉残余速度阻尼。已识别持续负载时保留保持力，不能为了安静而让轴滑落。 */
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
