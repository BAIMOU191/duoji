/* C_Pos_Ctrl.c 舵机位置闭环控制器，纯算法，无硬件依赖 */

#include "C_Pos_Ctrl.h"
#include "C_Friction.h"
#include "C_Fixed.h"
#include "A_Config.h"

#define POSCTRL_Q15        15U
#define POSCTRL_Q16        16U
#define POSCTRL_Q20        20U

/* ===================== 控制器系数(编译期由 A_Config.h 的标定/调参算出) =====================
 * 位置环带宽 -> 速度环带宽(x TUNE_DAMPING/10) -> 观测器带宽，三级带宽拉开是稳定的前提，
 * 断言在 A_Config.h 第7节。这里只做单位换算和定点化，没有第二套设计参数。 */
#define CTRL_VEL_BW_RADS PLANT_VEL_BW_RADS                                            /* 速度环带宽 */

#define CTRL_KP_POS_Q8 (TUNE_STIFFNESS_RADS * 256)                                    /** 位置环P，Q8 */
#define CTRL_KP_VEL_RAW ((int32_t)(((int64_t)CTRL_VEL_BW_RADS * 65536 * 256) / PLANT_KA_Q8))
#define CTRL_KP_VEL_Q16 ((CTRL_KP_VEL_RAW > 0) ? CTRL_KP_VEL_RAW : 1)                 /** 速度环P，Q16 */
/* 慢衰减H桥制动侧出力较弱，减速时用另一组增益，比例就是标定出来的制动增益 */
#define CTRL_KP_VEL_BRAKE_Q16 ((int32_t)(((int64_t)CTRL_KP_VEL_Q16 * MODEL_BRAKE_GAIN_Q8) >> 8))
#define CTRL_KI_VEL_RAW ((int32_t)(((int64_t)PLANT_INTEGRAL_RADS * CTRL_KP_VEL_Q16 * CFG_TICK_HZ) / 1000000))
#define CTRL_KI_VEL_Q16 ((CTRL_KI_VEL_RAW > 0) ? CTRL_KI_VEL_RAW : 1)                 /** 速度环I，Q16 */
#define CTRL_INTEGRAL_MAX (CFG_PWM_FULL / 3)                                          /** 积分限幅，PWM */
#define CTRL_INTEGRAL_ACC_GATE (PLANT_A0_CDPSS / 19)                                  /** 大加速度时冻结积分的阈值 */

#define CTRL_KA_Q20 ((int32_t)((1048576LL * 256) / PLANT_KA_Q8))                      /** 加速度前馈，Q20 */
#define CTRL_KA_BRAKE_Q20 ((int32_t)(((int64_t)CTRL_KA_Q20 * MODEL_BRAKE_GAIN_Q8) >> 8))
/* 制动放行：轴比参考每慢1%，制动前馈就少给2%。模型把惯量估大时轴会先于参考停住，
 * 满额制动前馈会把它刹在目标之前再慢慢挪过去。1~4 扫描取2，取4减速段PWM抖动明显变大 */
#define CTRL_BRAKE_RELAX_X 2
#define CTRL_PREVIEW_Q16 ((int32_t)(((int64_t)MODEL_DELAY_TICKS * 65536) / CFG_TICK_HZ)) /** 加速度预演时长，Q16秒 */
#define CTRL_POS_LEAD_Q16 ((MODEL_DELAY_TICKS + MODEL_TAU_MS / 8) * 65536 / CFG_TICK_HZ) /** 相对速度预测时长，Q16秒 */

#define CTRL_IN_VEL_CDPS (CAL_MIN_STEP_CDEG * 50)           /** 到位零速阈值，厘度/秒 */
#define CTRL_WCORR_MAX ((int32_t)(((int64_t)PLANT_VSS_CDPS * 21) / 100)) /** 位置环速度修正限幅 */

/* 位置窗口全部是最小可靠位移的整数倍：进窗小、出窗大，中间的回差挡住静止噪声 */
#define CTRL_POS_DEADBAND (TUNE_RESOLUTION_CDEG)            /** 位置死区 = 1倍最小步长 */
#define CTRL_HOLD_DEADBAND (TUNE_RESOLUTION_CDEG * 3 / 2)   /** 静音捕获窗 = 1.5倍 */
#define CTRL_IN_WIN_CDEG (TUNE_RESOLUTION_CDEG * 5 / 2)     /** 到位窗 = 2.5倍 */
#define CTRL_OUT_WIN_CDEG (TUNE_RESOLUTION_CDEG * 4)        /** 退出窗 = 4倍，与到位窗构成滞环 */
#define CTRL_IN_HOLD_MS 20                                  /** 进入到位状态前需连续满足条件的拍数 */
#define CTRL_NUDGE_PWM ((CTRL_FRIC_STATIC > CAL_FRICTION_PWM) ? CTRL_FRIC_STATIC : 0)    /** 静止时值得驱动的最小PWM */

/* 静止保持状态，见 PosCtrl_HoldMode */
enum { HM_ACTIVE = 0, HM_PROBE, HM_QUIET, HM_LOADED };
#define CTRL_PROBE_STEP_Q15 (32767 / 128)   /** 试探卸力：总出力128ms线性降到0 */
/* 蠕动判据：偏离停稳点这么多就算滑了。不能只等静音退出窗(2倍分辨率，15kg为2度)：
 * 动摩擦小于负载时轴一旦蠕动就一路滑下去 */
#define CTRL_SETTLE_MS 100   /** 停稳时残差还在位置死区外，先留给摩擦补偿挪动的时间(顶起值抬满约70ms) */
/** 值得等着挪的残差下限：比最小可靠位移还小的残差顶不准，粘滑一窜就过头 */
#define CTRL_SETTLE_EMIN ((CAL_MIN_STEP_CDEG > CTRL_POS_DEADBAND) ? CAL_MIN_STEP_CDEG : CTRL_POS_DEADBAND)
#define CTRL_CREEP_CDEG ((CAL_MIN_STEP_CDEG / 2 > 0) ? (CAL_MIN_STEP_CDEG / 2) : 1)

/* 到位保持弹簧：对外统一为0~12 PWM/厘度。弹簧死区固定为0.5度，不能再随传感器；详见 docs/ALGORITHM_NOTES.md。 */
#define CTRL_HOLD_SPRING_DEADBAND_CDEG 50  /** 保持弹簧对外统一使用0.5度死区 */
#define CTRL_HOLD_STIFF_SLEW_MS 10U                  /** 运行时改值每10ms最多变化1，避免力矩阶跃 */
/* 弹簧阻尼 = TUNE_HOLD_DAMP_X 倍速度环P增益，Q16 PWM/(厘度/秒)；为0则弹簧不带阻尼 */
#define CTRL_HOLD_DAMP_Q16 (CTRL_KP_VEL_Q16 * TUNE_HOLD_DAMP_X)
#define CTRL_HOLD_I_SHIFT 6                        /** 弹簧出力时积分统一降到1/64，不在刚度8产生行为突变 */
#define CTRL_HOLD_SPRING_MAX (CFG_PWM_FULL / 2)

_Static_assert (CTRL_FRIC_MOVE_CDPS > 0, "速度噪声底算出来是0，检查 TUNE_OBS_BW_HZ/CAL_MIN_STEP_CDEG");
/* 制动放行比例在int32内算：参考速度上限左移15位不能溢出 */
_Static_assert (PLANT_VMAX_HW_CDPS < 65536, "硬件最大速度太大，制动放行的Q15比例会溢出");
_Static_assert (CTRL_OUT_WIN_CDEG > CTRL_IN_WIN_CDEG, "到位滞环反了");

static struct {
    /* 速度PI积分项，Q16 PWM。跨指令保持，只在 Reset 时清零 */
    int32_t vel_i_q16;

    /* 到位保持状态机 */
    uint8_t  state;
    uint16_t hold_cnt;
    uint8_t  hm;             /* 静止保持状态 HM_* */
    uint8_t  loaded;         /* 本次保持已确认要靠出力扛负载，再停稳时不再试探 */
    int16_t  probe_q15;      /* 试探卸力的出力比例 */
    uint8_t  settle_ms;      /* 停稳且残差在位置死区外的拍数 */
    int8_t   dir;            /* 最近一段走位的方向，判欠位/冲过头 */
    int32_t  e_ref;          /* 停稳(或转静音)那一刻的残差，判蠕动用 */
    int32_t  i_ref_q16;      /* 停稳那一刻的积分：负载方向，滑了按它还原 */
    int16_t  u_prev;         /* 闭环最后一拍的实际出力：停稳时据此决定试不试，试探就从它线性卸到0 */
    uint8_t  pushed;         /* 上一拍保持弹簧在出力(被外力推开) */
    uint8_t  held;           /* 1=本段轨迹走完后已进入过保持，此后被外力推开才启用保持弹簧 */
    uint8_t  hold_stiff;     /* 当前实际刚度；缓慢跟随 SRAM 实时参数 */
    uint8_t  hold_slew_cnt;  /* 刚度每次变化1之前的毫秒计数 */

    /* 本拍允许的输出上限，由限功率逻辑下发 */
    int16_t  out_limit;

    /* 与对象模型有关的增益：标定值乘自适应得到的对象参数比，见 C_PosCtrl_SetPlant */
    int32_t  kp_vel_q16;     /* 速度环P */
    int32_t  kp_brake_q16;   /* 减速侧速度环P */
    int32_t  ki_vel_q16;     /* 速度环I，每拍 */
    int32_t  ka_q20;         /* 加速度前馈 */
    int32_t  ka_brake_q20;   /* 减速侧加速度前馈 */
    int32_t  slope_q16;      /* 速度前馈 1/Kv */
} s_ctl;

/* 清空积分与到位状态，参数不动 */
void C_PosCtrl_Reset(void)
{
    s_ctl.vel_i_q16 = 0;
    s_ctl.state    = (uint8_t)POSCTRL_RUN;
    s_ctl.hm       = HM_ACTIVE;
    s_ctl.hold_cnt = s_ctl.loaded = s_ctl.probe_q15 = s_ctl.settle_ms = 0;
    s_ctl.dir = s_ctl.e_ref = s_ctl.i_ref_q16 = s_ctl.u_prev = 0;
    s_ctl.pushed = s_ctl.held = 0;
    C_Fric_Reset();
}

/* 模型增益 = 标定值 x 对象参数比。比值为1时与编译期常数逐位相同。 */
void C_PosCtrl_SetPlant(int32_t ka_q12, int32_t kv_q12)
{
    int32_t ki;

    if (ka_q12 <= 0) ka_q12 = OBS_PLANT_ONE;
    if (kv_q12 <= 0) kv_q12 = OBS_PLANT_ONE;
    s_ctl.kp_vel_q16   = (int32_t)(((int64_t)CTRL_KP_VEL_Q16 * ka_q12) >> 12);
    s_ctl.kp_brake_q16 = (int32_t)(((int64_t)CTRL_KP_VEL_BRAKE_Q16 * ka_q12) >> 12);
    ki = (int32_t)(((int64_t)PLANT_INTEGRAL_RADS * s_ctl.kp_vel_q16 * CFG_TICK_HZ) / 1000000);
    s_ctl.ki_vel_q16   = (ki > 0) ? ki : 1;
    s_ctl.ka_q20       = (int32_t)(((int64_t)CTRL_KA_Q20 * ka_q12) >> 12);
    s_ctl.ka_brake_q20 = (int32_t)(((int64_t)CTRL_KA_BRAKE_Q20 * ka_q12) >> 12);
    s_ctl.slope_q16    = (int32_t)(((int64_t)CAL_SPEED_SLOPE_Q16 << 12) / kv_q12);
}

/* 增益回到标定值并复位 */
void C_PosCtrl_Init(void)
{
    C_PosCtrl_SetPlant(OBS_PLANT_ONE, OBS_PLANT_ONE);

    s_ctl.out_limit = CFG_PWM_FULL;

    C_PosCtrl_Reset();
    s_ctl.hold_stiff = (g_runtime_config.hold_stiff <= CFG_HOLD_STIFF_MAX)
                     ? g_runtime_config.hold_stiff : CFG_DEF_HOLD_STIFF;
    s_ctl.hold_slew_cnt = 0U;
}

/* 总线在线修改保持刚度时平滑接入；控制周期为1ms，10拍只走1个刻度。 */
static void PosCtrl_HoldStiffStep(void)
{
    uint8_t target = g_runtime_config.hold_stiff;

    if (target > CFG_HOLD_STIFF_MAX) target = CFG_HOLD_STIFF_MAX;
    if (target == s_ctl.hold_stiff)
    {
        s_ctl.hold_slew_cnt = 0U;
        return;
    }
    if (++s_ctl.hold_slew_cnt < CTRL_HOLD_STIFF_SLEW_MS) return;

    s_ctl.hold_slew_cnt = 0U;
    if (s_ctl.hold_stiff < target) s_ctl.hold_stiff++;
    else                           s_ctl.hold_stiff--;
}

void C_PosCtrl_Set_Output_Limit(int16_t limit)
{
    s_ctl.out_limit = (limit <= 0 || limit > CFG_PWM_FULL) ? CFG_PWM_FULL : limit;
}

/* 轴是被负载逆着出力方向拖走的(滑落)，而不是被自己的出力推过了头：出窗看残差方向，速度先超限看速度方向 */
static uint8_t PosCtrl_Slipped(int32_t e_meas, int32_t obs_vel, uint8_t out_of_win, int32_t force)
{
    if (force == 0) return 0U;
    if (out_of_win) return (uint8_t)((e_meas > 0) == (force > 0));
    return (uint8_t)((obs_vel > 0) != (force > 0));
}

/* 静止保持。停稳 = HOLD态、残差在静音捕获窗内、速度压到噪声底。欠位且残差在位置死区外时先等至多；详见 docs/ALGORITHM_NOTES.md。 */
static void PosCtrl_HoldMode(int32_t e_meas, int32_t obs_vel, uint8_t traj_done)
{
    int32_t ref = s_ctl.i_ref_q16;                    /* 停稳时的积分：它的方向就是在扛负载的方向 */
    int32_t dev = Fx_WrapDiff(e_meas - s_ctl.e_ref, CFG_WRAP_RANGE_CDEG);              /* 离停稳点的偏移 */
    uint8_t out_of_win = (uint8_t)(Fx_Abs32(e_meas) >= TUNE_RESOLUTION_CDEG * 2);
    uint8_t moved = (uint8_t)(out_of_win || Fx_Abs32(obs_vel) > CTRL_IN_VEL_CDPS);
    /* 滑落：停稳那拍确有扛负载的出力被卸掉(不小于起转值1/8；本来就没出力的是还在惯性滑行)，
     * 已在目标的负载一侧(往目标方向滑是回到目标，不算)，且顺着负载方向(与积分相反)偏离停稳点
     * 半个最小可靠位移，或者已滑出静音退出窗(分辨率小于最小可靠位移时出窗先到，不能当成被外力推开) */
    uint8_t crept = (uint8_t)(ref != 0 && Fx_Abs32(s_ctl.u_prev) >= CTRL_NUDGE_PWM / 8
                              && (int64_t)e_meas * ref > 0
                              && (out_of_win || ((ref > 0) ? dev : -dev) >= CTRL_CREEP_CDEG));

    if (!traj_done) { s_ctl.hm = HM_ACTIVE; s_ctl.loaded = 0U; s_ctl.settle_ms = 0U; return; }
    if (s_ctl.hm != HM_ACTIVE && crept)   /* 滑了：负载要靠出力扛，还原停稳时的积分回闭环，本次保持记住 */
    {
        s_ctl.hm        = HM_ACTIVE;
        s_ctl.loaded    = 1U;
        s_ctl.vel_i_q16 = ref;
        return;
    }
    switch (s_ctl.hm)
    {
    case HM_ACTIVE:
        /* 停稳要真停：速度压到噪声底。带载走完常还在死区里慢滑，这时卸力会滑出死区 */
        if (s_ctl.state != (uint8_t)POSCTRL_HOLD || Fx_Abs32(e_meas) > CTRL_HOLD_DEADBAND
            || Fx_Abs32(obs_vel) > CTRL_VEL_NOISE_CDPS) { s_ctl.settle_ms = 0; break; }
        ref = s_ctl.vel_i_q16;
        /* 积分把轴往远离目标推(与残差反号)：是走位留下的残余，不是在扛负载，清掉；
         * 不清的话会和往目标顶的弹簧/摩擦补偿互相抵消，合力随噪声正负翻转 */
        if ((int64_t)e_meas * ref < 0) s_ctl.vel_i_q16 = 0;
        /* 欠位且残差在位置死区外：摩擦补偿还在顶，等它挪进来，挪不动再试。
         * 冲过头的不往回拉(那就是回转)，直接试；比最小可靠位移还小的残差也不挪 */
        if (Fx_Abs32(e_meas) > CTRL_SETTLE_EMIN && (int32_t)s_ctl.dir * e_meas > 0
            && ++s_ctl.settle_ms < CTRL_SETTLE_MS) break;
        s_ctl.settle_ms = 0;
        s_ctl.held      = 1U;   /* 停稳之后保持弹簧才参与，不对到位残差出力 */
        s_ctl.e_ref     = e_meas;
        s_ctl.i_ref_q16 = ref;  /* 清之前的积分：试探中顺着它的方向滑了就按它还原 */
        s_ctl.probe_q15 = 32767;
        if (s_ctl.loaded || Fx_Abs32(s_ctl.vel_i_q16) >= ((int32_t)CTRL_FRIC_STATIC << POSCTRL_Q16))
            s_ctl.hm = HM_LOADED;
        else if (!s_ctl.pushed || Fx_Abs32(s_ctl.u_prev) < CTRL_NUDGE_PWM)
            s_ctl.hm = HM_PROBE;
        /* 保持弹簧顶着外力且出力不小于起转值：留在闭环，刚度不打折 */
        break;
    case HM_PROBE:          /* 被推动(任何方向)也立刻回闭环，卸力不能让外力把轴推走 */
        if (moved || Fx_Abs32(dev) >= CTRL_CREEP_CDEG) s_ctl.hm = HM_ACTIVE;
        else if ((s_ctl.probe_q15 -= CTRL_PROBE_STEP_Q15) <= 0
                 || (s_ctl.probe_q15 <= 24576 && Fx_Abs32(s_ctl.u_prev) < CTRL_NUDGE_PWM / 8))
        {   /* 卸完(起点只有几PWM的，跟随期过了就不必再慢慢卸)转静音。积分不小于起转值1/4才是在扛负载，留给下一段走位起步；否则是残余，清掉 */
            s_ctl.hm    = HM_QUIET;
            s_ctl.e_ref = e_meas;
            if (Fx_Abs32(s_ctl.vel_i_q16) < ((int32_t)CTRL_FRIC_STATIC << POSCTRL_Q16) / 4) s_ctl.vel_i_q16 = 0;
        }
        else if (s_ctl.probe_q15 > 24576) s_ctl.e_ref = e_meas;   /* 前32ms跟着走：刚停稳时还在噪声底以下滑行 */
        break;
    case HM_QUIET:          /* 被推开：回闭环，不留历史力矩 */
        if (moved || Fx_Abs32(dev) >= CTRL_CREEP_CDEG) { s_ctl.hm = HM_ACTIVE; s_ctl.vel_i_q16 = 0; }
        break;
    default:                /* HM_LOADED：被出力自己推开(负载卸了)才忘掉 loaded */
        if (moved)
        {
            if (!PosCtrl_Slipped(e_meas, obs_vel, out_of_win, ref)) s_ctl.loaded = 0U;
            s_ctl.hm = HM_ACTIVE;
        }
        break;
    }
}

/*
 * 到位锁存，全部用不带预测的实际残差判定(预测项只服务运动控制，不能让速度噪声改变窗口)：
 *   state —— RUN/HOLD，进窗小、出窗大，回差挡住静止噪声
 *   held  —— 本段轨迹走完后进入过保持，此后被外力推开才允许保持弹簧出力
 */
static void PosCtrl_Latches(int32_t e_meas, int32_t obs_vel, uint8_t traj_done)
{
    int32_t av_obs = Fx_Abs32(obs_vel);
    int32_t ae = Fx_Abs32(e_meas);

    if (s_ctl.state == (uint8_t)POSCTRL_RUN)
    {
        if (traj_done && ae <= CTRL_IN_WIN_CDEG && av_obs <= CTRL_IN_VEL_CDPS)
        {
            if (s_ctl.hold_cnt < 0xFFFFU) s_ctl.hold_cnt++;
            if (s_ctl.hold_cnt >= CTRL_IN_HOLD_MS) s_ctl.state = (uint8_t)POSCTRL_HOLD;
        }
        else s_ctl.hold_cnt = 0;
    }
    else if (!traj_done || ae >= CTRL_OUT_WIN_CDEG)
    {
        s_ctl.state    = (uint8_t)POSCTRL_RUN;
        s_ctl.hold_cnt = 0;
    }

    if (!traj_done) s_ctl.held = 0U;   /* 置位在 PosCtrl_HoldMode 判定停稳那一拍 */
    PosCtrl_HoldMode(e_meas, obs_vel, traj_done);
}

/* 加速度前馈。减速段(acc 与 vel 反号)是主动制动，慢衰减H桥这一侧出力较弱，单独用 Ka_brake；；详见 docs/ALGORITHM_NOTES.md。 */
static int32_t PosCtrl_AccFF(const TrajRef_t *ref, int32_t obs_vel)
{
    uint8_t braking = (uint8_t)((int64_t)ref->acc * ref->vel < 0);
    int32_t u_acc = (int32_t)(((int64_t)(braking ? s_ctl.ka_brake_q20 : s_ctl.ka_q20)
                               * ref->acc) >> POSCTRL_Q20);
    int32_t v_along, scale, g_q15;

    if (!braking) return u_acc;

    v_along = (ref->vel >= 0) ? obs_vel : -obs_vel;   /* 行进方向上的实测速度 */
    scale   = Fx_Abs32(ref->vel);                /* 尺度取参考速度，太低时取门限 */
    if (scale < CTRL_FRIC_MOVE_CDPS) scale = CTRL_FRIC_MOVE_CDPS;

    if (v_along <= 0)          g_q15 = 0;
    else if (v_along >= scale) g_q15 = 1 << POSCTRL_Q15;
    else g_q15 = Fx_RatioQ15((uint32_t)v_along, (uint32_t)scale); /* 0<v_along<scale<2^16 */

    /* 每慢 1% 就少给 CTRL_BRAKE_RELAX_X 个 1% 的刹车 */
    g_q15 = (1 << POSCTRL_Q15) - CTRL_BRAKE_RELAX_X * ((1 << POSCTRL_Q15) - g_q15);
    if (g_q15 < 0) g_q15 = 0;
    return (int32_t)(((int64_t)u_acc * g_q15) >> POSCTRL_Q15);
}

/* 保持弹簧：到位后被外力推开时按实际残差直接回推。级联环为压传感器噪声调得很软，靠它补保持刚度；
 * 只在进入过保持之后生效，所以走位和到位过程完全不受影响；试探和静音时不出力(被推动会先回到闭环)。 */
static int32_t PosCtrl_Spring(int32_t e_meas, int32_t obs_vel)
{
    int32_t e, u;

    if (s_ctl.hold_stiff == 0U || (!s_ctl.held && !s_ctl.settle_ms)
        || s_ctl.hm == HM_QUIET || s_ctl.hm == HM_PROBE) return 0;

    e = Fx_Deadband(e_meas, CTRL_HOLD_SPRING_DEADBAND_CDEG);
    if (e == 0) return 0;

    u = Fx_Clamp32((int32_t)s_ctl.hold_stiff * e, CTRL_HOLD_SPRING_MAX);
    /* 还没停稳(等摩擦补偿挪残差的窗口里)：弹簧随等待时间线性加力，软顶残差，不一拍砸上去 */
    if (!s_ctl.held) u = (int32_t)(((int64_t)u * s_ctl.settle_ms * ((1 << 15) / CTRL_SETTLE_MS)) >> 15);
    /* 带阻尼：轴被负载带动时按速度反向出力，吸收手臂的摆动而不是把它弹回去 */
    return u - (int32_t)(((int64_t)CTRL_HOLD_DAMP_Q16 * obs_vel) >> POSCTRL_Q16);
}

/* 速度PI的积分项，全系统唯一的积分器(观测器的负载估计不外送)。四条分支各挡一种失效；详见 docs/ALGORITHM_NOTES.md。 */
static void PosCtrl_Integrate(int32_t e_pos, int32_t e_vel, int32_t ref_acc, int32_t av_obs,
                              uint8_t traj_done, uint8_t holding_load,
                              int32_t u_spring, int32_t u_unsat, int32_t u_sat)
{
    int32_t di;

    if (s_ctl.hm != HM_ACTIVE) return;
    /* 外力已经撤掉、弹簧也回到0.5度死区后，快速卸掉刚才为抗外力积下的力矩。 */
    if (traj_done && s_ctl.held && !holding_load && u_spring == 0)
        s_ctl.vel_i_q16 -= s_ctl.vel_i_q16 >> 4;
    if (Fx_Abs32(e_pos) <= CTRL_POS_DEADBAND)
    {
        if (!holding_load) s_ctl.vel_i_q16 -= s_ctl.vel_i_q16 >> 12;
        return;
    }
    if (Fx_Abs32(ref_acc) > CTRL_INTEGRAL_ACC_GATE) return;

    di = s_ctl.ki_vel_q16 * e_vel;               /* 本拍积分增量，Q16 PWM */
    /* 弹簧在出力时积分放慢到 1/64，避免与弹簧叠加过冲。
     * 统计(18种外力)：刚度1~12时 推开量/5秒静差/卸力回摆 随刚度单调变好；刚度0没有弹簧、全靠积分，
     * 静差最小但松手回摆最大。试过"刚度越小积分放得越开"：静差拉平了，回摆却涨到 40~100 厘度，未采用。 */
    if (u_spring != 0) di >>= CTRL_HOLD_I_SHIFT;
    /* 轨迹结束、轴已低速却还在捕获窗外：加快把缺的那点静摩擦力建起来 */
    if (traj_done && av_obs <= CTRL_VEL_DB_CDPS
        && Fx_Abs32(e_pos) > CTRL_HOLD_DEADBAND) di *= 4;

    if ((u_unsat > u_sat && di > 0) || (u_unsat < u_sat && di < 0)) return;  /* 饱和侧冻结 */
    s_ctl.vel_i_q16 = Fx_Clamp32(s_ctl.vel_i_q16 + di,
                                      CTRL_INTEGRAL_MAX << POSCTRL_Q16);
}

/* 保持态的出力闸门：静止就短路制动；轴停住且净输出还不够起转时先不驱动，让积分慢慢建起来。
 * 带负载(holding_load)时一律保留出力，否则静音后会滑落。 */
static int32_t PosCtrl_OutputGate(int32_t u_sat, int32_t e_pos, int32_t av_obs,
                                  uint8_t holding_load, int32_t u_spring)
{
    if (s_ctl.hm == HM_QUIET) return 0;   /* 静音=短路制动、不出力 */
    if (s_ctl.hm == HM_PROBE) return (int32_t)(((int64_t)s_ctl.u_prev * s_ctl.probe_q15) >> 15);
    if (s_ctl.state != (uint8_t)POSCTRL_HOLD || holding_load || s_ctl.hm == HM_LOADED) return u_sat;

    if (Fx_Abs32(e_pos) <= CTRL_POS_DEADBAND && av_obs <= CTRL_VEL_DB_CDPS) return 0;
    if (av_obs <= CTRL_VEL_DB_CDPS && Fx_Abs32(u_sat) < CTRL_NUDGE_PWM
        && u_spring == 0) return 0;   /* 还不够起转，先积累 */
    return u_sat;
}

/* 推进一拍，返回下发给H桥的PWM。无除法、无浮点；详见 docs/ALGORITHM_NOTES.md。 */
int16_t C_PosCtrl_Update(const TrajRef_t *ref, const SpeedObsOut_t *obs,
                         uint8_t traj_done, PosCtrlDbg_t *dbg)
{
    int32_t av_obs   = Fx_Abs32(obs->vel);
    int32_t av_meas  = C_Fric_MeasVel(obs->pos);
    int32_t e_pos, e_meas, e_pos_ctl, e_vel, e_vel_fb;
    int32_t w_corr, w_cmd;
    int32_t u_fric, u_vel, u_acc, u_fb, u_i, u_spring, u_unsat, u_sat, u_out;
    uint8_t decelerating, holding_load, ahead;

    PosCtrl_HoldStiffStep();

    /* 用扰动估计识别持续负载(只做判断不前馈)：带载时保留闭环和积分，防止静音后滑落 */
    holding_load = (uint8_t)(traj_done && CTRL_NUDGE_PWM > 0
                             && Fx_Abs32(obs->load) >= CTRL_FRIC_STATIC / 2);

    /* 1. 位置误差。运动控制用带预测的：对 obs-ref 的相对速度做外推，匀速吻合时附加项为0，
     *    超速时提前加制动。到位判据用不带预测的实际残差。 */
    e_meas = Fx_WrapDiff(ref->pos - obs->pos, CFG_WRAP_RANGE_CDEG);
    e_pos  = Fx_WrapDiff(e_meas - (int32_t)(((int64_t)(obs->vel - ref->vel) * CTRL_POS_LEAD_Q16) >> POSCTRL_Q16), CFG_WRAP_RANGE_CDEG);

    /* 2. 到位/静音锁存 */
    if (ref->vel != 0) s_ctl.dir = (int8_t)((ref->vel > 0) ? 1 : -1);
    PosCtrl_Latches(e_meas, obs->vel, traj_done);

    /* 3. 位置死区任何状态都生效：比最小可靠位移还小的残差去踹静摩擦必然窜过头，形成极限环 */
    e_pos_ctl = Fx_Deadband(e_pos, CTRL_POS_DEADBAND);

    /* 4. 位置P -> 速度修正，单独限幅：跟踪误差再大也不把速度指令推满 */
    w_corr = Fx_Clamp32((int32_t)(((int64_t)CTRL_KP_POS_Q8 * e_pos_ctl) >> 8), CTRL_WCORR_MAX);

    /* 5. 速度指令：参考速度按参考加速度提前 Tact 补执行延迟(只作用于前馈)，再加位置修正 */
    w_cmd = Fx_Clamp32(ref->vel
                            + (int32_t)(((int64_t)ref->acc * CTRL_PREVIEW_Q16) >> POSCTRL_Q16)
                            + w_corr, PLANT_VMAX_HW_CDPS);
    e_vel = (ref->vel + w_corr) - obs->vel;   /* 观测器速度无滞后，直接比 */

    /* 6. 减速段或轨迹已结束却仍有残余速度：速度环P改用制动侧增益 */
    decelerating = (uint8_t)(av_obs > PLANT_WCMD_MIN_CDPS
                             && (((int64_t)ref->acc * obs->vel < 0) || traj_done));

    /* 7. 前馈。摩擦前馈放到第9步(粘住时需先知道其余各项之和)，且不按HOLD关闭：末端挪动正需要它 */
    u_vel = (int32_t)(((int64_t)s_ctl.slope_q16 * w_cmd) >> POSCTRL_Q16);
    u_acc = PosCtrl_AccFF(ref, obs->vel);

    /* 8. 速度PI的P项。保持态给 e_vel 加噪声底死区(轴停住后 obs.vel 只剩量化噪声)，积分仍用原始值 */
    e_vel_fb = s_ctl.state == POSCTRL_HOLD ? Fx_Deadband(e_vel, CTRL_VEL_DB_CDPS) : e_vel;
    u_fb = (int32_t)(((int64_t)(decelerating ? s_ctl.kp_brake_q16 : s_ctl.kp_vel_q16)
                      * e_vel_fb) >> POSCTRL_Q16);

    u_spring = PosCtrl_Spring(e_meas, obs->vel);

    /* 9. 摩擦前馈与合成限幅 */
    u_i     = s_ctl.vel_i_q16 >> POSCTRL_Q16;
    ahead   = (uint8_t)((int64_t)e_meas * w_cmd < 0);   /* 实测残差与速度指令反号：轴已超到参考前面 */
    u_fric  = C_Fric_Update(s_ctl.hm != HM_ACTIVE ? 0 : w_cmd, av_meas, obs->pos,
                            u_vel + u_acc + u_fb + u_i + u_spring, ahead);
    u_unsat = u_fric + u_vel + u_acc + u_fb + u_i + u_spring;
    u_sat   = Fx_Clamp32(u_unsat, s_ctl.out_limit);

    /* 10. 积分与出力闸门 */
    PosCtrl_Integrate(e_pos, e_vel, ref->acc, av_obs, traj_done, holding_load,
                      u_spring, u_unsat, u_sat);
    u_out = PosCtrl_OutputGate(u_sat, e_pos, av_obs, holding_load, u_spring);
    if (s_ctl.hm == HM_ACTIVE) { s_ctl.u_prev = (int16_t)u_out; s_ctl.pushed = (uint8_t)(u_spring != 0); }

    /* 11. 遥测 */
    if (dbg != 0)
    {
#define DEBUG(field) dbg->field = field;
        DEBUG(e_pos) DEBUG(e_vel) DEBUG(w_corr) DEBUG(w_cmd)
        DEBUG(u_fric) DEBUG(u_vel) DEBUG(u_acc) DEBUG(u_fb) DEBUG(u_i) DEBUG(u_out)
#undef DEBUG
        dbg->state   = s_ctl.state;
        dbg->sat     = (uint8_t)(u_unsat != u_sat);
        dbg->braking = decelerating;
    }

    return (int16_t)(u_out * CAL_MOTOR_SIGN);
}
