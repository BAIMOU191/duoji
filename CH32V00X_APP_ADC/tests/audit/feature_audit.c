/*
 * feature_audit.c —— 本轮新增协议功能的行为审计
 *
 * 复用 servo_sim.c 的被控对象模型和生产代码本体，查的是**契约**而不是调参质量：
 *     脉冲边界(SMI/SMX)   越界指令必须整条不响应，不是夹到边界上
 *     角度校准(SCK/SCZ)   校准值必须生效；电位器版按物理极限收缩量程且不叠加
 *     多圈指令(N形式)     转够圈数、方向与普通指令一致、每圈时间恒定
 *     旋向反转开关        同一条指令的目标必须落在行程的另一头
 *
 * 每一项都同时验证"该生效的生效"和"该拒绝的拒绝"：只测前者的话，一个把所有
 * 指令都放行的实现照样能全绿。
 */
#define main sim_previous_main
#include "../sim/servo_sim.c"
#undef main

/* 走够拍数让轨迹跑完并停稳。多圈指令最长要转十圈，给的余量按最慢的算。 */
static void settle(int ms) { while (ms--) step(); }

/* 当前物理角度换算成行程坐标，用来判"轴到底转到哪了"。
 * 圆周坐标的零点存的是回绕值，直接相减会得到一个负的假坐标，必须折回来。 */
static double travel_diff(double a, double b);
static double travel_pos(void) {
    double p = plant_pos - Servo_Offset();
#if ENCODER_IS_CIRCULAR
    while (p < 0)            p += CDEG_RANGE;
    while (p >= CDEG_RANGE)  p -= CDEG_RANGE;
#endif
    return p;
}

/* 行程坐标下的最短差值。圆周坐标里 0 和 35999 只差 1 厘度，直接相减会差一整圈，
 * 端点附近的断言会因此假失败。 */
static double travel_diff(double a, double b) {
    double d = a - b;
#if ENCODER_IS_CIRCULAR
    while (d >  CDEG_RANGE / 2) d -= CDEG_RANGE;
    while (d < -CDEG_RANGE / 2) d += CDEG_RANGE;
#endif
    return d;
}

/* 某个脉宽实际会被采纳成什么目标：目标会被夹进轨迹范围(行程与可转入区的交集)。 */
static int expect_target(int pwm) {
    int a = target_of(pwm);
    if (a < s_servo.traj_lo) a = s_servo.traj_lo;
    if (a > s_servo.traj_hi) a = s_servo.traj_hi;
    return a;
}

/* ==================== 1. 可响应的脉冲窗口(SMI/SMX + 越界拒收) ==================== */

/* 协议量程之外的脉宽必须整条丢弃。夹到边界上会让配错的上位机看起来"指令生效了"。 */
static void check_pulse_range_rejected(void) {
    setup(SIM_MID, 0);
    A_Servo_Submit(2000, 0); settle(2500);
    int landed = s_servo.target_angle;

    A_Servo_Submit(2600, 0);  CHECK(s_servo.target_angle == landed); /* 高于量程 */
    A_Servo_Submit(400, 0);   CHECK(s_servo.target_angle == landed); /* 低于量程 */
    A_Servo_Submit(9999, 0);  CHECK(s_servo.target_angle == landed); /* 4位十进制上限 */
    A_Servo_Submit(0, 0);     CHECK(s_servo.target_angle == landed);
    /* 拒收不能留下副作用：紧接着的合法指令必须照常生效 */
    A_Servo_Submit(1000, 0);  CHECK(s_servo.target_angle == expect_target(1000));
}

/* SMI/SMX 把当前角度对应的脉宽设成边界，之后框外的指令一条都不响应。 */
static void check_pulse_limit(void) {
    setup(SIM_MID, 0);
    CHECK(A_Servo_SetMode(3));                 /* 180度模式，与需求描述一致 */

    /* 需求里的场景：在 50 度和 150 度两处分别框出边界。哪一头是"下界"由旋向
     * 决定——映射反过来时，角度大的那一头脉宽反而小——所以按脉宽大小分配 SMI/SMX，
     * 而不是按角度大小写死。 */
    int p50 = pulse_for(5000), p150 = pulse_for(15000);
    int lo_pwm = (p50 < p150) ? p50 : p150;
    int hi_pwm = (p50 < p150) ? p150 : p50;

    /* 存进去的必须**就是当前位置的脉宽**。不拿命令值做期望：轴未必真走到命令的
     * 位置(有死区的板子两端各有禁入余量)，而 SMI/SMX 标的是"此刻脚下"这个位置。 */
    A_Servo_Submit((uint16_t)lo_pwm, 0); settle(3000);
    int at_lo = A_Servo_GetPositionPwm();
    CHECK(A_Servo_SetPulseLimit(1));
    int lo = g_config.pulse_lo;
    CHECK(lo == at_lo);

    A_Servo_Submit((uint16_t)hi_pwm, 0); settle(3000);
    int at_hi = A_Servo_GetPositionPwm();
    CHECK(A_Servo_SetPulseLimit(0));
    int hi = g_config.pulse_hi;
    CHECK(hi == at_hi);
    CHECK(lo < hi);

    /* 框内照常响应 */
    A_Servo_Submit((uint16_t)((lo + hi) / 2), 0);
    CHECK(s_servo.target_angle == expect_target((lo + hi) / 2));
    int inside = s_servo.target_angle;

    /* 框外一条都不响应——目标一动不动，不是被夹到边界上 */
    A_Servo_Submit((uint16_t)(lo - 1), 0); CHECK(s_servo.target_angle == inside);
    A_Servo_Submit((uint16_t)(hi + 1), 0); CHECK(s_servo.target_angle == inside);
    A_Servo_Submit(SERVO_PWM_MIN, 0);      CHECK(s_servo.target_angle == inside);
    A_Servo_Submit(SERVO_PWM_MAX, 0);      CHECK(s_servo.target_angle == inside);
    /* 边界本身属于框内 */
    A_Servo_Submit((uint16_t)lo, 0); CHECK(s_servo.target_angle == expect_target(lo));
    A_Servo_Submit((uint16_t)hi, 0); CHECK(s_servo.target_angle == expect_target(hi));

    /* 两端不许交叉或重合：窗口一旦为空，舵机从此拒收一切运动指令，只能靠 CLE 救。
     * 把另一端就设在脚下这个位置，交叉判据必然成立，与轴到底停在哪无关。 */
    A_Servo_Submit((uint16_t)((lo + hi) / 2), 0); settle(3000);
    int here = A_Servo_GetPositionPwm();
    g_config.pulse_hi = (uint16_t)here;
    CHECK(!A_Servo_SetPulseLimit(1));           /* 下界要压到上界上，必须失败 */
    CHECK(g_config.pulse_lo == lo);             /* 失败不许留下半改的状态 */
    g_config.pulse_hi = (uint16_t)hi;
    g_config.pulse_lo = (uint16_t)here;
    CHECK(!A_Servo_SetPulseLimit(0));
    CHECK(g_config.pulse_hi == hi);
    g_config.pulse_lo = (uint16_t)lo;

    /* 窗口放回整个量程后必须立刻恢复响应(CLE 走的就是这条路，见 config_audit) */
    g_config.pulse_lo = SERVO_PWM_MIN; g_config.pulse_hi = SERVO_PWM_MAX;
    A_Servo_Submit(SERVO_PWM_MAX, 0);
    CHECK(s_servo.target_angle == expect_target(SERVO_PWM_MAX));
}

/* 上电动作不是上位机发的指令，被边界挡掉就成了"开不了机"，必须夹进边界照常执行。 */
static void check_boot_clamped_into_window(void) {
    setup(SIM_MID, 0);
    g_config.boot_mode   = SERVO_BOOT_GOTO_START;
    g_config.startup_pwm = SERVO_PWM_MAX;        /* 落在边界之外 */
    g_config.pulse_lo = 1200; g_config.pulse_hi = 1800;
    A_Servo_Init();
    CHECK(s_servo.target_angle == expect_target(1800)); /* 夹到上界而不是静默失效 */
}

/* ==================== 2. 角度校准(SCK/SCZ) ==================== */

/* SCK 的对外契约：校完之后脚下这个位置就是 1500us。 */
static void check_mid_calibration(void) {
    setup(SIM_MID, 0);
    plant_pos = SIM_MID - 4000;
    CHECK(A_Servo_CalibrateMid());
    CHECK(labs((long)A_Servo_GetPositionPwm() - SERVO_PWM_MID) <= 2);
    CHECK(fabs(travel_diff(travel_pos(), s_servo.span_cdeg / 2)) <= 3);

    /* 校准后必须还能正常动，而且 1500us 就回到校准点 */
    A_Servo_Submit(pulse_for(s_servo.span_cdeg / 4), 0); settle(4000);
    A_Servo_Submit(SERVO_PWM_MID, 0); settle(4000);
    CHECK(fabs(plant_pos - (SIM_MID - 4000)) < 120);   /* 中点不受端点余量影响 */
}

/* SCZ 的对外契约：校完之后脚下这个位置就是 500us 对应的那个"0度"。
 * 反向模式下 500us 落在行程高端，行程坐标因此是 span 而不是 0 —— 两种方向都要对。 */
static void check_zero_calibration(void) {
    for (int mode = 1; mode <= 2; mode++) {          /* 1=正向 2=反向 */
        setup(SIM_MID, 0);
        CHECK(A_Servo_SetMode((uint8_t)mode));
        plant_pos = SIM_MID;
        CHECK(A_Servo_CalibrateZero());
        CHECK(labs((long)A_Servo_GetPositionPwm() - SERVO_PWM_MIN) <= 2);
        CHECK(fabs(travel_diff(travel_pos(),
                   s_servo.reverse ? s_servo.span_cdeg : 0)) <= 3);
        /* 500us 必须能回到校准点 */
        A_Servo_Submit(pulse_for(s_servo.span_cdeg / 2), 0); settle(4000);
        /* 500us 必须能回到校准点 */
        A_Servo_Submit(SERVO_PWM_MIN, 0); settle(6000);
        CHECK(fabs(plant_pos - SIM_MID) < 150);
    }
}

/* 行程两端必须可达：P500/P2500 停止误差不超过5us，且停稳后不再通电。
 * 电位器版曾在两端再收2度禁入余量，270度模式下 P500 实际停在 P517。 */
static void check_endpoint_reach(void) {
    static const uint8_t modes[] = { SERVO_MODE_270_CW, SERVO_MODE_180_CW };
    noise = SIM_NOISE_CDEG;
    for (size_t m = 0; m < 2; m++)
    for (int end = 0; end < 2; end++) {
        int pwm = end ? SERVO_PWM_MAX : SERVO_PWM_MIN, nz = 0;
        setup(SIM_MID, 0);
        CHECK(A_Servo_SetMode(modes[m]));
        A_Servo_Submit((uint16_t)pwm, 0); settle(3000);
        for (int k = 0; k < 500; k++) { step(); if (applied != 0) nz++; }
        CHECK(labs((long)A_Servo_GetPositionPwm() - pwm) <= 5);
        CHECK(fabs(travel_diff(travel_pos(), target_of(pwm))) <= 5.0 * s_servo.span_cdeg / 2000.0);
        CHECK(nz == 0);
    }
    noise = 0;
}

/* 保持刚度：到位后被手推开要顶得住、松手不反弹、推完和带载时都不能抖。
 * 只在开了保持弹簧(TUNE_HOLD_STIFF>0)的工程里有意义，关闭时直接通过。 */
static void check_hold_stiff(void) {
#if TUNE_HOLD_STIFF > 0
    noise = SIM_NOISE_CDEG;
    for (int dir = -1; dir <= 1; dir += 2) {
        setup(SIM_MID, 0); A_Servo_Submit(1500, 0); settle(3000);
        double p0 = plant_pos, defl = 0, over = 0; int nz = 0, kicks = 0, prev = applied;
        for (int k = 0; k < 4100; k++) {
            load_pwm = k < 400 ? dir * 800.0 * k / 400 : k < 2400 ? dir * 800.0
                     : k < 2600 ? dir * 800.0 * (2600 - k) / 200 : 0;
            step();
            double d = -dir * (plant_pos - p0);
            if (k < 2400 && d > defl) defl = d;
            if (k >= 2600 && -d > over) over = -d;
            if (k >= 3600) { if (applied) nz++; if (applied && !prev) kicks++; }
            prev = applied;
        }
        CHECK(defl < 200);          /* 现状约255厘度，保持弹簧后约120 */
        CHECK(over < 50);           /* 现状松手反冲约320厘度 */
        CHECK(nz == 0 && kicks == 0);

        /* 持续负载：顶住之后不能周期性地滑出去再顶回来 */
        setup(SIM_MID, 0); A_Servo_Submit(1500, 0); settle(3000);
        p0 = plant_pos; kicks = 0; prev = applied;
        int slips = 0, moving = 0;
        for (int k = 0; k < 6000; k++) {
            load_pwm = k < 300 ? dir * 500.0 * k / 300 : dir * 500.0;
            step();
            if (k >= 2000) {
                if (applied && !prev) kicks++;
                if (fabs(plant_vel) > 30 && !moving) { slips++; moving = 1; } else if (fabs(plant_vel) < 5) moving = 0;
            }
            prev = applied;
        }
        load_pwm = 0;
        CHECK(kicks == 0 && slips == 0);
        CHECK(fabs(plant_pos - p0) < 150);

        /* 空载长时间保持完全不出力 */
        setup(SIM_MID, 0); A_Servo_Submit(1500, 0); settle(3000);
        nz = 0; for (int k = 0; k < 5000; k++) { step(); if (applied) nz++; }
        CHECK(nz == 0);
    }
    noise = 0;
#endif
}

/* 反复校准不叠加，换模式后按新模式的标称量程重推。 */
static void check_calibration_not_cumulative(void) {
    setup(SIM_MID, 0);
    plant_pos = 5000;  CHECK(A_Servo_CalibrateMid());
    int span_at_5000 = s_servo.span_cdeg;

    /* 再在别处校一次，基准仍是模式标称量程而不是上一次收缩后的行程 */
    plant_pos = 10000; CHECK(A_Servo_CalibrateMid());
    int span_at_10000 = s_servo.span_cdeg;

    /* 回到第一个点重校，必须复现第一次的结果——叠加的实现这里会一次比一次小 */
    plant_pos = 5000;  CHECK(A_Servo_CalibrateMid());
    CHECK(s_servo.span_cdeg == span_at_5000);
    CHECK(span_at_10000 >= span_at_5000);

    /* 换模式后重推：180度模式的量程不可能超过标称的18000 */
    plant_pos = 10000; CHECK(A_Servo_CalibrateMid());
    CHECK(A_Servo_SetMode(3));
    CHECK(s_servo.span_cdeg <= 18000);
    CHECK(s_servo.span_cdeg >= SERVO_CUSTOM_SPAN_MIN);
    CHECK(A_Servo_SetMode(1));
    CHECK(s_servo.span_cdeg == span_at_10000);   /* 切回来必须一模一样 */
}

/* 从没校准过时，标称量程该摆在可转入区的什么位置。
 * 有死区的编码器上这是个真问题：180度模式的量程只占可转入区的三分之二，靠在
 * 低端就意味着上面90度永远用不到、下面一点余量也没有。整圈可测的编码器没有
 * "物理中心"可言，沿用编码器零点。 */
static void check_default_placement(void) {
    setup(SIM_MID, 0);
    CHECK(A_Servo_SetMode(1));                       /* 270度：量程与可转入区等长 */
    CHECK(s_servo.span_cdeg == 27000);
    CHECK(Servo_Offset() == 0);

    CHECK(A_Servo_SetMode(3));                       /* 180度 */
    CHECK(s_servo.span_cdeg == 18000);
#if ENCODER_IS_CIRCULAR
    CHECK(Servo_Offset() == 0);                      /* 整圈可测：零点即编码器零点 */
#else
    CHECK(Servo_Offset() == 4500);                   /* 居中：45~225度，中点135度 */
    /* 1500us 必须正好落在机构中点上 */
    CHECK(labs((long)(target_of(SERVO_PWM_MID) + Servo_Offset())
               - (ENCODER_TRAVEL_LO + ENCODER_TRAVEL_HI) / 2) <= 2);
    /* 两端物理余量还剩45度，禁入余量不该再从逻辑行程里砍：整整180度都要能用到 */
    CHECK(s_servo.traj_lo == 0 && s_servo.traj_hi == 18000);
    A_Servo_Submit(SERVO_PWM_MIN, 0); settle(8000);
    CHECK(fabs(plant_pos - 4500) < 150);             /* 500us -> 物理45度 */
    A_Servo_Submit(SERVO_PWM_MAX, 0); settle(8000);
    CHECK(fabs(plant_pos - 22500) < 150);            /* 2500us -> 物理225度 */
    A_Servo_Submit(SERVO_PWM_MID, 0); settle(8000);
    CHECK(fabs(plant_pos - 13500) < 150);            /* 1500us -> 物理135度 */
#endif

    /* 反向模式必须落在同一段行程上，只是两端对调 */
    CHECK(A_Servo_SetMode(4));
    CHECK(s_servo.span_cdeg == 18000);
    CHECK(Servo_Offset() == (ENCODER_IS_CIRCULAR ? 0 : 4500));

    /* 270<->180 来回切，回到原样，不会一次比一次偏 */
    CHECK(A_Servo_SetMode(1)); CHECK(Servo_Offset() == 0);
    CHECK(A_Servo_SetMode(3));
    CHECK(Servo_Offset() == (ENCODER_IS_CIRCULAR ? 0 : 4500));

    /* 校准过之后就以锚点为准，不再套默认居中 */
    plant_pos = 10000; CHECK(A_Servo_CalibrateMid());
    CHECK(labs((long)(Servo_Offset() + s_servo.span_cdeg / 2) - 10000) <= 2);
}

#if !ENCODER_IS_CIRCULAR
/* 电位器版的量程被物理卡死在 [0,270]度，偏心校准必须对称收缩量程让校准值生效，
 * 而不是拒绝校准。需求里给的两个例子直接写成断言。 */
static void check_pot_shrink(void) {
    setup(SIM_MID, 0);

    /* 50度处校中位 -> 行程 0~100度 */
    plant_pos = 5000; CHECK(A_Servo_CalibrateMid());
    CHECK(s_servo.span_cdeg == 10000);
    CHECK(Servo_Offset() == 0);

    /* 150度处校中位 -> 行程 30~270度 */
    plant_pos = 15000; CHECK(A_Servo_CalibrateMid());
    CHECK(s_servo.span_cdeg == 24000);
    CHECK(Servo_Offset() == 3000);

    /* 正中心校准 -> 拿满标称量程 */
    plant_pos = 13500; CHECK(A_Servo_CalibrateMid());
    CHECK(s_servo.span_cdeg == 27000);

    /* 贴死在物理端点上推不出可用行程：必须拒绝，并且不留下半改的状态 */
    plant_pos = 13500; CHECK(A_Servo_CalibrateMid());
    int good_span = s_servo.span_cdeg, good_off = Servo_Offset();
    plant_pos = 2;    CHECK(!A_Servo_CalibrateMid());
    CHECK(s_servo.span_cdeg == good_span && Servo_Offset() == good_off);

    /* 外扩的安全余量禁止用作锚点：那一段读数虽真，但不是可转入区域 */
    plant_pos = -500; CHECK(!A_Servo_CalibrateMid());
    CHECK(s_servo.span_cdeg == good_span && Servo_Offset() == good_off);

    /* 轨迹范围同样不许伸进外扩段 */
    CHECK((int32_t)s_servo.traj_lo + Servo_Offset() >= ENCODER_TRAVEL_LO);
    CHECK((int32_t)s_servo.traj_hi + Servo_Offset() <= ENCODER_TRAVEL_HI);
}
#endif

/* ==================== 3. 多圈指令 ==================== */

#if SERVO_MULTITURN
/* 转够圈数、落在目标上，且方向与同目标的普通指令一致。
 * 位移直接用未回绕的 plant_pos 做差：绕了几圈只有它数得清，回绕坐标做差会把
 * 每一圈都算成0。 */
static void check_multiturn_distance(void) {
    for (int turns = 0; turns <= 3; turns++) {
        setup(SIM_MID, 0);
        double p0 = plant_pos;        /* 物理起点，未回绕 */
        double t0 = travel_pos();     /* 同一时刻的行程坐标 */
        int pwm = pulse_for(20000);
        double plain_dir = (target_of(pwm) > t0) ? 1 : -1;

        A_Servo_SubmitTurns((uint16_t)pwm, (uint8_t)turns, 0);
        settle(20000);

        double moved = plant_pos - p0;
        double want  = plain_dir * (turns * 36000.0
                                    + fabs(target_of(pwm) - t0));
        CHECK(fabs(moved - want) < 200);          /* 圈数走够 */
        CHECK(plain_dir * moved > 0);             /* 方向与普通指令一致 */
        CHECK(C_Traj_Is_Done() && !s_servo.mt_active);
        CHECK(applied == 0);                      /* 停稳后不再通电 */
        /* 收尾后参考必须已经折回行程坐标，否则下一条普通指令会差好几圈 */
        CHECK(labs((long)s_servo.ref.pos - (long)s_servo.angle) <= 60);
        CHECK(s_servo.target_angle == target_of(pwm));
    }
}

/* T 是"每转一圈的时间"：总时长必须随圈数成比例增长，而不是固定不变。
 * 一圈的物理最快时间约 36000/TRAJ_VMAX_CDPS 秒，所以请求值必须挑得比它慢，
 * 否则测到的是"顶着物理极限"而不是"按请求的角速度走"。 */
static void check_multiturn_timing(void) {
    const int fastest = (int)(36000L * 1000 / TRAJ_VMAX_CDPS); /* 单圈物理下限(ms) */
    const int per_turn = 3 * fastest / 2;                      /* 留 50% 余量的请求值 */
    int t_one, t_three, pwm;

    setup(SIM_MID, 0);
    pwm = pulse_for(13500);       /* 目标就在起点，位移纯粹由圈数贡献 */
    A_Servo_SubmitTurns((uint16_t)pwm, 1, (uint16_t)per_turn);
    t_one = 0; while (t_one < 60000 && !(C_Traj_Is_Done() && !s_servo.mt_active))
                   { step(); t_one++; }

    setup(SIM_MID, 0);
    A_Servo_SubmitTurns((uint16_t)pwm, 3, (uint16_t)per_turn);
    t_three = 0; while (t_three < 60000 && !(C_Traj_Is_Done() && !s_servo.mt_active))
                     { step(); t_three++; }

    CHECK(abs(t_one - per_turn) <= per_turn / 20);            /* 一圈 = 请求值 */
    CHECK(abs(t_three - 3 * per_turn) <= 3 * per_turn / 20);  /* 三圈 = 3倍 */
    CHECK(t_three > 2 * t_one);          /* 按圈计时，不是"总时长固定" */

    /* N0 形式：T 仍是"每圈时间"，四分之一圈就该只花四分之一 */
    setup(SIM_MID, 0);
    A_Servo_SubmitTurns((uint16_t)pulse_for(SIM_MID - 9000), 0, (uint16_t)(4 * per_turn));
    int t_part = 0;
    while (t_part < 60000 && !(C_Traj_Is_Done() && !s_servo.mt_active))
        { step(); t_part++; }
    CHECK(abs(t_part - per_turn) <= per_turn / 10);

    /* 请求得比机器能跑的还快：退化成最大速度，而不是报错或跑飞 */
    setup(SIM_MID, 0);
    A_Servo_SubmitTurns((uint16_t)pwm, 1, 1);
    int t_cap = 0; while (t_cap < 60000 && !(C_Traj_Is_Done() && !s_servo.mt_active))
                       { step(); t_cap++; }
    CHECK(t_cap >= fastest && t_cap < fastest + 400);

    /* T=0 同样是"能多快多快"，两者应当一致 */
    setup(SIM_MID, 0);
    A_Servo_SubmitTurns((uint16_t)pwm, 1, 0);
    int t_fast = 0; while (t_fast < 60000 && !(C_Traj_Is_Done() && !s_servo.mt_active))
                        { step(); t_fast++; }
    CHECK(abs(t_fast - t_cap) <= 5);
    CHECK(t_fast < t_one);
}

/* 暂停/继续不能丢圈数，也不能改角速度。 */
static void check_multiturn_pause(void) {
    setup(SIM_MID, 0);
    double p0 = plant_pos;
    int pwm = pulse_for(13500);

    A_Servo_SubmitTurns((uint16_t)pwm, 2, 1000);
    settle(700);
    A_Servo_Pause();
    settle(400);                               /* 先让它刹住，再量"停住了没有" */
    double at_pause = plant_pos;
    settle(500);
    CHECK(fabs(plant_pos - at_pause) < 100);   /* 暂停期间不再走 */
    CHECK(s_servo.mt_active);                           /* 圈数还记着 */
    A_Servo_Resume();
    settle(8000);
    CHECK(fabs((plant_pos - p0) - 2 * 36000.0) < 250);
    CHECK(!s_servo.mt_active);
}

/* 普通指令必须能打断多圈，且打断后坐标系要换回行程坐标。 */
static void check_multiturn_interrupt(void) {
    setup(SIM_MID, 0);
    A_Servo_SubmitTurns((uint16_t)pulse_for(13500), 5, 1000);
    settle(1500);
    CHECK(s_servo.mt_active);

    int pwm = pulse_for(6000);
    A_Servo_Submit((uint16_t)pwm, 0);
    CHECK(!s_servo.mt_active);
    settle(6000);
    CHECK(s_servo.target_angle == target_of(pwm));
    CHECK(fabs(travel_diff(travel_pos(), target_of(pwm))) < 150); /* 真的去了新目标 */
    CHECK(labs((long)s_servo.ref.pos - (long)s_servo.angle) <= 60);

    /* 打断后再发"去多圈指令原来那个目标"，不能被目标死区当成"没变化"吃掉 */
    A_Servo_SubmitTurns((uint16_t)pulse_for(13500), 2, 800);
    settle(1500);
    CHECK(s_servo.mt_active);
    A_Servo_Submit((uint16_t)pulse_for(13500), 0);
    settle(8000);
    CHECK(fabs(travel_diff(travel_pos(), 13500)) < 150);

    /* DST 同样要收回坐标系 */
    A_Servo_SubmitTurns((uint16_t)pulse_for(13500), 3, 1000);
    settle(1200);
    A_Servo_Stop();
    CHECK(!s_servo.mt_active);
    CHECK(labs((long)s_servo.ref.pos - (long)s_servo.angle) <= 60);
    settle(1000);
    CHECK(applied == 0 || fabs(plant_vel) < 500);
}

/* 圈数上限、越界脉宽、卸力/故障状态下的拒收。 */
static void check_multiturn_rejects(void) {
    setup(SIM_MID, 0);
    A_Servo_SubmitTurns((uint16_t)pulse_for(13500), 10, 1000); /* N>9 */
    CHECK(!s_servo.mt_active);
    A_Servo_SubmitTurns(2600, 1, 1000);                        /* 脉宽越界 */
    CHECK(!s_servo.mt_active);

    g_config.pulse_lo = 1200; g_config.pulse_hi = 1800;
    A_Servo_SubmitTurns(1000, 1, 1000);                        /* 脉冲边界之外 */
    CHECK(!s_servo.mt_active);
    g_config.pulse_lo = SERVO_PWM_MIN; g_config.pulse_hi = SERVO_PWM_MAX;

    simulated_fault = 1;
    A_Servo_SubmitTurns((uint16_t)pulse_for(13500), 1, 1000);
    CHECK(!s_servo.mt_active);
    simulated_fault = 0;

    /* 电机模式没有位置概念，多圈无从谈起 */
    int pwm = pulse_for(13500);   /* 换模式前先算好：电机模式下没有行程可换算 */
    CHECK(A_Servo_SetMode(7));
    A_Servo_SubmitTurns((uint16_t)pwm, 1, 1000);
    CHECK(!s_servo.mt_active);
}

/* 极端参数：9圈 x 9999ms 要 90 秒，超出规划器16位时间参数，必须分段续跑而不是
 * 时间被截断或干脆跑飞。这里只查"确实分了段且圈数走够"，不苛求段间那一下速度归零。 */
static void check_multiturn_longest(void) {
    setup(SIM_MID, 0);
    double p0 = plant_pos;
    A_Servo_SubmitTurns((uint16_t)pulse_for(13500), 9, 9999);
    CHECK(!s_servo.mt_final);                   /* 第一段一定不是最后一段 */

    int ms = 0;
    while (ms < 200000 && !(C_Traj_Is_Done() && !s_servo.mt_active)) { step(); ms++; }
    CHECK(!s_servo.mt_active);
    CHECK(fabs((plant_pos - p0) - 9 * 36000.0) < 400);
    CHECK(ms > 60000);                          /* 没有被16位时间参数截短 */
}
#endif /* SERVO_MULTITURN */

/* ==================== 4. 整机旋向 ==================== */

/* CFG_DIR_INVERT 只该翻转"脉宽对应哪一头"，不该动闭环：两种设置下舵机都必须
 * 稳稳到位，区别只在同一条指令落在行程的哪一端。 */
static void check_direction(void) {
    setup(SIM_MID, 0);
    CHECK(s_servo.reverse == (uint8_t)(CFG_DIR_INVERT ? 1 : 0)); /* 模式1的方向 */
    A_Servo_Submit(SERVO_PWM_MAX, 0); settle(6000);
    CHECK(fabs(travel_diff(travel_pos(), expect_target(SERVO_PWM_MAX))) < 150);
    A_Servo_Submit(SERVO_PWM_MIN, 0); settle(6000);
    CHECK(fabs(travel_diff(travel_pos(), expect_target(SERVO_PWM_MIN))) < 150);
    /* 方向本身：最大脉宽必须落在行程的哪一头 */
    CHECK((expect_target(SERVO_PWM_MAX) < expect_target(SERVO_PWM_MIN))
          == (CFG_DIR_INVERT ? 1 : 0));

    /* 模式2 必须与模式1 相反 */
    setup(SIM_MID, 0); CHECK(A_Servo_SetMode(2));
    CHECK(s_servo.reverse == (uint8_t)(CFG_DIR_INVERT ? 0 : 1));

    /* 自定义行程存的是未翻转的原始方向：标定完再读回来，方向不能自己变 */
    setup(SIM_MID, 0);
    plant_pos = SIM_MID - 4000; CHECK(A_Servo_SetTravelEnd(1));
    uint8_t rev = s_servo.reverse;
    A_Servo_ApplyConfig();
    CHECK(s_servo.reverse == rev);
    CHECK(g_config.custom_reverse == (uint8_t)(rev ^ CFG_DIR_INVERT));
}

int main(int argc, char **argv) {
    noise = 0;
    if (argc < 2) return 2;
    if      (!strcmp(argv[1], "pulse_range"))  check_pulse_range_rejected();
    else if (!strcmp(argv[1], "pulse_limit"))  check_pulse_limit();
    else if (!strcmp(argv[1], "boot_window"))  check_boot_clamped_into_window();
    else if (!strcmp(argv[1], "endpoint"))     check_endpoint_reach();
    else if (!strcmp(argv[1], "hold_stiff"))   check_hold_stiff();
    else if (!strcmp(argv[1], "cal_mid"))      check_mid_calibration();
    else if (!strcmp(argv[1], "cal_zero"))     check_zero_calibration();
    else if (!strcmp(argv[1], "cal_repeat"))   check_calibration_not_cumulative();
    else if (!strcmp(argv[1], "cal_default"))  check_default_placement();
#if !ENCODER_IS_CIRCULAR
    else if (!strcmp(argv[1], "cal_shrink"))   check_pot_shrink();
#endif
#if SERVO_MULTITURN
    else if (!strcmp(argv[1], "mt_distance"))  check_multiturn_distance();
    else if (!strcmp(argv[1], "mt_timing"))    check_multiturn_timing();
    else if (!strcmp(argv[1], "mt_pause"))     check_multiturn_pause();
    else if (!strcmp(argv[1], "mt_interrupt")) check_multiturn_interrupt();
    else if (!strcmp(argv[1], "mt_reject"))    check_multiturn_rejects();
    else if (!strcmp(argv[1], "mt_longest"))   check_multiturn_longest();
#endif
    else if (!strcmp(argv[1], "direction"))    check_direction();
    else return 2;
    printf("%s failures=%d\n", argv[1], failures); return failures ? 1 : 0;
}
