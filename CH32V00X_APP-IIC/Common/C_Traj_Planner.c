/* C_Traj_Planner.c 在线轨迹规划器(运动学限幅型)，无硬件依赖 */

#include "C_Traj_Planner.h"

#define TRAJ_VEL_SHIFT   16U  /* Q16：内部速度(厘度/拍)定点小数位数 */
#define TRAJ_U_SHIFT     15U  /* Q15：段内归一化时间u及其幂次的定点小数位数 */
#define TRAJ_INV_SHIFT   30U  /* Q30：1/段长 的定点小数位数 */
#define TRAJ_W_SHIFT      8U  /* 解二次方程时把Q16速度降到Q8，防止平方溢出 */

/* 恒加速度段：t=|dv|/amax。 */
static int32_t Traj_K_Linear_Q32(int32_t amax_cdpss)
{
    return (int32_t)(((int64_t)CFG_TICK_HZ * CFG_TICK_HZ * 65536) /
                     amax_cdpss);
}

/*
 * ============ 加速度上限的自动推导 ============
 * "硬件最大加速度"不是常数：维持速度v本身要吃掉 v/Kv 的PWM，剩下的才能
 * 用于加速。该关系是**精确线性**的：
 *     a(v) = (PWM_MAX - coulomb - v/Kv) * Ka = a0 * (1 - v/v_ss)
 * 其中 a0 = v_ss/tau 是零速时的加速能力，v=v_ss 时归零。
 *
 * 恒加速度的参考只能取一个值，取哪个？实测扫描(tests/servo_sim.c 全场景
 * 平均整定时间)给出一个很浅但明确的最优：
 *     a=5648(=a0)   1163ms  饱和713ms   参考冲太前，巡航段只能以
 *                                       (v_ss-vmax)=3°/s 追赶，误差追不回来
 *     a=4000        1029ms  饱和536ms
 *     a=2500        1024ms  饱和279ms   <- 最优区
 *     a=1800        1040ms  饱和202ms
 *     a=1350(整段平均) 1048ms 饱和184ms  参考太保守，低速段把电机压住了
 * 偏高的惩罚远大于偏低，因为一旦攒下位置误差，饱和的巡航段追不回来。
 *
 * 最优值对应的正是**速度中点的可用加速度** a(vmax/2) = a0*(1-f/2)：
 * 恒加速度参考在前半段略低于电机能力、后半段略高，两边正好抵消。
 * 取整段平均(a0*f/(-ln(1-f)))是错的——那个值低估了前半段能出的力。
 *
 * vp 与 amax 互相依赖，但不动点有闭式解：把 D=k*vp^2、k=(1+lam/2)/a、
 * a=a0*(1-f/2) 消元得
 *     D = (1 + lam/2) * tau * v_ss * H(f),     H(f) = f^2/(1-f/2)
 * H 单调递增，反查一次即得 f，不需要迭代。
 *
 * 两张表都是纯数学函数，与具体舵机无关；变的只有 plant_vss_cdps 和
 * plant_tau_ms 两个标定值。
 */
#define TRAJ_TBL_N  12U
static const int32_t TRAJ_F_Q15[TRAJ_TBL_N] = {   /* f = vp/v_ss，Q15 */
     1638,  3277,  6554,  9830, 13107, 16384, 19661, 22938, 26214, 29491, 31130, 32112 };
static const int32_t TRAJ_H_Q12[TRAJ_TBL_N] = {   /* H(f) = f^2/(1-f/2)，Q12 */
       11,    43,   182,   434,   819,  1365,  2107,  3088,  4369,  6032,  7041,  7713 };
static const int32_t TRAJ_G_Q15[TRAJ_TBL_N] = {   /* g(f) = 1-f/2，Q15 */
    31949, 31130, 29491, 27853, 26214, 24576, 22938, 21299, 19661, 18022, 17203, 16712 };

/*
 * @fn      Traj_TblLerp
 * @brief   在单调表 xs[] 上反查 x，返回对应的 ys[] 插值结果
 * @return  x 超出表范围时钳在端点
 */
static int32_t Traj_TblLerp(int32_t x, const int32_t *xs, const int32_t *ys)
{
    uint8_t i;

    if (x <= xs[0]) return ys[0];
    if (x >= xs[TRAJ_TBL_N - 1U]) return ys[TRAJ_TBL_N - 1U];
    for (i = 1U; i < TRAJ_TBL_N; i++)
    {
        if (x <= xs[i])
        {
            int32_t span = xs[i] - xs[i - 1U];

            if (span <= 0) return ys[i];
            return ys[i - 1U] + (int32_t)(((int64_t)(ys[i] - ys[i - 1U])
                                         * (x - xs[i - 1U])) / span);
        }
    }
    return ys[TRAJ_TBL_N - 1U];
}

/*
 * @fn      Traj_AutoAmax
 * @brief   按本段行程与平滑度算出该用的加速度上限(厘度/秒^2)
 * @param   D       本段行程(厘度)，>=0
 * @param   shape_q15 S曲线段时长倍率，Q15，范围1.0~2.0
 * @param   vmax    本段允许的速度上限(厘度/秒)
 * @return  加速度上限；对象常数缺失时退回配置里的固定值
 *
 * 短行程根本爬不到高速区，于是自动拿到更大的加速度：实测 5° 移动得到
 * 3534°/s²，而 90° 移动得到 1425°/s²，都是各自峰值速度下的硬件极限。
 */
static int32_t Traj_AutoAmax(int32_t D, int32_t shape_q15, int32_t vmax, int32_t v0_cdps)
{
    int64_t denom;
    int32_t h_q12, f_q15, g_q15, f_cap, f_now;

    if (PLANT_VSS_CDPS <= 0 || PLANT_TAU_MS == 0U
        || PLANT_A0_CDPSS <= 0)
        return TRAJ_AMAX_ACC_CDPSS;      /* 未标定：退回固定值 */

    /* H = D*1000 / (shape * tau_ms * v_ss)，取Q12。shape 是标准
     * jerk 受限速度过渡相对恒加速度斜坡的时长倍率，范围 1..2。 */
    denom = ((int64_t)shape_q15 * PLANT_TAU_MS
             * PLANT_VSS_CDPS) >> 15;
    if (denom <= 0) return TRAJ_AMAX_ACC_CDPSS;
    h_q12 = (int32_t)(((int64_t)D * 1000 * 4096) / denom);

    f_q15 = Traj_TblLerp(h_q12, TRAJ_H_Q12, TRAJ_F_Q15);

    /* 峰值速度还受本段速度上限约束，取两者较小的那个来查 g */
    f_cap = (int32_t)(((int64_t)vmax << 15) / PLANT_VSS_CDPS);
    if (f_q15 > f_cap) f_q15 = f_cap;

    /* **起点速度也要算进来**。上式只由行程反推峰值速度，隐含假设"从静止起步"。
     * 在线重规划(云台跟随、运动中改目标)时轴已经在跑，剩余行程可能很短，
     * 于是查出一个很小的 f、进而给出一个接近 a0 的加速度上限——可实际上
     * 此刻电机的反电势已经吃掉了大部分PWM，根本出不了这么大的加速度。
     * 结果是参考跑在前面、误差累积、输出饱和。取"由行程推出的"和"当前已有的"
     * 两者中较大的速度去查可用加速度，才是这一段真正的工作点。 */
    f_now = (int32_t)(((int64_t)v0_cdps << 15) / PLANT_VSS_CDPS);
    if (f_now > 32767) f_now = 32767;
    if (f_q15 < f_now) f_q15 = f_now;

    g_q15 = Traj_TblLerp(f_q15, TRAJ_F_Q15, TRAJ_G_Q15);

    /* 本函数只回答一件事：**这段硬件最大能出多少加速度**。
     * "想要多柔"完全由平滑度对应的段时长倍率 peak=1/(1-r) 负责，
     * 不在这里重复压低加速度上限。旧版让同一个旋钮在两处相乘，
     * 使文档中的时长倍率与实际结果不一致，也让调参效果难以预测。
     * 也让两处的效果相乘、难以预测。 */
    return (int32_t)(((int64_t)PLANT_A0_CDPSS * g_q15) >> 15);
}

/*
 * @fn      Traj_VelCap
 * @brief   位移相关的速度上限(厘度/秒)
 *
 * **只限速度，不限加速度。** 小角度不需要高峰速——三角形轨迹的峰速是
 * sqrt(a*D)，1度移动在满加速下能甩到 6000 厘度/秒，起停就是这么抖出来的；
 * 但它**需要满加速**，否则大角度连续反转会跟不上。
 *
 * 取 v = D / T_move：位移以内的移动一律花 T_move 毫秒，等时、可预期。
 * 位移大到 v 触顶 vmax 之后本限幅自动失效，大角度完全按硬件极限跑。
 * 交叉点 D = vmax * T_move（当前约 9.3 度）就是"多大角度开始满性能"。
 *
 * 旧实现用一个 smoothstep 把**速度和加速度一起**按 D 缩放，两个副作用：
 *   1) 10 度移动只剩 39% 的加速度，大角度反转发木；
 *   2) span 和 floor 两个旋钮互相耦合，没有单独的物理含义，只能靠试。
 * 现在换成两条各自独立、各有物理判据的上限。
 */
static int32_t Traj_VelCap(int32_t D)
{
    int32_t t = TRAJ_MOVE_MIN_MS;
    int32_t v;

    if (t <= 0) return TRAJ_VMAX_CDPS;          /* 0 = 关闭本机制 */
    if (D <= 0) return TRAJ_VMAX_CDPS;

    v = (int32_t)(((int64_t)D * 1000) / t);     /* 厘度/毫秒 -> 厘度/秒 */
    if (v > TRAJ_VMAX_CDPS) v = TRAJ_VMAX_CDPS;
    return (v > 0) ? v : 1;
}

/*
 * @fn      Traj_AccelCeil
 * @brief   由速度上限推出的加速度上限(厘度/秒^2)
 *
 * 判据：加速段至少要持续 TRAJ_ACCEL_MIN_MS(=3L，由实测纯延迟派生)。
 * 加速指令比纯延迟还短的话，PWM 尖峰在轴开始响应之前就已经结束，
 * 参考是"物理上跟不了"的，反馈只能事后补——小角度抖动的另一个来源。
 *
 * 大位移时 v_cap = vmax，本式给出的上限远高于 TRAJ_AMAX_ACC_CDPSS，
 * 于是自动失效，加速度回到硬件极限。
 */
static int32_t Traj_AccelCeil(int32_t v_cap)
{
    int32_t a = (int32_t)(((int64_t)v_cap * 1000) / TRAJ_ACCEL_MIN_MS);

    return (a > 0) ? a : 1;
}

/* 规划器内部状态：板卡仅单实例，封装在本文件内，不对外暴露。 */
static struct {
    /* 指令状态：重规划时作为新段起点，保证位置/速度连续 */
    int32_t pos;         /* 当前参考位置，厘度 */
    int32_t vel_qt;      /* 当前参考速度，Q16 厘度/拍 */

    /* 平滑度换算结果，仅在C_Traj_Init初始化一次。r 是每个速度过渡段两端
     * 的 jerk 斜坡占比：0=恒加速度，0.5=三角加速度；peak=1/(1-r)。 */
    int32_t k_acc_q32;   /* 加速段：过渡时长/速度差 */
    int32_t k_dec_q32;   /* 减速段：过渡时长/速度差 */
    int32_t k_rev_q32;   /* max(k_acc,k_dec)：第一段需要先刹车/换向时用，
                          * 取更大的k即取更小的加速度，同时满足两个上限 */
    int32_t r_acc_q15;
    int32_t r_dec_q15;
    int32_t r_rev_q15;       /* 刹车/换向段：取两侧较柔的一档 */
    int32_t peak_acc_q15;    /* 1/(1-r)，也是段时长倍率，Q15 */
    int32_t peak_dec_q15;
    int32_t peak_rev_q15;
    int32_t inv_r_acc_q15;   /* 1/r，Q15；r=0时为0 */
    int32_t inv_r_dec_q15;
    int32_t inv_r_rev_q15;

    /* 本段计划(全部在C_Traj_Plan中一次算好) */
    int32_t p0;          /* 段起点位置(未回绕)，厘度 */
    int32_t v0_qt;       /* 段起点速度，Q16 厘度/拍 */
    int32_t vc_qt;       /* 巡航速度，Q16 厘度/拍(情形B下为负) */
    int32_t p_a;         /* 加速段末位置(未回绕)，厘度 */
    int32_t p_c;         /* 匀速段末位置(未回绕)，厘度 */
    int32_t p_end;       /* 段终点位置(未回绕)，厘度 */
    int32_t inv_t1_q30;  /* 1/t1 的Q30定点 */
    int32_t inv_t3_q30;  /* 1/t3 的Q30定点 */
    int32_t posk1;       /* (dv1*t1)>>15，加速段位置整形系数 */
    int32_t posk3;       /* (dv3*t3)>>15，减速段位置整形系数 */
    int32_t acck1;       /* 加速段 dv1/t1 换算成厘度/秒^2，配 6u(1-u) 用 */
    int32_t acck3;       /* 减速段同上 */
    int32_t dv1_qt;      /* 加速段速度增量 vc-v0 */
    int32_t dv3_qt;      /* 减速段速度增量 0-vc  */
    int32_t seg1_r_q15;
    int32_t seg3_r_q15;
    int32_t seg1_peak_q15;
    int32_t seg3_peak_q15;
    int32_t seg1_inv_r_q15;
    int32_t seg3_inv_r_q15;

    int32_t t1;          /* 加速段时长(拍) */
    int32_t tc;          /* 匀速段时长(拍) */
    int32_t t3;          /* 减速段时长(拍) */
    int32_t T;           /* 本段总时长(拍) */
    int32_t t;           /* 段内已走拍数 */

    /* 可用行程(厘度)。范围以外即死区，参考位置永不进入，轨迹也永不穿过。 */
    int32_t lo;
    int32_t hi;
} s_tj;

/*
 * @fn      Traj_Clamp
 * @brief   把位置夹进可用行程[lo,hi]
 *
 * 取代了旧版的角度回绕。规划器不再有"最短路径"概念：位移一律按
 * dp = target - p0 直线计算，因此永远不会为了走近路而穿过死区。
 * 360度模式下 359.5° -> 0.5° 只会反向走 359° 的长路，这正是要的行为。
 */
static int32_t Traj_Clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/*
 * @fn      Traj_Sqrt_Ceil
 * @brief   32位整数平方根，向上取整
 *
 * 逐位试探法：16次定长循环，只有移位与加减，无除法、无乘法、无浮点。
 * 每条指令最多一次(解三角形轨迹的峰值速度)。向上取整而非四舍五入：
 * 峰值速度略偏大 => 过渡段时长 t=k*vp 略偏长 => 加速度只会更小，安全侧。
 */
static uint32_t Traj_Sqrt_Ceil(uint32_t x)
{
    uint32_t rem  = 0;
    uint32_t root = 0;
    uint32_t v    = x;
    uint32_t i;

    for (i = 0; i < 16U; i++)
    {
        root <<= 1;
        rem    = (rem << 2) | (v >> 30);
        v    <<= 2;
        if (root < rem)
        {
            root++;
            rem -= root;
            root++;
        }
    }
    /* 循环结束时 root>>1 = floor(sqrt(x))，rem 非零即非完全平方 */
    return (root >> 1) + ((rem != 0U) ? 1U : 0U);
}

/* radix-16 每轮要把余数左移4位并试减 8 倍除数，要求 16*den 不溢出32位，
 * 即 den <= 2^28-1。本模块所有除数都远小于它，下面有编译期断言。 */
#define TRAJ_DIV_DEN_MAX  0x0FFFFFFFu

/*
 * @fn      Traj_Div_U64
 * @brief   64位被除数 / 32位除数，要求商放得下32位(等价于 num>>32 < den)
 * @param   num 被除数
 * @param   den 除数，必须 1 <= den <= TRAJ_DIV_DEN_MAX
 * @return  num/den，向零取整；越界时返回0xFFFFFFFF
 *
 * 为什么不直接写 num/den：目标核没有DIV指令，编译器会展开成 libgcc 的
 * __udivdi3，而它内部要调用4~8次逐位移位的 __udivsi3(每次约173条指令)，
 * 一次64位除法就是800~1500条，占掉整个Plan的三分之二，而且条数随操作数
 * 变化、耗时抖动大。
 *
 * 这里用 radix-16 移位相减：每轮吃进4位被除数，用"依次试减 8/4/2/1 倍
 * 除数"求出这一位的4bit商——这等价于对商的4个bit做二分，结果与逐位相减
 * 完全一致(4百万组随机数据 + 定向边界已与编译器的 / 逐一比对)。
 * 轮数固定8轮，riscv-gcc -Os 实测循环体11~19条，合计约120条指令：
 *     __udivdi3   800~1500 条，耗时随操作数抖动
 *     radix-2     288 条
 *     radix-16    120 条   <- 本实现，比 __udivdi3 快 7~12 倍
 * 代价是多用3个寄存器存 2/4/8 倍除数，代码大 62 字节。
 *
 * 本模块的三处调用都满足"商放得下32位"：
 *   三角形解 Wp2 = num/(k1+k3)：num>>32 <= 7.2e4 < k1+k3 >= 6.6e5
 *   巡航速度 vc = numv/den    ：den>=2，商 <= 1.2e9
 *   速度上限 need = an/VMAX_QT：商 <= 6.8e4
 */
static uint32_t Traj_Div_U64(uint64_t num, uint32_t den)
{
    uint32_t r  = (uint32_t)(num >> 32);
    uint32_t lo = (uint32_t)num;
    uint32_t q  = 0;
    uint32_t d2, d4, d8;
    uint32_t i;

    /* 商放不下32位，或除数超出 radix-16 的适用范围(见上)，纯防御 */
    if (den == 0U || den > TRAJ_DIV_DEN_MAX || r >= den) return 0xFFFFFFFFu;

    d2 = den << 1;
    d4 = den << 2;
    d8 = den << 3;

    for (i = 0; i < 8U; i++)
    {
        r    = (r << 4) | (lo >> 28);   /* 一次吃进4位被除数 */
        lo <<= 4;
        q  <<= 4;
        /* 余数 r < 16*den，用"从高位往低位试减 8/4/2/1 倍"贪心求这一位商，
         * 等价于对商的4个bit做二分，结果与逐位相减完全一致。 */
        if (r >= d8)  { r -= d8;  q |= 8U; }
        if (r >= d4)  { r -= d4;  q |= 4U; }
        if (r >= d2)  { r -= d2;  q |= 2U; }
        if (r >= den) { r -= den; q |= 1U; }
    }
    return q;
}

/*
 * @fn      Traj_Div_S64
 * @brief   带符号版本，语义与C的整数除法一致(向零取整)
 */
static int32_t Traj_Div_S64(int64_t num, int32_t den)
{
    int32_t s = 1;
    uint32_t q;

    if (num < 0) { num = -num; s = -s; }
    if (den < 0) { den = -den; s = -s; }
    q = Traj_Div_U64((uint64_t)num, (uint32_t)den);
    /* 行程放开到整圈后，den 取最小值2时商可达约2.4e9，超出int32。这里饱和到
     * INT32_MAX，让调用方的 |vc|>vmax 分支去把总时间顶开，而不是回绕成负数。 */
    if (q > 0x7FFFFFFFu) q = 0x7FFFFFFFu;
    return (s > 0) ? (int32_t)q : -(int32_t)q;
}

/*
 * @fn      Traj_Seg_Ticks
 * @brief   跨越速度差 dv_qt(>=0) 所需的过渡段时长(拍)，向上取整且至少1拍
 * @param   k_q32  形状常数(Q32)，见文件头
 * @param   dv_qt  速度差绝对值，Q16 厘度/拍
 *
 * 这是全模块唯一决定"加减速段有多长"的地方——它只与速度差成正比，
 * 不含任何与行程、与总时间无关的绝对下限。
 */
static int32_t Traj_Seg_Ticks(int32_t k_q32, int32_t dv_qt)
{
    int32_t t = (int32_t)((((int64_t)k_q32 * dv_qt) + 0xFFFFFFFFLL) >> 32);
    return (t < 1) ? 1 : t;
}

/*
 * @fn      C_Traj_Init
 * @brief   初始化规划器、停在初始位置，并设置加/减速平滑度
 * @param   init_pos      初始位置(厘度)
 * @param   accel_smooth  加速平滑度，0~TRAJ_SMOOTH_MAX，越大越柔
 * @param   decel_smooth  减速平滑度，0~TRAJ_SMOOTH_MAX，越大越柔
 *
 * 平滑度只在初始化时设置，不再提供运行期修改入口。smooth=0使用恒加速度
 * 速度斜坡，整段持续使用硬件允许的最大加/减速度；smooth>0继续使用原来的
 * jerk受限梯形加速度型线，保证段首段尾加速度为0。传入值超过上限时自动夹紧。
 */
void C_Traj_Init(int32_t init_pos, uint8_t accel_smooth, uint8_t decel_smooth)
{
    int32_t max = TRAJ_SMOOTH_MAX;
    int32_t sa = (accel_smooth > max) ? max : (int32_t)accel_smooth;
    int32_t sd = (decel_smooth > max) ? max : (int32_t)decel_smooth;

    /*
     * ============ 平滑度 -> 标准 jerk 受限 S 曲线 ============
     * r = 0.5*s/max 是一个速度过渡段中“加速度爬升”和“加速度回落”各自
     * 占用的时间比例：
     *     s=0    r=0   ：恒加速度，严格使用硬件上限，时间最短；
     *     s=max  r=0.5 ：三角加速度，没有恒加速度平台，最柔和。
     * 中间值是七段式 S 曲线的标准梯形加速度。为保持峰值加速度不越限，
     * 段时长倍率为 peak=1/(1-r)，从 1 连续变到 2；不存在形状切换点，
     * 也不再需要第二个 SPREAD 旋钮。
     */
    if (max <= 0) max = 1;
    s_tj.r_acc_q15 = (int32_t)(((int64_t)sa << 14) / max);
    s_tj.r_dec_q15 = (int32_t)(((int64_t)sd << 14) / max);
    /* 换向段(首段需要先刹车再反向)的形状只跟随 r_acc，**不取 max**。
     * 到位冲击发生在末段减速归零(seg3，只认 r_dec)，与本段无关；
     * 若这里取 max，停车的平滑度就会漏到在线改目标的换向上，
     * 实测 DEC=25 时参考翻转 73->81ms、到达 366->393ms。 */
    s_tj.r_rev_q15 = s_tj.r_acc_q15;

    s_tj.peak_acc_q15 = (int32_t)(((int64_t)32768 * 32768)
                                   / (32768 - s_tj.r_acc_q15));
    s_tj.peak_dec_q15 = (int32_t)(((int64_t)32768 * 32768)
                                   / (32768 - s_tj.r_dec_q15));
    s_tj.peak_rev_q15 = (int32_t)(((int64_t)32768 * 32768)
                                   / (32768 - s_tj.r_rev_q15));
    s_tj.inv_r_acc_q15 = (s_tj.r_acc_q15 > 0)
                       ? (int32_t)(((int64_t)32768 * 32768) / s_tj.r_acc_q15) : 0;
    s_tj.inv_r_dec_q15 = (s_tj.r_dec_q15 > 0)
                       ? (int32_t)(((int64_t)32768 * 32768) / s_tj.r_dec_q15) : 0;
    s_tj.inv_r_rev_q15 = (s_tj.r_rev_q15 > 0)
                       ? (int32_t)(((int64_t)32768 * 32768) / s_tj.r_rev_q15) : 0;

    /* k 不再在这里定死：它随每段行程由 Traj_AutoAmax 算出，见 C_Traj_Plan。
     * 这里先给一份保底值，避免 Plan 之前就调用 Step。 */
    s_tj.k_acc_q32 = Traj_K_Linear_Q32(TRAJ_AMAX_ACC_CDPSS);
    s_tj.k_dec_q32 = Traj_K_Linear_Q32(TRAJ_AMAX_DEC_CDPSS);
    s_tj.k_rev_q32 = (s_tj.k_acc_q32 > s_tj.k_dec_q32)
                   ? s_tj.k_acc_q32 : s_tj.k_dec_q32;

    /* 默认放开整圈；A_Servo 会按舵机模式立刻调 C_Traj_Set_Range 收紧到实际行程 */
    s_tj.lo     = 0;
    s_tj.hi     = CDEG_RANGE;
    s_tj.pos    = Traj_Clamp(init_pos, s_tj.lo, s_tj.hi);
    s_tj.vel_qt = 0;

    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = 0;
    s_tj.vc_qt = 0;
    s_tj.p_a   = s_tj.pos;
    s_tj.p_c   = s_tj.pos;
    s_tj.p_end = s_tj.pos;
    s_tj.inv_t1_q30 = 0;
    s_tj.inv_t3_q30 = 0;
    s_tj.posk1  = 0;
    s_tj.posk3  = 0;
    s_tj.dv1_qt = 0;
    s_tj.dv3_qt = 0;
    s_tj.seg1_r_q15 = s_tj.r_acc_q15;
    s_tj.seg3_r_q15 = s_tj.r_dec_q15;
    s_tj.seg1_peak_q15 = s_tj.peak_acc_q15;
    s_tj.seg3_peak_q15 = s_tj.peak_dec_q15;
    s_tj.seg1_inv_r_q15 = s_tj.inv_r_acc_q15;
    s_tj.seg3_inv_r_q15 = s_tj.inv_r_dec_q15;

    s_tj.t1 = 0;
    s_tj.tc = 0;
    s_tj.t3 = 0;
    s_tj.T  = 0;
    s_tj.t  = 0;   /* t >= T 即视为已走完 */

}

/*
 * @fn      C_Traj_Set_Range
 * @brief   设置可用行程(厘度)，范围以外即死区
 * @param   lo 下限，@param hi 上限(必须 > lo)
 *
 * 调用后当前指令位置被夹进[lo,hi]、速度归零并停住，避免换模式的瞬间
 * 参考还带着旧行程的速度往死区里冲。参数非法时保持原设置不动。
 */
void C_Traj_Set_Range(int32_t lo, int32_t hi)
{
    if (hi <= lo) return;

    s_tj.lo = lo;
    s_tj.hi = hi;

    s_tj.pos    = Traj_Clamp(s_tj.pos, lo, hi);
    s_tj.vel_qt = 0;
    s_tj.p0     = s_tj.pos;
    s_tj.p_a    = s_tj.pos;
    s_tj.p_c    = s_tj.pos;
    s_tj.p_end  = s_tj.pos;
    s_tj.vc_qt  = 0;
    s_tj.t1 = 1; s_tj.tc = 0; s_tj.t3 = 1;
    s_tj.T  = 0; s_tj.t  = 0;   /* T=0 => 立即视为走完，停在当前位置 */
}

void C_Traj_Hold(int32_t pos)
{
    s_tj.pos = Traj_Clamp(pos, s_tj.lo, s_tj.hi);
    s_tj.vel_qt = 0;
    s_tj.p0 = s_tj.p_a = s_tj.p_c = s_tj.p_end = s_tj.pos;
    s_tj.v0_qt = s_tj.vc_qt = s_tj.dv1_qt = s_tj.dv3_qt = 0;
    s_tj.t1 = s_tj.t3 = 1;
    s_tj.tc = s_tj.T = s_tj.t = 0;
}

/*
 * @fn      Traj_Acc_Coef
 * @brief   把段的 dv/t_seg 换算成"厘度/秒^2 每单位形状函数"
 * @param   dv_qt    该段速度增量，Q16 厘度/拍
 * @param   inv_q30  1/段长 的Q30定点
 *
 * 恒加速度段直接输出本系数；jerk受限段再乘梯形归一化加速度A(u)。
 * 中间量最大 2.36e6*1e6=2.4e12，落在int64内；结果被 amax 约束，落在int32内。
 */
static int32_t Traj_Acc_Coef(int32_t dv_qt, int32_t inv_q30)
{
    int64_t product = (int64_t)dv_qt * inv_q30;
    int64_t per_tick2;
    int64_t scaled;

    /* 负数算术右移通常向负无穷舍入，会把减速度绝对值多算1个Q16计数，最终可
     * 比配置上限大约10厘度/秒²。显式向零截断，保证正反向都只低不高。 */
    per_tick2 = (product >= 0)
              ? (product >> TRAJ_INV_SHIFT)
              : -((-product) >> TRAJ_INV_SHIFT);
    scaled = per_tick2 * CFG_TICK_HZ * CFG_TICK_HZ;
    return (int32_t)((scaled >= 0)
                   ? (scaled >> TRAJ_VEL_SHIFT)
                   : -((-scaled) >> TRAJ_VEL_SHIFT));
}

/*
 * @fn      C_Traj_Plan
 * @brief   规划一段新轨迹，以当前指令状态为起点(在线重定向)
 * @param   target_pos 目标位置(厘度)
 * @param   time_ms    期望总时间(ms)，0=最快
 *
 * 三步：
 *   1) 解"最速轨迹"(满限幅)，得到 t1/tc/t3 与最短总时间 T_fast；
 *   2) 若请求时间 > T_fast，按 lambda=T_req/T_fast 均匀拉伸三段(比例守恒，
 *      加速度自动降为 1/lambda^2)；否则直接用 T_fast——t=0、时间给太短、
 *      硬件根本做不到，走的是同一条路径；
 *   3) 用位移方程反解巡航速度 vc(保证准时到位)，并复核 vmax/加速度上限，
 *      不满足就加长对应过渡段后重解，最多4轮。
 *
 * 全部除法集中在本函数，每条指令执行一次；C_Traj_Step每拍无除法。
 */
void C_Traj_Plan(int32_t target_pos, uint16_t time_ms)
{
    int32_t dp, sgn, D, u0, au0, W0;
    int32_t k1, k3;
    int32_t t1, tc, t3, T, T_fast, T_req;
    int32_t vp, vc, vmax_qt;
    int32_t n1, n3, i;
    int64_t num, numv;
    uint32_t lam_q8;
    uint8_t  case_b;

    /* ---- 新段起点取规划器自身的指令状态，保证位置/速度连续 ---- */
    s_tj.p0    = s_tj.pos;
    s_tj.v0_qt = s_tj.vel_qt;

    /* 目标先夹进可用行程：死区以内的目标一律被收到边界上，绝不下发到死区里。
     * 位移直接相减、不做任何最短路径折算，所以轨迹永远不会穿过死区。 */
    target_pos = Traj_Clamp(target_pos, s_tj.lo, s_tj.hi);
    dp  = target_pos - s_tj.p0;
    sgn = (dp >= 0) ? 1 : -1;
    D   = dp * sgn;                 /* 运动方向上的行程，>=0 */
    u0  = s_tj.v0_qt * sgn;         /* 起点速度沿运动方向的分量，反向时为负 */

    /* ---- 本段的动力学上限：按行程自动确定，不再取固定配置值 ----
     * 两件事在这里发生：
     *   1) 小位移缩放：行程小于 traj_small_span_cdeg 时整体降低动力学，
     *      防止 1° 的指令也冲到 67°/s 造成抖动；
     *   2) 加速度上限按峰值速度自动推导，短行程自动获得更大的加速度。
     * 都只在收到指令时算一次，不进每拍路径。 */
    {
        int32_t vmax_use = Traj_VelCap(D);
        int32_t a_ceil   = Traj_AccelCeil(vmax_use);
        int32_t amax_a, amax_d, kl_a, kl_d, v0_cdps;

        if (vmax_use < 1) vmax_use = 1;
        vmax_qt = (int32_t)(((int64_t)vmax_use << TRAJ_VEL_SHIFT) / CFG_TICK_HZ);

        /* 起点速度换算成厘度/秒，供加速度上限推导用(见 Traj_AutoAmax) */
        v0_cdps = (int32_t)((((int64_t)((u0 < 0) ? -u0 : u0)) * CFG_TICK_HZ)
                            >> TRAJ_VEL_SHIFT);

        amax_a = Traj_AutoAmax(D, s_tj.peak_acc_q15, vmax_use, v0_cdps);
        if (amax_a > a_ceil) amax_a = a_ceil;
        if (amax_a < 1) amax_a = 1;

        /* 减速侧基准取配置的减速上限(= a0 * DSG_DEC_RATIO)：制动能力
         * 与速度同向增长，不受"高速时余量不足"的限制。 */
        amax_d = TRAJ_AMAX_DEC_CDPSS;

        /* 但同样套 a_ceil：减速段也至少要持续 TRAJ_ACCEL_MIN_MS。
         * 两个指标在这里是对立的，取舍取决于要治哪个症状：
         *   不套 —— 位置过冲更小(2度 39->6、5度 50->29、1.35度 23->0，
         *          整定各快 40~90ms)，因为参考尽快让路、由无模型误差的
         *          反馈接手刹车；
         *   套上 —— 到位瞬间的 PWM 单拍跳变从 1860 降到 263(约7倍)，
         *          治的是机械惯性震动，不是轴的来回摆。
         * 现取"套上"：震动是实机可感的，几十厘度的过冲不是。 */
        if (amax_d > a_ceil) amax_d = a_ceil;
        if (amax_d < 1) amax_d = 1;

        /* 标准 jerk 受限过渡的段时长：k = peak/a，peak=1/(1-r)。 */
        kl_a = Traj_K_Linear_Q32(amax_a);
        kl_d = Traj_K_Linear_Q32(amax_d);
        s_tj.k_acc_q32 = (int32_t)(((int64_t)kl_a * s_tj.peak_acc_q15) >> 15);
        s_tj.k_dec_q32 = (int32_t)(((int64_t)kl_d * s_tj.peak_dec_q15) >> 15);
        /* 换向段的段时长：取两侧中较弱的物理加速度(kl 越大越弱)保守，
         * 但拉长倍率用 peak_rev(=peak_acc)。不能直接 max(k_acc,k_dec)：
         * k_dec 里已经乘过 peak_dec=1/(1-r_dec)，那个拉长是为了停车柔和、
         * 不应该跟到换向上。DEC=25 时 peak_dec=1.143，就是另一半滞后。 */
        {
            int32_t kl_r = (kl_a > kl_d) ? kl_a : kl_d;
            s_tj.k_rev_q32 = (int32_t)(((int64_t)kl_r * s_tj.peak_rev_q15) >> 15);
        }
    }

    T_req = ((int32_t)time_ms * CFG_TICK_HZ) / 1000;

    /* 已经停在目标上：保持不动。云台连续微调时目标常常与上一拍相同，
     * 这里直接返回可以避免每帧都重建一条长度为0的轨迹。 */
    if (D == 0 && u0 == 0)
    {
        s_tj.p_end = s_tj.p0;
        s_tj.vc_qt = 0;
        s_tj.t1 = 1; s_tj.tc = 0; s_tj.t3 = 1;
        s_tj.T  = 0; s_tj.t  = 0;
        return;
    }

    au0 = (u0 < 0) ? -u0 : u0;
    W0  = au0 >> TRAJ_W_SHIFT;      /* Q8，用于平方时不溢出 */
    k3  = s_tj.k_dec_q32;           /* 末段恒为"减速到0"，永远用减速上限 */

    /* ---- 情形判定：正向运动且刹车距离已经超过剩余行程 => 必然过冲 ----
     * 刹车距离(厘度) = k3*u0^2/2 ，两边同乘 2^33 化成整数比较，无除法。 */
    case_b = 0;
    if (u0 > 0)
    {
        if (((int64_t)D << 33) < ((int64_t)k3 * W0 * W0)) case_b = 1;
    }
    /* 第一段若需要刹车或换向，取更保守的k(=更小的加速度)同时满足两个上限 */
    k1 = (case_b || u0 < 0) ? s_tj.k_rev_q32 : s_tj.k_acc_q32;
    s_tj.seg1_r_q15 = (case_b || u0 < 0) ? s_tj.r_rev_q15 : s_tj.r_acc_q15;
    s_tj.seg1_peak_q15 = (case_b || u0 < 0) ? s_tj.peak_rev_q15 : s_tj.peak_acc_q15;
    s_tj.seg1_inv_r_q15 = (case_b || u0 < 0) ? s_tj.inv_r_rev_q15 : s_tj.inv_r_acc_q15;
    s_tj.seg3_r_q15 = s_tj.r_dec_q15;
    s_tj.seg3_peak_q15 = s_tj.peak_dec_q15;
    s_tj.seg3_inv_r_q15 = s_tj.inv_r_dec_q15;

    /* ---- 第1步：最速轨迹 ---- */
    if (case_b)
    {
        /* 冲过目标再倒回来：vp<0，且 |vp|<=|u0|<=vmax，永远不需要匀速段 */
        num = (int64_t)k1 * W0 * W0 - ((int64_t)D << 33);
        if (num < 0) num = 0;                       /* 纯防御，判据已保证>0 */
        vp = -(int32_t)(Traj_Sqrt_Ceil(Traj_Div_U64((uint64_t)num,
                                       (uint32_t)(k1 + k3))) << TRAJ_W_SHIFT);
        if (vp < -vmax_qt) vp = -vmax_qt;
        t1 = Traj_Seg_Ticks(k1, u0 - vp);
        t3 = Traj_Seg_Ticks(k3, -vp);
        tc = 0;
    }
    else
    {
        /* 先按 vp=vmax 试梯形：两段过渡位移放得下就是梯形，否则是三角形 */
        int32_t d_ramp;
        t1 = Traj_Seg_Ticks(k1, vmax_qt - u0);
        t3 = Traj_Seg_Ticks(k3, vmax_qt);
        d_ramp = (int32_t)((((int64_t)(u0 + vmax_qt) * t1)
                          + ((int64_t)vmax_qt * t3)) >> (TRAJ_VEL_SHIFT + 1));

        if (d_ramp <= D)
        {
            /* 梯形：匀速段补足剩余行程。分子<=18000<<16 落在uint32内，
             * 这是一次32位除法而不是64位除法。 */
            uint32_t rest = ((uint32_t)(D - d_ramp) << TRAJ_VEL_SHIFT);
            tc = (int32_t)((rest + (uint32_t)vmax_qt - 1U) / (uint32_t)vmax_qt);
            vp = vmax_qt;
        }
        else
        {
            /* 三角形：vp = sqrt((2D + k1*u0^2)/(k1+k3))，闭式解，一次开方 */
            num = ((int64_t)D << 33) + (int64_t)k1 * W0 * W0;
            vp  = (int32_t)(Traj_Sqrt_Ceil(Traj_Div_U64((uint64_t)num,
                                           (uint32_t)(k1 + k3))) << TRAJ_W_SHIFT);
            if (vp > vmax_qt) vp = vmax_qt;
            t1 = Traj_Seg_Ticks(k1, (vp > u0) ? (vp - u0) : (u0 - vp));
            t3 = Traj_Seg_Ticks(k3, vp);
            tc = 0;
        }
    }

    T_fast = t1 + tc + t3;

    /* ---- 第2步：请求时间比最快还长 => 均匀时间缩放 ----
     * 三段同乘 lambda，比例守恒；速度自动降为1/lambda、加速度降为1/lambda^2。
     * t1*lam_q8 <= T_req<<8 <= 65535*256，不会溢出。
     *
     * 唯一的例外是加速段承担的是"抵消已有速度"而不是"跑行程"的时候——
     * 即情形B(刹不住)或起点速度与行程反向(u0<0)。这种恢复过程必须顶着
     * 加速度上限走：把它按lambda拉长既不会更平滑(加速度本来就该顶满)，
     * 又会让参考朝错误方向多跑很多。实测全速巡航时把目标改到眼前，
     * T=0 时正向冲出28°(=物理刹车距离)，而拉伸后会冲出125°。
     * 因此这两种情况下加速段只取"抵消u0所需的最短时长"作为种子，
     * 由第3步的可行性迭代长到刚好够用为止，多出来的时间全部给匀速段。 */
    T = T_fast;
    if (T_req > T_fast)
    {
        lam_q8 = ((uint32_t)T_req << 8) / (uint32_t)T_fast;
        if (!case_b && u0 >= 0) t1 = (int32_t)(((uint32_t)t1 * lam_q8) >> 8);
        else                    t1 = Traj_Seg_Ticks(k1, au0);
        t3 = (int32_t)(((uint32_t)t3 * lam_q8) >> 8);
        T  = T_req;
        if (t1 < 1) t1 = 1;
        if (t1 + t3 > T) t3 = T - t1;   /* 加速段不缩放时减速段可能挤不下 */
        if (t3 < 1) t3 = 1;
    }

    /* ---- 第3步：反解巡航速度并复核上限 ----
     * 位移方程 D = (u0+vc)*t1/2 + vc*tc + vc*t3/2 ，整理得
     *     vc = (2D - u0*t1) / (2T - t1 - t3)
     * 分母 2T-t1-t3 >= t1+t3 >= 2 恒正。解出的vc使轨迹**精确**在T拍到位。
     * 复核两件事：|vc|<=vmax(否则加长T)，两段过渡跨得过去(否则加长该段)。
     * 过渡段只增不减，单调收敛；最后一轮只解vc不再改t1/t3，保证输出的
     * vc 与 t1/t3/T 自洽(否则末端会出现一个位置跳变)。 */
    vc = 0;
    for (i = 0; i < 4; i++)
    {
        int32_t den;
        if (T < t1 + t3) T = t1 + t3;
        den  = 2 * T - t1 - t3;
        numv = ((int64_t)D << (TRAJ_VEL_SHIFT + 1)) - (int64_t)u0 * t1;
        vc   = Traj_Div_S64(numv, den);

        if (vc > vmax_qt || vc < -vmax_qt)
        {
            /* T短到巡航速度顶破vmax：把T顶开到刚好不越限，宁可晚到 */
            int64_t an   = (numv < 0) ? -numv : numv;
            int32_t need = (int32_t)((Traj_Div_U64((uint64_t)(an + vmax_qt - 1),
                                                  (uint32_t)vmax_qt)
                                      + (uint32_t)(t1 + t3 + 1)) >> 1);
            if (need > T) { T = need; continue; }
            vc = (vc > 0) ? vmax_qt : -vmax_qt;
        }
        if (i == 3) break;

        n1 = Traj_Seg_Ticks(k1, (vc > u0) ? (vc - u0) : (u0 - vc));
        n3 = Traj_Seg_Ticks(k3, (vc < 0) ? -vc : vc);
        if (n1 <= t1 && n3 <= t3) break;
        if (n1 > t1) t1 = n1;
        if (n3 > t3) t3 = n3;
        /* 加速段变长会挤掉匀速段。只要减速段还有富余(它是可自由拉伸的那段)，
         * 就先把它压回去保住"准时"；压到减速上限还放不下，才由循环顶部的
         * T = t1+t3 把总时间顶开——那说明这个时间物理上真的做不到。 */
        if (t1 + t3 > T && (T - t1) >= n3) t3 = T - t1;
    }
    tc = T - t1 - t3;

    /* ---- 换回全局符号并预计算每拍要用的常量 ---- */
    s_tj.vc_qt  = vc * sgn;
    s_tj.dv1_qt = s_tj.vc_qt - s_tj.v0_qt;
    s_tj.dv3_qt = -s_tj.vc_qt;
    s_tj.t1 = t1;
    s_tj.tc = tc;
    s_tj.t3 = t3;
    s_tj.T  = T;
    s_tj.t  = 0;

    /* 位置整形系数：过渡段位移 = v0*tt + (dv*t_seg)*P(u)，P是S的积分(Q15)。
     * (dv*t_seg)>>15 最大约 2.2e6*65535/32768 = 4.5e6，落在int32内。 */
    s_tj.posk1 = (int32_t)(((int64_t)s_tj.dv1_qt * t1) >> TRAJ_U_SHIFT);
    s_tj.posk3 = (int32_t)(((int64_t)s_tj.dv3_qt * t3) >> TRAJ_U_SHIFT);

    /* 段末位置用与逐拍公式**完全相同**的表达式算(P(1)=0.5 即Q15的16384)，
     * 这样段与段的接缝上不会因为舍入差出1厘度。 */
    s_tj.p_a = s_tj.p0
             + (int32_t)((((int64_t)s_tj.v0_qt * t1)
                        + ((int64_t)s_tj.posk1 << (TRAJ_U_SHIFT - 1))) >> TRAJ_VEL_SHIFT);
    s_tj.p_c = s_tj.p_a
             + (int32_t)(((int64_t)s_tj.vc_qt * tc) >> TRAJ_VEL_SHIFT);
    s_tj.p_end = s_tj.p0 + dp;   /* 终点用解析值，避免逐段舍入累积 */

    /* 这两个倒数的商有23~27位，libgcc的__udivsi3是"每多一位商多9条指令"，
     * 实测要210/206条；而 Traj_Div_U64 是定长约130条，所以这里反过来用它更快
     * (省约156条/条指令)。上面 tc 和 lam_q8 的商只有10位左右，__udivsi3 只要
     * 88/100条，比130条便宜，就保持除号不动——不是"能换就换"，是按商的位数选。 */
    s_tj.inv_t1_q30 = (int32_t)Traj_Div_U64((uint64_t)1 << TRAJ_INV_SHIFT, (uint32_t)t1);
    s_tj.inv_t3_q30 = (int32_t)Traj_Div_U64((uint64_t)1 << TRAJ_INV_SHIFT, (uint32_t)t3);

    /* 加速度系数：先把dv/t_seg换算成厘度/秒^2。恒加速度段直接输出该值；
     * jerk受限段再乘归一化梯形加速度A(u)。复用上面的Q30倒数，不额外做除法。
     *   dv_qt/t_seg  -> Q16 厘度/拍^2 : (dv_qt * inv_q30) >> 30
     *   再 *TICK_HZ^2 >>16 换成 厘度/秒^2
     * 物理上，t_seg=peak*|dv|/amax，peak随平滑度从1连续增至2。 */
    s_tj.acck1 = Traj_Acc_Coef(s_tj.dv1_qt, s_tj.inv_t1_q30);
    s_tj.acck3 = Traj_Acc_Coef(s_tj.dv3_qt, s_tj.inv_t3_q30);
}

/*
 * @fn      Traj_Emit
 * @brief   把本拍算出的参考位置夹进行程后输出，并同步内部指令状态
 *
 * 撞到死区边界时把参考速度一并归零：一是不要把速度前馈往"墙"里送，二是
 * 下一次重规划会以这个静止的边界位置为起点，不会带着旧速度继续往死区冲。
 * 正常工况下钳位不会触发——目标已经在 Plan 里被夹过一次了，只有在线反向
 * 重规划、刹不住而必须冲过目标(情形B)时才可能碰到边界。
 */
static void Traj_Emit(TrajRef_t *out, int32_t pos, int32_t acc)
{
    if (pos < s_tj.lo || pos > s_tj.hi)
    {
        pos = Traj_Clamp(pos, s_tj.lo, s_tj.hi);
        s_tj.vel_qt = 0;
        acc = 0;          /* 撞死区边界：速度和加速度前馈都不该再往墙里送 */
    }
    s_tj.pos = pos;
    out->pos = pos;
    out->vel = (int32_t)(((int64_t)s_tj.vel_qt * CFG_TICK_HZ) >> TRAJ_VEL_SHIFT);
    out->acc = acc;
}

/* 标准 jerk 受限速度过渡的归一化型线。
 * r=0 时精确退化为恒加速度；r>0 时加速度按“线性升—恒定—线性降”变化，
 * 因而速度、位置、加速度在段边界都连续。S 是速度比例，P 是其积分，A 是
 * 归一化加速度；恒有 S(1)=1、P(1)=1/2、A(0)=A(1)=0(r>0)。 */
static void Traj_SCurve(int32_t u, int32_t r, int32_t peak, int32_t inv_r,
                        int32_t *s, int32_t *p, int32_t *a)
{
    int32_t w, ratio, pr, sr;

    if (r <= 0)
    {
        *s = u;
        *p = (int32_t)(((int64_t)u * u) >> 16);
        *a = 32768;
        return;
    }

    if (u > 32768 - r)
    {
        int32_t sw, pw, aw;

        w = 32768 - u;
        Traj_SCurve(w, r, peak, inv_r, &sw, &pw, &aw);
        *s = 32768 - sw;
        *p = 16384 - w + pw;
        *a = aw;
        return;
    }

    if (u < r)
    {
        ratio = (int32_t)(((int64_t)u * inv_r) >> 15); /* u/r，Q15 */
        *a = (int32_t)(((int64_t)peak * ratio) >> 15);
        *s = (int32_t)(((int64_t)peak * u * ratio) >> 31);
        /* S=peak*u^2/(2r)，故P=∫Sdu=S*u/3。这里四个量都是Q15。
         * 旧式少乘了一个u，虽能保持接缝连续，却不再满足dp/dt=v。 */
        /* 21845≈2^16/3，因此 >>31 等价于 /(3*2^15)，避免RV32上的软除法。 */
        *p = (int32_t)(((int64_t)(*s) * u * 21845) >> 31);
        return;
    }

    /* 中段加速度恒为 peak。P(r)=peak*r^2/6，之后积分
     * S(u)=peak*(u-r/2)，化简得增量 peak*u*(u-r)/2。 */
    sr = (int32_t)(((int64_t)peak * r) >> 16); /* S(r)=peak*r/2 */
    pr = (int32_t)(((int64_t)sr * r * 21845) >> 31);
    *a = peak;
    *s = (int32_t)(((int64_t)peak * (u - (r >> 1))) >> 15);
    *p = pr + (int32_t)(((int64_t)peak * u * (u - r)) >> 31);
}

/*
 * @fn      C_Traj_Step
 * @brief   推进一拍并输出参考位置/速度
 * @param   out 参考轨迹输出
 *
 * 每拍无一般除法、无sqrt、无查表、无浮点；只计算分段多项式和解析加速度。
 */
void C_Traj_Step(TrajRef_t *out, uint8_t hold)
{
    int32_t tt;
    int32_t u_q15;
    int32_t s_q15;      /* 归一化速度 */
    int32_t p_q15;      /* S(u)从0到u的积分 */
    int32_t seg_p0, seg_v0_qt, seg_dv_qt, seg_inv_q30, seg_posk, seg_acck;
    int32_t a_q15;      /* 归一化加速度，Q15，1.0=32768 */
    int32_t pos, acc;
    int32_t seg_r = 0, seg_peak = 32768, seg_inv_r = 0;

    if (out == 0) return;

    /* ---- 饱和保持：执行器已经出满力时，**不要再推进参考** ----
     * 这是全套算法里唯一不依赖任何标定精度的鲁棒性机制。
     * 规划器的加速度上限来自 a0=(PWM_FULL-coulomb)/(Kv*tau)，只要 Kv 或 tau
     * 标小了，a0 就偏大，参考会要求一个硬件做不出来的加速度；反馈救不了
     * 一条对象根本跟不上的参考，误差一路累积到末段变成过冲。
     * 让轨迹时钟在饱和期间停摆，参考就自动退化成"硬件能做到的最快"，
     * 总时长按缺多少力自动拉长。不需要知道缺了多少，也不需要任何参数。
     * 代价：饱和期间 ref.vel/acc 保持不变，前馈仍然自洽。
     *
     * **只在加速段和匀速段冻结，减速段绝不冻结。**方向完全相反：
     *   加速/匀速饱和 = 推不动，参考等一等是对的；
     *   减速段饱和   = 刹不住，此时冻结会把 ref.vel 钉在高位、速度前馈
     *                  继续要求高速，等于帮着冲过去。实测(仿真全场景总过冲)
     *                  不分段冻结 173->1153，分段后回到 173 且整定时间照样改善。
     * 减速刹不住是另一类问题，由情形B(冲过去再倒回来)负责，不归这里管。 */
    /* 饱和等待只允许发生在加速/巡航段。减速段即使制动饱和也必须继续把
     * ref.vel 拉向 0；冻结减速参考会持续输出高速前馈，反而扩大过冲。 */
    if (s_tj.t < s_tj.T &&
        (!hold || s_tj.t >= s_tj.t1 + s_tj.tc))
    {
        s_tj.t++;
    }

    if (s_tj.t >= s_tj.T)
    {
        /* 最后一拍及其之后：锁在解析终点、速度归零。终点不由逐拍公式算出，
         * 而是直接取p_end，消除定点舍入的累积残差。 */
        s_tj.pos    = Traj_Clamp(s_tj.p_end, s_tj.lo, s_tj.hi);
        s_tj.vel_qt = 0;
        out->pos    = s_tj.pos;
        out->vel    = 0;
        out->acc    = 0;
        return;
    }

    if (s_tj.t <= s_tj.t1)
    {
        tt          = s_tj.t;                       /* 加速段 */
        seg_p0      = s_tj.p0;
        seg_v0_qt   = s_tj.v0_qt;
        seg_dv_qt   = s_tj.dv1_qt;
        seg_inv_q30 = s_tj.inv_t1_q30;
        seg_posk    = s_tj.posk1;
        seg_acck    = s_tj.acck1;
        seg_r       = s_tj.seg1_r_q15;
        seg_peak    = s_tj.seg1_peak_q15;
        seg_inv_r   = s_tj.seg1_inv_r_q15;
    }
    else if (s_tj.t <= s_tj.t1 + s_tj.tc)
    {
        /* 匀速段：无过渡整形，1次乘法 */
        tt = s_tj.t - s_tj.t1;
        s_tj.vel_qt = s_tj.vc_qt;
        pos = s_tj.p_a + (int32_t)(((int64_t)s_tj.vc_qt * tt) >> TRAJ_VEL_SHIFT);
        Traj_Emit(out, pos, 0);          /* 匀速段加速度恒为0 */
        return;
    }
    else
    {
        tt          = s_tj.t - s_tj.t1 - s_tj.tc;   /* 减速段 */
        seg_p0      = s_tj.p_c;
        seg_v0_qt   = s_tj.vc_qt;
        seg_dv_qt   = s_tj.dv3_qt;
        seg_inv_q30 = s_tj.inv_t3_q30;
        seg_posk    = s_tj.posk3;
        seg_acck    = s_tj.acck3;
        seg_r       = s_tj.seg3_r_q15;
        seg_peak    = s_tj.seg3_peak_q15;
        seg_inv_r   = s_tj.seg3_inv_r_q15;
    }

    /* u = tt/段长，用预存的Q30倒数换成乘法。tt<=段长 => 乘积<=2^30，不溢出。 */
    u_q15 = (int32_t)(((uint32_t)tt * (uint32_t)seg_inv_q30) >> TRAJ_U_SHIFT);
    if (u_q15 > (1 << TRAJ_U_SHIFT)) u_q15 = (1 << TRAJ_U_SHIFT); /* 舍入兜底 */

    Traj_SCurve(u_q15, seg_r, seg_peak, seg_inv_r, &s_q15, &p_q15, &a_q15);
    acc = (int32_t)(((int64_t)seg_acck * a_q15) >> TRAJ_U_SHIFT);

    s_tj.vel_qt = seg_v0_qt + (int32_t)(((int64_t)seg_dv_qt * s_q15) >> TRAJ_U_SHIFT);

    pos = seg_p0 + (int32_t)((((int64_t)seg_v0_qt * tt)
                            + ((int64_t)seg_posk * p_q15)) >> TRAJ_VEL_SHIFT);

    /* 减速段禁止越过终点：定点舍入会让位置多走1个计数。减速段速度由vc单调
     * 收敛到0、不换向，位置必然单调朝p_end逼近，因此按vc方向做单侧钳位是
     * 安全的(加速段可能换向，不能这样钳)。 */
    if (s_tj.t > s_tj.t1 + s_tj.tc)
    {
        if (s_tj.vc_qt >= 0) { if (pos > s_tj.p_end) pos = s_tj.p_end; }
        else                 { if (pos < s_tj.p_end) pos = s_tj.p_end; }
    }

    Traj_Emit(out, pos, acc);
}

/*
 * @fn      C_Traj_Is_Done
 * @brief   查询本段轨迹是否已走完
 */
uint8_t C_Traj_Is_Done(void)
{
    return (uint8_t)(s_tj.t >= s_tj.T);
}

/*
 * @fn      C_Traj_Get_Ticks
 * @brief   本段轨迹的实际总时长(拍)，指令时间不可达时会大于请求值
 */
uint16_t C_Traj_Get_Ticks(void)
{
    return (uint16_t)((s_tj.T > 0xFFFF) ? 0xFFFF : s_tj.T);
}
