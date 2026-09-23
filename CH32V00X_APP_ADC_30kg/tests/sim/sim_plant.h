/*
 * sim_plant.h —— 本工程主机仿真的被控对象(硬件)参数
 * 只描述舵机本体，不引用 A_Parameter.h 的控制器参数：调控制器参数时对象不跟着变，仿真才有判别力。
 * servo_sim.c / feature_audit.c / run.py 四个工程逐字相同，差异全部在本文件。
 * 方向下标：_POS = 角度增大方向(标定日志 dir0)，_NEG = 角度减小方向(dir1)。
 *
 * 本机：电位器30kg(11:24:09单次标定)。T1分向斜率9340/9047、截距68/90(低档PWM700起拟合，
 * 截距是外推值)；T2两向L+tau 19.22/37.19ms，T3回归L=1.86ms(工具自报残差2.26ms、不可信)，
 * 对象tau取两向L+tau扣L；纯延迟取2拍(=L四舍五入)，鲁棒性扫描覆盖1~3拍；T9起转240/244；
 * 噪声为静止峰峰7.44厘度的一半。T4制动增益166(制动比线性模型猛约1.5倍)本文件不建模，
 * 带载仿真 load_sim.c 可用 brake=1.54 另行扫描。
 * arrival_soft：在两向实测对象上再把tau放大1.27倍(慢向约45ms)。
 */
#pragma once

#define SIM_PLANT_SLOPE_POS_Q16 9340.0  /** PWM-速度斜率，Q16 */
#define SIM_PLANT_SLOPE_NEG_Q16 9047.0
#define SIM_PLANT_FRIC_POS_PWM  68.0    /** 动摩擦，PWM */
#define SIM_PLANT_FRIC_NEG_PWM  90.0 
#define SIM_PLANT_BRK_POS_PWM   240.0  /** 起转(静摩擦)，按出力方向取，PWM */
#define SIM_PLANT_BRK_NEG_PWM   244.0
#define SIM_PLANT_TAU_POS_MS    17.4   /** 机械时间常数，按运动方向取，ms */
#define SIM_PLANT_TAU_NEG_MS    35.3 
#define SIM_PLANT_DELAY_TICKS   2      /** 指令至轴响应的纯延迟，控制拍 */
#define SIM_NOISE_CDEG          3.72   /** 位置读数均匀噪声幅值(±)，厘度 */

/* arrival_soft 审计：实际惯性大于模型时，小角度(100us)到位后的反向回拉不能过猛。
 * 失配工况与门限按本机给出；门限取本版控制器(2026-09-22 带载改动后)仿真实测值留约20%余量，用于发现回退，
 * 不是硬件指标。本版实测：回拉721厘度/秒、超调36厘度；改动前的控制器在本对象上为回拉1986、超调164(不过审)。 */
#define SIM_ARRIVAL_NOISE        3.72
#define SIM_ARRIVAL_TAU_SCALE    1.27
#define SIM_ARRIVAL_DELAY_TICKS  2
#define SIM_ARRIVAL_DONE_MS      240   /** 参考轨迹走完时间上限，ms */
#define SIM_ARRIVAL_REVERSE_CDPS 900   /** 参考走完后的反向峰值速度上限，厘度/秒 */
#define SIM_ARRIVAL_OVER_CDEG    50    /** 超调上限，厘度 */
#define SIM_ARRIVAL_ERROR_CDEG   50    /** 3.5秒时的终点误差上限，厘度 */
#define SIM_PLANT_MOTOR_SIGN 1 /* 实测接线方向，独立于控制器参数 */
