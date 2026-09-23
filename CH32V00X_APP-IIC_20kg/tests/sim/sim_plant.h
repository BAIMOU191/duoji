/*
 * sim_plant.h —— 本工程主机仿真的被控对象(硬件)参数
 * 只描述舵机本体，不引用 A_Parameter.h 的控制器参数：调控制器参数时对象不跟着变，仿真才有判别力。
 * servo_sim.c / feature_audit.c / run.py 四个工程逐字相同，差异全部在本文件。
 * 方向下标：_POS = 角度增大方向(标定日志 dir0)，_NEG = 角度减小方向(dir1)。
 *
 * 本机：MT6701磁编码20kg。对象取标定值(两向差异在重复性以内，不分方向)；
 * 起转负向取正向的0.77，保留旧仿真的方向不对称假设。噪声为运动中 sigma。
 * arrival_soft：tau放大1.27倍(实际惯性比模型大27%)。
 */
#pragma once

#define SIM_PLANT_SLOPE_POS_Q16 7434.0  /** PWM-速度斜率，Q16 */
#define SIM_PLANT_SLOPE_NEG_Q16 7434.0
#define SIM_PLANT_FRIC_POS_PWM  78.0   /** 动摩擦，PWM */
#define SIM_PLANT_FRIC_NEG_PWM  78.0
#define SIM_PLANT_BRK_POS_PWM   200.0  /** 起转(静摩擦)，按出力方向取，PWM */
#define SIM_PLANT_BRK_NEG_PWM   154.0
#define SIM_PLANT_TAU_POS_MS    37.0   /** 机械时间常数，按运动方向取，ms */
#define SIM_PLANT_TAU_NEG_MS    37.0
#define SIM_PLANT_DELAY_TICKS   3      /** 指令至轴响应的纯延迟，控制拍 */
#define SIM_NOISE_CDEG          0.8   /** 位置读数均匀噪声幅值(±)，厘度 */

/* arrival_soft 审计：实际惯性大于模型时，小角度(100us)到位后的反向回拉不能过猛。
 * 失配工况与门限按本机给出；门限取本版控制器(已还原老版本控制链路)仿真实测值留余量，用于发现回退，不是硬件指标。 */
#define SIM_ARRIVAL_NOISE        0.8
#define SIM_ARRIVAL_TAU_SCALE    1.27
#define SIM_ARRIVAL_DELAY_TICKS  3
#define SIM_ARRIVAL_DONE_MS      180   /** 参考轨迹走完时间上限，ms */
#define SIM_ARRIVAL_REVERSE_CDPS 1300  /** 参考走完后的反向峰值速度上限，厘度/秒 */
#define SIM_ARRIVAL_OVER_CDEG    75   /** 超调上限，厘度 */
#define SIM_ARRIVAL_ERROR_CDEG   25    /** 3.5秒时的终点误差上限，厘度 */
#define SIM_PLANT_MOTOR_SIGN 1 /* 实测接线方向，独立于控制器参数 */
