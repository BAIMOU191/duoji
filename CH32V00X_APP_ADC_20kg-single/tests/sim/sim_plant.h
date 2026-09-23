/*
 * sim_plant.h —— 本工程主机仿真的被控对象(硬件)参数
 * 只描述舵机本体，不引用 A_Parameter.h 的控制器参数：调控制器参数时对象不跟着变，仿真才有判别力。
 * servo_sim.c / feature_audit.c / run.py 四个工程逐字相同，差异全部在本文件。
 * 方向下标：_POS = 角度增大方向(标定日志 dir0)，_NEG = 角度减小方向(dir1)。
 *
 * 本机：电位器20kg单轴版(2026-09-21两次标定，日志见 tests/fixtures/calibration_20kg_single_20260921.log)。
 * T1两次共20点分向拟合：dir0 斜率4797/截距67，dir1 斜率5036/截距-10。负的动摩擦物理上不存在，
 * dir1各点按零摩擦反算斜率5032~5061、与拟合一致，截距-10在拟合误差内，取0；dir1低速阻力由起转值体现；
 * T2两向L+tau 60.2/32.6ms 减 T3纯延迟约2.4ms 作两向时间常数；T9起转取两次上界165/192；延迟2拍；
 * 噪声为静止峰峰9.42厘度的一半。arrival_soft：tau再放大1.27倍(慢向约73ms)。
 */
#pragma once

#define SIM_PLANT_SLOPE_POS_Q16 4797.0  /** PWM-速度斜率，Q16 */
#define SIM_PLANT_SLOPE_NEG_Q16 5036.0
#define SIM_PLANT_FRIC_POS_PWM  67.0   /** 动摩擦，PWM */
#define SIM_PLANT_FRIC_NEG_PWM  0.0
#define SIM_PLANT_BRK_POS_PWM   165.0  /** 起转(静摩擦)，按出力方向取，PWM */
#define SIM_PLANT_BRK_NEG_PWM   192.0
#define SIM_PLANT_TAU_POS_MS    57.8   /** 机械时间常数，按运动方向取，ms */
#define SIM_PLANT_TAU_NEG_MS    30.2
#define SIM_PLANT_DELAY_TICKS   2      /** 指令至轴响应的纯延迟，控制拍 */
#define SIM_NOISE_CDEG          4.71   /** 位置读数均匀噪声幅值(±)，厘度 */

/* arrival_soft 审计：实际惯性大于模型时，小角度(100us)到位后的反向回拉不能过猛。
 * 失配工况与门限按本机给出；门限取本版控制器(已还原老版本控制链路)仿真实测值留约20%余量，用于发现回退，不是硬件指标；
 * 老链路在老版20kg标称对象上同一工况为回拉约1550厘度/秒、超调约87厘度。 */
#define SIM_ARRIVAL_NOISE        4.71
#define SIM_ARRIVAL_TAU_SCALE    1.27
#define SIM_ARRIVAL_DELAY_TICKS  2
#define SIM_ARRIVAL_DONE_MS      200   /** 参考轨迹走完时间上限，ms */
#define SIM_ARRIVAL_REVERSE_CDPS 1750  /** 参考走完后的反向峰值速度上限，厘度/秒 */
#define SIM_ARRIVAL_OVER_CDEG    90   /** 超调上限，厘度 */
#define SIM_ARRIVAL_ERROR_CDEG   45    /** 3.5秒时的终点误差上限，厘度 */

#define SIM_PLANT_MOTOR_SIGN -1 /* 实测接线方向，独立于控制器参数 */
