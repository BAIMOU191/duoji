/*
 * sim_plant.h —— 本工程主机仿真的被控对象(硬件)参数
 * 只描述舵机本体，不引用 A_Parameter.h 的控制器参数：调控制器参数时对象不跟着变，仿真才有判别力。
 * servo_sim.c / feature_audit.c / run.py 四个工程逐字相同，差异全部在本文件。
 * 方向下标：_POS = 角度增大方向(标定日志 dir0)，_NEG = 角度减小方向(dir1)。
 *
 * 本机：电位器20kg双轴版，两次原始日志见 tests/fixtures/calibration_20kg_double_20260921.log。
 * T1联合拟合两向斜率/截距；T2的L+tau均值减T3=2.03ms得46.405/19.23ms。
 * T9起转取两次各方向上界208/213；噪声取较大峰峰8.93厘度的一半。
 * 模型独立于控制器参数；一阶模型不含齿轮柔性或电源动态，不能证明实物完全无回摆。
 */
#pragma once

#define SIM_PLANT_SLOPE_POS_Q16 7754.0
#define SIM_PLANT_SLOPE_NEG_Q16 8080.0
#define SIM_PLANT_FRIC_POS_PWM  98.0
#define SIM_PLANT_FRIC_NEG_PWM  41.0
#define SIM_PLANT_BRK_POS_PWM   208.0
#define SIM_PLANT_BRK_NEG_PWM   213.0
#define SIM_PLANT_TAU_POS_MS    46.405
#define SIM_PLANT_TAU_NEG_MS    19.23
#define SIM_PLANT_DELAY_TICKS   2      /** 指令至轴响应的纯延迟，控制拍 */
#define SIM_NOISE_CDEG          4.465

/* arrival_soft 审计：实际惯性大于模型时，小角度(100us)到位后的反向回拉不能过猛。
 * 失配工况与门限按本机给出；门限取本版控制器(已还原老版本控制链路)仿真实测值留约20%余量，用于发现回退，不是硬件指标；
 * 老链路在老版20kg标称对象上同一工况为回拉约1550厘度/秒、超调约87厘度。 */
#define SIM_ARRIVAL_NOISE        4.71
#define SIM_ARRIVAL_TAU_SCALE    1.27
#define SIM_ARRIVAL_DELAY_TICKS  2
#define SIM_ARRIVAL_DONE_MS      200   /** 参考轨迹走完时间上限，ms */
#define SIM_ARRIVAL_REVERSE_CDPS 2000  /** 参考走完后的反向峰值速度上限，厘度/秒 */
#define SIM_ARRIVAL_OVER_CDEG    170   /** 超调上限，厘度 */
#define SIM_ARRIVAL_ERROR_CDEG   45    /** 3.5秒时的终点误差上限，厘度 */

#define SIM_PLANT_MOTOR_SIGN 1 /* 实测接线方向，独立于控制器参数 */
