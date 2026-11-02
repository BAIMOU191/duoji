#pragma once

/* A_Servo_Ctrl.h —— 舵机执行层："怎么转"。
 * 坐标(模式/校准→行程方向零点) / 算法链(Common层装配) / 输出级(母线、mV↔占空比、限幅、阻尼)。
 * 不含指令、暂停、扭矩、脱困——在 A_Servo.c。 */

#include <stdint.h>
#include "A_Config.h"
#include "C_Traj_Planner.h"
#include "C_Speed_Observer.h"
#include "C_Pos_Ctrl.h"

/* 脉宽内部用 1/8us(Q3)，给PWM输入的回差滤波留精度 */
#define Q3(us)       ((uint16_t)((uint16_t)(us) << 3))
#define Q3_MIN       Q3(SERVO_PWM_MIN)
#define Q3_MID       Q3(SERVO_PWM_MID)
#define Q3_MAX       Q3(SERVO_PWM_MAX)
#define Q3_SPAN      (Q3_MAX - Q3_MIN)

/* 当前几何与位置；只读，由本模块维护 */
typedef struct {
    ServoMode_t mode;
    uint8_t  reverse;           /* 1=脉宽增大角度减小 */
    uint8_t  range_valid;       /* 1=行程与可测范围有交集，否则禁止驱动 */
    uint8_t  read_ok;           /* 诊断：最近一次位置读取是否成功 */
    uint16_t span_cdeg;         /* 有效行程，厘度；0=电机模式 */
    uint16_t angle;             /* 行程坐标[0,span]，只给目标和上报用 */
    uint16_t traj_lo, traj_hi;  /* 与规划器一致的目标范围 */
    int32_t  offset_cdeg;       /* 行程坐标0对应的编码器原始角度 */
    int32_t  angle_circ;        /* 未截断的反馈坐标，闭环只用它 */
    int32_t  raw_angle;         /* 编码器原始角度 */
} ServoGeom_t;

extern const ServoGeom_t *const g_servo_geom;

/* ---- 上电 ---- */
uint8_t  A_ServoCtrl_Init(ServoMode_t mode);      /* 展开模式、读位置、测母线、初始化算法链并静止在实测位置；返回读位置是否成功 */

/* ---- 1. 模式与坐标 ---- */
void     A_ServoCtrl_SetMode(ServoMode_t mode);   /* 按模式号与校准锚点展开行程、方向、零点 */
uint8_t  A_ServoCtrl_IsPosition(void);            /* 位置模式(1~4, 11) */
uint8_t  A_ServoCtrl_IsTurns(void);               /* 电机定圈模式 */
uint8_t  A_ServoCtrl_ReadPosition(void);          /* 读编码器刷新坐标；失败返回0，坐标保持上次值 */
void     A_ServoCtrl_UpdateRange(void);           /* 重算目标范围，有效时同步给规划器 */
uint16_t A_ServoCtrl_PulseToAngle(uint16_t q3);   /* 脉宽(Q3) -> 行程坐标 */
uint16_t A_ServoCtrl_AngleToPwm(uint16_t angle);  /* 行程坐标 -> 协议脉宽；电机模式返回中位 */
uint8_t  A_ServoCtrl_SetCalAnchor(uint8_t kind);  /* SCK/SCZ：校验当前位置并写入锚点(g_config)，不改当前几何 */
uint8_t  A_ServoCtrl_SetTravelEnd(uint8_t is_min);/* AMI/AMX：校验并写入自定义行程(g_config)，不改当前几何 */

/* ---- 2. 算法链 ---- */
void     A_ServoCtrl_ApplyTrajConfig(void);       /* V/A/D、平滑时间、最短运动时间：下一次规划生效 */
void     A_ServoCtrl_ApplyGains(void);            /* 控制带宽(固件整定常量) */
void     A_ServoCtrl_Reset(void);                 /* 观测器从实测位置重新起步，控制器清状态 */
void     A_ServoCtrl_ResetLoop(void);             /* 只清控制器状态 */
void     A_ServoCtrl_Rebase(void);                /* 参考就地静止在实测位置并刷新一拍，清诊断 */
void     A_ServoCtrl_HoldRef(void);               /* 参考立即静止在实测位置 */
void     A_ServoCtrl_Begin(void);                 /* 新运动：清到位资格；参考已停时从实测位置起步 */
void     A_ServoCtrl_StallHold(void);             /* 堵转保持：去掉积分托举力，只留窗外弹簧 */
void     A_ServoCtrl_Command(uint16_t target, uint16_t time_ms);  /* 唯一位置指令入口：time_ms=到达时间(0=最快) */
void     A_ServoCtrl_Brake(void);                 /* 沿当前方向按减速度刹停 */
uint8_t  A_ServoCtrl_TrajDone(void);              /* 规划器参考已停 */
uint8_t  A_ServoCtrl_Holding(void);               /* 参考已停且控制器在保持 */
uint8_t  A_ServoCtrl_Arrived(void);               /* 参考已停且本条运动满足过到位条件 */
void     A_ServoCtrl_Observe(uint8_t measured);   /* 控制拍第1步：观测器吃本拍实测与上拍电压 */
void     A_ServoCtrl_Update(uint8_t measured);    /* 控制拍第2步：推进参考、跑控制器、驱动H桥 */
uint16_t A_ServoCtrl_TakeSaturation(void);        /* 取出并清零"输出饱和且跟踪落后"的累计拍数 */

/* ---- 3. 输出级 ---- */
void     A_ServoCtrl_Output(int16_t duty);        /* 驱动H桥(物理方向占空比)，记下实际施加电压 */
void     A_ServoCtrl_Release(uint8_t high);       /* 卸力：0=两路低(自由转)，1=短路制动 */
void     A_ServoCtrl_Damping(uint8_t percent);    /* 自定义阻尼一拍：按百分比在短路制动和自由释放间交替 */
void     A_ServoCtrl_ResetDamping(void);
void     A_ServoCtrl_SetLimit(int16_t limit);     /* 保护模块给的PWM上限 */
int16_t  A_ServoCtrl_Limit(void);
int16_t  A_ServoCtrl_MvToDuty(int32_t mv);        /* 电压 -> 占空比(±满量程) */
int16_t  A_ServoCtrl_AppliedPwm(void);            /* 诊断：D_Motor_Set 返回的实际PWM */

/* ---- 只读快照(诊断/上报) ---- */
const TrajRef_t     *A_ServoCtrl_Ref(void);
const SpeedObsOut_t *A_ServoCtrl_Obs(void);
const PosCtrlDbg_t  *A_ServoCtrl_Dbg(void);
