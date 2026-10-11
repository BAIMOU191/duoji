#pragma once

#include <stdint.h>

/* A_Servo.h —— 舵机编排层：模式、运动指令、控制拍、卸力/暂停/停止 */

#define SERVO_INPUT_TX  0U   /* 输入源：串口总线 */
#define SERVO_INPUT_PWM 1U   /* 输入源：PWM脉宽捕获 */

typedef enum {
    SERVO_WORK_IDLE = 0, SERVO_WORK_MOVING, SERVO_WORK_PAUSED, SERVO_WORK_RELEASE_LOW,
    SERVO_WORK_RELEASE_HIGH, SERVO_WORK_RECOVERING, SERVO_WORK_FAULT
} ServoWorkState_t;

/* 一个堵转检测区间内的运动量 */
typedef struct {
    int32_t  ref_delta;   /* 参考位移，厘度 */
    int32_t  act_delta;   /* 实测位移，厘度 */
    uint16_t hold_ticks;  /* 轨迹时钟被饱和冻结的拍数 */
    uint8_t  valid;       /* 1=整个区间都在跑同一条未走完的轨迹 */
} ServoMotion_t;

/* 集合查询用的缓存快照；位置为协议脉宽，速度按位置值增大方向为正 */
typedef struct {
    uint16_t target_pwm;
    uint16_t position_pwm;
    int32_t  speed_cdps;  /* 0.01度/秒；INT32_MIN=观测器当前不在更新 */
} ServoTelemetry_t;

void     A_Servo_Init(void);
void     A_Servo_Control(void);                         /* 1ms控制拍 */
void     A_Servo_Submit(uint16_t pwm, uint16_t value);  /* 运动指令：脉宽 + 时间(ms，电机定圈模式为圈数) */
void     A_Servo_Pause(void);
void     A_Servo_Resume(void);
void     A_Servo_Stop(void);
void     A_Servo_Release(uint8_t high_resistance);      /* 卸力：0=自由转，1=短路制动 */
void     A_Servo_RestoreTorque(void);
uint8_t  A_Servo_SetTorqueMode(uint8_t mode);           /* 0=自由，1=正常，2=自定义阻尼 */
uint8_t  A_Servo_SetDamping(uint8_t percent);           /* 自定义阻尼 0~100% */
uint8_t  A_Servo_SetMode(uint8_t mode);                 /* 切换工作模式并落盘 */
uint8_t  A_Servo_CalibrateMid(void);                    /* SCK：当前位置为行程中点 */
uint8_t  A_Servo_CalibrateZero(void);                   /* SCZ：当前位置为500us端 */
uint8_t  A_Servo_SetTravelEnd(uint8_t is_min);          /* AMI/AMX：行程一端收到当前位置 */
uint8_t  A_Servo_SetPulseLimit(uint8_t is_min);         /* SMI/SMX：当前位置的脉宽设为可响应边界 */
uint8_t  A_Servo_SaveStartup(void);                     /* 当前位置存为上电目标 */
void     A_Servo_ApplyConfig(void);                     /* 让模式/校准参数立即生效 */
void     A_Servo_ApplyTrajConfig(void);                 /* 让轨迹参数从下一次规划起生效 */
uint8_t  A_Servo_TorqueOn(void);
uint8_t  A_Servo_GetWorkState(void);                    /* ServoWorkState_t */
void     A_Servo_GetTelemetry(ServoTelemetry_t *out);   /* 只读缓存，不改变控制状态 */
uint16_t A_Servo_GetPositionPwm(void);                  /* 重读位置并换算为协议脉宽 */
void     A_Servo_SetOutputLimit(int16_t limit);         /* 保护模块设PWM上限 */
void     A_Servo_SampleMotion(ServoMotion_t *out);      /* 堵转采样，并把区间基准推到本拍 */
void     A_Servo_HoldHere(void);                        /* 堵转：放弃当前动作就地保持 */
