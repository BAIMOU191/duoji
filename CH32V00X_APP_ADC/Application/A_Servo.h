#pragma once

#include <stdint.h>

/* A_Servo.h —— 舵机编排层：模式状态机、控制拍链路、卸力/暂停/停止 */

#define SERVO_INPUT_TX  0U /** 输入源：串口总线 */
#define SERVO_INPUT_PWM 1U /** 输入源：PWM脉宽捕获 */

/* 一个检测区间内的运动量，供保护模块判堵转。位移由本模块自己做差：圆周坐标的
 * 跨0折算只有这里知道怎么算，而保护模块本来就不该碰坐标。 */
typedef struct {
    int32_t  ref_delta;  /** 区间内参考位置的位移，厘度(已折算最短路径) */
    int32_t  act_delta;  /** 区间内实测位置的位移，厘度 */
    uint16_t hold_ticks; /** 区间内轨迹时钟被饱和冻结的控制拍数 */
    uint8_t  valid;      /** 1=整个区间都在跑一条未走完的位置轨迹 */
} ServoMotion_t;

void    A_Servo_Init(void);                       /* 上电初始化：选输入源、建环路、执行上电动作 */
uint8_t A_Servo_InputSource(void);                /* 读本次上电选定的输入源 */
void    A_Servo_Submit(uint16_t pwm, uint16_t value); /* 下发一条运动指令 */
void    A_Servo_Control(void);                    /* 1ms控制拍入口 */

uint8_t  A_Servo_SetMode(uint8_t mode);           /* 切换工作模式并落盘 */
uint16_t A_Servo_GetPositionPwm(void);            /* 读当前位置并换算回协议脉宽 */
uint8_t  A_Servo_CalibrateMid(void);              /* SCK：把当前位置标定为行程中点 */
uint8_t  A_Servo_SetTravelEnd(uint8_t is_min);    /* SMI/SMX：把行程一端收到当前位置 */
uint8_t  A_Servo_SaveStartup(void);               /* 把当前位置存为上电目标 */
void     A_Servo_ApplyConfig(void);               /* 让新的模式/中位参数立即生效 */

uint8_t A_Servo_TorqueOn(void);                   /* 1=当前有扭矩输出(非卸力) */
void    A_Servo_SetOutputLimit(int16_t limit);    /* 设对称PWM上限，位置/电机模式都生效 */
void    A_Servo_SampleMotion(ServoMotion_t *out); /* 取一次运动采样，同时把区间基准推到本拍 */
void    A_Servo_HoldHere(void);                   /* 堵转保护：丢掉剩余轨迹，就地保持 */
void    A_Servo_printf(void);                     /* 按FireWater格式发一帧遥测，队列满则丢帧 */

void A_Servo_Pause(void);                         /* 暂停：输出刹车，保留目标以便继续 */
void A_Servo_Resume(void);                        /* 继续：以当前位置为起点重新规划到原目标 */
void A_Servo_Stop(void);                          /* 停止：丢弃目标，按当前扭矩状态收尾 */
void A_Servo_Release(uint8_t high_resistance);    /* 卸力，0=低阻力自由转，1=高阻力短路制动 */
void A_Servo_RestoreTorque(void);                 /* 从卸力恢复扭矩并重建全部环路 */
