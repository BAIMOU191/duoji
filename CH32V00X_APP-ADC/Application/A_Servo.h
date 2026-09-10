#pragma once

#include <stdint.h>

#define SERVO_INPUT_TX  0U
#define SERVO_INPUT_PWM 1U

void    A_Servo_Init(void);
uint8_t A_Servo_InputSource(void);
void    A_Servo_Submit(uint16_t pwm, uint16_t value);
void    A_Servo_Control(void);

uint8_t  A_Servo_SetMode(uint8_t mode);
uint16_t A_Servo_GetPositionPwm(void);
uint8_t  A_Servo_CalibrateMid(void);
uint8_t  A_Servo_SetTravelEnd(uint8_t is_min); /* SMI/SMX：把行程一端收到当前位置 */
uint8_t  A_Servo_SaveStartup(void);
void     A_Servo_ApplyConfig(void);

uint8_t A_Servo_TorqueOn(void);                 /* 1=当前有扭矩输出(非卸力) */
void    A_Servo_SetOutputLimit(int16_t limit);  /* 设对称PWM上限，位置/电机模式都生效 */

/* 一个检测区间内的运动量，供保护模块判堵转。位移由本模块自己做差，
 * 一是圆周坐标的跨0折算只有这里知道怎么算，二是保护模块本来就不该碰坐标。 */
typedef struct {
    int32_t  ref_delta;  /* 区间内参考位置的位移，厘度(已折算最短路径) */
    int32_t  act_delta;  /* 区间内实测位置的位移，厘度                 */
    uint16_t hold_ticks; /* 区间内轨迹时钟被饱和冻结的控制拍数         */
    uint8_t  valid;      /* 1=整个区间都在跑一条未走完的位置轨迹       */
} ServoMotion_t;

void A_Servo_SampleMotion(ServoMotion_t *out); /* 取一次运动采样，同时把区间基准推到本拍 */
void A_Servo_HoldHere(void);                   /* 堵转保护：丢掉剩余轨迹，就地保持 */

void A_Servo_printf(void); /* 按FireWater格式发一帧遥测，队列满则丢帧 */

void A_Servo_Pause(void);
void A_Servo_Resume(void);
void A_Servo_Stop(void);
void A_Servo_Release(uint8_t high_resistance);
void A_Servo_RestoreTorque(void);
