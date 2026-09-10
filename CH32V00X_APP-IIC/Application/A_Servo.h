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

void A_Servo_Pause(void);
void A_Servo_Resume(void);
void A_Servo_Stop(void);
void A_Servo_Release(uint8_t high_resistance);
void A_Servo_RestoreTorque(void);
