#pragma once

#include <stdint.h>
#include "A_Config.h"

/* A_Servo.h —— 舵机编排层：模式、运动指令、控制拍、卸力/暂停/停止 */

#define SERVO_INPUT_TX  0U   /* 输入源：串口总线 */
#define SERVO_INPUT_PWM 1U   /* 输入源：PWM脉宽捕获 */

typedef enum {
    SERVO_WORK_IDLE = 0, SERVO_WORK_MOVING, SERVO_WORK_PAUSED, SERVO_WORK_RELEASE_LOW,
    SERVO_WORK_RELEASE_HIGH, SERVO_WORK_RECOVERING, SERVO_WORK_FAULT
} ServoWorkState_t;
/* 位置模式：接受新动作为MOVING，参考停止且控制器首次进入保持后为IDLE；保持中被推动仍为IDLE。 */

/* 一个堵转检测区间内的运动量 */
typedef struct {
    int32_t  ref_delta;   /* 参考位移，厘度 */
    int32_t  act_delta;   /* 实测位移，厘度 */
    uint16_t saturation_ticks; /* 输出饱和且跟踪落后的拍数；参考仍正常推进 */
    uint8_t  valid;       /* 1=整个区间都在跑同一条未走完的轨迹 */
} ServoMotion_t;

/* 集合查询用的缓存快照，内部单位；位置为脉宽 Q3(1/8us)，由参数表层换算对外单位；速度按位置值增大方向为正 */
typedef struct {
    uint16_t target_q3;
    uint16_t position_q3;
    int32_t  speed_cdps;  /* 厘度/秒；INT32_MIN=观测器当前不在更新 */
} ServoTelemetry_t;

/* 诊断快照：位置/速度按行程增大为正，不按协议翻转。
 * flags: bit0=本拍闭环，bit1=位置有效，bit2=观测器粘住，bit3=电流测得到；bit0=0时速度/诊断输出置0。
 * state: 0=MOVE 1=SETTLE 2=HOLD；bit0=0时为3。工作状态用协议 0xC0 查。 */
typedef struct {
    int32_t target_cdeg, ref_cdeg, actual_cdeg;
    int32_t ref_cdps, actual_cdps;
    int16_t pwm;                         /* 实际限幅后的有符号PWM，物理方向，±2400 */
    int16_t ff_mv, fb_mv, i_mv, stick_mv, out_mv;
    int32_t dist_mv;                     /* 观测器扰动估计，仅供诊断 */
    int16_t cur_ma;                      /* 绕组电流 mA，角度增大方向为正(与 out_mv 同号=出力，异号=回馈)；flags bit3=0 时测不到 */
    int32_t enc_raw;                     /* 编码器原始读数(电位器：ADC块均值Q4计数；磁编码：14位计数)，未就绪为-1 */
    uint8_t saturated, state, flags, input_source;
} ServoDebugTelemetry_t;

void     A_Servo_Init(void);
void     A_Servo_Control(void);                         /* 1ms控制拍 */
/* 运动指令受理结果 */
typedef enum { SERVO_SUBMIT_OK = 0, SERVO_SUBMIT_VALUE, SERVO_SUBMIT_STATE } ServoSubmit_t;
/* 运动指令：脉宽 + 时间(ms，电机定圈模式为圈数)。返回 ServoSubmit_t：VALUE=脉宽超出协议量程或 SMI/SMX 边界，
 * STATE=故障/编码器失效/行程无效/扭矩上限不允许驱动。重复目标、被合并、多圈分段续接、脱困中排队都算受理 */
uint8_t  A_Servo_Submit(uint16_t pwm, uint16_t value);       /* 文本协议：pwm 单位 us，500~2500 */
uint8_t  A_Servo_SubmitQ3(uint16_t q3, uint16_t value);      /* 同上，脉宽 Q3(1/8us)，Q3_MIN~Q3_MAX */
#if ENCODER_IS_CIRCULAR
#define SERVO_TURNS_MAX 9U   /* 多圈运动额外整圈数上限(明文只有1位数字，二进制同口径) */
uint8_t  A_Servo_SubmitTurns(uint16_t pwm, uint8_t turns, uint16_t turn_ms);   /* 多圈：0~9圈，turn_ms为每圈时间 */
uint8_t  A_Servo_SubmitTurnsQ3(uint16_t q3, uint8_t turns, uint16_t turn_ms);  /* 同上，脉宽 Q3(1/8us) */
#endif
void     A_Servo_Pause(void);
void     A_Servo_Resume(void);
void     A_Servo_Stop(void);
void     A_Servo_Release(uint8_t high_resistance);      /* 卸力：0=自由转，1=短路制动 */
void     A_Servo_RestoreTorque(void);
void     A_Servo_SyncTorque(void);                      /* 用户请求与保护禁止合成实际状态 */
uint8_t  A_Servo_SetTorqueMode(uint8_t mode);           /* 0=自由，1=正常，2=自定义阻尼 */
uint8_t  A_Servo_SetDamping(uint8_t percent);           /* 自定义阻尼 0~100% */
uint8_t  A_Servo_SetMode(uint8_t mode);                 /* 切换工作模式并落盘 */
uint8_t  A_Servo_CalibrateMid(void);                    /* SCK：当前位置为行程中点 */
uint8_t  A_Servo_CalibrateZero(void);                   /* SCZ：当前位置为500us端 */
#define SERVO_TRIM_MAX_US 999   /* 中点微调幅度上限，us(明文3位数字) */
uint8_t  A_Servo_CalibrateTrim(int16_t us);             /* SCK±n：转到1500+n us，到位后按实测位置做SCK；返回 ServoSubmit_t */
uint8_t  A_Servo_SetTravelEnd(uint8_t is_min);          /* AMI/AMX：行程一端收到当前位置 */
uint8_t  A_Servo_SetPulseLimit(uint8_t is_min);         /* SMI/SMX：当前位置的脉宽设为可响应边界 */
uint8_t  A_Servo_SaveStartup(void);                     /* 当前位置存为上电目标 */
void     A_Servo_ApplyConfig(void);                     /* 让模式/校准参数立即生效 */
void     A_Servo_FactoryReset(uint8_t keep_id);         /* 恢复出厂并复刻上电动作；非0仅保留ID */
uint8_t  A_Servo_CanSave(void);                         /* 1=当前无运动，可安全写Flash(擦写冻结CPU数ms) */
void     A_Servo_ApplyTrajConfig(void);                 /* V/A/D、平滑时间、最短运动时间：下一次规划生效 */
uint8_t  A_Servo_GetWorkState(void);                    /* ServoWorkState_t */
void     A_Servo_GetTelemetry(ServoTelemetry_t *out);   /* 只读缓存，不改变控制状态 */
void     A_Servo_GetDebugTelemetry(ServoDebugTelemetry_t *out); /* 只读控制快照 */
uint16_t A_Servo_GetPositionQ3(void);                   /* 重读位置并换算为脉宽 Q3 */
int8_t   A_Servo_UserSign(void);                        /* 硬件方向(正PWM)→协议方向(位置值增大为正)的符号 ±1 */
void     A_Servo_SetOutputLimit(int16_t limit);         /* 保护模块设PWM上限 */
void     A_Servo_SampleMotion(ServoMotion_t *out);      /* 堵转采样，并把区间基准推到本拍 */
void     A_Servo_HoldHere(void);                        /* 堵转：当前角度为目标，直接进保持；同普通到位，无积分托举力，限流0.1A */
