#pragma once

#include <stdint.h>
#include "A_Parameter.h"

#define SERVO_VERSION   "Servo-V1(2026-09-07)"       /* 固件版本号      */


/*
 * 位置模式集中在前，电机模式居中，自定义模式排在最后。
 *
 * 自定义模式(11)是唯一不能用 MODn 手动切入的模式：它的行程由 SMI/SMX 现场
 * 标定出来，没有标定过的"11"没有意义。所以 A_Servo_SetMode 和协议解码都把
 * 可写范围卡在 SERVO_MODE_TIMED_CCW，只有 A_Servo_SetTravelEnd 能写进去；
 * MOD 查询照常返回 11。
 */
typedef enum {
    SERVO_MODE_270_CW = 1, /* 270°位置，P增大时CW */
    SERVO_MODE_270_CCW,    /* 270°位置，P增大时CCW */
    SERVO_MODE_180_CW,     /* 180°位置，P增大时CW */
    SERVO_MODE_180_CCW,    /* 180°位置，P增大时CCW */
    SERVO_MODE_360_CW,     /* 【已停用】360°位置，P增大时CW  */
    SERVO_MODE_360_CCW,    /* 【已停用】360°位置，P增大时CCW */
    SERVO_MODE_TURNS_CW,   /* CW定圈，T=0无限 */
    SERVO_MODE_TURNS_CCW,  /* CCW定圈，T=0无限 */
    SERVO_MODE_TIMED_CW,   /* CW定时，T=0无限 */
    SERVO_MODE_TIMED_CCW,  /* CCW定时，T=0无限 */
    SERVO_MODE_CUSTOM      /* 自定义行程位置模式，只能由SMI/SMX进入 */
} ServoMode_t;

/* 360度模式(5/6)已停用：电位器整圈里有一段抽头脱离碳膜的死区，那一段根本
 * 测不出角度，整圈绝对定位无从谈起。
 *
 * 编号**保持不变**，只是拒绝切进去：MODn 是对外协议，重新编号会让已经烧过
 * 参数的舵机和上位机脚本全部错位，代价远大于省下两个空号。
 * Flash里遗留的5/6不会导致整份参数作废(那会连ID和波特率一起清掉，可能直接
 * 失联)，而是在 Servo_SetModeFields 里静默退回270度。 */
#define SERVO_MODE_IS_SUPPORTED(m)                                  \
    ((m) >= SERVO_MODE_270_CW    && (m) <= SERVO_MODE_CUSTOM        \
  && (m) != SERVO_MODE_360_CW    && (m) != SERVO_MODE_360_CCW)

/* 自定义行程的下限。协议只有2000个脉宽档，行程再小每档就远低于机构
 * 0.1度的分辨率，收到这种指令按无效丢弃，避免把舵机标成一碰就满量程。 */
#define SERVO_CUSTOM_SPAN_MIN  100U  /* 厘度，1度 */

typedef enum {
    SERVO_BOOT_GOTO_START = 1, /* 上电转到startup_pwm */
    SERVO_BOOT_HOLD,           /* 上电保持实际位置 */
    SERVO_BOOT_RELEASE         /* 上电小阻力释放，首条控制指令恢复 */
} ServoBootMode_t;

typedef struct {
    uint16_t startup_pwm;          /* 上电目标，500~2500 */
    uint16_t position_offset_cdeg; /* SCK中值校正偏移，0~35999厘度 */
    uint16_t custom_offset_cdeg;   /* 自定义行程坐标零点对应的编码器角度，0~35999 */
    uint16_t custom_span_cdeg;     /* 自定义行程长度，厘度 */
    uint8_t  servo_id;             /* 总线ID，0~254 */
    uint8_t  servo_mode;           /* ServoMode_t，1~11 */
    uint8_t  baud_code;            /* 1~8，默认5=115200 */
    uint8_t  boot_mode;            /* ServoBootMode_t，1~3 */
    uint8_t  torque_limit;         /* 扭矩上限百分比，0~100，SP指令设置 */
    uint8_t  custom_reverse;       /* 自定义模式的方向，1=脉宽增大对应角度减小 */
} Config_t;

extern Config_t g_config;

void     A_Config_Init(void);                 /* Flash有效则加载，否则写入默认值 */
void     A_Config_Default(uint8_t keep_id);   /* 恢复默认，keep_id=1保留ID */
void     A_Config_MarkDirty(void);            /* 只投递保存请求，不在指令路径写Flash */
void     A_Config_SaveTask(void);             /* 低优先级异步任务消费保存请求 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定周期0任务的run标志 */
uint32_t A_Config_Baudrate(void);              /* baud_code换算为实际波特率 */
