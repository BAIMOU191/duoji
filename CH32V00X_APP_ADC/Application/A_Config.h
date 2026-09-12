#pragma once

#include <stdint.h>
#include "A_Parameter.h"

#define SERVO_VERSION "Servo-V1(2026-09-10)" /** 固件版本号 */

/*
 * A_Config.h —— 掉电保存参数的定义与访问接口
 *
 * 位置模式集中在前，电机模式居中，自定义模式排在最后。自定义模式(11)是唯一
 * 不能用 MODn 手动切入的模式：它的行程由 SMI/SMX 现场标定出来，没标定过的
 * "11"没有意义，所以可写范围卡在 SERVO_MODE_TIMED_CCW，MOD 查询照常返回11。
 *
 * 360度模式(5/6)只在整圈可测的编码器上有意义：电位器整圈里有一段抽头脱离
 * 碳膜的死区，那一段根本测不出角度，整圈绝对定位无从谈起。编号**保持不变**，
 * 只是在不支持的板子上拒绝切进去——MODn 是对外协议，重新编号会让已烧参数的
 * 舵机和上位机脚本全部错位。Flash里遗留的5/6也不会导致整份参数作废(那会连ID
 * 和波特率一起清掉，可能直接失联)，而是在 Servo_SetModeFields 里静默退回270度。
 */

typedef enum {
    SERVO_MODE_270_CW = 1, /* 270度位置，P增大时CW */
    SERVO_MODE_270_CCW,    /* 270度位置，P增大时CCW */
    SERVO_MODE_180_CW,     /* 180度位置，P增大时CW */
    SERVO_MODE_180_CCW,    /* 180度位置，P增大时CCW */
    SERVO_MODE_360_CW,     /* 360度位置，P增大时CW，仅整圈可测的编码器支持 */
    SERVO_MODE_360_CCW,    /* 360度位置，P增大时CCW，仅整圈可测的编码器支持 */
    SERVO_MODE_TURNS_CW,   /* CW定圈，T=0无限 */
    SERVO_MODE_TURNS_CCW,  /* CCW定圈，T=0无限 */
    SERVO_MODE_TIMED_CW,   /* CW定时，T=0无限 */
    SERVO_MODE_TIMED_CCW,  /* CCW定时，T=0无限 */
    SERVO_MODE_CUSTOM      /* 自定义行程位置模式，只能由SMI/SMX进入 */
} ServoMode_t;

#if CFG_WRAP_RANGE_CDEG > 0
#define SERVO_MODE_IS_SUPPORTED(m) \
    ((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_CUSTOM) /** 整圈可测：全部模式可用 */
#else
#define SERVO_MODE_IS_SUPPORTED(m)                              \
    ((m) >= SERVO_MODE_270_CW && (m) <= SERVO_MODE_CUSTOM       \
  && (m) != SERVO_MODE_360_CW && (m) != SERVO_MODE_360_CCW)     /** 有死区：360度模式停用 */
#endif

#define SERVO_CUSTOM_SPAN_MIN 100U /** 自定义行程下限(厘度)，再小每个脉宽档都低于机构分辨率 */

typedef enum {
    SERVO_BOOT_GOTO_START = 1, /* 上电转到startup_pwm */
    SERVO_BOOT_HOLD,           /* 上电保持实际位置 */
    SERVO_BOOT_RELEASE         /* 上电小阻力释放，首条控制指令恢复 */
} ServoBootMode_t;

typedef struct {
    uint16_t startup_pwm;          /** 上电目标脉宽，500~2500 */
    uint16_t position_offset_cdeg; /** SCK中值校正偏移，厘度 */
    uint16_t custom_offset_cdeg;   /** 自定义行程零点对应的编码器角度，厘度 */
    uint16_t custom_span_cdeg;     /** 自定义行程长度，厘度 */
    uint8_t  servo_id;             /** 总线ID，0~254 */
    uint8_t  servo_mode;           /** ServoMode_t，1~11 */
    uint8_t  baud_code;            /** 1~8，默认5=115200 */
    uint8_t  boot_mode;            /** ServoBootMode_t，1~3 */
    uint8_t  torque_limit;         /** 扭矩上限百分比，0~100，SP指令设置 */
    uint8_t  custom_reverse;       /** 自定义模式方向，1=脉宽增大对应角度减小 */
} Config_t;

extern Config_t g_config; /** 全工程唯一的参数实例，各模块直接读，改完须MarkDirty */

void     A_Config_Init(void);                             /* 上电加载；Flash无效或越界则写默认值 */
void     A_Config_Default(uint8_t keep_id);               /* 恢复默认，keep_id=1保留ID */
void     A_Config_MarkDirty(void);                        /* 登记改动，唤醒保存任务，不在指令路径写Flash */
void     A_Config_SaveTask(void);                         /* 最低优先级任务：参数变脏时落盘 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定保存任务的run标志 */
uint32_t A_Config_Baudrate(void);                         /* baud_code换算为实际波特率 */
