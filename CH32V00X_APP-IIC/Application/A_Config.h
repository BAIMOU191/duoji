#pragma once

#include <stdint.h>
#include "A_Parameter.h"

#define SERVO_VERSION "Servo-V1(2026-09-10)" /** 固件版本号 */

/*
 * A_Config.h —— 掉电保存参数与访问接口
 * 模式编号是对外协议，不重新编号：11(自定义)只能由 AMI/AMX 进入；360度(5/6)仅整圈可测的编码器支持，
 * Flash 里遗留的5/6由 Servo_SetModeFields 退回270度。
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
    SERVO_MODE_CUSTOM      /* 自定义行程位置模式，只能由AMI/AMX进入 */
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

/* 校准记账：Flash 存校准时的编码器原始角度(锚点)，有效行程与零点每次由"锚点+模式标称行程"现推，
 * 所以反复校准、切换180/270度都不会叠加。 */
typedef enum {
    SERVO_CAL_NONE = 0, /* 未校准，零点取 Flash 里的历史偏移值 */
    SERVO_CAL_MID,      /* SCK：锚点是行程中点(1500us 对应的角度) */
    SERVO_CAL_ZERO      /* SCZ：锚点是行程起点(500us 对应的角度) */
} ServoCalKind_t;

typedef enum {
    SERVO_BOOT_GOTO_START = 1, /* 上电转到startup_pwm */
    SERVO_BOOT_HOLD,           /* 上电保持实际位置 */
    SERVO_BOOT_RELEASE         /* 上电小阻力释放，首条控制指令恢复 */
} ServoBootMode_t;

/* 新增字段只能追加在末尾：升级时按旧长度 CONFIG_LEGACY_LEN 兼容读取旧记录 */
typedef struct {
    uint16_t startup_pwm;          /** 上电目标脉宽，500~2500 */
    uint16_t position_offset_cdeg; /** 标准模式零点偏移(厘度)；已校准时由锚点现推 */
    uint16_t custom_offset_cdeg;   /** 自定义行程零点对应的编码器角度，厘度 */
    uint16_t custom_span_cdeg;     /** 自定义行程标称长度，厘度，只由 AMI/AMX 写 */
    uint8_t  servo_id;             /** 总线ID，0~254 */
    uint8_t  servo_mode;           /** ServoMode_t，1~11 */
    uint8_t  baud_code;            /** 1~8，默认5=115200 */
    uint8_t  boot_mode;            /** ServoBootMode_t，1~3 */
    uint8_t  torque_limit;         /** 扭矩上限百分比，0~100，SP指令设置 */
    uint8_t  custom_reverse;       /** 自定义模式方向(未经 CFG_DIR_INVERT 翻转的原始值) */
    /* ---- 以下为后加字段，只能继续往下追加 ---- */
    uint16_t pulse_lo;             /** SMI 标定的脉冲下界，500~2500 */
    uint16_t pulse_hi;             /** SMX 标定的脉冲上界，500~2500，须 > pulse_lo */
    uint16_t cal_anchor_cdeg;      /** 校准锚点的编码器原始角度，厘度 */
    uint8_t  cal_kind;             /** ServoCalKind_t，0=未校准 */
} Config_t;

#define CONFIG_LEGACY_LEN 14U /** 加入 pulse_lo 之前的 Config_t 长度，升级时按它兼容读取 */

extern Config_t g_config; /** 全工程唯一的参数实例，各模块直接读，改完须MarkDirty */

void     A_Config_Init(void);                             /* 上电加载；Flash无效或越界则写默认值 */
void     A_Config_Default(uint8_t keep_id);               /* 恢复默认，keep_id=1保留ID */
void     A_Config_MarkDirty(void);                        /* 登记改动，唤醒保存任务，不在指令路径写Flash */
void     A_Config_SaveTask(void);                         /* 最低优先级任务：参数变脏时落盘 */
void     A_Config_BindSaveEvent(volatile uint8_t *event); /* 绑定保存任务的run标志 */
uint32_t A_Config_Baudrate(void);                         /* baud_code换算为实际波特率 */
