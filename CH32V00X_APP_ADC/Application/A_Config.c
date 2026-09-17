/* A_Config.c —— 掉电保存参数的加载、默认值与异步落盘(改参数只置脏，由最低优先级任务写Flash) */

#include "A_Config.h"
#include "A_Protect.h"
#include "A_Sensor.h"
#include "flash.h"
#include <stddef.h>
#include <string.h>

Config_t g_config; /** 全工程唯一的参数实例，各模块直接读，改完须MarkDirty */

/* 编译期护栏：兼容旧记录的前提 */
/* 第一个新增字段必须恰好落在旧长度处 */
typedef char guard_config_legacy_prefix_moved
    [(CONFIG_LEGACY_LEN == offsetof(Config_t, pulse_lo)) ? 1 : -1];
/* 参数须放得进Flash记录载荷区 */
typedef char guard_config_exceeds_flash_payload
    [(sizeof(Config_t) <= CONFIG_PAYLOAD_MAX) ? 1 : -1];

static uint32_t s_sequence;            /** 已落盘记录的序号，双备份靠它判新旧 */
static uint8_t  s_dirty;               /** 1=内存里的参数比Flash新，待保存 */
static volatile uint8_t *s_save_event; /** 指向保存任务的就绪标志，置1即触发调度 */

/* 逐项范围检查，返回1=全部合法(CRC正确不代表内容有意义) */
static uint8_t A_Config_Valid(const Config_t *cfg)
{
    return cfg->startup_pwm >= SERVO_PWM_MIN
        && cfg->startup_pwm <= SERVO_PWM_MAX
        && cfg->pulse_lo >= SERVO_PWM_MIN
        && cfg->pulse_hi <= SERVO_PWM_MAX
        && cfg->pulse_lo < cfg->pulse_hi
        && cfg->cal_kind <= SERVO_CAL_ZERO
        /* 未校准时锚点为0不必检查；下限恒为0不用比 */
        && (cfg->cal_kind == SERVO_CAL_NONE
            || (int32_t)cfg->cal_anchor_cdeg <= ENCODER_TRAVEL_HI)
        && cfg->position_offset_cdeg < CDEG_RANGE
        && cfg->servo_id <= 254U          /* 255保留给广播地址 */
        && cfg->servo_mode >= SERVO_MODE_270_CW
        && cfg->servo_mode <= SERVO_MODE_CUSTOM
        && cfg->custom_offset_cdeg < CDEG_RANGE
        && cfg->custom_span_cdeg < CDEG_RANGE
        && cfg->custom_reverse <= 1U
        /* 自定义行程只在处于该模式时才必须有效 */
        && (cfg->servo_mode != SERVO_MODE_CUSTOM
            || cfg->custom_span_cdeg >= SERVO_CUSTOM_SPAN_MIN)
        && cfg->baud_code >= 1U && cfg->baud_code <= 8U
        && cfg->boot_mode >= SERVO_BOOT_GOTO_START
        && cfg->boot_mode <= SERVO_BOOT_RELEASE
        && cfg->torque_limit <= PROT_TORQUE_MAX;
}

/* 恢复出厂参数，keep_id=1保留总线ID(CLE0)，0=连ID一起清零(CLE) */
void A_Config_Default(uint8_t keep_id)
{
    uint8_t id = g_config.servo_id; /** 先存下来，清空后再按需要写回 */

    g_config.startup_pwm          = SERVO_PWM_MID;
    g_config.position_offset_cdeg = 0U;
    g_config.servo_id             = keep_id ? id : 0U;
    g_config.servo_mode           = SERVO_MODE_270_CW;
    g_config.custom_offset_cdeg   = 0U;  /* 自定义行程一并作废，回到标准270度 */
    g_config.custom_span_cdeg     = 0U;
    g_config.custom_reverse       = 0U;
    g_config.baud_code            = 5U;                /* 5 = 115200 */
    g_config.boot_mode            = SERVO_BOOT_HOLD;
    g_config.torque_limit         = PROT_TORQUE_DEFAULT;
    g_config.pulse_lo             = SERVO_PWM_MIN;     /* 脉冲边界回到整个协议量程 */
    g_config.pulse_hi             = SERVO_PWM_MAX;
    g_config.cal_anchor_cdeg      = 0U;
    g_config.cal_kind             = SERVO_CAL_NONE;    /* 角度校准一并作废 */

    A_Config_MarkDirty();
}

/* 上电加载，Flash无效或越界写默认值；须在驱动初始化之前调用，此时调度器未运行故同步落盘 */
void A_Config_Init(void)
{
    if (!FLASH_Config_Load(&g_config, sizeof(g_config), &s_sequence))
    {
        /* 兼容旧记录：按旧长度再读一次、新字段补默认值并标脏，避免升级后ID/波特率被清导致失联 */
        memset(&g_config, 0, sizeof(g_config));
        if (FLASH_Config_Load(&g_config, CONFIG_LEGACY_LEN, &s_sequence))
        {
            g_config.pulse_lo        = SERVO_PWM_MIN;
            g_config.pulse_hi        = SERVO_PWM_MAX;
            g_config.cal_anchor_cdeg = 0U;
            g_config.cal_kind        = SERVO_CAL_NONE;
            s_dirty = 1U;
        }
    }

    if (!A_Config_Valid(&g_config))
    {
        A_Config_Default(0U);
    }
    if (s_dirty) A_Config_SaveTask(); /* 调度器尚未运行，只能就地同步写入 */
}

/* 登记参数已改动，唤醒保存任务 */
void A_Config_MarkDirty(void)
{
    s_dirty = 1U;
    if (s_save_event != 0) *s_save_event = 1U;
}

/* 绑定保存任务的就绪标志；初始化阶段攒下的dirty在此补触发 */
void A_Config_BindSaveEvent(volatile uint8_t *event)
{
    s_save_event = event;
    if (s_dirty && s_save_event != 0) *s_save_event = 1U;
}

/* 保存任务：参数变脏时写Flash，失败保持dirty下次重试 */
void A_Config_SaveTask(void)
{
    if (s_dirty && FLASH_Config_Save(&g_config, sizeof(g_config), s_sequence + 1U))
    {
        s_sequence++;
        s_dirty = 0U;
    }
}

/* 把配置里的波特率编号换算成实际波特率，编号越界时返回115200 */
uint32_t A_Config_Baudrate(void)
{
    /* 下标 = baud_code - 1，与协议的 BD1~BD8 一一对应 */
    static const uint32_t baud[] = {
        9600U, 19200U, 38400U, 57600U,
        115200U, 128000U, 256000U, 1000000U
    };

    return baud[(g_config.baud_code >= 1U && g_config.baud_code <= 8U)
              ? g_config.baud_code - 1U : 4U];
}
