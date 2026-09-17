/* A_Config.c —— 掉电保存参数的加载、默认值与异步落盘(改参数只置脏，由最低优先级任务写Flash) */

#include "A_Config.h"
#include "A_Protect.h"
#include "A_Sensor.h"
#include "flash.h"
#include <stddef.h>
#include <string.h>

Config_t g_config; /** 全工程唯一的参数实例，各模块直接读，改完须MarkDirty */

/* 编译期护栏：版本、ID、波特率的偏移一旦变动，已出厂舵机的ID和波特率就读不回来 */
typedef char guard_config_head_moved
    [(offsetof(Config_t, version) == 0U && offsetof(Config_t, servo_id) == 3U
      && offsetof(Config_t, baud_code) == 4U) ? 1 : -1];
/* 末尾不许有填充字节：否则追加的字段会落进填充区，旧记录长度不变，新字段读到的是填充里的旧值。
 * 追加字段后把这里换成新的最后一个字段 */
typedef char guard_config_tail_padding
    [(sizeof(Config_t) == offsetof(Config_t, cal_anchor_cdeg) + sizeof(uint16_t)) ? 1 : -1];
/* 参数须放得进Flash记录载荷区 */
typedef char guard_config_exceeds_flash_payload
    [(sizeof(Config_t) <= CONFIG_PAYLOAD_MAX) ? 1 : -1];

static const uint8_t SERVO_VERSION[3] = {
    SERVO_VERSION_MAJOR, SERVO_VERSION_MINOR, SERVO_VERSION_PATCH
}; /** 当前固件版本，按参数记录里的字节顺序排好 */

static uint32_t s_sequence;           /** 已落盘记录的序号，双备份靠它判新旧 */
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

/* 把出厂值填进 g_config，不登记保存 */
static void Config_FillDefaults(void)
{
    memcpy(g_config.version, SERVO_VERSION, sizeof(g_config.version));
    g_config.servo_id             = 0U;
    g_config.baud_code            = 5U;                /* 5 = 115200 */
    g_config.servo_mode           = SERVO_MODE_270_CW;
    g_config.boot_mode            = SERVO_BOOT_HOLD;
    g_config.torque_limit         = PROT_TORQUE_DEFAULT;
    g_config.custom_reverse       = 0U;
    g_config.cal_kind             = SERVO_CAL_NONE;    /* 角度校准一并作废 */
    g_config.startup_pwm          = SERVO_PWM_MID;
    g_config.position_offset_cdeg = 0U;
    g_config.custom_offset_cdeg   = 0U;  /* 自定义行程一并作废，回到标准270度 */
    g_config.custom_span_cdeg     = 0U;
    g_config.pulse_lo             = SERVO_PWM_MIN;     /* 脉冲边界回到整个协议量程 */
    g_config.pulse_hi             = SERVO_PWM_MAX;
    g_config.cal_anchor_cdeg      = 0U;
}

/* 恢复出厂参数，keep_id=1保留总线ID(CLE0)，0=连ID一起清零(CLE) */
void A_Config_Default(uint8_t keep_id)
{
    uint8_t id = g_config.servo_id; /** 先存下来，清空后再按需要写回 */

    Config_FillDefaults();
    if (keep_id) g_config.servo_id = id;

    A_Config_MarkDirty();
}

/* 上电加载；须在驱动初始化之前调用，此时调度器未运行故同步落盘
 *   内容合法：全部沿用(不论哪个版本的固件写的)，比当前结构短的部分保持出厂值(后加的字段)
 *   内容越界：其余恢复出厂值，ID和波特率只要合法就保留，避免总线上失联
 *   版本号始终改成当前固件；和Flash里不一致(空白、补过字段、越界、换过固件)就写回 */
void A_Config_Init(void)
{
    Config_t stored; /** Flash 里的记录，先铺出厂值，短记录缺的尾部就是出厂值 */
    uint16_t length; /** 记录实际长度，0=两页都无效 */

    Config_FillDefaults();
    stored = g_config;
    length = FLASH_Config_Load(&stored, sizeof(stored), &s_sequence);

    if (A_Config_Valid(&stored))
    {
        g_config = stored;
        memcpy(g_config.version, SERVO_VERSION, sizeof(g_config.version));
    }
    else
    {
        if (stored.servo_id <= 254U) g_config.servo_id = stored.servo_id;
        if (stored.baud_code >= 1U && stored.baud_code <= 8U) g_config.baud_code = stored.baud_code;
    }

    if (length != sizeof(Config_t) || memcmp(&stored, &g_config, sizeof(Config_t)) != 0)
    {
        s_dirty = 1U;
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
