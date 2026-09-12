/*
 * A_Config.c —— 掉电保存参数的加载、默认值与异步落盘
 *
 * 写Flash要擦一整页再写，耗时是毫秒级的，绝对不能放在指令路径或1ms控制拍里。
 * 所以这里把"改参数"和"写Flash"拆成两步：
 *
 *     指令处理 -> 改 g_config -> A_Config_MarkDirty() 置事件
 *                                          |
 *                             最低优先级任务 A_Config_SaveTask() 真正落盘
 *
 * 谁都可以直接读 g_config；改完必须调 MarkDirty，否则断电就丢。
 */

#include "A_Config.h"
#include "A_Protect.h"
#include "flash.h"

Config_t g_config; /** 全工程唯一的参数实例，各模块直接读，改完须MarkDirty */

static uint32_t s_sequence;            /** 已落盘记录的序号，双备份靠它判新旧 */
static uint8_t  s_dirty;               /** 1=内存里的参数比Flash新，待保存 */
static volatile uint8_t *s_save_event; /** 指向保存任务的就绪标志，置1即触发调度 */

/* 逐项检查参数是否落在合法范围内，返回1=全部合法。
 * CRC只能证明"没有位翻转"，证明不了"内容有意义"：跨版本升级、手工烧录、
 * 结构体改动都可能留下校验正确但语义越界的记录，所以加载后还要过一遍范围检查。 */
static uint8_t A_Config_Valid(const Config_t *cfg)
{
    return cfg->startup_pwm >= SERVO_PWM_MIN
        && cfg->startup_pwm <= SERVO_PWM_MAX
        && cfg->position_offset_cdeg < CDEG_RANGE
        && cfg->servo_id <= 254U          /* 255保留给广播地址 */
        && cfg->servo_mode >= SERVO_MODE_270_CW
        && cfg->servo_mode <= SERVO_MODE_CUSTOM
        && cfg->custom_offset_cdeg < CDEG_RANGE
        && cfg->custom_span_cdeg < CDEG_RANGE
        && cfg->custom_reverse <= 1U
        /* 自定义行程只有在真的处于该模式时才必须有效：其它模式下这几项
         * 是上一次标定留下的历史值(或出厂的0)，不该因此判定整份参数无效。 */
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

    A_Config_MarkDirty();
}

/* 上电加载参数，Flash无效或内容越界时写入默认值。
 * 必须在所有驱动初始化之前调用：波特率、工作模式都要按加载结果配置。此时任务表
 * 还没建立，MarkDirty的事件没人消费，所以这里直接同步落一次盘。 */
void A_Config_Init(void)
{
    if (!FLASH_Config_Load(&g_config, sizeof(g_config), &s_sequence)
            || !A_Config_Valid(&g_config))
    {
        A_Config_Default(0U);
        A_Config_SaveTask(); /* 调度器尚未运行，只能就地同步写入 */
    }
}

/* 登记一次"参数已改动"，唤醒最低优先级的保存任务 */
void A_Config_MarkDirty(void)
{
    s_dirty = 1U;
    if (s_save_event != 0) *s_save_event = 1U;
}

/* 绑定保存任务的就绪标志。该任务周期为0不会被时基唤醒，只靠这个标志触发；
 * 绑定发生在 A_Config_Init 之后，所以要补一次，初始化阶段攒下的dirty不能漏掉。 */
void A_Config_BindSaveEvent(volatile uint8_t *event)
{
    s_save_event = event;
    if (s_dirty && s_save_event != 0) *s_save_event = 1U;
}

/* 保存任务入口：参数变脏时写Flash。写失败(擦除超时、回读不一致)时保持dirty，
 * 下次事件到来自动重试，不会把一次偶发失败变成永久丢参数。 */
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
