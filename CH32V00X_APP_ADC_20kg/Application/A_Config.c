/* A_Config.c —— 掉电保存参数的加载、默认值与异步落盘(改参数只置脏，由最低优先级任务写Flash) */

#include "A_Config.h"
#include "A_Protect.h"
#include "A_Sensor.h"
#include "flash.h"
#include "iap.h"
#include <stddef.h>
#include <string.h>

Config_t        g_config;
RuntimeConfig_t g_runtime_config;
static const uint8_t k_version[3] = {
    SERVO_VERSION_MAJOR, SERVO_VERSION_MINOR, SERVO_VERSION_PATCH
};

/* 设置与上报共用同一硬件扭矩来源，防止型号信息与固件默认值不一致。 */
uint16_t A_Config_TorqueMax(void)
{
    IAP_HwInfo_t hw;
    uint8_t torque = FW_TORQUE;
    if (IAP_HwInfoRead(&hw) && hw.torque != 0U && hw.torque != 0xFFU)
        torque = hw.torque;
    return (uint16_t)((uint16_t)torque * TORQUE_SCALE);
}

typedef char guard_config_head
    [(offsetof(Config_t, servo_id) == 0U && offsetof(Config_t, baud_code) == 1U) ? 1 : -1];
typedef char guard_config_tail_padding
    [(sizeof(Config_t) == offsetof(Config_t, image_tag)
                       + sizeof(g_config.image_tag)) ? 1 : -1];
typedef char guard_config_exceeds_flash_payload
    [(sizeof(Config_t) <= CONFIG_PAYLOAD_MAX) ? 1 : -1];

/* 固件镜像指纹：对整个固件镜像(向量表、代码、常量、.data 初值)做 FNV-1a 散列。；详见 docs/ALGORITHM_NOTES.md。 */
#ifndef CONFIG_IMAGE_TAG
extern const uint8_t _data_lma[], _data_vma[], _edata[]; /* 链接脚本：.data 初值在 flash 里的位置与长度 */

static uint32_t Config_ImageTag(void)
{
    uint32_t len = (uint32_t)_data_lma + ((uint32_t)_edata - (uint32_t)_data_vma); /* 镜像从地址0起算 */
    const uint32_t *p = (const uint32_t *)FLASH_BASE;
    uint32_t h = 2166136261UL ^ len;
    uint32_t i;

    for (i = 0U; i < len / 4U; i++) h = (h ^ p[i]) * 16777619UL;
    return h;
}
#define CONFIG_IMAGE_TAG() Config_ImageTag()
#endif

static uint32_t s_image_tag;           /** 当前固件的镜像指纹，A_Config_Init 里算一次 */
static uint8_t  s_dirty;               /** 1=内存里的参数比Flash新，待保存 */
static volatile uint8_t *s_save_event; /** 指向保存任务的就绪标志，置1即触发调度 */

/* 把出厂值填进 g_config，不登记保存 */
static void Config_FillDefaults(void)
{
    memset(&g_config, 0, sizeof(g_config));
    g_config.baud_code            = 5U;                /* 5 = 115200 */
    g_config.servo_mode           = SERVO_MODE_270_CW;
    g_config.boot_mode            = SERVO_BOOT_GOTO_START;
    g_config.torque_limit_ckgcm         = A_Config_TorqueMax();
    g_config.startup_pwm          = SERVO_PWM_MID;
    g_config.pulse_lo             = SERVO_PWM_MIN;     /* 脉冲边界回到整个协议量程 */
    g_config.pulse_hi             = SERVO_PWM_MAX;
    g_config.traj_amax_dps2       = CFG_DEF_AMAX_DPS2;
    g_config.traj_dmax_dps2       = CFG_DEF_DMAX_DPS2;
    g_config.traj_jmax_dps3       = CFG_DEF_JMAX_DPS3;
    g_config.traj_vmax_dps        = CFG_DEF_VMAX_DPS;
    g_config.traj_move_min_ms     = CFG_DEF_MOVE_MIN_MS;
    g_config.prot_current_limit_ma= PROT_CURRENT_LIMIT_DEFAULT_MA;
    g_config.prot_current_time_ms = PROT_CURRENT_TIME_DEFAULT_MS;
    g_config.prot_temp_limit_dC   = PROT_TEMP_LIMIT_DEFAULT_DC;
    g_config.prot_temp_time_ms    = PROT_TEMP_TIME_DEFAULT_MS;
    g_config.prot_stall_ref_cdeg  = PROT_STALL_REF_DEFAULT_CDEG;
    g_config.prot_stall_act_cdeg  = PROT_STALL_ACT_DEFAULT_CDEG;
    g_config.prot_stall_trip      = PROT_STALL_TRIP_DEFAULT;
    g_config.hold_stiff           = CFG_DEF_HOLD_STIFF;
    g_config.protect_enable       = PROT_ENABLE_DEFAULT;
    g_config.voltage_min_dv       = PROT_VOLTAGE_MIN_DEFAULT_DV;
    g_config.voltage_max_dv       = PROT_VOLTAGE_MAX_DEFAULT_DV;
    g_config.image_tag            = s_image_tag;       /* 出厂值属于当前这份固件 */
}

void A_Config_ReloadRuntime(uint8_t reset_transient)
{
#define COPY(field) g_runtime_config.field = g_config.field;
    COPY(torque_limit_ckgcm)
    COPY(traj_amax_dps2) COPY(traj_dmax_dps2) COPY(traj_jmax_dps3)
    COPY(traj_vmax_dps) COPY(traj_move_min_ms)
    COPY(prot_current_limit_ma) COPY(prot_current_time_ms)
    COPY(prot_temp_limit_dC) COPY(prot_temp_time_ms)
    COPY(prot_stall_act_cdeg) COPY(prot_stall_trip)
    COPY(hold_stiff) COPY(protect_enable)
    COPY(voltage_min_dv) COPY(voltage_max_dv) COPY(min_start_torque_ckgcm)
#undef COPY
    /* 对外只保留一个堵转阈值；参考位移固定为实际位移阈值的4倍。 */
    g_runtime_config.prot_stall_ref_cdeg   = (uint16_t)(g_config.prot_stall_act_cdeg * 4U);
    if (reset_transient)
    {
        g_runtime_config.torque_mode       = 1U;
        g_runtime_config.damping_percent   = 100U;
    }
}

/* 恢复出厂参数，keep_id=1保留总线ID(CLE0)，0=连ID一起清零(CLE) */
void A_Config_Default(uint8_t keep_id)
{
    uint8_t id = g_config.servo_id; /** 先存下来，清空后再按需要写回 */

    Config_FillDefaults();
    if (keep_id) g_config.servo_id = id;

    A_Config_ReloadRuntime(1U);
    A_Config_MarkDirty();
}

/* 全部取值规则只写这一处：非法字段或字段组修回出厂值(调用时 g_config 里放的是出厂值)；
 * 合法的配置原样不动，所以"修完与原来不同"就是"原来有非法字段"。CRC正确不代表内容有意义。
 * 相互关联的坐标和限位必须一起修复，避免非法行程保留了不匹配的标定锚点。 */
static void Config_Repair(Config_t *c)
{
#define REPAIR(field, valid) do { if (!(valid)) c->field = g_config.field; } while (0)
    REPAIR(servo_id, c->servo_id <= 254U);
    REPAIR(baud_code, c->baud_code >= 1U && c->baud_code <= 8U);
    REPAIR(servo_mode, SERVO_MODE_IS_SUPPORTED(c->servo_mode));
    REPAIR(boot_mode, c->boot_mode >= SERVO_BOOT_GOTO_START && c->boot_mode <= SERVO_BOOT_RELEASE);
    REPAIR(startup_pwm, c->startup_pwm >= SERVO_PWM_MIN && c->startup_pwm <= SERVO_PWM_MAX);
    REPAIR(torque_limit_ckgcm, c->torque_limit_ckgcm <= A_Config_TorqueMax());
    if (c->pulse_lo < SERVO_PWM_MIN || c->pulse_hi > SERVO_PWM_MAX || c->pulse_lo >= c->pulse_hi)
    { c->pulse_lo = g_config.pulse_lo; c->pulse_hi = g_config.pulse_hi; }
    if (c->cal_kind > SERVO_CAL_ZERO || (c->cal_kind != SERVO_CAL_NONE && c->cal_anchor_cdeg > ENCODER_TRAVEL_HI))
    { c->cal_kind = SERVO_CAL_NONE; c->cal_anchor_cdeg = 0U; }
    REPAIR(position_offset_cdeg, c->position_offset_cdeg < CDEG_RANGE);
    if (c->custom_offset_cdeg >= CDEG_RANGE || c->custom_span_cdeg >= CDEG_RANGE
        || c->custom_reverse > 1U || (c->servo_mode == SERVO_MODE_CUSTOM && c->custom_span_cdeg < SERVO_CUSTOM_SPAN_MIN))
    {
        c->custom_offset_cdeg = c->custom_span_cdeg = 0U; c->custom_reverse = 0U;
        if (c->servo_mode == SERVO_MODE_CUSTOM) c->servo_mode = SERVO_MODE_270_CW;
    }
    REPAIR(traj_vmax_dps, (int32_t)c->traj_vmax_dps * 100 <= PLANT_VMAX_HW_CDPS);
    REPAIR(traj_amax_dps2, c->traj_amax_dps2 <= PLANT_AMAX_HW_DPS2);
    REPAIR(traj_dmax_dps2, c->traj_dmax_dps2 <= PLANT_DMAX_HW_DPS2);
    REPAIR(traj_jmax_dps3, c->traj_jmax_dps3 <= PLANT_JMAX_HW_DPS3);
    REPAIR(traj_move_min_ms, c->traj_move_min_ms <= 200U);
    REPAIR(prot_current_limit_ma, c->prot_current_limit_ma >= PROT_CURRENT_LIMIT_MIN_MA && c->prot_current_limit_ma <= PROT_CURRENT_LIMIT_MAX_MA);
    REPAIR(prot_current_time_ms, c->prot_current_time_ms >= PROT_CURRENT_TIME_MIN_MS && c->prot_current_time_ms <= PROT_CURRENT_TIME_MAX_MS);
    REPAIR(prot_temp_limit_dC, c->prot_temp_limit_dC >= PROT_TEMP_LIMIT_MIN_DC && c->prot_temp_limit_dC <= PROT_TEMP_LIMIT_MAX_DC);
    REPAIR(prot_temp_time_ms, c->prot_temp_time_ms >= PROT_TEMP_TIME_MIN_MS && c->prot_temp_time_ms <= PROT_TEMP_TIME_MAX_MS);
    REPAIR(prot_stall_act_cdeg, c->prot_stall_act_cdeg >= PROT_STALL_THRESHOLD_MIN_CDEG && c->prot_stall_act_cdeg <= PROT_STALL_THRESHOLD_MAX_CDEG);
    c->prot_stall_ref_cdeg = (uint16_t)(c->prot_stall_act_cdeg * 4U);
    REPAIR(prot_stall_trip, c->prot_stall_trip >= PROT_STALL_TRIP_MIN && c->prot_stall_trip <= PROT_STALL_TRIP_MAX);
    REPAIR(hold_stiff, c->hold_stiff <= CFG_HOLD_STIFF_MAX);
    c->reserved0 = 0U; memset(c->reserved1, 0, sizeof(c->reserved1));
    if (c->voltage_min_dv < PROT_VOLTAGE_CFG_MIN_DV || c->voltage_max_dv > PROT_VOLTAGE_CFG_MAX_DV
        || (uint16_t)c->voltage_min_dv + PROT_VOLTAGE_HYST_DV > c->voltage_max_dv)
    { c->voltage_min_dv = g_config.voltage_min_dv; c->voltage_max_dv = g_config.voltage_max_dv; }
    REPAIR(min_start_torque_ckgcm, c->min_start_torque_ckgcm <= A_Config_TorqueMax());
#undef REPAIR
}

void A_Config_Init(void)
{
    Config_t stored;
    uint8_t stored_version[3] = { 0xFFU, 0xFFU, 0xFFU };
    uint8_t keep_id = 0U;
    uint8_t keep_baud = 5U;
    uint8_t loaded;

    s_image_tag = CONFIG_IMAGE_TAG();
    Config_FillDefaults();
    stored = g_config;
    loaded = FLASH_Config_Load(&stored, sizeof(stored), stored_version) == sizeof(stored);

    /* 同一份固件写的参数才沿用；换过固件(版本号或镜像任一不同)一律回出厂值，只留ID和波特率 */
    if (loaded && memcmp(stored_version, k_version, sizeof(k_version)) == 0
        && stored.image_tag == s_image_tag)
    {
        Config_t raw;                    /* 修复前的原样(连填充字节)，比较出有没有非法字段 */
        memcpy(&raw, &stored, sizeof(raw));
        Config_Repair(&stored);          /* 此时 g_config 仍是出厂值，修复从它取 */
        if (memcmp(&raw, &stored, sizeof(raw)) != 0) s_dirty = 1U;
        g_config = stored;
    }
    else
    {
        if (loaded)
        {
            keep_id = stored.servo_id;
            keep_baud = stored.baud_code;
        }
        else
        {
            (void)IAP_PreservedIdentityRead(&keep_id, &keep_baud);
        }
        if (keep_id <= 254U) g_config.servo_id = keep_id;
        if (keep_baud >= 1U && keep_baud <= 8U) g_config.baud_code = keep_baud;
        s_dirty = 1U;
    }

    A_Config_ReloadRuntime(1U);
    if (s_dirty) A_Config_SaveTask();
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
    if (s_dirty && FLASH_Config_Save(&g_config, sizeof(g_config), k_version))
    {
        s_dirty = 0U;
    }
    /* 调度器在调用前已消费事件；写入失败必须重新安排低优先级重试，
     * 否则脏配置将一直得不到保存。 */
    if (s_dirty && s_save_event != 0) *s_save_event = 1U;
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
