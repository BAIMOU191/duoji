/* A_Config.c —— 掉电保存参数的加载、默认值与异步落盘(改参数只置脏，由最低优先级任务写Flash)。
 * 各参数的范围与校验在 A_Param.c 的参数表里，这里只管存取。 */

#include "A_Config.h"
#include "A_Param.h"
#include "flash.h"
#include "iap.h"
#include <stddef.h>
#include <string.h>

Config_t        g_config;
RuntimeConfig_t g_runtime_config;
static const uint8_t k_version[3] = {
    SERVO_VERSION_MAJOR, SERVO_VERSION_MINOR, SERVO_VERSION_PATCH
};

/* ==================== 编译期校验(只检查，不产生代码；A_Config.h / App_Model.h 的参数改错在这里报错) ==================== */
_Static_assert(MOTOR_KV_UV > 0 && MOTOR_KA_NV > 0 && MOTOR_FS_MV >= MOTOR_FC_MV && MOTOR_DELAY_US < 7000
               && MOTOR_CAL_MV > 0 && MOTOR_MIN_STEP_CDEG >= 1 && MOTOR_MIN_STEP_CDEG <= 255, "Invalid motor calibration");
_Static_assert(MOTOR_SIGN == 1 || MOTOR_SIGN == -1, "Invalid motor direction");
_Static_assert(TUNE_POS_BW_RADS * 4U <= TUNE_VEL_BW_RADS && TUNE_INT_BW_RADS <= TUNE_POS_BW_RADS && TUNE_INT_BW_RADS >= 1U
               && TUNE_VEL_BW_RADS * 1000U <= TUNE_OBS_BW_HZ * 6283U
               && TUNE_OBS_DIST_HZ >= 1U && TUNE_OBS_DIST_HZ <= TUNE_OBS_BW_HZ && TUNE_OBS_LOAD_RATIO_X10 <= 40U, "Controller bandwidths violate constraints");
_Static_assert(CFG_DEF_RESOLUTION_CDEG >= CFG_RESOLUTION_MIN_CDEG && CFG_DEF_RESOLUTION_CDEG <= CFG_RESOLUTION_MAX_CDEG
               && CFG_RESOLUTION_MIN_CDEG >= 1U
               && CFG_DEF_HOLD_KP <= CFG_HOLD_KP_MAX
               && CFG_DEF_HOLD_KI <= CFG_HOLD_KI_MAX
               && (CFG_DEF_SMOOTH_MS == 0U || (CFG_DEF_SMOOTH_MS >= CFG_SMOOTH_MIN_MS && CFG_DEF_SMOOTH_MS <= CFG_SMOOTH_MAX_MS))
               && CFG_DEF_MOVE_MIN_MS <= CFG_MOVE_MIN_MAX_MS,
               "Invalid default motion/hold parameters");
/* 平滑时间自动值 τ = ka/kv 也不能超过上限 */
_Static_assert(CFG_SMOOTH_MIN_MS >= 1U && CFG_SMOOTH_MIN_MS <= CFG_SMOOTH_MAX_MS
               && (MOTOR_KA_NV + MOTOR_KV_UV - 1) / MOTOR_KV_UV <= CFG_SMOOTH_MAX_MS, "Invalid smoothing time range");
_Static_assert((PROT_CURRENT_LIMIT_DEFAULT_MA >= PROT_CURRENT_LIMIT_MIN_MA) && (PROT_CURRENT_LIMIT_DEFAULT_MA <= PROT_CURRENT_LIMIT_MAX_MA)
               && (PROT_CURRENT_TIME_DEFAULT_MS >= PROT_CURRENT_TIME_MIN_MS) && (PROT_CURRENT_TIME_DEFAULT_MS <= PROT_CURRENT_TIME_MAX_MS)
               && (PROT_TEMP_LIMIT_DEFAULT_DC >= PROT_TEMP_LIMIT_MIN_DC) && (PROT_TEMP_LIMIT_DEFAULT_DC <= PROT_TEMP_LIMIT_MAX_DC)
               && (PROT_TEMP_TIME_DEFAULT_MS >= PROT_TEMP_TIME_MIN_MS) && (PROT_TEMP_TIME_DEFAULT_MS <= PROT_TEMP_TIME_MAX_MS)
               && (PROT_STALL_TRIP_DEFAULT >= PROT_STALL_TRIP_MIN) && (PROT_STALL_TRIP_DEFAULT <= PROT_STALL_TRIP_MAX)
               && (PROT_STALL_ACT_DEFAULT_CDEG >= PROT_STALL_THRESHOLD_MIN_CDEG)
               && (PROT_STALL_ACT_DEFAULT_CDEG <= PROT_STALL_THRESHOLD_MAX_CDEG)
               && (PROT_STALL_REF_RATIO > 1U) && (PROT_STALL_THRESHOLD_MIN_CDEG >= 1U)
               && (PROT_ENABLE_DEFAULT & (uint8_t)~PROT_ENABLE_IMPLEMENTED) == 0U, "Invalid protection defaults");
_Static_assert(PROT_VOLTAGE_CFG_MIN_DV + 2U * PROT_VOLTAGE_HYST_DV < PROT_VOLTAGE_CFG_MAX_DV
               && PROT_VOLTAGE_CFG_MIN_DV > 10U && HW_BUS_SAFE_MV < HW_BUS_MEAS_MAX_MV, "Invalid supply voltage range");
_Static_assert(offsetof(Config_t, servo_id) == 0U && offsetof(Config_t, baud_code) == 1U,
               "ID and baud must stay at the head of Config_t (IAP reads them)");
_Static_assert(offsetof(Config_t, run) + sizeof(RunParams_t) == offsetof(Config_t, image_tag)
               && sizeof(Config_t) == offsetof(Config_t, image_tag) + sizeof(((Config_t *)0)->image_tag),
               "Config_t has padding: Flash compare would see uninitialised bytes");
_Static_assert(sizeof(Config_t) <= CONFIG_PAYLOAD_MAX, "Config_t exceeds the Flash payload");

/* 固件镜像指纹：对 APP 镜像(从链接起点 _sinit 起，不含前面的 Boot)做 FNV-1a 散列。 */
#ifndef CONFIG_IMAGE_TAG
extern const uint8_t _sinit[], _data_lma[], _data_vma[], _edata[]; /* 链接脚本：镜像起点、.data 初值位置与长度 */

static uint32_t Config_ImageTag(void)
{
    uint32_t len = (uint32_t)_data_lma + ((uint32_t)_edata - (uint32_t)_data_vma) - (uint32_t)_sinit;
    const uint32_t *p = (const uint32_t *)(FLASH_BASE + (uint32_t)_sinit);
    uint32_t h = 2166136261UL ^ len;
    uint32_t i;

    for (i = 0U; i < len / 4U; i++) h = (h ^ p[i]) * 16777619UL;
    return h;
}
#define CONFIG_IMAGE_TAG() Config_ImageTag()
#endif

static uint32_t s_image_tag;           /** 当前固件镜像指纹 */
static uint8_t  s_dirty;               /** 1=内存比Flash新，待保存 */
static volatile uint8_t *s_save_event; /** 保存任务就绪标志，置1触发调度 */

/* 出厂值，不登记保存 */
void A_Config_Defaults(Config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->baud_code                    = CFG_BAUD_DEFAULT;
    c->servo_mode                   = SERVO_MODE_270_CW;
    c->boot_mode                    = SERVO_BOOT_GOTO_START;
    c->startup_q3                   = Q3_MID;
    c->pulse_lo_q3                  = Q3_MIN;            /* 脉冲边界回到协议量程 */
    c->pulse_hi_q3                  = Q3_MAX;
    c->run.torque_limit_pct         = TORQUE_MAX_PERCENT;
    c->run.traj_vmax_dps            = CFG_DEF_VMAX_DPS;
    c->run.traj_amax_dps2           = CFG_DEF_AMAX_DPS2;
    c->run.traj_dmax_dps2           = CFG_DEF_DMAX_DPS2;
    c->run.traj_smooth_ms           = CFG_DEF_SMOOTH_MS;
    c->run.traj_move_min_ms         = CFG_DEF_MOVE_MIN_MS;
    c->run.hold_kp                  = CFG_DEF_HOLD_KP;
    c->run.hold_ki                  = CFG_DEF_HOLD_KI;
    c->run.position_resolution_cdeg = CFG_DEF_RESOLUTION_CDEG;
    c->run.prot_current_limit_ma    = PROT_CURRENT_LIMIT_DEFAULT_MA;
    c->run.prot_current_time_ms     = PROT_CURRENT_TIME_DEFAULT_MS;
    c->run.prot_temp_limit_dC       = PROT_TEMP_LIMIT_DEFAULT_DC;
    c->run.prot_temp_time_ms        = PROT_TEMP_TIME_DEFAULT_MS;
    c->run.prot_stall_act_cdeg      = PROT_STALL_ACT_DEFAULT_CDEG;
    c->run.prot_stall_trip          = PROT_STALL_TRIP_DEFAULT;
    c->run.protect_enable           = PROT_ENABLE_DEFAULT;
    c->run.voltage_min_dv           = PROT_VOLTAGE_CFG_MIN_DV;
    c->run.voltage_max_dv           = PROT_VOLTAGE_CFG_MAX_DV;
    c->image_tag                    = s_image_tag;       /* 出厂值属于当前固件 */
}

void A_Config_ReloadRuntime(uint8_t reset_transient)
{
    g_runtime_config.run = g_config.run;
    if (reset_transient)
    {
        g_runtime_config.torque_mode     = 1U;
        g_runtime_config.damping_percent = 100U;
    }
}

/* 用户恢复默认：keep_id=1仅保留ID(及从机模式，二者同属总线身份)；波特率一律回出厂值。
 * IAP换固件保留ID与波特率的规则在A_Config_Init处理。 */
void A_Config_Default(uint8_t keep_id)
{
    uint8_t id = g_config.servo_id, slave = g_config.slave_mode;

    A_Config_Defaults(&g_config);
    if (keep_id) { g_config.servo_id = id; g_config.slave_mode = slave; }

    A_Config_ReloadRuntime(1U);
    A_Config_MarkDirty();
}

void A_Config_Init(void)
{
    Config_t stored, defaults;
    uint8_t stored_version[3] = { 0xFFU, 0xFFU, 0xFFU };
    uint8_t keep_id = 0U;
    uint8_t keep_baud = CFG_BAUD_DEFAULT;
    uint8_t loaded;

    s_image_tag = CONFIG_IMAGE_TAG();
    A_Config_Defaults(&defaults);
    stored = defaults;
    loaded = FLASH_Config_Load(&stored, sizeof(stored), stored_version) == sizeof(stored);
    g_config = defaults;

    /* 版本与镜像指纹一致时沿用保存参数，非法字段修回出厂值；"修完与原来不同"即原配置非法，要重新落盘 */
    if (loaded && memcmp(stored_version, k_version, sizeof(k_version)) == 0
        && stored.image_tag == s_image_tag)
    {
        Config_t raw = stored;           /* 修复前原样 */
        A_Param_Repair(&stored, &defaults);
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
        if (keep_id <= CFG_ID_MAX) g_config.servo_id = keep_id;
        if (keep_baud >= CFG_BAUD_MIN && keep_baud <= CFG_BAUD_MAX) g_config.baud_code = keep_baud;
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

/* 绑定保存任务就绪标志；初始化阶段攒下的dirty在此补触发 */
void A_Config_BindSaveEvent(volatile uint8_t *event)
{
    s_save_event = event;
    if (s_dirty && s_save_event != 0) *s_save_event = 1U;
}

/* 保存任务：变脏时写Flash，失败保持dirty下次重试 */
uint8_t A_Config_SaveTask(void)
{
    if (s_dirty && FLASH_Config_Save(&g_config, sizeof(g_config), k_version))
    {
        s_dirty = 0U;
    }
    /* 写入失败必须重新安排低优先级重试，否则脏配置一直得不到保存 */
    if (s_dirty && s_save_event != 0) *s_save_event = 1U;
    return (uint8_t)!s_dirty;
}

/* 波特率编号换算实际波特率，越界时返回115200 */
uint32_t A_Config_Baudrate(void)
{
    /* 下标 = baud_code - 1，对应协议 BD1~BD8 */
    static const uint32_t baud[] = {
        9600U, 19200U, 38400U, 57600U,
        115200U, 128000U, 256000U, 1000000U
    };

    return baud[(g_config.baud_code >= CFG_BAUD_MIN && g_config.baud_code <= CFG_BAUD_MAX)
              ? g_config.baud_code - 1U : CFG_BAUD_DEFAULT - 1U];
}
