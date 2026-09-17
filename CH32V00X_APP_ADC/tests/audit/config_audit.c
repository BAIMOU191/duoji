/*
 * config_audit.c —— 掉电参数的默认值、范围校验与**跨版本升级**
 *
 * 升级路径是这轮改动里最容易悄悄出事的一段：Config_t 多了几个字段，长度变了，
 * 旧固件写下的记录会被 FLASH_Config_Load 判成无效。处理不当的后果不是"新功能
 * 不工作"，而是**整台舵机回到出厂状态**——ID 和波特率一起被清掉，总线上直接失联。
 * 所以这里专门盯住"旧记录能不能原样读回来"。
 *
 * Flash 换成内存里的一条记录，其余全是生产代码本体。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "A_Config.h"

static int failures;
#define CHECK(test) do { if (!(test)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); failures++; } } while (0)

/* ---- Flash 桩：一条记录 + 它的长度，模拟"旧固件写的短记录" ---- */
static uint8_t  slot[64];
static uint16_t slot_len;      /* 0 = 空白Flash */
static uint32_t slot_seq;
static int      save_calls;

uint8_t FLASH_Config_Load(void *data, uint16_t length, uint32_t *sequence) {
    if (slot_len == 0U || slot_len != length) return 0U;  /* 长度不符即判无效 */
    memcpy(data, slot, length);
    *sequence = slot_seq;
    return 1U;
}
uint8_t FLASH_Config_Save(const void *data, uint16_t length, uint32_t sequence) {
    save_calls++;
    if (length > sizeof(slot)) return 0U;
    memcpy(slot, data, length);
    slot_len = length; slot_seq = sequence;
    return 1U;
}

#include "A_Config.c"

/* 旧版固件的参数布局：新字段全部追加在末尾，所以它就是新结构体的前缀 */
typedef struct {
    uint16_t startup_pwm;
    uint16_t position_offset_cdeg;
    uint16_t custom_offset_cdeg;
    uint16_t custom_span_cdeg;
    uint8_t  servo_id;
    uint8_t  servo_mode;
    uint8_t  baud_code;
    uint8_t  boot_mode;
    uint8_t  torque_limit;
    uint8_t  custom_reverse;
} LegacyConfig_t;

static void reset_flash(void) {
    memset(slot, 0, sizeof(slot));
    slot_len = 0U; slot_seq = 0U; save_calls = 0;
    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U; s_sequence = 0U;
}

/* 空白Flash：落出厂值，并且必须当场写回去 */
static void check_blank_flash(void) {
    reset_flash();
    A_Config_Init();
    CHECK(g_config.servo_id == 0U);
    CHECK(g_config.servo_mode == SERVO_MODE_270_CW);
    CHECK(g_config.baud_code == 5U);
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN);
    CHECK(g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.cal_kind == SERVO_CAL_NONE);
    CHECK(save_calls >= 1);
    CHECK(slot_len == sizeof(Config_t));
}

/* 旧记录升级：ID、波特率、模式、上电位一个都不能丢，新字段补出厂值。 */
static void check_legacy_upgrade(void) {
    LegacyConfig_t old;

    reset_flash();
    memset(&old, 0, sizeof(old));
    old.startup_pwm          = 1234U;
    old.position_offset_cdeg = 5000U;
    old.custom_offset_cdeg   = 700U;
    old.custom_span_cdeg     = 9000U;
    old.servo_id             = 37U;    /* 丢了它就等于总线上失联 */
    old.servo_mode           = SERVO_MODE_180_CW;
    old.baud_code            = 7U;     /* 丢了它连话都说不上 */
    old.boot_mode            = SERVO_BOOT_RELEASE;
    old.torque_limit         = 60U;
    old.custom_reverse       = 1U;
    memcpy(slot, &old, sizeof(old));
    slot_len = (uint16_t)sizeof(old);
    slot_seq = 9U;
    CHECK(sizeof(old) == CONFIG_LEGACY_LEN);

    A_Config_Init();

    CHECK(g_config.servo_id == 37U);
    CHECK(g_config.baud_code == 7U);
    CHECK(g_config.servo_mode == SERVO_MODE_180_CW);
    CHECK(g_config.startup_pwm == 1234U);
    CHECK(g_config.position_offset_cdeg == 5000U);
    CHECK(g_config.custom_offset_cdeg == 700U);
    CHECK(g_config.custom_span_cdeg == 9000U);
    CHECK(g_config.boot_mode == SERVO_BOOT_RELEASE);
    CHECK(g_config.torque_limit == 60U);
    CHECK(g_config.custom_reverse == 1U);
    /* 新字段拿出厂值 */
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN);
    CHECK(g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.cal_kind == SERVO_CAL_NONE);
    CHECK(g_config.cal_anchor_cdeg == 0U);
    /* 升级完必须就地写成新格式，否则每次上电都要再走一遍兼容路径 */
    CHECK(slot_len == sizeof(Config_t));
    CHECK(slot_seq == 10U);

    /* 再上电一次：走的应该是正常路径，内容一字不差 */
    save_calls = 0;
    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U;
    A_Config_Init();
    CHECK(g_config.servo_id == 37U && g_config.baud_code == 7U);
    CHECK(save_calls == 0);
}

/* 新记录原样读回，一个字段都不许变 */
static void check_roundtrip(void) {
    reset_flash();
    A_Config_Init();
    g_config.servo_id        = 21U;
    g_config.pulse_lo        = 800U;
    g_config.pulse_hi        = 2200U;
    g_config.cal_kind        = SERVO_CAL_ZERO;
    g_config.cal_anchor_cdeg = 4321U;
    A_Config_MarkDirty();
    A_Config_SaveTask();

    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U;
    A_Config_Init();
    CHECK(g_config.servo_id == 21U);
    CHECK(g_config.pulse_lo == 800U && g_config.pulse_hi == 2200U);
    CHECK(g_config.cal_kind == SERVO_CAL_ZERO);
    CHECK(g_config.cal_anchor_cdeg == 4321U);
}

/* 内容越界的记录必须被判无效。位翻转过不了CRC，但跨版本/手工烧录留下的
 * "校验正确、语义荒唐"的记录只有范围检查拦得住。 */
static void check_range_validation(void) {
    /* 脉冲边界交叉：舵机会拒收一切运动指令，绝不能让它活下来 */
    reset_flash(); A_Config_Init();
    g_config.pulse_lo = 2000U; g_config.pulse_hi = 1000U;
    CHECK(!A_Config_Valid(&g_config));
    /* 边界落在协议量程之外 */
    reset_flash(); A_Config_Init();
    g_config.pulse_lo = 100U;
    CHECK(!A_Config_Valid(&g_config));
    reset_flash(); A_Config_Init();
    g_config.pulse_hi = 4000U;
    CHECK(!A_Config_Valid(&g_config));
    /* 两端相等 = 空窗口，同样不许 */
    reset_flash(); A_Config_Init();
    g_config.pulse_lo = g_config.pulse_hi = 1500U;
    CHECK(!A_Config_Valid(&g_config));
    /* 校准种类越界 */
    reset_flash(); A_Config_Init();
    g_config.cal_kind = 9U;
    CHECK(!A_Config_Valid(&g_config));
    /* 校准锚点落在可转入行程之外 */
    reset_flash(); A_Config_Init();
    g_config.cal_kind = SERVO_CAL_MID;
    g_config.cal_anchor_cdeg = (uint16_t)(ENCODER_TRAVEL_HI + 1);
    CHECK(!A_Config_Valid(&g_config));
    /* 未校准时锚点是什么都不该导致整份参数作废 */
    reset_flash(); A_Config_Init();
    g_config.cal_kind = SERVO_CAL_NONE;
    g_config.cal_anchor_cdeg = 60000U;
    CHECK(A_Config_Valid(&g_config));

    /* 坏记录上电：落出厂值而不是带着荒唐参数跑起来 */
    reset_flash(); A_Config_Init();
    g_config.pulse_lo = 2400U; g_config.pulse_hi = 600U;
    A_Config_MarkDirty(); A_Config_SaveTask();
    memset(&g_config, 0, sizeof(g_config)); s_dirty = 0U;
    A_Config_Init();
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN && g_config.pulse_hi == SERVO_PWM_MAX);
}

/* CLE 恢复出厂必须把脉冲边界和校准一起清掉——误设的边界只有这一条自救路 */
static void check_factory_reset(void) {
    reset_flash(); A_Config_Init();
    g_config.servo_id        = 88U;
    g_config.pulse_lo        = 1200U;
    g_config.pulse_hi        = 1800U;
    g_config.cal_kind        = SERVO_CAL_MID;
    g_config.cal_anchor_cdeg = 1000U;

    A_Config_Default(1U);                       /* CLE0：保留ID */
    CHECK(g_config.servo_id == 88U);
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN && g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.cal_kind == SERVO_CAL_NONE && g_config.cal_anchor_cdeg == 0U);

    g_config.pulse_lo = 1200U; g_config.cal_kind = SERVO_CAL_ZERO;
    A_Config_Default(0U);                       /* CLE：连ID一起清 */
    CHECK(g_config.servo_id == 0U);
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN && g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.cal_kind == SERVO_CAL_NONE);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    check_blank_flash();
    check_legacy_upgrade();
    check_roundtrip();
    check_range_validation();
    check_factory_reset();
    printf("config failures=%d\n", failures);
    return failures ? 1 : 0;
}
