/*
 * config_audit.c —— 掉电参数的默认值、范围校验与**跨版本升级**
 *
 * 升级路径最容易悄悄出事：换了固件或者 Config_t 追加了字段，旧固件写下的记录
 * 处理不当，后果不是"新功能不工作"，而是**整台舵机回到出厂状态**——ID 和波特率
 * 一起被清掉，总线上直接失联。所以这里专门盯住：
 *   - 其他版本固件写的合法记录全部沿用，只把版本号更新成当前固件
 *   - 短记录(后来追加了字段)能原样读回，新字段拿出厂值
 *   - 内容越界时，ID 和波特率照样保住
 *   - 版本、ID、波特率在记录里的字节位置固定在 0~2/3/4
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

uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint32_t *sequence) {
    if (slot_len == 0U) return 0U;
    memcpy(data, slot, slot_len < capacity ? slot_len : capacity);
    *sequence = slot_seq;
    return slot_len;
}
uint8_t FLASH_Config_Save(const void *data, uint16_t length, uint32_t sequence) {
    save_calls++;
    if (length > sizeof(slot)) return 0U;
    memcpy(slot, data, length);
    slot_len = length; slot_seq = sequence;
    return 1U;
}

#include "A_Config.c"

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
    CHECK(g_config.version[0] == SERVO_VERSION_MAJOR);
    CHECK(g_config.version[1] == SERVO_VERSION_MINOR);
    CHECK(g_config.version[2] == SERVO_VERSION_PATCH);
    CHECK(g_config.servo_id == 0U);
    CHECK(g_config.servo_mode == SERVO_MODE_270_CW);
    CHECK(g_config.baud_code == 5U);
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN);
    CHECK(g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.cal_kind == SERVO_CAL_NONE);
    CHECK(save_calls >= 1);
    CHECK(slot_len == sizeof(Config_t));
}

/* 记录里第 0~2 字节是版本{主,次,修订}，第 3、4 字节是ID、波特率——Boot 或上位机按偏移读的就是它们 */
static void check_fixed_head(void) {
    reset_flash();
    A_Config_Init();
    g_config.servo_id  = 123U;
    g_config.baud_code = 8U;
    A_Config_MarkDirty();
    A_Config_SaveTask();
    CHECK(slot[0] == SERVO_VERSION_MAJOR);
    CHECK(slot[1] == SERVO_VERSION_MINOR);
    CHECK(slot[2] == SERVO_VERSION_PATCH);
    CHECK(slot[3] == 123U);
    CHECK(slot[4] == 8U);
}

/* 短记录 = 写它的固件还没有后来追加的字段：已有字段一个不丢，新字段拿出厂值 */
static void check_short_record(void) {
    Config_t cfg;
    uint16_t short_len = (uint16_t)offsetof(Config_t, pulse_lo);

    reset_flash();
    A_Config_Init();
    cfg = g_config;
    cfg.servo_id     = 37U;    /* 丢了它就等于总线上失联 */
    cfg.baud_code    = 7U;     /* 丢了它连话都说不上 */
    cfg.servo_mode   = SERVO_MODE_180_CW;
    cfg.boot_mode    = SERVO_BOOT_RELEASE;
    cfg.torque_limit = 60U;
    cfg.startup_pwm  = 1234U;
    cfg.custom_span_cdeg = 9000U;
    cfg.pulse_lo     = 900U;   /* 在短记录长度之外，不该被读到 */
    memset(slot, 0, sizeof(slot));
    memcpy(slot, &cfg, short_len);
    slot_len = short_len;
    slot_seq = 9U;

    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U; save_calls = 0;
    A_Config_Init();

    CHECK(g_config.servo_id == 37U);
    CHECK(g_config.baud_code == 7U);
    CHECK(g_config.servo_mode == SERVO_MODE_180_CW);
    CHECK(g_config.boot_mode == SERVO_BOOT_RELEASE);
    CHECK(g_config.torque_limit == 60U);
    CHECK(g_config.startup_pwm == 1234U);
    CHECK(g_config.custom_span_cdeg == 9000U);
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN);
    CHECK(g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.cal_anchor_cdeg == 0U);
    /* 补齐后必须就地写成当前长度，否则每次上电都要再补一遍 */
    CHECK(slot_len == sizeof(Config_t));
    CHECK(slot_seq == 10U);

    /* 再上电一次：走正常路径，不再写Flash */
    save_calls = 0;
    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U;
    A_Config_Init();
    CHECK(g_config.servo_id == 37U && g_config.baud_code == 7U);
    CHECK(save_calls == 0);
}

/* 其他版本固件写的合法记录：换固件不许清参数，全部沿用，只把版本号改成当前固件并写回 */
static void check_other_firmware(void) {
    reset_flash();
    A_Config_Init();
    g_config.servo_id    = 37U;
    g_config.baud_code   = 7U;
    g_config.servo_mode  = SERVO_MODE_180_CW;
    g_config.startup_pwm = 1234U;
    g_config.version[0] = 0U; g_config.version[1] = 9U; g_config.version[2] = 3U; /* 0.9.3 写的 */
    A_Config_MarkDirty();
    A_Config_SaveTask();

    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U; save_calls = 0;
    A_Config_Init();
    CHECK(g_config.servo_id == 37U && g_config.baud_code == 7U);
    CHECK(g_config.servo_mode == SERVO_MODE_180_CW);
    CHECK(g_config.startup_pwm == 1234U);
    CHECK(save_calls == 1);
    CHECK(slot[0] == SERVO_VERSION_MAJOR && slot[1] == SERVO_VERSION_MINOR
          && slot[2] == SERVO_VERSION_PATCH);

    /* 再上电一次：版本已一致，不再写Flash */
    save_calls = 0;
    memset(&g_config, 0, sizeof(g_config));
    s_dirty = 0U;
    A_Config_Init();
    CHECK(save_calls == 0);
}

/* 布局对不上的垃圾记录：范围检查拦下，只按固定偏移取回合法的ID和波特率，其余出厂值 */
static void check_garbage_record(void) {
    reset_flash();
    memset(slot, 0x5A, sizeof(slot));
    slot[3] = 37U;
    slot[4] = 7U;
    slot_len = sizeof(Config_t);
    slot_seq = 4U;

    A_Config_Init();
    CHECK(g_config.servo_id == 37U);
    CHECK(g_config.baud_code == 7U);
    CHECK(g_config.servo_mode == SERVO_MODE_270_CW);
    CHECK(g_config.startup_pwm == SERVO_PWM_MID);
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN && g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(slot[0] == SERVO_VERSION_MAJOR && slot_seq == 5U);  /* 已按当前版本写回 */

    /* ID、波特率本身越界时不能照搬：255是广播地址，波特率档位只有1~8 */
    reset_flash();
    memset(slot, 0x5A, sizeof(slot));
    slot[3] = 255U;
    slot[4] = 0U;
    slot_len = sizeof(Config_t);
    A_Config_Init();
    CHECK(g_config.servo_id == 0U);
    CHECK(g_config.baud_code == 5U);
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

    /* 坏记录上电：落出厂值而不是带着荒唐参数跑起来，但ID和波特率照样保住 */
    reset_flash(); A_Config_Init();
    g_config.servo_id = 42U; g_config.baud_code = 8U;
    g_config.pulse_lo = 2400U; g_config.pulse_hi = 600U;
    A_Config_MarkDirty(); A_Config_SaveTask();
    memset(&g_config, 0, sizeof(g_config)); s_dirty = 0U;
    A_Config_Init();
    CHECK(g_config.pulse_lo == SERVO_PWM_MIN && g_config.pulse_hi == SERVO_PWM_MAX);
    CHECK(g_config.servo_id == 42U && g_config.baud_code == 8U);
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
    CHECK(memcmp(g_config.version, SERVO_VERSION, sizeof(g_config.version)) == 0);
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
    check_fixed_head();
    check_short_record();
    check_other_firmware();
    check_garbage_record();
    check_roundtrip();
    check_range_validation();
    check_factory_reset();
    printf("config failures=%d\n", failures);
    return failures ? 1 : 0;
}
