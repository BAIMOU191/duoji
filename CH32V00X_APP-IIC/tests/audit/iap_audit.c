/*
 * iap_audit.c —— APP 侧 IAP 命令审计：查询、进入升级、写硬件信息、固件信息块
 *
 * System/iap.c 原样编译；串口发送换成抓包，硬件信息页换成内存数组，复位换成记录。
 * 盯住的是"总线上不乱说话"和"进入升级不误触发"：
 *   - 坏帧、别人的 ID、广播进入、APP 不认识的帧一律不应答
 *   - 硬件信息 4 项有一项不符就不进 Boot；符合时先卸力，应答发完才复位
 *   - 超长帧(Boot 的写页)不缓存、不越界，也不影响下一帧
 *   - 固件信息块布局与协议文档一致
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "ch32v00X.h"
#include "A_Config.h"
#include "flash.h"

static int failures;
#define CHECK(test) do { if (!(test)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); failures++; } } while (0)

Config_t g_config;

/* ---- WCH 库桩：硬件信息页落在内存数组里 ---- */
typedef enum { FLASH_BUSY = 1, FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE, FLASH_TIMEOUT } FLASH_Status;
#define Start_Mode_BOOT ((uint32_t)0x00004000)

static uint8_t hw_page[256];
#define IAP_HWINFO_PAGE ((const uint8_t *)hw_page)

static int corrupt_write;               /* 1=写入后故意改坏一个字节，模拟写入失败 */
static int write_calls;

FLASH_Status FLASH_ROM_ERASE(uint32_t addr, uint32_t len)
{
    CHECK(addr == HW_INFO_ADDR && len == 256U);
    memset(hw_page, 0xFF, sizeof(hw_page));
    return FLASH_COMPLETE;
}
FLASH_Status FLASH_ROM_WRITE(uint32_t addr, uint32_t *buf, uint32_t len)
{
    CHECK(addr == HW_INFO_ADDR && len == 256U);
    memcpy(hw_page, buf, sizeof(hw_page));
    if (corrupt_write) hw_page[6] ^= 0x01U;
    write_calls++;
    return FLASH_COMPLETE;
}

static int      reset_calls, clear_calls;
static uint32_t reset_mode;
void RCC_ClearFlag(void) { clear_calls++; }
void SystemReset_StartMode(uint32_t mode) { reset_mode = mode; }
static void NVIC_SystemReset(void) { reset_calls++; }

static int     release_calls;
static uint8_t release_high;
void A_Servo_Release(uint8_t high) { release_calls++; release_high = high; }

static uint8_t tx[64];
static int     tx_len;
static uint8_t tx_idle = 1U;
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len) { memcpy(tx, data, len); tx_len = len; return 1U; }
uint8_t D_UART_Tx_Idle(void) { return tx_idle; }

#include "iap.c"

/* ---- 参考实现：查表法 CRC-32 ---- */
static uint32_t ref_crc32(const uint8_t *p, size_t n)
{
    static uint32_t table[256];
    uint32_t crc = 0xFFFFFFFFU, c;
    int i, k;

    if (table[1] == 0U)
        for (i = 0; i < 256; i++)
        {
            for (c = (uint32_t)i, k = 0; k < 8; k++) c = (c & 1U) ? 0xEDB88320U ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
    while (n--) crc = table[(crc ^ *p++) & 0xFFU] ^ (crc >> 8);
    return ~crc;
}

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static void feed(const uint8_t *bytes, size_t n) { while (n--) IAP_Rx_Deal(*bytes++); }

/* 组帧逐字节喂入；返回应答状态码，-1 = 没有应答。应答帧格式在这里统一核对 */
static int send_raw(uint8_t cmd, const uint8_t *data, uint16_t len, int bad_crc)
{
    uint8_t  f[300];
    uint16_t n = 0, rlen;

    f[n++] = 0xAA; f[n++] = 0x55; f[n++] = cmd; f[n++] = (uint8_t)len; f[n++] = (uint8_t)(len >> 8);
    memcpy(f + n, data, len); n += len;
    put32(f + n, ref_crc32(f + 2, 3U + len) ^ (bad_crc ? 1U : 0U)); n += 4;
    f[n++] = 0x55; f[n++] = 0xAA;

    tx_len = 0;
    feed(f, n);
    if (tx_len == 0) return -1;

    rlen = (uint16_t)(tx[3] | tx[4] << 8);
    CHECK(tx[0] == 0xAA && tx[1] == 0x55);
    CHECK(tx[2] == (cmd | 0x80));
    CHECK(tx_len == 11 + rlen && rlen >= 1);
    CHECK(get32(tx + 5 + rlen) == ref_crc32(tx + 2, 3U + rlen));
    CHECK(tx[9 + rlen] == 0x55 && tx[10 + rlen] == 0xAA);
    return tx[5];
}
static int send(uint8_t cmd, const uint8_t *data, uint16_t len) { return send_raw(cmd, data, len, 0); }

static int cmd5(uint8_t cmd, uint8_t id, uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    uint8_t buf[5] = { id, a, b, c, d };
    return send(cmd, buf, 5);
}

/* 进入升级：ID + 硬件信息4项 + 固件版本3项 */
static int enter(uint8_t id, uint8_t type, uint8_t hw, uint8_t volt, uint8_t torque,
                 uint8_t major, uint8_t minor, uint8_t patch)
{
    uint8_t buf[8] = { id, type, hw, volt, torque, major, minor, patch };
    return send(IAP_CMD_ENTER, buf, 8);
}

static void reset_state(void)
{
    memset(hw_page, 0xFF, sizeof(hw_page));
    memset(&g_config, 0, sizeof(g_config));
    g_config.servo_id = 7U;
    tx_len = 0; tx_idle = 1U; corrupt_write = 0; write_calls = 0;
    reset_calls = clear_calls = release_calls = 0; reset_mode = 0; release_high = 0xEE;
    s_pos = 0U; s_reset_pending = 0U;
}

/* ============================== 用例 ============================== */

/* 固件信息块：16 字节，布局与协议文档一致，内容取自版本宏和板级宏 */
static void check_fw_info(void)
{
    const uint8_t *p = (const uint8_t *)&IAP_FW_INFO;
    int i;

    CHECK(sizeof(IAP_FwInfo_t) == 16U);
    CHECK(offsetof(IAP_FwInfo_t, servo_type) == 4U && offsetof(IAP_FwInfo_t, version) == 8U);
    CHECK(memcmp(p, "SVFW", 4) == 0);
    CHECK(p[4] == FW_SERVO_TYPE && p[5] == FW_HW_VERSION && p[6] == FW_VOLTAGE && p[7] == FW_TORQUE);
    CHECK(p[8] == SERVO_VERSION_MAJOR && p[9] == SERVO_VERSION_MINOR && p[10] == SERVO_VERSION_PATCH);
    for (i = 11; i < 16; i++) CHECK(p[i] == 0xFF);
}

/* 上电自动写：出厂空白时写入本固件适用的硬件，已写过就不动；带 0xFF 的固件不写 */
static void check_auto_provision(void)
{
    uint8_t id = 7U;
    uint8_t wildcard[4] = { 1U, 0xFFU, 3U, 4U };
    uint8_t known[4]    = { 1U, 2U, 3U, 4U };
    uint8_t before[256];

    CHECK(!Iap_HwInfoKnown(wildcard));
    CHECK(Iap_HwInfoKnown(known));

    reset_state();                                   /* 有 0xFF 的固件：宁可不写，也不写进错的身份 */
    Iap_Provision(wildcard);
    CHECK(write_calls == 0 && hw_page[0] == 0xFF);
    Iap_Provision(known);
    CHECK(write_calls == 1 && hw_page[5] == 1U && hw_page[8] == 4U);

    reset_state();                                   /* 出厂空白 */
    IAP_Init();
    CHECK(write_calls == 1);
    CHECK(hw_page[5] == FW_SERVO_TYPE && hw_page[6] == FW_HW_VERSION);
    CHECK(hw_page[7] == FW_VOLTAGE && hw_page[8] == FW_TORQUE);
    CHECK(send(IAP_CMD_QUERY, &id, 1) == 0);
    CHECK(tx[7] == FW_SERVO_TYPE && tx[10] == FW_TORQUE);

    write_calls = 0;                                 /* 再次上电：已写过，不再动 Flash */
    memcpy(before, hw_page, sizeof(before));
    IAP_Init();
    CHECK(write_calls == 0 && memcmp(hw_page, before, sizeof(before)) == 0);

    memset(hw_page, 0x5A, sizeof(hw_page));          /* 旧记录残留：当成没写过，覆盖掉 */
    IAP_Init();
    CHECK(write_calls == 1 && hw_page[5] == FW_SERVO_TYPE);

    reset_state();                                   /* 写入失败不应答也不死循环，下次上电再试 */
    corrupt_write = 1;
    IAP_Init();
    CHECK(write_calls == 1);
}

/* 查询：点名本机或广播 255 都回 ID、硬件信息 4 项、固件版本；别人的 ID 不理 */
static void check_query(void)
{
    uint8_t id;

    reset_state();
    id = 7U;
    CHECK(send(IAP_CMD_QUERY, &id, 1) == 0);
    CHECK(tx[3] == 9 && tx[6] == 7);
    CHECK(tx[7] == 0xFF && tx[8] == 0xFF && tx[9] == 0xFF && tx[10] == 0xFF);   /* 硬件信息页未写 */
    CHECK(tx[11] == SERVO_VERSION_MAJOR && tx[12] == SERVO_VERSION_MINOR && tx[13] == SERVO_VERSION_PATCH);

    id = 255U;
    CHECK(send(IAP_CMD_QUERY, &id, 1) == 0 && tx[6] == 7);
    id = 8U;
    CHECK(send(IAP_CMD_QUERY, &id, 1) == -1);
}

/* 写硬件信息：页有效就拒绝覆盖；无效(空白或垃圾)才写，写完读回 */
static void check_hwinfo(void)
{
    uint8_t id = 7U;
    uint8_t before[256];
    int i;

    reset_state();
    CHECK(cmd5(IAP_CMD_HWINFO, 7, 1, 2, 3, 4) == 0);
    CHECK(memcmp(hw_page, "HWIF", 4) == 0 && hw_page[4] == 1);
    CHECK(hw_page[5] == 1 && hw_page[6] == 2 && hw_page[7] == 3 && hw_page[8] == 4);
    for (i = 9; i < 28; i++) CHECK(hw_page[i] == 0xFF);
    CHECK(get32(hw_page + 28) == ref_crc32(hw_page, 28));
    for (i = 32; i < 256; i++) CHECK(hw_page[i] == 0xFF);

    CHECK(send(IAP_CMD_QUERY, &id, 1) == 0);
    CHECK(tx[7] == 1 && tx[8] == 2 && tx[9] == 3 && tx[10] == 4);

    memcpy(before, hw_page, sizeof(before));
    CHECK(cmd5(IAP_CMD_HWINFO, 7, 9, 9, 9, 9) == 7);                    /* 已写入，不许覆盖 */
    CHECK(memcmp(hw_page, before, sizeof(before)) == 0);
    CHECK(cmd5(IAP_CMD_HWINFO, 8, 9, 9, 9, 9) == -1);
    CHECK(cmd5(IAP_CMD_HWINFO, 255, 9, 9, 9, 9) == -1);                 /* 广播写会把整条总线写成一样的 */

    memset(hw_page, 0x5A, sizeof(hw_page));                             /* 旧参数之类的残留：不是有效信息页 */
    CHECK(cmd5(IAP_CMD_HWINFO, 7, 5, 6, 7, 8) == 0 && hw_page[5] == 5);

    reset_state();
    corrupt_write = 1;
    CHECK(cmd5(IAP_CMD_HWINFO, 7, 1, 2, 3, 4) == 4);
}

/* 进入升级：硬件信息4项 + 固件版本3项都要与查询回去的一致，且必须点名本机；
 * 通过后先卸力，应答发完才复位进 Boot */
static void check_enter(void)
{
    const uint8_t VER_MAJOR = SERVO_VERSION_MAJOR, VER_MINOR = SERVO_VERSION_MINOR,
                  VER_PATCH = SERVO_VERSION_PATCH;
    uint8_t id = 7U;

    reset_state();
    CHECK(cmd5(IAP_CMD_HWINFO, 7, 1, 2, 3, 4) == 0);

    /* 先查询一次，进入升级要原样带回查询到的 7 个字节 */
    CHECK(send(IAP_CMD_QUERY, &id, 1) == 0);
    CHECK(tx[7] == 1 && tx[10] == 4 && tx[11] == VER_MAJOR);

    CHECK(enter(7, 1, 2, 3, 5, VER_MAJOR, VER_MINOR, VER_PATCH) == 2);   /* 扭力型号不符 */
    CHECK(enter(7, 1, 2, 3, 4, (uint8_t)(VER_MAJOR + 1), VER_MINOR, VER_PATCH) == 2); /* 版本不符 */
    CHECK(release_calls == 0);
    IAP_Service();
    CHECK(reset_calls == 0);

    CHECK(enter(255, 1, 2, 3, 4, VER_MAJOR, VER_MINOR, VER_PATCH) == -1); /* 广播不能让整条总线进 Boot */
    CHECK(enter(8, 1, 2, 3, 4, VER_MAJOR, VER_MINOR, VER_PATCH) == -1);
    IAP_Service();
    CHECK(reset_calls == 0 && release_calls == 0);

    CHECK(enter(7, 1, 2, 3, 4, VER_MAJOR, VER_MINOR, VER_PATCH) == 0);
    CHECK(release_calls == 1 && release_high == 0);
    tx_idle = 0U;                                                       /* 应答还在发 */
    IAP_Service();
    CHECK(reset_calls == 0);
    tx_idle = 1U;
    IAP_Service();
    CHECK(reset_calls == 1 && reset_mode == Start_Mode_BOOT && clear_calls == 1);

    reset_state();                                                      /* 硬件信息页没写：查询报 FF，带 FF 进入 */
    CHECK(enter(7, 1, 2, 3, 4, VER_MAJOR, VER_MINOR, VER_PATCH) == 2);
    CHECK(enter(7, 0xFF, 0xFF, 0xFF, 0xFF, VER_MAJOR, VER_MINOR, VER_PATCH) == 0);
}

/* 坏帧与不相干的数据：一律不应答，也不会让接收状态卡住 */
static void check_bad_frames(void)
{
    uint8_t id = 7U, two[2] = { 7, 0 };
    uint8_t write_page[5 + 260 + 6];
    const char *ascii = "#007PVER!#007P1500T1000!";
    uint8_t tail_bad[] = { 0xAA, 0x55, IAP_CMD_QUERY, 1, 0, 7, 0, 0, 0, 0, 0x55, 0x55 };
    int i;

    reset_state();
    CHECK(send_raw(IAP_CMD_QUERY, &id, 1, 1) == -1);                     /* CRC 错 */
    CHECK(send(IAP_CMD_QUERY, two, 2) == -1);                           /* LEN 不对 */
    CHECK(cmd5(IAP_CMD_ENTER, 7, 1, 2, 3, 4) == -1);                    /* 进入升级必须带满 8 字节 */
    CHECK(send(IAP_CMD_QUERY, 0, 0) == -1);
    CHECK(send(IAP_CMD_QUERY | 0x80, &id, 1) == -1);                    /* 应答帧(别的舵机发的) */
    CHECK(send(0x30, &id, 1) == -1);                                    /* APP 不认识的命令 */
    CHECK(send(0x01, 0, 0) == -1);                                      /* Boot 的握手 */

    feed((const uint8_t *)ascii, strlen(ascii));                        /* 舵机 ASCII 指令 */
    CHECK(tx_len == 0 && s_pos == 0);

    put32(tail_bad + 6, ref_crc32(tail_bad + 2, 4));
    feed(tail_bad, sizeof(tail_bad));                                   /* 帧尾不对 */
    CHECK(tx_len == 0);

    write_page[0] = 0xAA; write_page[1] = 0x55; write_page[2] = 0x03; write_page[3] = 0x04; write_page[4] = 0x01;
    for (i = 5; i < (int)sizeof(write_page); i++) write_page[i] = (uint8_t)(i * 37);
    feed(write_page, 5);
    CHECK(s_pos == 0);                                                  /* LEN=260 不缓存 */
    feed(write_page + 5, sizeof(write_page) - 5);
    CHECK(tx_len == 0);

    IAP_Rx_Deal(0xAA);                                                  /* AA AA 55 也要对上帧头 */
    CHECK(send(IAP_CMD_QUERY, &id, 1) == 0);
}

int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    check_fw_info();
    check_auto_provision();
    check_query();
    check_hwinfo();
    check_enter();
    check_bad_frames();
    printf("iap failures=%d\n", failures);
    return failures ? 1 : 0;
}
