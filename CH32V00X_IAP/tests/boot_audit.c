/*
 * boot_audit.c —— Bootloader 升级协议的主机端审计
 *
 * System/iap.c 原样编译，只把 Flash 换成内存数组(擦除写0xFF、编程只能把1写成0)，
 * 串口发送换成抓包，跳转 APP 换成计数。盯住的是：
 *   - 断电安全：开始后标志页立刻是 UPGD，完成且整包 CRC 对了才写 APOK
 *   - 擦除范围：模式0只擦 APP，模式1连参数页，硬件信息页和标志页永远不被当成 APP 擦
 *   - 任何时候都能重来：同一页重发、乱序写、中途"断电"后重新开始
 *   - 坏帧、越界、顺序错误都有明确的状态码，不会写坏 Flash
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <setjmp.h>
#include "ch32v00X.h"                   /* tests/stubs 里的桩 */

#define BOOT_HOST_TEST
#define __FLASH_H                       /* 跳过工程的 flash.h，Flash 函数由下面的桩提供 */

static uint8_t sim_flash[0xF800];
#define FLASH_PTR(off) ((const uint8_t *)(sim_flash + (off)))
#define IWDG_FEED()    ((void)0)

static int failures;
#define CHECK(test) do { if (!(test)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); failures++; } } while (0)

/* ---- Flash 桩 ---- */
static int      unlocked;
static uint32_t corrupt_addr;           /* 非0：编程这一页时故意把该为1的位写成0，模拟写入失败(只有擦除才能恢复) */

void FLASH_Unlock_Fast(void) { unlocked = 1; }
void FLASH_ErasePage_Fast(uint32_t addr)
{
    CHECK(unlocked && addr >= FLASH_BASE && addr - FLASH_BASE <= 0xF700U);
    memset(sim_flash + (addr - FLASH_BASE), 0xFF, 256);
}
void FLASH_EraseBlock_32K_Fast(uint32_t addr)
{
    CHECK(unlocked && addr == FLASH_BASE);
    memset(sim_flash, 0xFF, 0x8000);
}
void CH32_IAP_Program(uint32_t adr, uint32_t *buf)
{
    uint8_t  bytes[256];
    uint32_t off = adr - FLASH_BASE;
    int      i;

    CHECK(unlocked && (off & 0xFFU) == 0U && off <= 0xF700U);
    memcpy(bytes, buf, sizeof(bytes));
    for (i = 0; i < 256; i++) sim_flash[off + i] &= bytes[i];
    if (corrupt_addr == adr)
        for (i = 0; i < 256; i++)
            if (bytes[i] != 0U) { sim_flash[off + i] &= (uint8_t)~bytes[i]; break; }
}

/* ---- 串口、计时与跳转桩 ---- */
static uint8_t tx[64];
static int     tx_len;
static int     jumped;

static uint8_t rxq[1024];               /* 待 Loop 取走的字节 */
static int     rxq_head, rxq_tail;
static int     tick_budget;             /* 还能过多少个 1ms，用尽就跳回测试 */
static int     ticks_used;
static uint8_t soft_reset_flag;         /* Reset_WasSoftware 的返回值 */
static jmp_buf esc;                     /* Loop 不会返回，靠它跳回测试 */
static int     escape_on_jump;          /* 1=Jump_App 时跳回测试 */

static void Uart_Send(const uint8_t *data, uint8_t len) { memcpy(tx, data, len); tx_len = len; }
static void Jump_App(void) { jumped++; if (escape_on_jump) longjmp(esc, 2); }
static void Hw_Init(void) {}
static uint8_t Reset_WasSoftware(void) { return soft_reset_flag; }
static uint8_t Uart_Poll(uint8_t *byte)
{
    if (rxq_head == rxq_tail) return 0U;
    *byte = rxq[rxq_head++];
    return 1U;
}
static uint8_t Tick_Poll(void)
{
    if (tick_budget <= 0) longjmp(esc, 1);
    tick_budget--;
    ticks_used++;
    return 1U;
}

#include "iap.c"

/* ---- 参考实现：查表法 CRC-32，和 iap.c 的逐位算法互相印证 ---- */
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

/* ---- 发帧与收应答 ---- */
static int last_ret;                    /* Frame_Process 的返回值：1=处理了握手 */

/* 按协议组帧逐字节喂给 Boot；返回应答状态码，-1 = 没有应答。应答帧格式在这里统一核对 */
static int send_raw(uint8_t cmd, const uint8_t *data, uint16_t len, int hello_only, int bad_crc)
{
    uint8_t  f[300];
    uint16_t n = 0, i, rlen;

    f[n++] = 0xAA; f[n++] = 0x55; f[n++] = cmd; f[n++] = (uint8_t)len; f[n++] = (uint8_t)(len >> 8);
    memcpy(f + n, data, len); n += len;
    put32(f + n, ref_crc32(f + 2, 3U + len) ^ (bad_crc ? 1U : 0U)); n += 4;
    f[n++] = 0x55; f[n++] = 0xAA;

    tx_len = 0; last_ret = 0;
    for (i = 0; i < n; i++)
        if (Frame_Byte(f[i])) last_ret = Frame_Process((uint8_t)hello_only);
    if (tx_len == 0) return -1;

    rlen = (uint16_t)(tx[3] | tx[4] << 8);
    CHECK(tx[0] == 0xAA && tx[1] == 0x55);
    CHECK(tx[2] == (cmd | 0x80));
    CHECK(tx_len == 11 + rlen && rlen >= 1);
    CHECK(get32(tx + 5 + rlen) == ref_crc32(tx + 2, 3U + rlen));
    CHECK(tx[9 + rlen] == 0x55 && tx[10 + rlen] == 0xAA);
    return tx[5];
}
static int send(uint8_t cmd, const uint8_t *data, uint16_t len) { return send_raw(cmd, data, len, 0, 0); }

static int cmd_start(uint32_t img_len, uint32_t crc, uint8_t mode)
{
    uint8_t d[9];
    put32(d, img_len); put32(d + 4, crc); d[8] = mode;
    return send(CMD_START, d, 9);
}

static int cmd_write(uint32_t off, const uint8_t *page)
{
    uint8_t d[260];
    put32(d, off); memcpy(d + 4, page, 256);
    return send(CMD_WRITE, d, 260);
}

/* ---- 场景搭建 ---- */
static uint8_t image[0xF400];

/* 模拟"重新上电"：Boot 的 RAM 状态全部丢失，Flash 保留 */
static void reboot(void)
{
    s_pos = 0; s_started = 0; s_img_len = 0; s_img_crc = 0;
    unlocked = 0; corrupt_addr = 0; jumped = 0;
    rxq_head = rxq_tail = 0; tick_budget = 0; ticks_used = 0; tx_len = 0;
}

/* 把字节排进"串口收到"的队列 */
static void rx_push(const uint8_t *data, int n) { while (n--) rxq[rxq_tail++] = *data++; }

/* 按协议组一帧塞进接收队列 */
static void rx_push_frame(uint8_t cmd, const uint8_t *data, uint16_t len)
{
    uint8_t f[300];
    uint16_t n = 0;

    f[n++] = 0xAA; f[n++] = 0x55; f[n++] = cmd; f[n++] = (uint8_t)len; f[n++] = (uint8_t)(len >> 8);
    memcpy(f + n, data, len); n += len;
    put32(f + n, ref_crc32(f + 2, 3U + len)); n += 4;
    f[n++] = 0x55; f[n++] = 0xAA;
    rx_push(f, n);
}

static void flash_blank(void) { memset(sim_flash, 0xFF, sizeof(sim_flash)); reboot(); }

/* 生成一份固件镜像：填充图样 + 0x100 处的固件信息块，补 0xFF 到 256 整数倍，返回长度 */
static uint32_t make_image(uint32_t raw_len, uint8_t type, uint8_t hwver, uint8_t volt, uint8_t torque, int with_info)
{
    uint32_t i, len = (raw_len + 255U) & ~255U;

    memset(image, 0xFF, sizeof(image));
    for (i = 0; i < raw_len; i++) image[i] = (uint8_t)(i * 7U + 3U);
    if (with_info)
    {
        memcpy(image + 0x100, "SVFW", 4);
        image[0x104] = type; image[0x105] = hwver; image[0x106] = volt; image[0x107] = torque;
        image[0x108] = 1; image[0x109] = 2; image[0x10A] = 3;
        memset(image + 0x10B, 0xFF, 5);
    }
    return len;
}

static void set_hwinfo(uint8_t type, uint8_t hwver, uint8_t volt, uint8_t torque)
{
    uint8_t *p = sim_flash + HWINFO_OFFSET;
    memset(p, 0xFF, 256);
    memcpy(p, "HWIF", 4);
    p[4] = 1; p[5] = type; p[6] = hwver; p[7] = volt; p[8] = torque;
    put32(p + 28, ref_crc32(p, 28));
}

static void set_flag(const char *magic) { memset(sim_flash + FLAG_OFFSET, 0xFF, 256); memcpy(sim_flash + FLAG_OFFSET, magic, 4); }

/* 整套升级：开始 -> 逐页写 -> 完成，返回完成命令的状态码 */
static int upgrade(uint32_t len, uint8_t mode)
{
    uint32_t off;
    int      st = cmd_start(len, ref_crc32(image, len), mode);

    CHECK(st == ST_OK);
    for (off = 0; off < len; off += 256) CHECK(cmd_write(off, image + off) == ST_OK);
    return send(CMD_FINISH, 0, 0);
}

/* ============================== 用例 ============================== */

/* 两种 CRC 算法和标准测试向量一致——上位机用 zlib.crc32 算出来的必须对得上 */
static void check_crc(void)
{
    CHECK(Crc32((const uint8_t *)"123456789", 9) == 0xCBF43926U);
    CHECK(ref_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926U);
}

/* 魔术字按小端读出就是 ASCII，文档里写的字节序列不会和代码对不上 */
static void check_magic(void)
{
    CHECK(get32((const uint8_t *)"APOK") == FLAG_APOK);
    CHECK(get32((const uint8_t *)"UPGD") == FLAG_UPGD);
    CHECK(get32((const uint8_t *)"HWIF") == HWINFO_MAGIC);
    CHECK(get32((const uint8_t *)"SVFW") == FWINFO_MAGIC);
}

/* 握手：Boot 版本、标志页状态、硬件信息；硬件信息页无效时报 FF */
static void check_hello(void)
{
    flash_blank();
    CHECK(send(CMD_HELLO, 0, 0) == ST_OK);
    CHECK(tx[3] == 7 && tx[6] == BOOT_VERSION && tx[7] == 0);
    CHECK(tx[8] == 0xFF && tx[9] == 0xFF && tx[10] == 0xFF && tx[11] == 0xFF);
    CHECK(last_ret == 1);

    set_hwinfo(1, 2, 3, 4);
    set_flag("UPGD");
    CHECK(send(CMD_HELLO, 0, 0) == ST_OK);
    CHECK(tx[7] == 1 && tx[8] == 1 && tx[9] == 2 && tx[10] == 3 && tx[11] == 4);

    sim_flash[HWINFO_OFFSET + 6] ^= 0xFF;               /* 内容被改过，CRC 对不上 */
    set_flag("APOK");
    CHECK(send(CMD_HELLO, 0, 0) == ST_OK);
    CHECK(tx[7] == 2 && tx[8] == 0xFF && tx[11] == 0xFF);

    set_flag("XXXX");
    CHECK(send(CMD_HELLO, 0, 0) == ST_OK && tx[7] == 3);
}

/* 上电判断：只有 APOK，或"标志页空白且 APP 区有东西"(量产刚烧录)才进 APP */
static void check_bootable(void)
{
    flash_blank();
    CHECK(!App_Bootable());                             /* 全新芯片只有 Boot */
    sim_flash[0] = 0x6F;
    CHECK(App_Bootable());                              /* 量产烧录：标志页空白 */
    set_flag("UPGD");
    CHECK(!App_Bootable());                             /* 升级中断电 */
    set_flag("APOK");
    CHECK(App_Bootable());
    set_flag("junk");
    CHECK(!App_Bootable());
}

/* 没有开始就写页或完成：拒绝并且什么都不写 */
static void check_order(void)
{
    uint8_t page[256];

    flash_blank();
    memset(page, 0x00, sizeof(page));
    CHECK(cmd_write(0, page) == ST_ORDER);
    CHECK(get32(tx + 6) == 0);                          /* 写页应答带回偏移 */
    CHECK(sim_flash[0] == 0xFF);
    CHECK(send(CMD_FINISH, 0, 0) == ST_ORDER);
    CHECK(jumped == 0);
}

/* 开始命令的参数越界：不擦不写，之前的开始也作废 */
static void check_start_range(void)
{
    uint8_t page[256];

    flash_blank();
    sim_flash[0] = 0x6F;
    set_flag("APOK");
    CHECK(cmd_start(0, 0, 0) == ST_RANGE);
    CHECK(cmd_start(300, 0, 0) == ST_RANGE);            /* 不是 256 的整数倍 */
    CHECK(cmd_start(0xF500, 0, 0) == ST_RANGE);         /* 超出 APP 区 */
    CHECK(cmd_start(256, 0, 2) == ST_RANGE);            /* 擦除模式只有 0/1 */
    CHECK(sim_flash[0] == 0x6F && App_Bootable());

    CHECK(cmd_start(0xF400, 0, 0) == ST_OK);            /* 最大长度正好放满 APP 区 */
    CHECK(cmd_start(300, 0, 0) == ST_RANGE);
    memset(page, 0, sizeof(page));
    CHECK(cmd_write(0, page) == ST_ORDER);
}

/* 模式0完整升级：APP 区擦掉重写，参数页、硬件信息页保持不动，完成后写 APOK 并进 APP */
static void check_upgrade_mode0(void)
{
    uint32_t len;
    uint8_t  hw_before[256];

    flash_blank();
    memset(sim_flash, 0x11, 0xF400);                    /* 旧 APP 占满 */
    memset(sim_flash + 0xF400, 0x22, 0x200);            /* 参数页 A/B */
    set_hwinfo(1, 2, 3, 4);
    set_flag("APOK");
    memcpy(hw_before, sim_flash + HWINFO_OFFSET, 256);

    len = make_image(3000, 0xFF, 2, 0xFF, 0xFF, 1);
    CHECK(cmd_start(len, ref_crc32(image, len), 0) == ST_OK);
    CHECK(get32(sim_flash + FLAG_OFFSET) == FLAG_UPGD);   /* 从这一刻起断电也会留在 Boot */
    CHECK(!App_Bootable());
    CHECK(sim_flash[0x7FFF] == 0xFF && sim_flash[0x8000] == 0xFF && sim_flash[0xF3FF] == 0xFF);
    CHECK(sim_flash[0xF400] == 0x22 && sim_flash[0xF5FF] == 0x22);
    CHECK(memcmp(sim_flash + HWINFO_OFFSET, hw_before, 256) == 0);

    CHECK(upgrade(len, 0) == ST_OK);
    CHECK(jumped == 1);
    CHECK(memcmp(sim_flash, image, len) == 0);
    CHECK(get32(sim_flash + FLAG_OFFSET) == FLAG_APOK);
    CHECK(get32(sim_flash + FLAG_OFFSET + 4) == len);
    CHECK(get32(sim_flash + FLAG_OFFSET + 8) == ref_crc32(image, len));
    CHECK(sim_flash[0xF400] == 0x22);
    CHECK(memcmp(sim_flash + HWINFO_OFFSET, hw_before, 256) == 0);
    CHECK(App_Bootable());
}

/* 模式1：参数页一起擦掉，硬件信息页照样不动 */
static void check_upgrade_mode1(void)
{
    uint32_t len;

    flash_blank();
    memset(sim_flash + 0xF400, 0x22, 0x200);
    set_hwinfo(1, 2, 3, 4);
    len = make_image(1000, 1, 2, 3, 4, 1);
    CHECK(upgrade(len, 1) == ST_OK);
    CHECK(sim_flash[0xF400] == 0xFF && sim_flash[0xF5FF] == 0xFF);
    CHECK(get32(sim_flash + HWINFO_OFFSET) == HWINFO_MAGIC);
    CHECK(jumped == 1);
}

/* 同一页重发、乱序写都不影响结果；偏移越界拒绝 */
static void check_retry_and_range(void)
{
    uint32_t len, off;
    uint8_t  zeros[256];

    flash_blank();
    len = make_image(2000, 0xFF, 0xFF, 0xFF, 0xFF, 1);
    CHECK(cmd_start(len, ref_crc32(image, len), 0) == ST_OK);
    memset(zeros, 0, sizeof(zeros));
    CHECK(cmd_write(256, zeros) == ST_OK);                /* 同一偏移先写别的内容，后面重写必须先擦 */
    for (off = len; off > 0; off -= 256) CHECK(cmd_write(off - 256, image + off - 256) == ST_OK);
    CHECK(cmd_write(256, image + 256) == ST_OK);          /* 应答丢了、上位机重发 */
    CHECK(cmd_write(256, image + 256) == ST_OK);
    CHECK(get32(tx + 6) == 256);
    CHECK(cmd_write(0x10, image) == ST_RANGE);            /* 没有按 256 对齐 */
    CHECK(get32(tx + 6) == 0x10);
    CHECK(cmd_write(len, image) == ST_RANGE);             /* 超出本次镜像长度 */
    CHECK(cmd_write(0xFFFFFF00U, image) == ST_RANGE);
    CHECK(send(CMD_FINISH, 0, 0) == ST_OK && jumped == 1);
    CHECK(memcmp(sim_flash, image, len) == 0);
}

/* 漏写一页：整包 CRC 报错、不写 APOK、不跳转；补上这一页就能完成，不用从头来 */
static void check_missing_page(void)
{
    uint32_t len, off;

    flash_blank();
    len = make_image(4000, 0xFF, 0xFF, 0xFF, 0xFF, 1);
    CHECK(cmd_start(len, ref_crc32(image, len), 0) == ST_OK);
    for (off = 0; off < len; off += 256)
        if (off != 1024) CHECK(cmd_write(off, image + off) == ST_OK);
    CHECK(send(CMD_FINISH, 0, 0) == ST_IMAGE_CRC);
    CHECK(get32(sim_flash + FLAG_OFFSET) == FLAG_UPGD && jumped == 0);
    CHECK(cmd_write(1024, image + 1024) == ST_OK);
    CHECK(send(CMD_FINISH, 0, 0) == ST_OK && jumped == 1);
}

/* 升级途中断电：上电后标志页是 UPGD 不进 APP，没重新开始前完成被拒，重新开始就能升完 */
static void check_power_loss(void)
{
    uint32_t len, off;

    flash_blank();
    sim_flash[0] = 0x6F;
    len = make_image(6000, 0xFF, 0xFF, 0xFF, 0xFF, 1);
    CHECK(cmd_start(len, ref_crc32(image, len), 0) == ST_OK);
    for (off = 0; off < 2048; off += 256) CHECK(cmd_write(off, image + off) == ST_OK);

    reboot();
    CHECK(!App_Bootable());
    CHECK(send(CMD_HELLO, 0, 0) == ST_OK && tx[7] == 1);
    CHECK(send(CMD_FINISH, 0, 0) == ST_ORDER);
    CHECK(upgrade(len, 0) == ST_OK && jumped == 1);
    CHECK(App_Bootable());
}

/* 固件与硬件信息页逐项比对，0xFF 表示不限；硬件信息页没写过时不比对；镜像缺固件信息块一律拒绝 */
static void check_compat(void)
{
    uint32_t len;

    flash_blank();
    set_hwinfo(1, 2, 3, 4);
    len = make_image(1500, 9, 2, 3, 4, 1);                /* 舵机类型不对 */
    CHECK(upgrade(len, 0) == ST_MISMATCH);
    CHECK(get32(sim_flash + FLAG_OFFSET) == FLAG_UPGD && jumped == 0);

    len = make_image(1500, 1, 2, 0xFF, 4, 1);             /* 电压型号不限 */
    CHECK(upgrade(len, 0) == ST_OK && jumped == 1);

    flash_blank();                                        /* 硬件信息页没写：不比对 */
    len = make_image(1500, 9, 9, 9, 9, 1);
    CHECK(upgrade(len, 0) == ST_OK && jumped == 1);

    flash_blank();
    len = make_image(1500, 0, 0, 0, 0, 0);                /* 不是本产品的固件 */
    CHECK(upgrade(len, 0) == ST_MISMATCH && jumped == 0);
}

/* 写入读回不一致报 4，重发同一页即可恢复 */
static void check_verify_fail(void)
{
    uint32_t len;

    flash_blank();
    len = make_image(600, 0xFF, 0xFF, 0xFF, 0xFF, 1);
    CHECK(cmd_start(len, ref_crc32(image, len), 0) == ST_OK);
    corrupt_addr = FLASH_BASE + 256;
    CHECK(cmd_write(256, image + 256) == ST_VERIFY);
    corrupt_addr = 0;
    CHECK(cmd_write(0, image) == ST_OK);
    CHECK(cmd_write(256, image + 256) == ST_OK);
    CHECK(cmd_write(512, image + 512) == ST_OK);
    CHECK(send(CMD_FINISH, 0, 0) == ST_OK);
}

/* 坏帧：CRC 错、未知命令、LEN 不对都回 1；应答帧不理；超长帧直接丢弃且不影响下一帧 */
static void check_bad_frames(void)
{
    uint8_t d[9] = {0};
    uint8_t junk[] = { 0x12, 0xAA, 0x00, 0xAA, 0xAA };
    uint8_t big[] = { 0xAA, 0x55, CMD_WRITE, 0x05, 0x01 };  /* LEN=261 */
    size_t  i;

    flash_blank();
    CHECK(send_raw(CMD_HELLO, 0, 0, 0, 1) == ST_FRAME);
    CHECK(send(0x09, 0, 0) == ST_FRAME);
    CHECK(send(CMD_START, d, 8) == ST_FRAME);
    CHECK(send(CMD_HELLO, d, 1) == ST_FRAME);
    CHECK(send(CMD_START | 0x80, d, 9) == -1);
    CHECK(sim_flash[FLAG_OFFSET] == 0xFF);

    tx_len = 0;
    for (i = 0; i < sizeof(big); i++) CHECK(!Frame_Byte(big[i]));
    CHECK(s_pos == 0);
    for (i = 0; i < sizeof(junk); i++) (void)Frame_Byte(junk[i]);   /* 以 AA AA 结尾，紧接的 55 也要能对上帧头 */
    CHECK(send(CMD_HELLO, 0, 0) == ST_OK);
}

/* 上电握手窗口只认握手：别的命令(哪怕帧正确)和坏帧都不应答、不执行 */
static void check_hello_window(void)
{
    uint8_t d[9] = {0};

    flash_blank();
    sim_flash[0] = 0x6F;
    put32(d, 256);
    CHECK(send_raw(CMD_START, d, 9, 1, 0) == -1);
    CHECK(send_raw(CMD_HELLO, 0, 0, 1, 1) == -1);
    CHECK(sim_flash[0] == 0x6F && sim_flash[FLAG_OFFSET] == 0xFF);
    CHECK(send_raw(CMD_HELLO, 0, 0, 1, 0) == ST_OK && last_ret == 1);
}

/* 主循环的上电窗口：没握手就超时返回，收到握手立刻返回，窗口里别的命令不执行 */
static void check_loop_window(void)
{
    uint8_t start[9] = {0};

    flash_blank();
    escape_on_jump = 1;

    tick_budget = 1000;                                   /* 没有数据：正好等满窗口 */
    if (setjmp(esc) == 0) CHECK(Loop(HELLO_WINDOW_MS) == 0U);
    else CHECK(0);
    CHECK(ticks_used == HELLO_WINDOW_MS && tx_len == 0);

    reboot();                                             /* 窗口里收到握手：立即返回 1 */
    rx_push_frame(CMD_HELLO, 0, 0);
    tick_budget = 1000;
    if (setjmp(esc) == 0) CHECK(Loop(HELLO_WINDOW_MS) == 1U);
    else CHECK(0);
    CHECK(tx_len > 0 && tx[5] == ST_OK && ticks_used == 0);

    reboot();                                             /* 窗口里收到开始命令：不理不擦 */
    put32(start, 256);
    rx_push_frame(CMD_START, start, 9);
    tick_budget = 1000;
    if (setjmp(esc) == 0) CHECK(Loop(HELLO_WINDOW_MS) == 0U);
    else CHECK(0);
    CHECK(tx_len == 0 && sim_flash[FLAG_OFFSET] == 0xFF && ticks_used == HELLO_WINDOW_MS);
    escape_on_jump = 0;
}

/* 升级循环：10 秒没有数据且 APP 完好就回 APP；APP 不完好就一直等 */
static void check_loop_idle(void)
{
    flash_blank();
    sim_flash[0] = 0x6F;                                  /* 量产刚烧录：标志页空白，APP 有效 */
    escape_on_jump = 1;

    tick_budget = 20000;
    if (setjmp(esc) == 0) { (void)Loop(0U); CHECK(0); }
    CHECK(jumped == 1 && ticks_used == IDLE_TO_APP_MS);

    reboot();                                             /* 升级中断电过：APP 不可用，一直等 */
    set_flag("UPGD");
    tick_budget = 20000;
    if (setjmp(esc) == 0) { (void)Loop(0U); CHECK(0); }
    CHECK(jumped == 0 && ticks_used == 20000);
    escape_on_jump = 0;
}

/* 帧内字节间隔超过 50ms：半帧丢弃，后到的字节不会拼成一帧 */
static void check_loop_bytegap(void)
{
    uint8_t head[3] = { 0xAA, 0x55, CMD_HELLO };
    uint8_t rest[8] = { 0x00, 0x00, 0x25, 0xB3, 0x83, 0xFE, 0x55, 0xAA };

    flash_blank();
    set_flag("UPGD");                                     /* APP 不可用，免得空闲回 APP */
    escape_on_jump = 1;

    rx_push(head, 3);
    tick_budget = BYTE_GAP_MS + 10;
    if (setjmp(esc) == 0) { (void)Loop(0U); CHECK(0); }
    CHECK(s_pos == 0 && tx_len == 0);

    rx_push(rest, 8);                                     /* 剩下半截：拼不成帧，不应答 */
    tick_budget = 10;
    if (setjmp(esc) == 0) { (void)Loop(0U); CHECK(0); }
    CHECK(tx_len == 0);

    rx_push_frame(CMD_HELLO, 0, 0);                       /* 完整的一帧照常应答 */
    tick_budget = 10;
    if (setjmp(esc) == 0) { (void)Loop(0U); CHECK(0); }
    CHECK(tx_len > 0 && tx[5] == ST_OK);
    escape_on_jump = 0;
}

/* 复位原因分支：上电复位先走窗口，APP 请求升级(软件复位)直接进升级循环 */
static void check_run_entry(void)
{
    flash_blank();
    sim_flash[0] = 0x6F;
    set_flag("APOK");
    escape_on_jump = 1;

    soft_reset_flag = 0;                                  /* 上电：窗口内没人握手就进 APP */
    tick_budget = 1000;
    if (setjmp(esc) == 0) { IAP_Run(); CHECK(0); }
    CHECK(jumped == 1 && ticks_used == HELLO_WINDOW_MS);

    reboot();
    soft_reset_flag = 1;                                  /* APP 请求升级：没有窗口，等命令 */
    tick_budget = 1000;
    if (setjmp(esc) == 0) { IAP_Run(); CHECK(0); }
    CHECK(jumped == 0 && ticks_used == 1000);
    escape_on_jump = 0;
    soft_reset_flag = 0;
}

int main(void)
{
    check_crc();
    check_magic();
    check_hello();
    check_bootable();
    check_order();
    check_start_range();
    check_upgrade_mode0();
    check_upgrade_mode1();
    check_retry_and_range();
    check_missing_page();
    check_power_loss();
    check_compat();
    check_verify_fail();
    check_bad_frames();
    check_hello_window();
    check_loop_window();
    check_loop_idle();
    check_loop_bytegap();
    check_run_entry();
    printf("boot failures=%d\n", failures);
    return failures ? 1 : 0;
}
