/*
 * sim_main.c —— 整机仿真的主程序：一台舵机，按文本命令驱动，供 Python 场景脚本调用
 *
 * 命令(每行一条)：
 *   TX <hex> [ms]         上位机发出这些字节，然后让舵机跑 ms 毫秒(默认 5)
 *   TICK <ms>             只推进时间
 *   BAUD <n>              上位机当前用的波特率
 *   APPBAUD <n>           舵机参数里存的波特率(Boot 恒为 115200)
 *   POWER                 断电重新上电(选项字节：上电从 BOOT 区启动)
 *   ERASEALL              WCH-Link 全片擦除
 *   SETFLASH <off> <hex>  直接写 Flash，模拟 WCH-Link 烧录
 *   FLASH <off> <len>     读 Flash，回 DATA <hex>
 *   ID <n>                设定参数区里的总线 ID，下次 APP 启动生效
 *   STATE                 回 STATE <app|boot> <标志页首字hex>
 *   QUIT
 * 舵机发出的字节用 RX <hex> 输出，每条命令以 END 结尾。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim.h"

uint8_t sim_flash[SIM_FLASH_SIZE];

static uint8_t s_rxq[8192];   /* 上位机 -> 舵机 */
static int     s_rx_head, s_rx_tail;
static uint8_t s_txq[8192];   /* 舵机 -> 上位机 */
static int     s_tx_len;
static int      s_reset_req;   /* 1=进 Boot，2=进 APP */
static int      s_mode;        /* 0=APP，1=Boot */
static uint8_t  s_servo_id;
static uint32_t s_host_baud = 115200;  /* 上位机当前波特率 */
static uint32_t s_app_baud  = 115200;  /* 舵机参数里的波特率 */
static uint32_t s_tx_baud   = 115200;  /* 待发字节是用哪个波特率发出去的 */

/* 当前这一端在用的波特率：Boot 恒为 115200 */
static uint32_t device_baud(void) { return s_mode ? 115200U : s_app_baud; }

/* 波特率对不上时线上收到的就是乱码：字节被打乱，数量也对不上 */
static int garble(const uint8_t *in, int len, uint8_t *out)
{
    static uint32_t seed = 12345U;
    int i, n = 0;

    for (i = 0; i < len; i++)
    {
        seed = seed * 1103515245U + 12345U;
        if ((seed >> 16) % 3U == 0U) continue;              /* 采样错位，丢掉一些 */
        out[n++] = (uint8_t)(in[i] ^ (uint8_t)(seed >> 8));
        if ((seed >> 20) % 5U == 0U) out[n++] = (uint8_t)(seed >> 24); /* 偶尔多出一个 */
    }
    return n;
}

uint8_t sim_rx_pop(uint8_t *byte)
{
    if (s_rx_head == s_rx_tail) return 0U;
    *byte = s_rxq[s_rx_head++];
    if (s_rx_head == s_rx_tail) s_rx_head = s_rx_tail = 0;
    return 1U;
}

void sim_tx_push(const uint8_t *data, int len)
{
    if (s_tx_len == 0) s_tx_baud = device_baud();  /* 发完就复位的话，字节仍是复位前那个波特率发的 */
    while (len-- > 0 && s_tx_len < (int)sizeof(s_txq)) s_txq[s_tx_len++] = *data++;
}

void sim_request_boot(void) { s_reset_req = 1; }
void sim_request_app(void)  { s_reset_req = 2; }
int  sim_reset_pending(void) { return s_reset_req; }

static void rx_push(const uint8_t *data, int len)
{
    while (len-- > 0 && s_rx_tail < (int)sizeof(s_rxq)) s_rxq[s_rx_tail++] = *data++;
}

static void do_reset(void)
{
    s_rx_head = s_rx_tail = 0;                 /* 复位丢掉在途字节 */
    if (s_reset_req == 1)
    {
        s_mode = 1;
        boot_sim_power_on(1);
    }
    else
    {
        /* 参数页被擦掉了(擦除模式1)：APP 会落回出厂的 ID 和波特率 */
        if (sim_flash[0xF400] == 0xFF && sim_flash[0xF401] == 0xFF
            && sim_flash[0xF500] == 0xFF && sim_flash[0xF501] == 0xFF)
        {
            s_servo_id = 0U;
            s_app_baud = 115200U;
        }
        s_mode = 0;
        app_sim_power_on(s_servo_id);
    }
    s_reset_req = 0;
}

static void run(int ms)
{
    int i;

    for (i = 0; i < 4; i++)                    /* 一条命令里最多经历几次复位 */
    {
        if (s_mode) boot_sim_step(ms);
        else        app_sim_step(ms);
        if (!s_reset_req) return;
        do_reset();
    }
}

static void power_on(void)
{
    s_rx_head = s_rx_tail = 0;
    s_tx_len = 0;
    s_reset_req = 0;
    s_mode = 1;                                /* 选项字节设定为上电从 BOOT 区启动 */
    boot_sim_power_on(0);
}

static int parse_hex(const char *text, uint8_t *out, int max)
{
    int n = 0;
    unsigned value;

    while (n < max && sscanf(text, "%2x", &value) == 1)
    {
        out[n++] = (uint8_t)value;
        text += 2;
    }
    return n;
}

static void print_hex(const char *tag, const uint8_t *data, int len)
{
    int i;

    printf("%s ", tag);
    for (i = 0; i < len; i++) printf("%02X", data[i]);
    printf("\n");
}

int main(void)
{
    static char    line[80000];
    static char    hex[76000];
    static uint8_t buf[32768];
    char           word[32];

    memset(sim_flash, 0xFF, sizeof(sim_flash));
    power_on();
    setvbuf(stdout, NULL, _IOLBF, 0);

    while (fgets(line, sizeof(line), stdin))
    {
        if (sscanf(line, "%31s", word) != 1) continue;

        if (strcmp(word, "TX") == 0)
        {
            static uint8_t noise[65536];
            int ms = 5, n;

            hex[0] = '\0';
            sscanf(line, "%*s %75999s %d", hex, &ms);
            n = parse_hex(hex, buf, (int)sizeof(buf));
            if (s_host_baud != device_baud())              /* 波特率对不上：设备收到乱码 */
            {
                n = garble(buf, n, noise);
                rx_push(noise, n);
            }
            else
            {
                rx_push(buf, n);
            }
            run(ms);
        }
        else if (strcmp(word, "BAUD") == 0)
        {
            sscanf(line, "%*s %u", &s_host_baud);
        }
        else if (strcmp(word, "APPBAUD") == 0)
        {
            sscanf(line, "%*s %u", &s_app_baud);
        }
        else if (strcmp(word, "TICK") == 0)
        {
            int ms = 1;

            sscanf(line, "%*s %d", &ms);
            run(ms);
        }
        else if (strcmp(word, "POWER") == 0)
        {
            power_on();
        }
        else if (strcmp(word, "ERASEALL") == 0)
        {
            memset(sim_flash, 0xFF, sizeof(sim_flash));
        }
        else if (strcmp(word, "SETFLASH") == 0)
        {
            unsigned off = 0;
            int      n;

            hex[0] = '\0';
            sscanf(line, "%*s %x %75999s", &off, hex);
            n = parse_hex(hex, buf, (int)sizeof(buf));
            if (off + (unsigned)n <= SIM_FLASH_SIZE) memcpy(sim_flash + off, buf, (size_t)n);
        }
        else if (strcmp(word, "FLASH") == 0)
        {
            unsigned off = 0, len = 0;

            sscanf(line, "%*s %x %u", &off, &len);
            if (off + len <= SIM_FLASH_SIZE) print_hex("DATA", sim_flash + off, (int)len);
        }
        else if (strcmp(word, "ID") == 0)
        {
            unsigned id = 0;

            sscanf(line, "%*s %u", &id);
            s_servo_id = (uint8_t)id;
        }
        else if (strcmp(word, "STATE") == 0)
        {
            printf("STATE %s %02X%02X%02X%02X\n", s_mode ? "boot" : "app",
                   sim_flash[0xF700], sim_flash[0xF701], sim_flash[0xF702], sim_flash[0xF703]);
        }
        else if (strcmp(word, "QUIT") == 0)
        {
            break;
        }

        if (s_tx_len > 0)
        {
            static uint8_t noise[16384];

            if (s_host_baud != s_tx_baud)                  /* 波特率对不上：上位机收到乱码 */
                print_hex("RX", noise, garble(s_txq, s_tx_len, noise));
            else
                print_hex("RX", s_txq, s_tx_len);
        }
        s_tx_len = 0;
        printf("END\n");
        fflush(stdout);
    }
    return 0;
}
