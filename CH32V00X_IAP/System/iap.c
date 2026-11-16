/*
 * iap.c —— Bootloader 主流程与升级协议
 *   复位：软件复位 = APP 收到进入升级后主动复位，直接进升级循环；
 *         上电复位先等 200ms 握手(救砖用)，没有握手且 APP 完好就进 APP，否则留下。
 *   升级循环：握手 / 开始 / 写页 / 完成。
 *   10 秒没有收到有效 Boot 帧(握手/开始/写页/完成)且 APP 完好，自动回 APP；总线上别的流量不算。
 *   非 Boot 命令、CRC 不对的帧静默丢弃，不应答。
 *   升级镜像是 AES-128-CTR 密文：开始命令带 12 字节 IV，第 n 个 16 字节块的计数器 = IV || n(大端32位)，
 *   n = 镜像内偏移/16；写页前原地解密，整包 CRC32 按明文核对。密钥见 boot_key.h(与 gen_key.py 生成的 boot_key.txt 同一串)。
 *   分区：Boot 在主 Flash [0, APP_BASE)，APP 在 [APP_BASE, 0xF400)，跳 APP 是直接跳转，不复位。
 * 主机端测试定义 BOOT_HOST_TEST，并替换 FLASH_PTR / IWDG_FEED，硬件相关的函数由测试提供。
 */
#include "iap.h"
#include "flash.h"
#include "boot_config.h"
#include "boot_key.h"
#include <string.h>

#define HELLO_WINDOW_MS    200U     /** 上电握手窗口 */
#define IDLE_TO_APP_MS     10000U   /** 还没开始升级时，无数据多久回 APP */
#define BYTE_GAP_MS        50U      /** 帧内字节间隔超过它就丢弃半帧 */
#define TICK_CMP           6000U    /** SysTick 默认时钟 HCLK/8 = 6MHz，计满即 1ms */
#define USART_BRR_115200   0x01A1U  /** 48MHz 下 115200 */
#define PAGE_SIZE          256U
#define CONFIG_COMMITTED   0x5AA55AA5U
#define CONFIG_CRC_OFFSET  250U
#define CONFIG_COMMIT_OFFSET 252U
#define CONFIG_ID_OFFSET   3U
#define CONFIG_BAUD_OFFSET 4U
#define FLAG_KEEP_VALID_OFFSET 12U
#define FLAG_ID_OFFSET     13U
#define FLAG_BAUD_OFFSET   14U
#define FLAG_HW_VALID_OFFSET 15U
#define FLAG_HW_OFFSET     16U
#define FLAG_CRC_OFFSET    20U
#define FLAG_DONE_OFFSET   24U

#ifndef FLASH_PTR
#define FLASH_PTR(off) ((const uint8_t *)(FLASH_BASE + (off)))
#endif
#define APP_PTR(off)   FLASH_PTR(APP_BASE + (off))   /** 镜像内偏移 -> 指针 */
#define IV_SIZE        12U
#ifndef IWDG_FEED
#define IWDG_FEED() (IWDG->CTLR = 0xAAAAU) /* APP 开过独立看门狗时软复位后可能仍在计数；没开时写入无副作用 */
#endif

/* 接收缓冲按字对齐，CMD 存在第1字节：写页帧的 256 字节数据正好从第8字节开始，
 * 可直接当 uint32_t 数组交给 CH32_IAP_Program */
static uint32_t s_rx_words[(1U + 3U + IAP_DATA_MAX + 4U + 2U + 3U) / 4U];
#define RX ((uint8_t *)s_rx_words)

static uint32_t s_page[PAGE_SIZE / 4U]; /** 标志页写入缓冲 */
static uint16_t s_pos;                  /** 当前帧已收字节数(含帧头)，0=等帧头 */
static uint8_t  s_started;              /** 1=开始命令已成功，允许写页和完成 */
static uint32_t s_img_len;              /** 开始命令给的镜像长度 */
static uint32_t s_img_crc;              /** 开始命令给的镜像 CRC32(明文) */
static uint8_t  s_iv[IV_SIZE];          /** 开始命令给的 CTR 初始值，每个升级文件不同 */
static uint8_t  s_keep_id;
static uint8_t  s_keep_baud;
static uint8_t  s_keep_valid;
static uint8_t  s_hw_snapshot[4];
static uint8_t  s_hw_snapshot_valid;

/* ============================== 硬件 ============================== */
#ifndef BOOT_HOST_TEST

/* SysTick 1ms 基准 + USART1 重映射3(TX=PC0)，单线半双工，115200 8N1，查询方式收发 */
static void Hw_Init(void)
{
    SysTick->CMP  = TICK_CMP;
    SysTick->CNT  = 0U;
    SysTick->CTLR = 1U;                             /* 使能，时钟默认 HCLK/8 */

    RCC->PB2PCENR |= RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1;
    AFIO->PCFR1 = (AFIO->PCFR1 & ~AFIO_PCFR1_USART1_REMAP)
                | AFIO_PCFR1_USART1_REMAP_0 | AFIO_PCFR1_USART1_REMAP_1;
    GPIOC->CFGLR = (GPIOC->CFGLR & ~0x0FU) | 0x09U;  /* PC0 复用推挽 30MHz */
    USART1->BRR   = USART_BRR_115200;
    USART1->CTLR3 = USART_CTLR3_HDSEL;
    USART1->CTLR1 = USART_CTLR1_UE | USART_CTLR1_TE | USART_CTLR1_RE;
}

/* 取一个已收字节，0=没有 */
static uint8_t Uart_Poll(uint8_t *byte)
{
    if (!(USART1->STATR & USART_STATR_RXNE)) return 0U;
    *byte = (uint8_t)USART1->DATAR;
    return 1U;
}

/* 又过了 1ms 返回 1；顺便喂狗 */
static uint8_t Tick_Poll(void)
{
    if (!(SysTick->SR & 1U)) return 0U;
    SysTick->SR  = 0U;
    SysTick->CNT = 0U;
    IWDG_FEED();
    return 1U;
}

/* 本次复位是不是软件复位(APP 收到进入升级后主动复位)；读完清标志 */
static uint8_t Reset_WasSoftware(void)
{
    uint8_t soft = (RCC->RSTSCKR & RCC_SFTRSTF) != 0U;

    RCC->RSTSCKR |= RCC_RMVF;
    return soft;
}

/* 阻塞发送；发送期间关接收，免得单线回环把自己发的字节收回来 */
static void Uart_Send(const uint8_t *data, uint8_t len)
{
    USART1->CTLR1 &= ~USART_CTLR1_RE;
    while (len--)
    {
        while (!(USART1->STATR & USART_STATR_TXE)) {}
        USART1->DATAR = *data++;
    }
    while (!(USART1->STATR & USART_STATR_TC)) {}
    USART1->CTLR1 |= USART_CTLR1_RE;
}

/* 直接跳到 APP 入口(APP 启动代码会重设栈、mtvec、时钟)。Boot 用到的外设先复位回上电状态 */
static void Jump_App(void)
{
    const uint32_t periph = RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO | RCC_PB2Periph_USART1;

    __disable_irq();
    FLASH_Lock_Fast();
    FLASH_Lock();
    SysTick->CTLR = 0U;
    SysTick->SR   = 0U;
    RCC->PB2PRSTR |= periph;
    RCC->PB2PRSTR &= ~periph;
    RCC->PB2PCENR &= ~periph;
    ((void (*)(void))APP_BASE)();   /* 必须跳 0 起的执行地址(APP 按 0x1400 链接)，不能用 0x08000000 别名，否则取指卡死 */
    while (1) {}
}

/* 选项字节：上电从用户区启动(Boot 在主 Flash 0 地址)；BOOT_RDP_ENABLE=1 时同时打开读保护。
 * 只在需要时整组写一次(擦写选项字节时掉电有风险)；读保护已生效就不再碰。新值断电再上电后生效 */
void FLASH_OptionBytePR(u32 *pbuf);   /* 库函数(ch32v00X_flash.c)，头文件里没有声明 */
#define OB_USER_START_BOOT 0x20U   /** USER 字节里"上电从 BOOT 区启动"位 */
#define OB_RDPR_OFF        0xA5U   /** RDPR=0xA5 即未开读保护 */
static void OptionBytes_Check(void)
{
    uint32_t buf[4];
    uint8_t  rdpr, user;
    uint8_t  i;

    if (FLASH->OBR & 0x02U) return;                 /* 读保护已生效 */
    for (i = 0U; i < 4U; i++) buf[i] = *(const uint32_t *)(OB_BASE + 4U * i);
    rdpr = (uint8_t)buf[0];
    user = (uint8_t)(buf[0] >> 16);
    if (!(user & OB_USER_START_BOOT) && (!BOOT_RDP_ENABLE || rdpr != OB_RDPR_OFF)) return;

    user &= (uint8_t)~OB_USER_START_BOOT;
    if (BOOT_RDP_ENABLE) rdpr = 0xFFU;              /* 非 0xA5 即读保护；0xFF/0x00 与库函数一致 */
    buf[0] = (uint32_t)rdpr | (uint32_t)(uint8_t)~rdpr << 8
           | (uint32_t)user << 16 | (uint32_t)(uint8_t)~user << 24;
    FLASH_OptionBytePR(buf);
}

#endif /* BOOT_HOST_TEST */

/* ============================== 工具 ============================== */

static uint32_t Get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint16_t Get16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)p[1] << 8;
}

static void Put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

/* 标准 CRC-32(与 Python zlib.crc32 一致)，逐位计算不占表 */
static uint32_t Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint8_t  bit;

    while (len--)
    {
        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

static uint16_t Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint8_t bit;

    while (len--)
    {
        crc ^= (uint16_t)*data++ << 8;
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U)
                ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

/* ============================== AES-128-CTR ==============================
 * 只用加密方向：CTR 解密 = 用密钥加密计数器得到密钥流，再与密文异或。轮密钥边算边用，不占 RAM 表 */

static const uint8_t k_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static uint8_t Aes_Xtime(uint8_t x) { return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1BU)); }

/* AES-128 加密一个块(原地)。状态按列存放：s[4c+r] = 第 c 列第 r 行 */
static void Aes_Encrypt(uint8_t s[16])
{
    uint8_t rk[16], t[16], rc = 1U, r, i, a, b, c, d, e;

    memcpy(rk, k_boot_key, sizeof(rk));
    for (i = 0U; i < 16U; i++) s[i] ^= rk[i];
    for (r = 1U; r <= 10U; r++)
    {
        for (i = 0U; i < 16U; i++) t[i] = k_sbox[s[(i + 4U * (i & 3U)) & 15U]];   /* SubBytes + ShiftRows */
        if (r < 10U)
        {
            for (i = 0U; i < 16U; i += 4U)                                      /* MixColumns */
            {
                a = t[i]; b = t[i + 1U]; c = t[i + 2U]; d = t[i + 3U]; e = (uint8_t)(a ^ b ^ c ^ d);
                t[i]      ^= (uint8_t)(e ^ Aes_Xtime((uint8_t)(a ^ b)));
                t[i + 1U] ^= (uint8_t)(e ^ Aes_Xtime((uint8_t)(b ^ c)));
                t[i + 2U] ^= (uint8_t)(e ^ Aes_Xtime((uint8_t)(c ^ d)));
                t[i + 3U] ^= (uint8_t)(e ^ Aes_Xtime((uint8_t)(d ^ a)));
            }
        }
        rk[0] ^= (uint8_t)(k_sbox[rk[13]] ^ rc);                                /* 下一轮轮密钥 */
        rk[1] ^= k_sbox[rk[14]];
        rk[2] ^= k_sbox[rk[15]];
        rk[3] ^= k_sbox[rk[12]];
        for (i = 4U; i < 16U; i++) rk[i] ^= rk[i - 4U];
        rc = Aes_Xtime(rc);
        for (i = 0U; i < 16U; i++) s[i] = (uint8_t)(t[i] ^ rk[i]);
    }
}

/* 原地解密一页：off 为镜像内偏移(页对齐) */
static void Ctr_DecryptPage(uint8_t *page, uint32_t off)
{
    uint8_t  ks[16];
    uint32_t n = off >> 4;  /** 块号 */
    uint16_t i;

    for (i = 0U; i < PAGE_SIZE; i++)
    {
        if ((i & 15U) == 0U)
        {
            memcpy(ks, s_iv, IV_SIZE);
            ks[12] = (uint8_t)(n >> 24); ks[13] = (uint8_t)(n >> 16);
            ks[14] = (uint8_t)(n >> 8);  ks[15] = (uint8_t)n;
            Aes_Encrypt(ks);
            n++;
        }
        page[i] ^= ks[i & 15U];
    }
}

/* 应答帧：CMD 取当前帧并置最高位，DATA = 状态码 + extra */
static void Reply(uint8_t status, const uint8_t *extra, uint8_t n)
{
    uint8_t frame[12U + 6U]; /** 最长应答缓冲 */
    uint8_t i;

    frame[0] = IAP_HEAD1;
    frame[1] = IAP_HEAD2;
    frame[2] = RX[1] | 0x80U;
    frame[3] = (uint8_t)(n + 1U);
    frame[4] = 0U;
    frame[5] = status;
    for (i = 0U; i < n; i++) frame[6U + i] = extra[i];
    Put32(frame + 6U + n, Crc32(frame + 2U, 4U + n));
    frame[10U + n] = IAP_TAIL1;
    frame[11U + n] = IAP_TAIL2;
    Uart_Send(frame, (uint8_t)(12U + n));
}

/* ============================== Flash ============================== */

static uint8_t Flag_RecordValid(const uint8_t *page)
{
    return Get32(page) == FLAG_UPGD
        && Get32(page + FLAG_CRC_OFFSET) == Crc32(page, FLAG_CRC_OFFSET);
}

static uint8_t Flag_State(void)
{
    const uint8_t *page = FLASH_PTR(FLAG_OFFSET);

    if (Get32(page) == 0xFFFFFFFFU && Get32(page + FLAG_DONE_OFFSET) == 0xFFFFFFFFU) return 0U;
    if (!Flag_RecordValid(page)) return 3U;
    return Get32(page + FLAG_DONE_OFFSET) == FLAG_APOK ? 2U : 1U;
}

static uint8_t App_Bootable(void)
{
    uint8_t state = Flag_State();

    return state == 2U || (state == 0U && Get32(APP_PTR(0U)) != 0xFFFFFFFFU);
}

static uint8_t Flag_Start(void)
{
    uint8_t *bytes = (uint8_t *)s_page;
    const uint8_t *stored = FLASH_PTR(FLAG_OFFSET);

    /* 未完成升级的快照已经可靠落盘时直接复用，避免重试时擦掉唯一的身份副本。 */
    if (Flag_RecordValid(stored) && Get32(stored + FLAG_DONE_OFFSET) != FLAG_APOK)
        return 1U;

    memset(s_page, 0xFF, sizeof(s_page));
    s_page[0] = FLAG_UPGD;
    s_page[1] = s_img_len;
    s_page[2] = s_img_crc;
    bytes[FLAG_KEEP_VALID_OFFSET] = s_keep_valid;
    bytes[FLAG_ID_OFFSET] = s_keep_id;
    bytes[FLAG_BAUD_OFFSET] = s_keep_baud;
    bytes[FLAG_HW_VALID_OFFSET] = s_hw_snapshot_valid;
    memcpy(bytes + FLAG_HW_OFFSET, s_hw_snapshot, sizeof(s_hw_snapshot));
    s_page[FLAG_CRC_OFFSET / 4U] = Crc32(bytes, FLAG_CRC_OFFSET);
    FLASH_ErasePage_Fast(FLASH_BASE + FLAG_OFFSET);
    CH32_IAP_Program(FLASH_BASE + FLAG_OFFSET, s_page);
    return (uint8_t)(memcmp(FLASH_PTR(FLAG_OFFSET), s_page, PAGE_SIZE) == 0);
}

static uint8_t Flag_Complete(void)
{
    memset(s_page, 0xFF, sizeof(s_page));
    s_page[FLAG_DONE_OFFSET / 4U] = FLAG_APOK;
    CH32_IAP_Program(FLASH_BASE + FLAG_OFFSET, s_page);
    return Flag_State() == 2U;
}

/* 读硬件信息页的 4 项(舵机类型、硬件版本、电压型号、扭力型号)；页无效时填 0xFF 并返回 0 */
static uint8_t HwInfo_Read(uint8_t *info)
{
    const uint8_t *page  = FLASH_PTR(HWINFO_OFFSET);
    uint8_t        valid = Get32(page) == HWINFO_MAGIC && Crc32(page, 28U) == Get32(page + 28U);
    uint8_t        i;

    for (i = 0U; i < 4U; i++) info[i] = valid ? page[5U + i] : 0xFFU;
    return valid;
}

static uint8_t Flag_MetaRead(void)
{
    const uint8_t *page = FLASH_PTR(FLAG_OFFSET);

    if (!Flag_RecordValid(page)) return 0U;
    s_keep_valid = page[FLAG_KEEP_VALID_OFFSET];
    s_keep_id = page[FLAG_ID_OFFSET];
    s_keep_baud = page[FLAG_BAUD_OFFSET];
    s_hw_snapshot_valid = page[FLAG_HW_VALID_OFFSET];
    memcpy(s_hw_snapshot, page + FLAG_HW_OFFSET, sizeof(s_hw_snapshot));
    return 1U;
}

static uint8_t Config_IdentityRead(uint32_t off, uint8_t *id, uint8_t *baud)
{
    const uint8_t *page = FLASH_PTR(off);

    if (Get32(page + CONFIG_COMMIT_OFFSET) != CONFIG_COMMITTED
        || Get16(page + CONFIG_CRC_OFFSET) != Crc16(page, CONFIG_CRC_OFFSET)
        || page[CONFIG_ID_OFFSET] > 254U
        || page[CONFIG_BAUD_OFFSET] < 1U || page[CONFIG_BAUD_OFFSET] > 8U) return 0U;
    *id = page[CONFIG_ID_OFFSET];
    *baud = page[CONFIG_BAUD_OFFSET];
    return 1U;
}

static void Preserve_Snapshot(void)
{
    uint8_t hw[4];
    uint8_t id;
    uint8_t baud;

    s_keep_id = 0U;
    s_keep_baud = 5U;
    s_keep_valid = 0U;
    memset(s_hw_snapshot, 0xFF, sizeof(s_hw_snapshot));
    s_hw_snapshot_valid = 0U;
    (void)Flag_MetaRead();

    if (Config_IdentityRead(CONFIG_SLOT_A, &id, &baud)
        || Config_IdentityRead(CONFIG_SLOT_B, &id, &baud))
    {
        s_keep_id = id;
        s_keep_baud = baud;
        s_keep_valid = 1U;
    }
    if (HwInfo_Read(hw))
    {
        memcpy(s_hw_snapshot, hw, sizeof(s_hw_snapshot));
        s_hw_snapshot_valid = 1U;
    }
}

/* 镜像里的固件信息块魔术字正确，且 4 项逐一等于硬件信息页或为 0xFF(不限)；硬件信息页没写过时不比对 */
static uint8_t Fw_Compatible(void)
{
    const uint8_t *fw = APP_PTR(FWINFO_OFFSET);
    uint8_t        hw[4];
    uint8_t        valid = HwInfo_Read(hw);
    uint8_t        i;

    if (!valid && s_hw_snapshot_valid)
    {
        memcpy(hw, s_hw_snapshot, sizeof(hw));
        valid = 1U;
    }
    if (Get32(fw) != FWINFO_MAGIC) return 0U;
    for (i = 0U; i < 4U; i++)
    {
        if (valid && fw[4U + i] != 0xFFU && fw[4U + i] != hw[i]) return 0U;
    }
    return 1U;
}

/* 擦除 [APP_BASE, end)(Flash 绝对偏移)：逐页擦，不碰 Boot 自己所在的 [0, APP_BASE) */
static void App_Erase(uint32_t end)
{
    uint32_t off;

    for (off = APP_BASE; off < end; off += PAGE_SIZE)
    {
        IWDG_FEED();
        FLASH_ErasePage_Fast(FLASH_BASE + off);
    }
}

/* ============================== 协议 ============================== */

/* 喂入一个字节，收齐一帧(帧头帧尾正确)返回 1；帧内容在 RX，CMD 位于 RX[1] */
static uint8_t Frame_Byte(uint8_t byte)
{
    uint16_t len;

    if (s_pos == 0U)
    {
        if (byte == IAP_HEAD1) s_pos = 1U;
        return 0U;
    }
    if (s_pos == 1U)
    {
        s_pos = (byte == IAP_HEAD2) ? 2U : (byte == IAP_HEAD1); /* AA AA 55 也能对上帧头 */
        return 0U;
    }

    RX[s_pos - 1U] = byte;
    s_pos++;
    if (s_pos < 5U) return 0U;                          /* CMD、LEN 还没收齐 */
    len = (uint16_t)(RX[2] | (uint16_t)RX[3] << 8);
    if (len > IAP_DATA_MAX)
    {
        s_pos = 0U;
        return 0U;
    }
    if (s_pos < 11U + len) return 0U;
    s_pos = 0U;
    return RX[8U + len] == IAP_TAIL1 && RX[9U + len] == IAP_TAIL2;
}

/* 处理收齐的一帧。hello_only=1(上电窗口)时只认握手。
 * 返回 FRAME_NONE = 不是发给 Boot 的帧(静默丢弃)；FRAME_OK = Boot 命令；FRAME_HELLO = 握手 */
#define FRAME_NONE  0U
#define FRAME_OK    1U
#define FRAME_HELLO 2U
static uint8_t Frame_Process(uint8_t hello_only)
{
    uint8_t   cmd  = RX[1];
    uint16_t  len  = (uint16_t)(RX[2] | (uint16_t)RX[3] << 8);
    uint8_t  *data = RX + 4U;
    uint8_t   info[5];
    uint32_t  off;

    /* 总线上 APP 协议也用 AA 55 帧头(查询/进入升级)，应答帧、非 Boot 命令、CRC 不对的帧
     * 一律不回，免得和别的舵机的应答撞在一起 */
    if (cmd < CMD_HELLO || cmd > CMD_FINISH) return FRAME_NONE;
    if (Crc32(RX + 1U, 3U + len) != Get32(data + len)) return FRAME_NONE;
    if (hello_only && cmd != CMD_HELLO) return FRAME_NONE;

    switch (cmd)
    {
    case CMD_HELLO:
        if (len != 0U) break;
        info[0] = Flag_State();
        (void)HwInfo_Read(info + 1U);
        Reply(ST_OK, info, 5U);
        return FRAME_HELLO;

    case CMD_START:                                     /* 长度4 + 明文CRC32 4 + IV 12 */
        if (len != 8U + IV_SIZE) break;
        s_started = 0U;
        s_img_len = Get32(data);
        s_img_crc = Get32(data + 4U);
        memcpy(s_iv, data + 8U, IV_SIZE);
        if (s_img_len == 0U || (s_img_len & (PAGE_SIZE - 1U)) || s_img_len > APP_SIZE_MAX)
        {
            Reply(ST_RANGE, 0, 0U);
            return FRAME_OK;
        }
        FLASH_Unlock_Fast();
        Preserve_Snapshot();
        if (!Flag_Start())
        {
            Reply(ST_VERIFY, 0, 0U);
            return FRAME_OK;
        }
        App_Erase(APP_DATA_END);
        s_started = 1U;
        Reply(ST_OK, 0, 0U);
        return FRAME_OK;

    case CMD_WRITE:
        if (len != 4U + PAGE_SIZE) break;
        off = Get32(data);
        if (!s_started)
        {
            Reply(ST_ORDER, data, 4U);
        }
        else if ((off & (PAGE_SIZE - 1U)) || off >= s_img_len)
        {
            Reply(ST_RANGE, data, 4U);
        }
        else
        {
            Ctr_DecryptPage(data + 4U, off);            /* 原地解密；data+4 即 s_rx_words+2，字对齐 */
            FLASH_ErasePage_Fast(FLASH_BASE + APP_BASE + off);   /* 先擦再写，同一页重发多少次结果都一样 */
            CH32_IAP_Program(FLASH_BASE + APP_BASE + off, s_rx_words + 2U);
            Reply(memcmp(APP_PTR(off), data + 4U, PAGE_SIZE) ? ST_VERIFY : ST_OK, data, 4U);
        }
        return FRAME_OK;

    case CMD_FINISH:
        if (len != 0U) break;
        if (!s_started)
        {
            Reply(ST_ORDER, 0, 0U);
        }
        else if (Crc32(APP_PTR(0U), s_img_len) != s_img_crc)
        {
            Reply(ST_IMAGE_CRC, 0, 0U);
        }
        else if (!Fw_Compatible())
        {
            Reply(ST_MISMATCH, 0, 0U);
        }
        else
        {
            if (!Flag_Complete())
            {
                Reply(ST_VERIFY, 0, 0U);
                return FRAME_OK;
            }
            Reply(ST_OK, 0, 0U);                        /* 发送完才返回，应答不会被复位截断 */
            Jump_App();
        }
        return FRAME_OK;

    default:
        break;
    }
    Reply(ST_FRAME, 0, 0U);                             /* 未知命令或 LEN 不对 */
    return FRAME_OK;
}

/* ============================== 主流程 ============================== */

/* 收帧循环。window_ms!=0：上电握手窗口，只认握手，收到返回 1，超时返回 0；
 * window_ms==0：升级循环，不返回 */
static uint8_t Loop(uint16_t window_ms)
{
    uint16_t gap     = 0U; /** 距上一个字节的毫秒数(帧内超时) */
    uint16_t idle    = 0U; /** 距上一个有效 Boot 帧的毫秒数；总线上别的流量不算 */
    uint16_t elapsed = 0U; /** 窗口已过去的毫秒数   */
    uint8_t  byte;
    uint8_t  result;

    for (;;)
    {
        if (Uart_Poll(&byte))
        {
            gap = 0U;
            if (Frame_Byte(byte))
            {
                result = Frame_Process(window_ms != 0U);
                if (result != FRAME_NONE) idle = 0U;
                if (result == FRAME_HELLO && window_ms != 0U) return 1U;
            }
        }
        else if (Tick_Poll())                           /* 又过了 1ms */
        {
            if (gap < 0xFFFFU) gap++;
            if (gap > BYTE_GAP_MS) s_pos = 0U;
            if (idle < 0xFFFFU) idle++;
            if (window_ms != 0U)
            {
                if (++elapsed >= window_ms) return 0U;
            }
            else if (idle >= IDLE_TO_APP_MS && App_Bootable())
            {
                Jump_App();
            }
        }
    }
}

void IAP_Run(void)
{
    uint8_t app_request = Reset_WasSoftware(); /** 软件复位 = APP 收到进入升级后主动复位 */

    OptionBytes_Check();
    Hw_Init();
    if (!app_request && !Loop(HELLO_WINDOW_MS) && App_Bootable())
    {
        Jump_App();
    }
    (void)Loop(0U);
}
