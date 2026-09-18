/*
 * iap.c —— Bootloader 主流程与升级协议
 *   复位：软件复位 = APP 收到进入升级后主动复位，直接进升级循环；
 *         上电复位先等 200ms 握手(救砖用)，没有握手且 APP 完好就进 APP，否则留下。
 *   升级循环：握手 / 开始(写UPGD、擦除) / 写页(擦页、写入、读回) / 完成(整包CRC、兼容检查、写APOK、进APP)。
 *   还没开始升级时 10 秒没有数据且 APP 完好，自动回 APP。
 * 主机端测试定义 BOOT_HOST_TEST，并替换 FLASH_PTR / IWDG_FEED，硬件相关的函数由测试提供。
 */
#include "iap.h"
#include "flash.h"
#include <string.h>

#define HELLO_WINDOW_MS    200U     /** 上电握手窗口 */
#define IDLE_TO_APP_MS     10000U   /** 还没开始升级时，无数据多久回 APP */
#define BYTE_GAP_MS        50U      /** 帧内字节间隔超过它就丢弃半帧 */
#define TICK_CMP           6000U    /** SysTick 默认时钟 HCLK/8 = 6MHz，计满即 1ms */
#define USART_BRR_115200   0x01A1U  /** 48MHz 下 115200 */
#define PAGE_SIZE          256U

#ifndef FLASH_PTR
#define FLASH_PTR(off) ((const uint8_t *)(FLASH_BASE + (off)))
#endif
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
static uint32_t s_img_crc;              /** 开始命令给的镜像 CRC32 */

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

/* 下次复位从 APP 区启动 */
static void Jump_App(void)
{
    RCC->RSTSCKR |= RCC_RMVF;
    SystemReset_StartMode(Start_Mode_USER);
    NVIC_SystemReset();
    while (1) {}
}

#endif /* BOOT_HOST_TEST */

/* ============================== 工具 ============================== */

static uint32_t Get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
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

/* 应答帧：CMD 取当前帧并置最高位，DATA = 状态码 + extra */
static void Reply(uint8_t status, const uint8_t *extra, uint8_t n)
{
    uint8_t frame[12U + 6U]; /** 最长应答：握手带 6 字节 */
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

/* 标志页状态：0=空白 1=升级中 2=APP有效 3=其他(损坏) */
static uint8_t Flag_State(void)
{
    uint32_t flag = Get32(FLASH_PTR(FLAG_OFFSET));

    return (flag == 0xFFFFFFFFU) ? 0U : (flag == FLAG_UPGD) ? 1U : (flag == FLAG_APOK) ? 2U : 3U;
}

/* APP 可以运行：标志页 APOK，或标志页空白(量产刚烧录)且 APP 区不是空的 */
static uint8_t App_Bootable(void)
{
    uint8_t state = Flag_State();

    return state == 2U || (state == 0U && Get32(FLASH_PTR(0U)) != 0xFFFFFFFFU);
}

/* 标志页整页改写：状态、镜像长度、镜像 CRC32，其余 0xFF */
static void Flag_Write(uint32_t state)
{
    memset(s_page, 0xFF, sizeof(s_page));
    s_page[0] = state;
    s_page[1] = s_img_len;
    s_page[2] = s_img_crc;
    FLASH_ErasePage_Fast(FLASH_BASE + FLAG_OFFSET);
    CH32_IAP_Program(FLASH_BASE + FLAG_OFFSET, s_page);
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

/* 镜像里的固件信息块魔术字正确，且 4 项逐一等于硬件信息页或为 0xFF(不限)；硬件信息页没写过时不比对 */
static uint8_t Fw_Compatible(void)
{
    const uint8_t *fw = FLASH_PTR(FWINFO_OFFSET);
    uint8_t        hw[4];
    uint8_t        valid = HwInfo_Read(hw);
    uint8_t        i;

    if (Get32(fw) != FWINFO_MAGIC) return 0U;
    for (i = 0U; i < 4U; i++)
    {
        if (valid && fw[4U + i] != 0xFFU && fw[4U + i] != hw[i]) return 0U;
    }
    return 1U;
}

/* 擦除 [0, end)：前 32K 整块擦，其余逐页擦 */
static void App_Erase(uint32_t end)
{
    uint32_t off;

    FLASH_EraseBlock_32K_Fast(FLASH_BASE);
    for (off = 0x8000U; off < end; off += PAGE_SIZE)
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

/* 处理收齐的一帧。hello_only=1(上电窗口)时只认握手。返回 1 = 处理了握手 */
static uint8_t Frame_Process(uint8_t hello_only)
{
    uint8_t   cmd  = RX[1];
    uint16_t  len  = (uint16_t)(RX[2] | (uint16_t)RX[3] << 8);
    uint8_t  *data = RX + 4U;
    uint8_t   info[6];
    uint32_t  off;

    if (cmd & 0x80U) return 0U;                         /* 应答帧，不处理 */
    if (Crc32(RX + 1U, 3U + len) != Get32(data + len))
    {
        if (!hello_only) Reply(ST_FRAME, 0, 0U);
        return 0U;
    }
    if (hello_only && cmd != CMD_HELLO) return 0U;

    switch (cmd)
    {
    case CMD_HELLO:
        if (len != 0U) break;
        info[0] = BOOT_VERSION;
        info[1] = Flag_State();
        (void)HwInfo_Read(info + 2U);
        Reply(ST_OK, info, 6U);
        return 1U;

    case CMD_START:
        if (len != 9U) break;
        s_started = 0U;
        s_img_len = Get32(data);
        s_img_crc = Get32(data + 4U);
        if (s_img_len == 0U || (s_img_len & (PAGE_SIZE - 1U)) || s_img_len > APP_SIZE_MAX || data[8] > 1U)
        {
            Reply(ST_RANGE, 0, 0U);
            return 0U;
        }
        FLASH_Unlock_Fast();
        Flag_Write(FLAG_UPGD);                          /* 先作废 APP 再擦，中途断电也会留在 Boot */
        App_Erase(data[8] ? PARAM_END : APP_SIZE_MAX);
        s_started = 1U;
        Reply(ST_OK, 0, 0U);
        return 0U;

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
            FLASH_ErasePage_Fast(FLASH_BASE + off);     /* 先擦再写，同一页重发多少次结果都一样 */
            CH32_IAP_Program(FLASH_BASE + off, s_rx_words + 2U);
            Reply(memcmp(FLASH_PTR(off), data + 4U, PAGE_SIZE) ? ST_VERIFY : ST_OK, data, 4U);
        }
        return 0U;

    case CMD_FINISH:
        if (len != 0U) break;
        if (!s_started)
        {
            Reply(ST_ORDER, 0, 0U);
        }
        else if (Crc32(FLASH_PTR(0U), s_img_len) != s_img_crc)
        {
            Reply(ST_IMAGE_CRC, 0, 0U);
        }
        else if (!Fw_Compatible())
        {
            Reply(ST_MISMATCH, 0, 0U);
        }
        else
        {
            Flag_Write(FLAG_APOK);
            if (Flag_State() != 2U)
            {
                Reply(ST_VERIFY, 0, 0U);
                return 0U;
            }
            Reply(ST_OK, 0, 0U);                        /* 发送完才返回，应答不会被复位截断 */
            Jump_App();
        }
        return 0U;

    default:
        break;
    }
    Reply(ST_FRAME, 0, 0U);                             /* 未知命令或 LEN 不对 */
    return 0U;
}

/* ============================== 主流程 ============================== */

/* 收帧循环。window_ms!=0：上电握手窗口，只认握手，收到返回 1，超时返回 0；
 * window_ms==0：升级循环，不返回 */
static uint8_t Loop(uint16_t window_ms)
{
    uint16_t idle    = 0U; /** 距上一个字节的毫秒数 */
    uint16_t elapsed = 0U; /** 窗口已过去的毫秒数   */
    uint8_t  byte;

    for (;;)
    {
        if (Uart_Poll(&byte))
        {
            idle = 0U;
            if (Frame_Byte(byte) && Frame_Process(window_ms != 0U) && window_ms != 0U) return 1U;
        }
        else if (Tick_Poll())                           /* 又过了 1ms */
        {
            if (idle < 0xFFFFU) idle++;
            if (idle > BYTE_GAP_MS) s_pos = 0U;
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

    Hw_Init();
    if (!app_request && !Loop(HELLO_WINDOW_MS) && App_Bootable())
    {
        Jump_App();
    }
    (void)Loop(0U);
}
