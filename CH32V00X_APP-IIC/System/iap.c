/*
 * iap.c —— APP 侧 IAP：查询(0x20)、进入升级(0x21)、写硬件信息(0x22)，以及固件信息块
 * 帧：AA 55 | CMD | LEN(2,小端) | DATA | CRC32(4,小端) | 55 AA，应答 CMD = 命令 | 0x80，DATA[0] = 状态码。
 * 总线上挂着多个舵机：坏帧、别人 ID 的帧、APP 不认识的帧一律不应答。
 * 主机端测试可替换 IAP_HWINFO_PAGE，让硬件信息页落在内存数组里。
 */
#include "iap.h"
#include "A_Config.h"
#include "A_Servo.h"
#include "D_uart.h"
#include "flash.h"
#include <string.h>

#define IAP_HEAD1        0xAAU
#define IAP_HEAD2        0x55U
#define IAP_TAIL1        0x55U
#define IAP_TAIL2        0xAAU
#define IAP_DATA_MAX     8U           /** APP 命令 DATA 最长 8 字节；更长的帧(如 Boot 的写页)不缓存 */

#define IAP_CMD_QUERY    0x20U        /** 查询：ID -> ID、硬件信息4项、固件版本3项 */
#define IAP_CMD_ENTER    0x21U        /** 进入升级：ID + 硬件信息4项 + 固件版本3项，全部一致才复位进 Boot */
#define IAP_CMD_HWINFO   0x22U        /** 写硬件信息：ID + 4项，量产用，已写过不允许覆盖 */

#define IAP_ST_OK        0U
#define IAP_ST_MISMATCH  2U           /** 硬件信息不符 */
#define IAP_ST_VERIFY    4U           /** 写入后读回不一致 */
#define IAP_ST_EXISTS    7U           /** 硬件信息已写入 */

#define HWINFO_MAGIC     0x46495748UL /** "HWIF" 按小端读出 */
#define HWINFO_FORMAT    1U           /** 硬件信息页格式版本 */

#ifndef IAP_HWINFO_PAGE
#define IAP_HWINFO_PAGE  ((const uint8_t *)HW_INFO_ADDR)
#endif

const IAP_FwInfo_t IAP_FW_INFO __attribute__((section(".fw_info"), used)) = {
    { 'S', 'V', 'F', 'W' },
    FW_SERVO_TYPE, FW_HW_VERSION, FW_VOLTAGE, FW_TORQUE,
    { SERVO_VERSION_MAJOR, SERVO_VERSION_MINOR, SERVO_VERSION_PATCH },
    { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU }
};

static uint8_t s_rx[3U + IAP_DATA_MAX + 4U + 2U]; /** CMD LEN(2) DATA CRC32(4) 55 AA，不含帧头 */
static uint8_t s_pos;                             /** 当前帧已收字节数(含帧头)，0=等帧头 */
static uint8_t s_reset_pending;                   /** 1=已应答进入升级，发完就复位进 Boot */

static uint32_t Iap_Get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void Iap_Put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

/* 标准 CRC-32(与 Python zlib.crc32 一致)，逐位计算不占表 */
static uint32_t Iap_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint8_t  bit;

    while (len--)
    {
        crc ^= *data++;
        for (bit = 0U; bit < 8U; bit++)
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
    return ~crc;
}

/* 应答帧：CMD 取当前帧并置最高位，DATA = 状态码 + data，交给串口发送队列 */
static void Iap_Reply(uint8_t status, const uint8_t *data, uint8_t n)
{
    uint8_t frame[12U + 8U]; /** 最长应答：查询带 8 字节 */
    uint8_t i;

    frame[0] = IAP_HEAD1;
    frame[1] = IAP_HEAD2;
    frame[2] = s_rx[0] | 0x80U;
    frame[3] = (uint8_t)(n + 1U);
    frame[4] = 0U;
    frame[5] = status;
    for (i = 0U; i < n; i++) frame[6U + i] = data[i];
    Iap_Put32(frame + 6U + n, Iap_Crc32(frame + 2U, 4U + n));
    frame[10U + n] = IAP_TAIL1;
    frame[11U + n] = IAP_TAIL2;
    (void)D_UART1_Tx_Write(frame, (uint8_t)(12U + n));
}

/* 读硬件信息页的 4 项(舵机类型、硬件版本、电压型号、扭力型号)；页无效时填 0xFF 并返回 0 */
static uint8_t Iap_HwInfoRead(uint8_t *info)
{
    const uint8_t *page  = IAP_HWINFO_PAGE;
    uint8_t        valid = (uint8_t)(Iap_Get32(page) == HWINFO_MAGIC
                                  && Iap_Crc32(page, 28U) == Iap_Get32(page + 28U));
    uint8_t        i;

    for (i = 0U; i < 4U; i++) info[i] = valid ? page[5U + i] : 0xFFU;
    return valid;
}

/* 4 项都不是 0xFF 才算确定的硬件身份；带 0xFF(不限)的固件说明不了舵机是哪一种 */
static uint8_t Iap_HwInfoKnown(const uint8_t *info)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        if (info[i] == 0xFFU) return 0U;
    }
    return 1U;
}

/* 整页写硬件信息：魔术字、格式版本、4 项、预留 0xFF、CRC32，写完读回确认 */
static uint8_t Iap_HwInfoWrite(const uint8_t *info)
{
    uint32_t page[FLASH_PAGE_SIZE / sizeof(uint32_t)];
    uint8_t *bytes = (uint8_t *)page;
    uint8_t  i;

    memset(page, 0xFF, sizeof(page));
    Iap_Put32(bytes, HWINFO_MAGIC);
    bytes[4] = HWINFO_FORMAT;
    for (i = 0U; i < 4U; i++) bytes[5U + i] = info[i];
    Iap_Put32(bytes + 28U, Iap_Crc32(bytes, 28U));

    if (FLASH_ROM_ERASE(HW_INFO_ADDR, FLASH_PAGE_SIZE) != FLASH_COMPLETE) return 0U;
    if (FLASH_ROM_WRITE(HW_INFO_ADDR, page, FLASH_PAGE_SIZE) != FLASH_COMPLETE) return 0U;
    return (uint8_t)(memcmp(IAP_HWINFO_PAGE, bytes, FLASH_PAGE_SIZE) == 0);
}

/* 处理收齐的一帧 */
static void Iap_Process(uint16_t len)
{
    const uint8_t *data = s_rx + 3U;
    uint8_t        cmd  = s_rx[0];
    uint8_t        info[8]; /** 本机身份：ID、硬件信息4项、固件版本3项。查询照此应答，进入升级照此比对 */
    uint8_t        valid;

    if (s_rx[7U + len] != IAP_TAIL1 || s_rx[8U + len] != IAP_TAIL2) return;
    if (Iap_Crc32(s_rx, 3U + len) != Iap_Get32(data + len)) return;
    if (len == 0U) return;
    /* 查询允许广播 255(单独接线时找ID)，其余命令必须点名本机 */
    if (data[0] != g_config.servo_id && !(cmd == IAP_CMD_QUERY && data[0] == 255U)) return;

    valid = Iap_HwInfoRead(info + 1U);
    info[0] = g_config.servo_id;
    info[5] = SERVO_VERSION_MAJOR;
    info[6] = SERVO_VERSION_MINOR;
    info[7] = SERVO_VERSION_PATCH;

    switch (cmd)
    {
    case IAP_CMD_QUERY:
        if (len != 1U) return;
        Iap_Reply(IAP_ST_OK, info, 8U);
        break;

    case IAP_CMD_ENTER:
        if (len != 8U) return;
        /* 硬件信息4项和固件版本3项必须与查询回去的一模一样：确认上位机查的就是本机当前这套固件 */
        if (memcmp(data + 1U, info + 1U, 7U) != 0)
        {
            Iap_Reply(IAP_ST_MISMATCH, 0, 0U);
            break;
        }
        A_Servo_Release(0U); /* Boot 期间电机驱动引脚不受控，先卸力 */
        Iap_Reply(IAP_ST_OK, 0, 0U);
        s_reset_pending = 1U;
        break;

    case IAP_CMD_HWINFO:
        if (len != 5U) return;
        if (valid)
        {
            Iap_Reply(IAP_ST_EXISTS, 0, 0U);
            break;
        }
        Iap_Reply(Iap_HwInfoWrite(data + 1U) ? IAP_ST_OK : IAP_ST_VERIFY, 0, 0U);
        break;

    default:
        break;
    }
}

void IAP_Rx_Deal(uint8_t data)
{
    uint16_t len; /* DATA 长度 */

    if (s_pos == 0U)
    {
        if (data == IAP_HEAD1) s_pos = 1U;
        return;
    }
    if (s_pos == 1U)
    {
        s_pos = (data == IAP_HEAD2) ? 2U : (uint8_t)(data == IAP_HEAD1); /* AA AA 55 也能对上帧头 */
        return;
    }

    s_rx[s_pos - 2U] = data;
    s_pos++;
    if (s_pos < 5U) return;                             /* CMD、LEN 还没收齐 */
    len = (uint16_t)(s_rx[1] | (uint16_t)s_rx[2] << 8);
    if (len > IAP_DATA_MAX)
    {
        s_pos = 0U;
        return;
    }
    if (s_pos < 11U + len) return;
    s_pos = 0U;
    Iap_Process(len);
}

/* 硬件信息页出厂空白时，把 fw_hw 写进去 */
static void Iap_Provision(const uint8_t *fw_hw)
{
    uint8_t info[4];

    if (!Iap_HwInfoKnown(fw_hw)) return; /* 本固件不限型号，写不出确定的硬件身份 */
    if (Iap_HwInfoRead(info)) return;    /* 已经写过，写一次就固定，要改只能 WCH-Link 擦除 */
    (void)Iap_HwInfoWrite(fw_hw);        /* 写失败下次上电再试，不影响舵机运行 */
}

void IAP_Init(void)
{
    static const uint8_t fw_hw[4] = { FW_SERVO_TYPE, FW_HW_VERSION, FW_VOLTAGE, FW_TORQUE };

    Iap_Provision(fw_hw);
}

void IAP_Service(void)
{
    if (s_reset_pending && D_UART_Tx_Idle())
    {
        RCC_ClearFlag();                                /* Boot 靠"只有软件复位标志"认出这是升级请求 */
        SystemReset_StartMode(Start_Mode_BOOT);
        NVIC_SystemReset();
    }
}
