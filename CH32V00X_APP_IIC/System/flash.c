#include "flash.h"
#include <string.h>

#define CONFIG_COMMITTED 0x5AA55AA5UL

static uint16_t Flash_Get16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)p[1] << 8;
}

static uint32_t Flash_Get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8
         | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void Flash_Put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void Flash_Put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static uint16_t Flash_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t bit;

    while (length--)
    {
        crc ^= (uint16_t)*data++ << 8;
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U)
                ? ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
    }
    return crc;
}

static uint8_t Flash_PageValid(const uint8_t *page)
{
    return Flash_Get32(page + CONFIG_COMMIT_OFFSET) == CONFIG_COMMITTED
        && Flash_Get16(page + CONFIG_CRC_OFFSET) == Flash_Crc16(page, CONFIG_CRC_OFFSET);
}

static uint8_t Flash_PageWrite(uint32_t address, const uint8_t *bytes)
{
    uint32_t page[FLASH_PAGE_SIZE / sizeof(uint32_t)];

    memcpy(page, bytes, FLASH_PAGE_SIZE);
    if (FLASH_ROM_ERASE(address, FLASH_PAGE_SIZE) != FLASH_COMPLETE) return 0U;
    if (FLASH_ROM_WRITE(address, page, FLASH_PAGE_SIZE) != FLASH_COMPLETE) return 0U;
    return (uint8_t)(memcmp((const void *)address, bytes, FLASH_PAGE_SIZE) == 0);
}

uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint8_t version[3])
{
    const uint8_t *page_a = (const uint8_t *)CONFIG_SLOT_A_ADDR;
    const uint8_t *page_b = (const uint8_t *)CONFIG_SLOT_B_ADDR;
    const uint8_t *source;
    uint8_t valid_a = Flash_PageValid(page_a);
    uint8_t valid_b = Flash_PageValid(page_b);

    if (data == 0 || version == 0 || capacity == 0U || capacity > CONFIG_PAYLOAD_MAX) return 0U;
    if (!valid_a && !valid_b) return 0U;

    if (valid_a)
    {
        source = page_a;
        if (!valid_b || memcmp(page_a, page_b, FLASH_PAGE_SIZE) != 0)
            (void)Flash_PageWrite(CONFIG_SLOT_B_ADDR, source);
    }
    else
    {
        source = page_b;
        (void)Flash_PageWrite(CONFIG_SLOT_A_ADDR, source);
    }

    memcpy(version, source, CONFIG_VERSION_SIZE);
    memcpy(data, source + CONFIG_PAYLOAD_OFFSET, capacity);
    return capacity;
}

uint8_t FLASH_Config_Save(const void *data, uint16_t length, const uint8_t version[3])
{
    uint32_t page_words[FLASH_PAGE_SIZE / sizeof(uint32_t)];
    uint8_t *page = (uint8_t *)page_words;

    if (data == 0 || version == 0 || length == 0U || length > CONFIG_PAYLOAD_MAX) return 0U;

    memset(page, 0xFF, FLASH_PAGE_SIZE);
    memcpy(page, version, CONFIG_VERSION_SIZE);
    memcpy(page + CONFIG_PAYLOAD_OFFSET, data, length);
    Flash_Put16(page + CONFIG_CRC_OFFSET, Flash_Crc16(page, CONFIG_CRC_OFFSET));
    Flash_Put32(page + CONFIG_COMMIT_OFFSET, CONFIG_COMMITTED);

    if (memcmp((const void *)CONFIG_SLOT_A_ADDR, page, FLASH_PAGE_SIZE) != 0
        && !Flash_PageWrite(CONFIG_SLOT_A_ADDR, page)) return 0U;
    if (memcmp((const void *)CONFIG_SLOT_B_ADDR, page, FLASH_PAGE_SIZE) != 0
        && !Flash_PageWrite(CONFIG_SLOT_B_ADDR, page)) return 0U;
    return 1U;
}
