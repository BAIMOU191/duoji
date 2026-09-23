#include "flash.h"
#include <string.h>

/* "CFG2"：记录头只管存取不看内容，版本号由载荷开头自带。
 * 换魔术字后，分区调整前写下的 CFG1 记录一律判无效。 */
#define CONFIG_MAGIC       0x43464732UL
#define CONFIG_COMMITTED   0x5AA55AA5UL

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint16_t reserved0; /* CFG1 在这里存参数格式版本，已废弃 */
    uint16_t length;
    uint8_t  payload[CONFIG_PAYLOAD_MAX];
    uint16_t crc;
    uint16_t reserved1;
    uint32_t committed;
} FlashRecord_t;

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

/* 记录本身完整即有效；长度不必等于当前结构体，新旧长度的取舍交给 A_Config */
static uint8_t Flash_RecordValid(const FlashRecord_t *record)
{
    return record->magic == CONFIG_MAGIC
        && record->committed == CONFIG_COMMITTED
        && record->length != 0U
        && record->length <= CONFIG_PAYLOAD_MAX
        && record->crc == Flash_Crc16(record->payload, record->length);
}

uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint32_t *sequence)
{
    const FlashRecord_t *a = (const FlashRecord_t *)CONFIG_SLOT_A_ADDR;
    const FlashRecord_t *b = (const FlashRecord_t *)CONFIG_SLOT_B_ADDR;
    uint8_t va = Flash_RecordValid(a);
    uint8_t vb = Flash_RecordValid(b);
    const FlashRecord_t *latest;

    if (data == 0 || sequence == 0 || (!va && !vb)) return 0U;
    latest = (!vb || (va && (int32_t)(a->sequence - b->sequence) > 0)) ? a : b;
    memcpy(data, latest->payload, (latest->length < capacity) ? latest->length : capacity);
    *sequence = latest->sequence;
    return latest->length;
}

uint8_t FLASH_Config_Save(const void *data, uint16_t length, uint32_t sequence)
{
    FlashRecord_t record;
    uint32_t page[FLASH_PAGE_SIZE / sizeof(uint32_t)];
    const FlashRecord_t *a = (const FlashRecord_t *)CONFIG_SLOT_A_ADDR;
    const FlashRecord_t *b = (const FlashRecord_t *)CONFIG_SLOT_B_ADDR;
    uint8_t va = Flash_RecordValid(a);
    uint8_t vb = Flash_RecordValid(b);
    uint32_t target;

    if (data == 0 || length == 0U || length > CONFIG_PAYLOAD_MAX) return 0U;
    target = (!va || (vb && (int32_t)(b->sequence - a->sequence) > 0))
           ? CONFIG_SLOT_A_ADDR : CONFIG_SLOT_B_ADDR;

    memset(&record, 0xFF, sizeof(record));
    record.magic = CONFIG_MAGIC;
    record.sequence = sequence;
    record.length = length;
    memcpy(record.payload, data, length);
    record.crc = Flash_Crc16(record.payload, length);
    record.committed = CONFIG_COMMITTED;
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &record, sizeof(record));

    if (FLASH_ROM_ERASE(target, FLASH_PAGE_SIZE) != FLASH_COMPLETE) return 0U;
    if (FLASH_ROM_WRITE(target, page, FLASH_PAGE_SIZE) != FLASH_COMPLETE) return 0U;
    return memcmp((const void *)target, page, FLASH_PAGE_SIZE) == 0;
}
