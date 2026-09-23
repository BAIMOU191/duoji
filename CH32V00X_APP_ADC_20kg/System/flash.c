#include "flash.h"
#include <string.h>

/* CFG3 把载荷从32扩到64字节；读取端仍识别CFG2，升级时原ID、波特率和标定不会丢。 */
#define CONFIG_MAGIC       0x43464733UL
#define CONFIG_MAGIC_V2    0x43464732UL
#define CONFIG_COMMITTED   0x5AA55AA5UL
#define CONFIG_PAYLOAD_V2  32U

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

typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint16_t reserved0;
    uint16_t length;
    uint8_t  payload[CONFIG_PAYLOAD_V2];
    uint16_t crc;
    uint16_t reserved1;
    uint32_t committed;
} FlashRecordV2_t;

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
static uint8_t Flash_RecordInfo(const void *address, const uint8_t **payload,
                                uint16_t *length, uint32_t *sequence)
{
    const FlashRecord_t *record = (const FlashRecord_t *)address;

    if (record->magic == CONFIG_MAGIC
        && record->committed == CONFIG_COMMITTED
        && record->length != 0U && record->length <= CONFIG_PAYLOAD_MAX
        && record->crc == Flash_Crc16(record->payload, record->length))
    {
        *payload = record->payload;
        *length = record->length;
        *sequence = record->sequence;
        return 1U;
    }
    else
    {
        const FlashRecordV2_t *old = (const FlashRecordV2_t *)address;
        if (old->magic == CONFIG_MAGIC_V2
            && old->committed == CONFIG_COMMITTED
            && old->length != 0U && old->length <= CONFIG_PAYLOAD_V2
            && old->crc == Flash_Crc16(old->payload, old->length))
        {
            *payload = old->payload;
            *length = old->length;
            *sequence = old->sequence;
            return 1U;
        }
    }
    return 0U;
}

uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint32_t *sequence)
{
    const uint8_t *pa = 0, *pb = 0, *latest;
    uint16_t la = 0U, lb = 0U, latest_len;
    uint32_t sa = 0U, sb = 0U;
    uint8_t va = Flash_RecordInfo((const void *)CONFIG_SLOT_A_ADDR, &pa, &la, &sa);
    uint8_t vb = Flash_RecordInfo((const void *)CONFIG_SLOT_B_ADDR, &pb, &lb, &sb);

    if (data == 0 || sequence == 0 || (!va && !vb)) return 0U;
    if (!vb || (va && (int32_t)(sa - sb) > 0))
        latest = pa, latest_len = la, *sequence = sa;
    else
        latest = pb, latest_len = lb, *sequence = sb;
    memcpy(data, latest, (latest_len < capacity) ? latest_len : capacity);
    return latest_len;
}

uint8_t FLASH_Config_Save(const void *data, uint16_t length, uint32_t sequence)
{
    FlashRecord_t record;
    uint32_t page[FLASH_PAGE_SIZE / sizeof(uint32_t)];
    const uint8_t *payload;
    uint16_t old_len;
    uint32_t sa = 0U, sb = 0U;
    uint8_t va = Flash_RecordInfo((const void *)CONFIG_SLOT_A_ADDR, &payload, &old_len, &sa);
    uint8_t vb = Flash_RecordInfo((const void *)CONFIG_SLOT_B_ADDR, &payload, &old_len, &sb);
    uint32_t target;

    if (data == 0 || length == 0U || length > CONFIG_PAYLOAD_MAX) return 0U;
    target = (!va || (vb && (int32_t)(sb - sa) > 0))
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
