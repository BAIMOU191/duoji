#ifndef __FLASH_H__
#define __FLASH_H__

#include "ch32v00X.h"

#define FLASH_PAGE_SIZE          256U
#define CONFIG_SLOT_A_ADDR       (FLASH_BASE + 0xF400U)
#define CONFIG_SLOT_B_ADDR       (FLASH_BASE + 0xF500U)
#define HW_INFO_ADDR             (FLASH_BASE + 0xF600U)
#define IAP_FLAG_ADDR            (FLASH_BASE + 0xF700U)

#define CONFIG_VERSION_SIZE      3U
#define CONFIG_PAYLOAD_OFFSET    3U
#define CONFIG_CRC_OFFSET        250U
#define CONFIG_COMMIT_OFFSET     252U
#define CONFIG_PAYLOAD_MAX       (CONFIG_CRC_OFFSET - CONFIG_PAYLOAD_OFFSET)

uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint8_t version[3]);
uint8_t  FLASH_Config_Save(const void *data, uint16_t length, const uint8_t version[3]);

#endif
