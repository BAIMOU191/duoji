#ifndef __FLASH_H__
#define __FLASH_H__

#include "ch32v00X.h"

#define FLASH_PAGE_SIZE          256U
#define CONFIG_SLOT_A_ADDR       (FLASH_BASE + 0xF500U)
#define CONFIG_SLOT_B_ADDR       (FLASH_BASE + 0xF600U)
#define CONFIG_PAYLOAD_MAX       32U

uint8_t FLASH_Config_Load(void *data, uint16_t length, uint32_t *sequence);
uint8_t FLASH_Config_Save(const void *data, uint16_t length, uint32_t sequence);

#endif
