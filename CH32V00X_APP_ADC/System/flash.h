#ifndef __FLASH_H__
#define __FLASH_H__

#include "ch32v00X.h"

/*
 * 62K Flash 末尾 4 页保留，APP 链接长度止于 0xF400(见 Link.ld)：
 *   0x0800F400  参数页 A
 *   0x0800F500  参数页 B
 *   0x0800F600  硬件信息页，量产写一次，升级不擦
 *   0x0800F700  IAP 标志页，只由 Boot 写(iap.h 的 CalAddr 在这一页)
 */
#define FLASH_PAGE_SIZE          256U
#define CONFIG_SLOT_A_ADDR       (FLASH_BASE + 0xF400U)
#define CONFIG_SLOT_B_ADDR       (FLASH_BASE + 0xF500U)
#define HW_INFO_ADDR             (FLASH_BASE + 0xF600U)
#define CONFIG_PAYLOAD_MAX       32U

/* 读两页中较新的有效记录，最多拷 capacity 字节；返回记录实际长度，0=两页都无效 */
uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint32_t *sequence);
uint8_t  FLASH_Config_Save(const void *data, uint16_t length, uint32_t sequence);

#endif
