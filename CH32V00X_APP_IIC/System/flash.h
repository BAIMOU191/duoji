/* flash.h 参数页存取：A/B 两页互为备份，每页 = 版本3B + 参数 + CRC16 + 提交标记；另定义硬件信息页与升级标志页地址 */
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

uint16_t FLASH_Config_Load(void *data, uint16_t capacity, uint8_t version[3]); /* 读有效页(另一页坏则修复)，返回读出字节数，0=两页都无效 */
uint8_t  FLASH_Config_Save(const void *data, uint16_t length, const uint8_t version[3]); /* 先写A再写B，内容一致的页不擦写；1=成功 */

#endif
