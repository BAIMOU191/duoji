#ifndef __IAP_H
#define __IAP_H

#include "ch32v00X.h"

/* 主 Flash 分区：[0, APP_BASE) Boot 5K；[APP_BASE, 0xF400) APP；0xF400 起参数/硬件信息/标志页。
 * 下面的 CONFIG/HWINFO/FLAG 是 Flash 绝对偏移；FWINFO_OFFSET 和升级镜像里的偏移相对 APP_BASE */
#define APP_BASE           0x1400U
#define APP_SIZE_MAX       (0xF400U - APP_BASE)
#define CONFIG_SLOT_A      0xF400U
#define CONFIG_SLOT_B      0xF500U
#define HWINFO_OFFSET      0xF600U
#define APP_DATA_END       0xF700U
#define FLAG_OFFSET        0xF700U
#define FWINFO_OFFSET      0x0100U

#define FLAG_UPGD          0x44475055U
#define FLAG_APOK          0x4B4F5041U
#define HWINFO_MAGIC       0x46495748U
#define FWINFO_MAGIC       0x57465653U

#define IAP_HEAD1          0xAAU
#define IAP_HEAD2          0x55U
#define IAP_TAIL1          0x55U
#define IAP_TAIL2          0xAAU
#define IAP_DATA_MAX       260U

#define CMD_HELLO          0x01U
#define CMD_START          0x02U
#define CMD_WRITE          0x03U
#define CMD_FINISH         0x04U

#define ST_OK              0U
#define ST_FRAME           1U
#define ST_MISMATCH        2U
#define ST_RANGE           3U
#define ST_VERIFY          4U
#define ST_IMAGE_CRC       5U
#define ST_ORDER           6U

void IAP_Run(void);

#endif
