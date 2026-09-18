/*
 * iap.h —— Bootloader 升级协议定义：Flash 布局、帧格式、命令与状态码
 * 与 APP 工程 System/iap.c 及《舵机 IAP 升级协议》文档保持一致，改动须三处同步。
 */
#ifndef __IAP_H
#define __IAP_H

#include "ch32v00X.h"

#define BOOT_VERSION       1U          /** 握手应答带出；协议有不兼容改动时加1 */

/* ---- Flash 布局：相对 0x08000000 的偏移 ---- */
#define APP_SIZE_MAX       0xF400U     /** APP 区 [0, 0xF400) */
#define PARAM_END          0xF600U     /** 擦除模式1 连参数页 F400/F500 一起擦到这里 */
#define HWINFO_OFFSET      0xF600U     /** 硬件信息页：量产写入，Boot 只读 */
#define FLAG_OFFSET        0xF700U     /** 升级标志页：只有 Boot 写 */
#define FWINFO_OFFSET      0x0100U     /** APP 镜像内的固件信息块 */

/* ---- 魔术字：按小端读出的 uint32，Flash 里的字节序列就是这4个ASCII字符 ---- */
#define FLAG_APOK          0x4B4F5041U /** "APOK" APP 已通过整包校验 */
#define FLAG_UPGD          0x44475055U /** "UPGD" 升级进行中，APP 不可用 */
#define HWINFO_MAGIC       0x46495748U /** "HWIF" 硬件信息页 */
#define FWINFO_MAGIC       0x57465653U /** "SVFW" 固件信息块 */

/* ---- 帧：AA 55 | CMD(1) | LEN(2,小端) | DATA(LEN) | CRC32(4,小端) | 55 AA ---- */
#define IAP_HEAD1          0xAAU
#define IAP_HEAD2          0x55U
#define IAP_TAIL1          0x55U
#define IAP_TAIL2          0xAAU
#define IAP_DATA_MAX       260U        /** 写页帧：偏移4 + 数据256 */

/* ---- 命令，应答的 CMD = 命令 | 0x80 ---- */
#define CMD_HELLO          0x01U       /** 握手：留在 Boot，回 Boot 版本、标志页状态、硬件信息 */
#define CMD_START          0x02U       /** 开始：长度、CRC32、擦除模式 -> 写 UPGD、擦除 */
#define CMD_WRITE          0x03U       /** 写页：偏移 + 256 字节 -> 擦页、写入、读回 */
#define CMD_FINISH         0x04U       /** 完成：整包 CRC32 + 兼容检查 -> 写 APOK、进 APP */

/* ---- 状态码：应答 DATA 第一个字节 ---- */
#define ST_OK              0U
#define ST_FRAME           1U          /** 帧无效：CRC 错、长度不对、未知命令 */
#define ST_MISMATCH        2U          /** 硬件信息或固件类型不符 */
#define ST_RANGE           3U          /** 地址或长度越界 */
#define ST_VERIFY          4U          /** 写入后读回不一致 */
#define ST_IMAGE_CRC       5U          /** 整包 CRC32 不符 */
#define ST_ORDER           6U          /** 顺序错误：没有开始就写页或完成 */

void IAP_Run(void); /* Boot 主流程：判断复位原因，进 APP 或留在升级循环，不返回 */

#endif
