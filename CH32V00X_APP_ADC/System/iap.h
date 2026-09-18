#ifndef __IAP_H
#define __IAP_H

#include <stdint.h>

/*
 * iap.h —— APP 侧 IAP：查询、进入升级、写硬件信息，以及固件信息块
 * 帧格式、命令与 Boot 工程 System/iap.h 及《舵机 IAP 升级协议》文档一致，改动须三处同步。
 */

/* 固件信息块，16 字节，链接脚本固定在 APP 偏移 0x100；上位机从 hex 读，Boot 完成升级时读 */
typedef struct {
    uint8_t magic[4];    /** 'S','V','F','W' */
    uint8_t servo_type;  /** 本固件适用的舵机类型，0xFF=不限 */
    uint8_t hw_version;  /** 适用的硬件版本，0xFF=不限 */
    uint8_t voltage;     /** 适用的电压型号，0xFF=不限 */
    uint8_t torque;      /** 适用的扭力型号，0xFF=不限 */
    uint8_t version[3];  /** 固件版本 {主, 次, 修订} */
    uint8_t reserved[5]; /** 0xFF */
} IAP_FwInfo_t;

void IAP_Init(void);            /* 上电调用一次：硬件信息页出厂空白时，写入本固件适用的硬件 */
void IAP_Rx_Deal(uint8_t data); /* 串口任务逐字节喂入，与舵机 ASCII 协议并行嗅探同一串 */
void IAP_Service(void);         /* 串口任务每拍调用：进入升级的应答发完后复位进 Boot */

#endif
