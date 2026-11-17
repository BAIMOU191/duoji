#ifndef __IAP_H
#define __IAP_H

#include <stdint.h>

/*
 * iap.h —— APP 侧 IAP：查询、进入升级、固件信息块与硬件信息自动建立
 * 帧格式与命令由 APP、Boot 和《舵机IAP升级协议与流程》共同定义。
 */

/* 固件信息块，16 字节，链接脚本固定在 APP 偏移 0x100(绝对地址 0x1500)；打包脚本从 hex 读，Boot 完成升级时读 */
typedef struct {
    uint8_t magic[4];    /** 'S','V','F','W' */
    uint8_t servo_type;  /** 本固件适用的舵机类型，0xFF=不限 */
    uint8_t hw_version;  /** 适用的硬件版本，0xFF=不限 */
    uint8_t voltage;     /** 适用的电压型号，0xFF=不限 */
    uint8_t torque;      /** 适用的扭力型号，0xFF=不限 */
    uint8_t version[3];  /** 固件版本 {主, 次, 修订} */
    uint8_t reserved[5]; /** 0xFF */
} IAP_FwInfo_t;

/* 硬件信息页中的型号身份与开环标定能力。能力项始终等于当前固件的标定常量：上电发现不一致就重写整页 */
typedef struct {
    uint8_t  format;
    uint8_t  servo_type;
    uint8_t  hw_version;
    uint8_t  voltage;
    uint8_t  torque;
    uint16_t max_speed_dps;
    uint32_t max_accel_dps2;
    uint32_t max_decel_dps2;
    uint16_t smooth_ms;          /* 自动平滑时间 τ = ka/kv，ms */
    uint16_t min_step_cdeg;      /* 最小可靠位移，厘度 */
} IAP_HwInfo_t;

void IAP_Init(void);            /* 上电调用一次：硬件信息页与本固件不一致(空白、旧格式、换过标定)时重写 */
uint8_t IAP_HwInfoRead(IAP_HwInfo_t *info); /* 读并校验硬件信息页，1=有效 */
uint8_t IAP_PreservedIdentityRead(uint8_t *id, uint8_t *baud_code);
#define IAP_BOOT_REQ_ADDR  0x20000FF0U   /** 升级请求标志地址，必须与 Boot 的 BOOT_REQ_ADDR 一致 */
#define IAP_BOOT_REQ_MAGIC 0x51455242U   /** 必须与 Boot 的 BOOT_REQ_MAGIC 一致 */
#define IAP_FRAME_HEAD1 0xAAU
#define IAP_FRAME_HEAD2 0x55U

uint8_t IAP_Rx_Deal(uint8_t data); /* 喂入1字节，返回1=IAP半帧仍在接收 */
void    IAP_Rx_Reset(void);        /* 串口半帧超时时放弃，防止残帧污染下一条协议 */
/* 初始化时注册专用停机动作；NULL 使用正常舵机卸力。仅有效的进入升级请求会调用。 */
void IAP_SetStopHandler(void (*handler)(void));
void IAP_Service(void);         /* 串口任务每拍调用：进入升级的应答发完后复位进 Boot */
uint8_t IAP_UpgradePending(void); /* ENTER accepted: discard subsequent control commands until reset */

#endif
