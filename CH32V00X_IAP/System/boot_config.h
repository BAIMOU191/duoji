#ifndef __BOOT_CONFIG_H
#define __BOOT_CONFIG_H

/*
 * boot_config.h —— Boot 编译开关
 *
 * BOOT_RDP_ENABLE：1 = 量产版，Boot 每次上电检查选项字节，读保护没开就打开(断电再上电后生效)；
 *                  0 = 开发版，不碰读保护。
 *   开发板不要烧 1 的版本：芯片一旦上锁，WCH-Link 就下载不了 APP，只能用 WCH-LinkUtility 解除读保护(整片擦除)。
 *   量产版请同时使用正式密钥(boot_key.h)，不要出现"正式密钥 + 未开读保护"的 Boot。
 * 不论开关为何，Boot 都会把"上电启动位置"改成用户区(Boot 在主 Flash 0 地址)；已开读保护时不再改写选项字节。
 */
#define BOOT_RDP_ENABLE    1

#endif
