#ifndef __BOOT_KEY_H
#define __BOOT_KEY_H

/*
 * boot_key.h —— 升级文件解密密钥(AES-128，16 个字符)
 * 把 gen_key.py 生成的 boot_key.txt 里那一串原样粘贴到下面引号里(正好 16 个字符)，重新编译 Boot。
 * 打包脚本 fw_pack.py 读的也是那个 txt，两边必须是同一串，否则升级会因整包 CRC 不符被 Boot 拒绝。
 * 换密钥后，烧着旧 Boot 的舵机将无法再升级。
 */
static const uint8_t k_boot_key[16] = "SaPBjhf9nhUUJChf";

#endif
