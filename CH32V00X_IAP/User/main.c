/*
 * main.c —— Bootloader 入口，位于主 Flash 0x0000~0x13FF(5K)，APP 从 0x1400 起
 * 上电从用户区启动(Boot 运行时自动改选项字节)；流程与协议见 System/iap.c。
 */
#include "iap.h"

int main(void)
{
    IAP_Run();
    while (1) {}
}
