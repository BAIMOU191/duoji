/*
 * main.c —— Bootloader 入口，位于 BOOT 区(0x1FFF0000，3328 字节)
 * 选项字节须设为上电从 BOOT 区启动；流程与协议见 System/iap.c。
 */
#include "iap.h"

int main(void)
{
    IAP_Run();
    while (1) {}
}
