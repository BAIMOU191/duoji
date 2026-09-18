#pragma once
#include <stdint.h>

/* 主机端测试用的最小桩：只补 iap.h / iap.c 协议部分引用到的类型别名和常量。 */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define FLASH_BASE 0x08000000UL
