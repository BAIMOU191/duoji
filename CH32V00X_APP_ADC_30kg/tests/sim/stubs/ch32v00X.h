#pragma once
#include <stdint.h>

/* 主机端仿真用的最小桩：只补 WCH 头文件里被引用到的那几个短类型别名和常量。 */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define FLASH_BASE 0x08000000UL
