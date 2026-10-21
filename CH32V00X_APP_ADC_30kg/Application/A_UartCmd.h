#ifndef __A_UARTCMD_H__
#define __A_UARTCMD_H__

#include <stdint.h>

/* 串口应用层统一入口：明文帧、AA 66二进制帧和AA 55升级帧按帧头独占分流。 */

void A_Uart_Process(void); /* 串口任务入口：收帧、执行、回复入队、驱动收尾 */
uint8_t A_Uart_DebugReady(void); /* 无半帧且最近10ms无接收，允许诊断尝试发送 */

#endif /* __A_UARTCMD_H__ */
