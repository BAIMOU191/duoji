#ifndef __A_UARTCMD_H__
#define __A_UARTCMD_H__

#include <stdint.h>

/* A_UartCmd.h —— 总线舵机ASCII协议层，帧格式 '#' + 3位ID + 'P' + 指令体 + '!'，ID=255为广播 */

void A_Uart_Process(void); /* 串口任务入口：收帧、执行、回复入队、驱动收尾 */

#endif /* __A_UARTCMD_H__ */
