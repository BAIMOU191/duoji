#ifndef __A_UARTCMD_H__
#define __A_UARTCMD_H__

#include <stdint.h>

/*
 * A_UartCmd.h —— 总线舵机ASCII协议层
 *
 * 帧格式：'#' + 3位ID + 'P' + 指令体 + '!'，ID=255为广播。
 *
 * 本层只做"取字节 -> 组帧 -> 解码 -> 执行 -> 组回复"。半双工方向切换、
 * 收发队列和中断全部封在 D_uart 内部，所以整层没有一行硬件寄存器操作，
 * 可以直接在PC上编译测试。
 */

void A_Uart_Process(void); /* 串口任务入口：收帧、执行、回复入队、驱动收尾 */

#endif /* __A_UARTCMD_H__ */
