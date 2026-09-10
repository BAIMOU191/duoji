#ifndef __D_UART_H__
#define __D_UART_H__

#include "ch32v00X.h"
#include "C_Ring_Buf.h"

/*
 * D_uart.h —— USART1 单线半双工驱动 (PC0，AFIO PartialRemap3)
 *
 * 收发各挂一条环形队列，中断只负责搬字节，不认识任何协议：
 *
 *     上位机 --> PC0 --> RXNE中断 --> s_rx_rb --> D_UART1_Rx_Get()  --> 协议层
 *     协议层 --> D_UART1_Tx_Write() --> s_tx_rb --> TXE中断 --> PC0 --> 上位机
 *
 * 半双工只有一根线，发送期间必须关掉RXNE中断，否则会把自己发出的字节
 * 当成收到的数据；末字节靠TC中断确认已完全移出移位寄存器后再开回接收。
 * 这两步都在本文件内闭环，协议层不需要知道方向切换的存在。
 */

/* ---- 生命周期 ---- */
void    D_USART1_Cfg(uint32_t baudrate); /* 配置引脚/波特率/半双工/接收中断 */
void    D_UART_Enable(uint8_t en);       /* 1=接管PC0，0=停收发并释放PC0(共线仲裁用) */
uint8_t D_UART_Is_Enabled(void);         /* 当前串口是否由本驱动占用 */

/* ---- 数据通路 ---- */
uint8_t D_UART1_Rx_Get(uint8_t *data);                       /* 取1个已收字节，0=队列空 */
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len);  /* 整帧入队并启动发送，0=空间不足 */

/* ---- 周期服务 ---- */
void    D_UART_SetBaud_Deferred(uint32_t baudrate); /* 登记新波特率，等回复发完再切 */
void    D_UART_Service(void);                       /* 任务中周期调用，执行延迟切换 */

/* ---- 中断入口 ---- */
void    D_UART1_ISR(void); /* USART1_IRQHandler中直接调用，收发合一 */

#endif /* __D_UART_H__ */
