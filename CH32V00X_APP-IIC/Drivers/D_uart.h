#ifndef __D_UART_H__
#define __D_UART_H__

#include "ch32v00X.h"
#include "C_Ring_Buf.h"

/*
 * D_uart.h —— USART1 单线半双工驱动(PC0，AFIO PartialRemap3)
 * 收发各一条环形队列，中断只搬字节；发送期间关RXNE，TC确认末字节移出后再开回接收。
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
uint8_t D_UART_Tx_Idle(void);                       /* 1=发送队列已空且末字节已移出引脚 */

/* ---- 中断入口 ---- */
void    D_UART1_ISR(void); /* USART1_IRQHandler中直接调用，收发合一 */

#endif /* __D_UART_H__ */
