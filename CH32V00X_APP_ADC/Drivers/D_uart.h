#ifndef __D_UART_H__
#define __D_UART_H__

#include "ch32v00X.h"

/* D_uart.h USART1单线半双工驱动(PC0，AFIO PartialRemap3)：接收DMA1通道5循环缓冲 + IDLE通知，
 * 发送DMA1通道4整帧发出；发送期间关接收器，TC确认末字节移出引脚后再开回接收。
 */

#define D_UART_TX_FRAME_MAX 128U   /* 单次 D_UART1_Tx_Write 的整帧上限(不含)；调试打印按它分配缓冲 */

/* ---- 生命周期 ---- */
void    D_USART1_Cfg(uint32_t baudrate); /* 配置引脚/波特率/半双工/收发DMA */
void    D_UART_SetRxNotify(volatile uint8_t *flag); /* 总线空闲(一段数据收完)时把 *flag 置1 */
void    D_UART_Enable(uint8_t en);       /* 1=接管PC0，0=停收发并释放PC0(共线仲裁用) */

/* ---- 数据通路 ---- */
uint8_t D_UART1_Rx_Get(uint8_t *data);                       /* 取1个已收字节，0=没有新数据 */
uint8_t D_UART1_Tx_Write(const uint8_t *data, uint8_t len);  /* 整帧入队并启动发送，0=空间不足 */

/* ---- 周期服务 ---- */
void    D_UART_SetBaud_Deferred(uint32_t baudrate); /* 登记新波特率，等回复发完再切 */
void    D_UART_Service(void);                       /* 任务中周期调用，执行延迟切换 */
uint8_t D_UART_Tx_Idle(void);                       /* 1=发送队列已空且末字节已移出引脚 */

/* ---- 中断入口 ---- */
void    D_UART1_ISR(void);       /* USART1_IRQHandler：IDLE通知 + 发送收尾 */
void    D_UART1_TxDma_ISR(void); /* DMA1_Channel4_IRQHandler：整帧搬完，等TC */

#endif /* __D_UART_H__ */
