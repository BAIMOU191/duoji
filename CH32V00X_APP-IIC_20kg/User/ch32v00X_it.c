/* ch32v00X_it.c —— 中断服务，仅转发，业务在驱动/应用层 */

#include <ch32v00X_it.h>
#include "D_tim.h"
#include "D_adc.h"
#include "D_uart.h"
#include "C_Task_Scheduler.h"

void NMI_Handler(void)              __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)        __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)        __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void)       __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)       __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* NMI：复位恢复，避免死循环 */
void NMI_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

/* 硬件错误：复位恢复 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

/* USART1：驱动收发队列，解析在任务里做 */
void USART1_IRQHandler(void)
{
    D_UART1_ISR();
}

/* TIM1捕获：双沿测PWM脉宽 */
void TIM1_CC_IRQHandler(void)
{
    D_TIM1_CC_ISR();
}

/* 1ms时基主由ADC块产生(每20个PWM周期)，与控制拍对齐；
 * TIM1更新中断兜底：ADC停了也22个周期后出时基，系统不停 */
#define TICK_FALLBACK_PERIODS 22U
static volatile uint8_t s_tick_periods;   /* 距上次时基的PWM周期数 */

static void Tick(void)
{
    s_tick_periods = 0U;
    C_Task_Tick_Update();
}

/* TIM1更新：溢出计数 + 时基兜底 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        D_TIM1_UP_ISR();
        if (++s_tick_periods >= TICK_FALLBACK_PERIODS) Tick();
    }
}

/* ADC DMA：每块(1ms)即一个控制拍 */
void DMA1_Channel1_IRQHandler(void)
{
    if (D_ADC_DMA_ISR()) Tick();
}
