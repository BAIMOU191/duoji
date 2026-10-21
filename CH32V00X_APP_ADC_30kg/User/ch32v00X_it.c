/* ch32v00X_it.c — 中断服务程序，仅做转发，业务逻辑在各驱动/应用层实现 */

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

/* 不可屏蔽中断，复位恢复，避免永久停在死循环。 */
void NMI_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

/* 硬件错误，复位恢复 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

/* USART1收发中断：驱动内部收发队列，协议解析全部放在任务里做 */
void USART1_IRQHandler(void)
{
    D_UART1_ISR();
}

/* TIM1捕获中断：双沿测量PWM脉宽 */
void TIM1_CC_IRQHandler(void)
{
    D_TIM1_CC_ISR();
}

/* 1ms调度时基由ADC块完成产生(每20个PWM周期一次)，控制拍与电流/电位器数据块对齐。
 * TIM1更新中断数PWM周期作兜底：ADC块万一停了(DMA出错)，22个周期后照常出时基，系统不会停住 */
#define TICK_FALLBACK_PERIODS 22U
static volatile uint8_t s_tick_periods;   /* 距上次时基的PWM周期数 */

static void Tick(void)
{
    s_tick_periods = 0U;
    C_Task_Tick_Update();
}

/* TIM1更新中断：捕获溢出计数 + 时基兜底 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        D_TIM1_UP_ISR();
        if (++s_tick_periods >= TICK_FALLBACK_PERIODS) Tick();
    }
}

/* ADC DMA：每发布一块(1ms)就是一个控制拍 */
void DMA1_Channel1_IRQHandler(void)
{
    if (D_ADC_DMA_ISR()) Tick();
}
