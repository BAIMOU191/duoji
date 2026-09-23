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

/* TIM1更新中断：捕获溢出计数，并将20kHz更新事件20分频为1ms调度时基。 */
void TIM1_UP_IRQHandler(void)
{
    static uint8_t tick_cnt = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        D_TIM1_UP_ISR();
        if (++tick_cnt >= 20U)
        {
            tick_cnt = 0;
            C_Task_Tick_Update();
        }
    }
}

/* ADC规则组电流数据：DMA每填满一个控制周期的20点缓冲区更新均值。 */
void DMA1_Channel1_IRQHandler(void)
{
    D_ADC_DMA_ISR();
}
