/* D_iwdg.c 独立看门狗驱动：LSI时钟使能、初始化启动与喂狗 */

#include "D_iwdg.h"

/*
 * @fn      D_IWDG_Init
 * @brief   使能LSI时钟并初始化启动独立看门狗，LSI就绪超时则放弃
 * @param   无
 * @return  无
 */
void D_IWDG_Init(void)
{
    uint32_t timeout = 100000U;

    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET && --timeout);
    if (timeout == 0) return;

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(IWDG_RELOAD_VALUE);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/*
 * @fn      D_IWDG_Feed
 * @brief   重装看门狗计数器，由应用层最低优先级任务周期调用
 * @param   无
 * @return  无
 */
void D_IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}
