/*
 * main.c —— 程序入口。
 * 分层：User -> Application -> Common -> Drivers -> SRC(WCH库)
 */

#include "debug.h"
#include "D_uart.h"
#include "D_spi.h"
#include "D_i2c.h"
#include "D_tim.h"
#include "D_opa.h"
#include "D_adc.h"
#include "D_tmp112.h"
#include "D_ws2812.h"
#include "D_iwdg.h"
#include "C_Task_Scheduler.h"
#include "A_Config.h"
#include "A_Servo.h"
#include "A_Protect.h"
#include "A_Tasks.h"
#include "A_Calib_Min.h"
#include "iap.h"

/* APP_MODE_CALIB：开环标定固件，标定块替换 App_Model.h 对应型号电机标定区后注释掉本行 */
// #define APP_MODE_CALIB
/* 程序入口：分层初始化后进入调度主循环，不返回 */
int main(void)
{
    /* ---- 1. 系统层 ---- */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    RCC_ClearFlag();
    Delay_Ms(100);                     /* 等电源和外部器件上电 */

    /* ---- 2. 参数：驱动依赖已保存配置，必须在驱动前 ---- */
    A_Config_Init();
    IAP_Init();                        /* 硬件信息页空白时写入本固件适用硬件 */

    /* ---- 3. 驱动层：先总线，再外设，最后总线上的器件 ---- */
    D_Bus_I2C_Init();                  /* I2C1 400kHz */
    D_SPI1_Init();                     /* SPI1 6MHz，WS2812 */
    D_USART1_Cfg(A_Config_Baudrate()); /* USART1 已保存波特率，半双工PC0 */
    D_TIM2_PWM_Init();                 /* TIM2：20kHz PWM + ADC触发 */
    D_TIM1_PWM_IC_Init();              /* TIM1：PWM/捕获/1ms时基 */
    D_OPA1_Init();                     /* OPA1 电流检测 8x */
    D_ADC_DMA_Init();                  /* ADC1 规则组DMA */
    D_TMP112_Init();                   /* TMP112 温度 */
    D_WS2812_Init();                   /* WS2812 状态灯 */
    Delay_Ms(200);                     /* 等ADC滤波读数稳定 */

    /* ---- 4. 应用层 ---- */
#ifdef APP_MODE_CALIB
    A_Calib_Min_Init();
    A_Calib_Min_Tasks_Init();
#else
    A_Servo_Init();                    /* 初始化环路并执行上电动作 */
    A_Protect_Init();                  /* 使扭矩上限与硬件保护生效 */
    A_Tasks_Init();                    /* 注册业务任务 */
#endif
    D_IWDG_Init();                     /* 看门狗最后开，最低优先级任务喂 */

    /* ---- 5. 主循环：每轮一个最高优先级就绪任务，不抢占 ---- */
    while (1)
    {
        C_Task_Run();
    }
}
