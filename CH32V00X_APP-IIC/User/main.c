/*
 * main.c —— 程序入口。分层依赖自上而下：
 *   User(main/中断转发) -> Application(A_*) -> Common(C_*，纯算法可PC测试) -> Drivers(D_*) -> SRC(WCH外设库)
 */

#include "debug.h"
#include "D_uart.h"
#include "D_spi.h"
#include "D_i2c.h"
#include "D_tim.h"
#include "D_opa.h"
#include "D_adc.h"
#include "D_mt6701.h"
#include "D_motor.h"
#include "D_tmp112.h"
#include "D_ws2812.h"
#include "D_iwdg.h"
#include "C_Task_Scheduler.h"
#include "A_Config.h"
#include "A_Servo.h"
#include "A_Sensor.h"
#include "A_Protect.h"
#include "A_Tasks.h"
#include "A_Calib_Min.h"
#include "iap.h"

/* 定义 APP_MODE_CALIB 编译为开环硬件标定固件(见 A_Calib_Min.h)，标定完填回 A_Parameter.h 后注释掉 */
/* #define APP_MODE_CALIB */

/* 程序入口：分层初始化后进入协作式调度主循环，不返回 */
int main(void)
{
    /* ---- 1. 系统层 ---- */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    RCC_ClearFlag();
    Delay_Ms(100);                     /* 等电源和外部器件上电稳定        */

    /* ---- 2. 参数：驱动要按已保存的配置初始化，所以必须排在驱动前面 ---- */
    A_Config_Init();
    IAP_Init();                        /* 硬件信息页出厂空白时写入本固件适用的硬件 */

    /* ---- 3. 驱动层：先总线，再外设，最后挂在总线上的器件 ---- */
    D_Bus_I2C_Init();                  /* I2C1  400kHz，MT6701+TMP112共用  */
    D_SPI1_Init();                     /* SPI1  6MHz，WS2812数据线         */
    D_USART1_Cfg(A_Config_Baudrate()); /* USART1 已保存的波特率，半双工PC0 */
    D_TIM2_PWM_Init();                 /* TIM2：20kHz PWM + ADC采样触发    */
    D_TIM1_PWM_IC_Init();              /* TIM1：PWM/捕获/1ms时基 + 同步TIM2 */
    D_OPA1_Init();                     /* OPA1：电流检测运放，8x + VB      */
    D_ADC_DMA_Init();                  /* ADC1：规则组DMA + 电压注入 + 零点标定 */
    D_MT6701_Init();                   /* MT6701：磁编码器，仅ENCODER_MODE=1时使用 */
    D_TMP112_Init();                   /* TMP112：数字温度传感器            */
    D_WS2812_Init();                   /* WS2812：状态指示灯               */
    Delay_Ms(200);                     /* 等编码器磁场读数稳定             */

    /* ---- 4. 应用层 ---- */
#ifdef APP_MODE_CALIB
    A_Calib_Min_Init();                /* 唯一的自动标定状态机             */
    A_Calib_Min_Tasks_Init();          /* 只注册标定拍与喂狗任务           */
#else
    A_Servo_Init();                    /* 选输入源、初始化环路、执行上电动作 */
    A_Protect_Init();                  /* 让保存的扭矩上限生效，须排在Servo之后 */
    A_Tasks_Init();                    /* 注册正常业务任务                 */
#endif
    D_IWDG_Init();                     /* 看门狗最后开，由最低优先级任务喂  */

    /* ---- 5. 主循环：每轮只跑一个最高优先级的就绪任务，协作式不抢占 ---- */
    while (1)
    {
        C_Task_Run();
    }
}
