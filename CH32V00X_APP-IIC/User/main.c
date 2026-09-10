/*
 * main.c —— 程序入口
 *
 * 分层结构(依赖只允许自上而下，同层之间不互相调用)：
 *
 *     User        main / 中断向量转发
 *        |
 *     Application A_*  业务编排：舵机状态机、协议、参数、任务表
 *        |
 *     Common      C_*  纯算法：轨迹规划/观测器/控制器/环形队列/调度器
 *        |                (零硬件依赖，可以直接在PC上编译测试)
 *     Drivers     D_*  外设驱动：只认寄存器，不认业务
 *        |
 *     SRC         WCH标准外设库
 *
 * ================== 初始化顺序里有三处硬约束，改动前务必看清 ==================
 *
 *   1. A_Config_Init 必须最先跑。波特率、工作模式、中位偏移都存在Flash里，
 *      后面的驱动要按加载结果配置，顺序反了就会用默认值初始化一遍再改。
 *
 *   2. D_ADC_DMA_Init 必须在第一次 D_Motor_Set 之前。它内部要在"两路输出
 *      恒低、绕组确实无电流"的窗口里标定运放零点，这是全程唯一能保证真零
 *      电流的时刻；一旦电机动过，测到的就不是零点了。
 *
 *   3. A_Servo_Init 必须在 A_Tasks_Init 之前。前者会嗅探信号线判断输入源
 *      (PWM还是串口)，期间要独占引脚并阻塞60ms；此时调度器还没启动，不会
 *      有任务在背后动电机或读串口。
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

/* 定义 APP_MODE_CALIB 后编译出的固件不跑舵机闭环，改为开环硬件标定程序：
 * 上电约1秒自动开始；量产快标约10秒，金样全标约17秒。串口打印可粘贴参数。
 * 标定期间不注册舵机控制与串口命令任务，避免任何闭环动作干扰开环激励。
 * 标定完成、参数填回 A_Parameter.h 后注释掉本行重新编译即可恢复正常固件。 */
// #define APP_MODE_CALIB

/*
 * @fn      main
 * @brief   程序入口：分层初始化后进入协作式调度主循环
 * @param   无
 * @return  不返回
 */
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

    /* ---- 3. 驱动层：先总线，再外设，最后挂在总线上的器件 ---- */
    D_Bus_I2C_Init();                  /* I2C1  400kHz，MT6701+TMP112共用  */
    D_SPI1_Init();                     /* SPI1  6MHz，WS2812数据线         */
    D_USART1_Cfg(A_Config_Baudrate()); /* USART1 已保存的波特率，半双工PC0 */
    D_TIM2_PWM_Init();                 /* TIM2：20kHz PWM + ADC采样触发    */
    D_TIM1_PWM_IC_Init();              /* TIM1：PWM/捕获/1ms时基 + 同步TIM2 */
    D_OPA1_Init();                     /* OPA1：电流检测运放，8x + VB      */
    D_ADC_DMA_Init();                  /* ADC1：电流DMA + 电压注入 + 零点标定 */
    D_MT6701_Init();                   /* MT6701：磁编码器                 */
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
