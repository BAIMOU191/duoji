/*
 * A_Tasks.c —— 应用任务注册表(协作式，耗时操作放最低优先级)
 *   优先级  周期   任务          内容
 *     0     1ms   ServoControl  编码器 -> 观测器 -> 轨迹 -> 控制器 -> H桥
 *     1     1ms   Uart          收帧 -> 执行 -> 回复
 *     2     10ms  Protect       过流/过温/堵转 + 功率限制
 *     3     1s    State         LED心跳 + 喂狗
 *     4     事件  ConfigSave    参数变脏时写Flash
 * 加任务：写好入口函数，在 s_tasks 里加一行 TASK_DEF 即可。
 */

#include "A_Tasks.h"
#include "A_Servo.h"
#include "A_UartCmd.h"
#include "A_Protect.h"
#include "A_Config.h"
#include "C_Task_Scheduler.h"
#include "D_iwdg.h"
#include "D_ws2812.h"
#include "iap.h"


/* ---- 任务入口：只转发给所属模块 ---- */

static void Task_ServoControl(void)
{
    A_Servo_Control(); /* 1ms 控制拍 */
}

static void Task_Uart(void)
{
    A_Uart_Process();  /* 收帧->执行->回复->延迟波特率切换 */
    IAP_Service();     /* 进入升级的应答发完后复位进 Boot */
}

static void Task_Protect(void)
{
    A_Protect_Task(); /* 10ms 基拍，内部再分频 */
}

/* 状态：LED心跳 + 喂狗 */
static void Task_State(void)
{
    static uint8_t led_on;  /** 当前亮灭状态 */
    if (led_on) D_WS2812_Send(255U, 255U, 255U);
    else        D_WS2812_Off();
    led_on = (uint8_t)!led_on;
    D_IWDG_Feed();          /* 喂狗放最低优先级：高优先级任务卡死时能触发复位 */
}

static void Task_ConfigSave(void)
{
    A_Config_SaveTask(); /* 参数变脏时异步写 Flash */
}

/*                   入口函数            周期ms 优先级(0最高) */
static Task_t s_tasks[] = {
    TASK_DEF(Task_ServoControl, 1,    0),
    TASK_DEF(Task_Uart,         1,    1),
    TASK_DEF(Task_Protect,      10,   2),
    TASK_DEF(Task_State,        1000, 3),
    TASK_DEF(Task_ConfigSave,   0,    4), /* 周期0：只在参数变脏时被事件触发 */
};

/* 注册任务表，并把保存任务的就绪标志交给配置模块(事件触发，不占1ms控制拍) */
void A_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
    A_Config_BindSaveEvent(C_Task_EventFlag(Task_ConfigSave));
}
