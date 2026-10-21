/* A_Tasks.c —— 应用任务注册表：只决定"谁、多久、多优先"，业务逻辑全部在各自模块。
 *   优先级  周期   任务          内容
 *     0     1ms   ServoControl  实测位置 -> 观测器/轨迹 -> 控制器 -> H桥      (A_Servo)
 *     1     1ms   Uart          收帧、执行、回复、升级服务                 (A_UartCmd, iap)
 *     2     10ms  Protect       电流/电压/温度/堵转保护与功率限制          (A_Protect)
 *     3     20ms  State         LED状态/故障指示与喂狗                     (A_StatusLed)
 *     4     事件  ConfigSave    参数变脏时写Flash                          (A_Config)
 *     5     20ms  Print         FireWater运行诊断(非阻塞串口入队)          (A_Debug，默认关)
 */
#include "A_Tasks.h"
#include "A_Servo.h"
#include "A_UartCmd.h"
#include "A_Protect.h"
#include "A_StatusLed.h"
#include "A_Debug.h"
#include "A_Config.h"
#include "C_Task_Scheduler.h"
#include "D_iwdg.h"
#include "iap.h"

static void Task_ServoControl(void)
{
    A_Servo_Control(); /* 1ms控制拍 */
}

static void Task_Uart(void)
{
    A_Uart_Process();
    IAP_Service();
}

static void Task_Protect(void)
{
    A_Protect_Task(); /* 10ms基拍，内部再分频 */
}

static void Task_State(void)
{
    A_StatusLed_Task();
    D_IWDG_Feed(); /* 主循环卡住时让看门狗复位。 */
}

static void Task_ConfigSave(void)
{
    (void)A_Config_SaveTask(); /* 参数变脏时异步写Flash */
}

#if TASK_FIREWATER_ENABLE
static void Task_Print(void)
{
    A_Debug_Task();
}
#endif

/*                           入口函数        周期ms                  优先级(0最高) */
static Task_t s_tasks[] = {
    TASK_DEF(Task_ServoControl, 1,                        0),
    TASK_DEF(Task_Uart,         1,                        1),
    TASK_DEF(Task_Protect,      10,                       2),
    TASK_DEF(Task_State,        STATUS_LED_PERIOD_MS,     3),
    TASK_DEF(Task_ConfigSave,   0,                        4), /* 参数变脏时触发。 */
#if TASK_FIREWATER_ENABLE
    TASK_DEF(Task_Print,        TASK_FIREWATER_PERIOD_MS, 5),
#endif
};

void A_Tasks_Init(void)
{
    C_TASK_INIT(s_tasks);
    A_Config_BindSaveEvent(C_Task_EventFlag(Task_ConfigSave));
}
