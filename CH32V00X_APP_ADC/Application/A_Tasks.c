/*
 * A_Tasks.c 应用任务注册表 —— 整个工程唯一决定"谁多久跑一次"的地方。
 *
 * 任务入口只调用所属模块的API，不写业务逻辑：想知道某个功能怎么实现，
 * 从这张表找到模块名，再去那个模块看，不需要在多个文件之间来回跳。
 *
 *   优先级  周期    任务              做什么
 *     0     1ms    ServoControl      读编码器 -> 观测器 -> 轨迹 -> 控制器 -> H桥
 *     1     1ms    Uart              排空接收队列 -> 执行指令 -> 回复入队
 *     2     10ms   Protect           过流/过温检测 + 功率限制回路
 *     3     30ms   State             LED心跳 + 遥测输出 + 喂狗
 *     4     事件   ConfigSave        参数变脏时写Flash(耗时长，放最低优先级)
 *
 * 协作式调度：任何一个任务超时都会挤掉后面的任务，所以耗时操作(Flash写入)
 * 必须放在最低优先级，且不能放在1ms任务里。
 */

#include "A_Tasks.h"
#include "A_Servo.h"
#include "A_UartCmd.h"
#include "A_Protect.h"
#include "A_Config.h"
#include "C_Task_Scheduler.h"
#include "D_iwdg.h"
#include "D_ws2812.h"

#define LED_BLINK_DIV 10U /** 状态任务分频数：30ms x 10 = 300ms 亮灭一次 */

/* LED心跳：按 LED_BLINK_DIV 分频亮灭 */
static void App_LedHeartbeat(void)
{
    static uint8_t led_on;  /** 当前亮灭状态 */
    static uint8_t led_div; /** 30ms分频计数 */

    if (++led_div < LED_BLINK_DIV) return;
    led_div = 0U;

    if (led_on) D_WS2812_Send(255U, 255U, 255U);
    else        D_WS2812_Off();
    led_on = (uint8_t)!led_on;
}

/* ---- 任务入口：一律只转发给所属模块 ---- */

/* 1ms控制拍，全部闭环计算都在这条链上 */
static void Task_ServoControl(void)
{
    A_Servo_Control(); /* 1ms控制拍，全部闭环计算都在这条链上 */
}

/* 串口：收帧 -> 执行 -> 回复 -> 延迟波特率切换 */
static void Task_Uart(void)
{
    A_Uart_Process();  /* 收帧->执行->回复->延迟波特率切换 */
}

/* 保护：过流/限流/堵转/过温，内部再分频 */
static void Task_Protect(void)
{
    A_Protect_Task(); /* 内部再分频出250ms过温，见A_Protect.c */
}

/* 状态：LED心跳 + 可选遥测 + 喂狗 */
static void Task_State(void)
{
    App_LedHeartbeat();
    /* A_Servo_printf(); */ /* FireWater遥测，约33Hz；调试时取消注释，队列满或PWM输入模式下自动不发 */
    D_IWDG_Feed();          /* 喂狗放最低优先级：高优先级任务卡死时能触发复位 */
}

/* 任务表下标。用具名枚举代替裸数字，改动表的顺序不会静默改错绑定对象；
 * 枚举必须与 s_tasks 一一对应，下面有编译期断言兜底。 */
typedef enum {
    TASK_IDX_SERVO = 0, /* 舵机闭环 */
    TASK_IDX_UART,      /* 串口协议 */
    TASK_IDX_PROTECT,   /* 保护与限功率 */
    TASK_IDX_STATE,     /* 状态指示 */
    TASK_IDX_CFG_SAVE,  /* 参数保存 */
    TASK_IDX_COUNT      /* 任务总数 */
} TaskIndex_t;

static Task_t s_tasks[] = {
    TASK_DEF(Task_ServoControl, 1,  0), /* 周期1ms，最高优先级 */
    TASK_DEF(Task_Uart,         1,  1), /* 周期1ms             */
    TASK_DEF(Task_Protect,      10, 2), /* 周期10ms            */
    TASK_DEF(Task_State,        30, 3), /* 周期30ms            */
    TASK_DEF(A_Config_SaveTask, 0,  4), /* 周期0：只在参数变脏时被事件触发 */
};

/* 枚举和任务表必须同步增删，否则下面的绑定会指到别的任务上。 */
typedef char guard_task_table_size_mismatch
    [(TASK_ARRAY_COUNT(s_tasks) == (uint8_t)TASK_IDX_COUNT) ? 1 : -1];

/* 注册任务表，并把参数保存任务的就绪标志交给配置模块。保存任务周期为0不会被时基
 * 唤醒，A_Config_MarkDirty 直接把就绪标志置1，于是"改了参数"最快在下一拍就被最低
 * 优先级任务消费掉，既不占用1ms控制拍，又不用为它专门轮询一个周期。 */
void A_Tasks_Init(void)
{
    C_Task_Init(s_tasks, TASK_ARRAY_COUNT(s_tasks));
    A_Config_BindSaveEvent(&s_tasks[TASK_IDX_CFG_SAVE].run);
}
