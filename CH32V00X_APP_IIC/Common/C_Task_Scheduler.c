/* C_Task_Scheduler.c —— 协作式优先级任务调度器，临界区适配CH32V00X */

#include "C_Task_Scheduler.h"
#include "ch32v00X.h"

static Task_t *s_tasks;       /* 应用层注册的任务表，不拥有内存 */
static uint8_t s_task_count;  /* 已注册任务数 */

void C_Task_Init(Task_t *tasks, uint8_t task_count)
{
    uint8_t i;

    s_tasks = tasks;
    s_task_count = (tasks != NULL) ? task_count : 0U;

    for (i = 0; i < s_task_count; i++)
    {
        s_tasks[i].run = 0;
        s_tasks[i].active = 0;
        s_tasks[i].timer = s_tasks[i].period;
    }
}

/* 时基：倒计时归零置就绪并重载 */
void C_Task_Tick_Update(void)
{
    uint8_t i;

    for (i = 0; i < s_task_count; i++)
    {
        if (s_tasks[i].period == 0) continue; /* 周期0=仅事件触发 */

        if (--s_tasks[i].timer == 0)
        {
            s_tasks[i].timer = s_tasks[i].period;
            if (!s_tasks[i].run && !s_tasks[i].active)
            {
                s_tasks[i].run = 1;
            }
        }
    }
}

/* 每轮只执行最高优先级就绪任务，协作式不抢占 */
void C_Task_Run(void)
{
    uint8_t i;
    uint8_t best      = 0xFF;
    uint8_t best_prio = 0xFF;
    uint32_t irq_state = __get_MSTATUS();

    /* 扫描与清就绪标志须原子 */
    __disable_irq();
    for (i = 0; i < s_task_count; i++)
    {
        if (s_tasks[i].run && s_tasks[i].prio < best_prio)
        {
            best_prio = s_tasks[i].prio;
            best      = i;
        }
    }

    if (best != 0xFF)
    {
        s_tasks[best].run = 0;
        s_tasks[best].active = 1;
    }
    __set_MSTATUS(irq_state);

    /* 执行期间到期不重复排队，避免超时高优先级饿死其余任务 */
    if (best != 0xFF && s_tasks[best].task_hook != NULL)
        s_tasks[best].task_hook();

    if (best != 0xFF)
    {
        irq_state = __get_MSTATUS();
        __disable_irq();
        s_tasks[best].active = 0;
        __set_MSTATUS(irq_state);
    }
}

/* 按入口函数查任务就绪标志；只在初始化阶段调用 */
volatile uint8_t *C_Task_EventFlag(void (*task_hook)(void))
{
    uint8_t i;

    if (task_hook == NULL) return NULL;
    for (i = 0U; i < s_task_count; i++)
    {
        if (s_tasks[i].task_hook == task_hook) return &s_tasks[i].run;
    }
    return NULL;
}
