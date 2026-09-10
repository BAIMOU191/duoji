/* C_Task_Scheduler.c 协作式优先级任务调度器，临界区适配CH32V00X */

#include "C_Task_Scheduler.h"
#include "ch32v00X.h"

static Task_t *s_tasks;       /* 应用层注册的任务表，不拥有内存 */
static uint8_t s_task_count;  /* 已注册任务数量 */

/*
 * @fn      C_Task_Init
 * @brief   注册任务表并初始化各任务计时器
 * @param   tasks 应用层任务数组
 * @param   task_count 数组元素数量
 * @return  无
 */
void C_Task_Init(Task_t *tasks, uint8_t task_count)
{
    uint8_t i;

    s_tasks = tasks;
    s_task_count = (tasks != NULL) ? task_count : 0U;

    for (i = 0; i < s_task_count; i++)
    {
        s_tasks[i].run = 0;
        s_tasks[i].active = 0;
        s_tasks[i].overrun = 0;
        s_tasks[i].timer = s_tasks[i].period;
    }
}

/*
 * @fn      C_Task_Tick_Update
 * @brief   时基更新：周期任务倒计时归零则置就绪标志并重载
 * @param   无
 * @return  无
 */
void C_Task_Tick_Update(void)
{
    uint8_t i;

    for (i = 0; i < s_task_count; i++)
    {
        if (s_tasks[i].period == 0) continue; /* 防御无效配置 */

        if (--s_tasks[i].timer == 0)
        {
            s_tasks[i].timer = s_tasks[i].period;
            if (s_tasks[i].run || s_tasks[i].active)
            {
                if (s_tasks[i].overrun < 0xFFFFU)
                    s_tasks[i].overrun++;
            }
            else
            {
                s_tasks[i].run = 1;
            }
        }
    }
}

/*
 * @fn      C_Task_Run
 * @brief   执行最高优先级就绪任务，每轮只执行一个，协作式不抢占
 * @param   无
 * @return  无
 */
void C_Task_Run(void)
{
    uint8_t i;
    uint8_t best      = 0xFF;
    uint8_t best_prio = 0xFF;
    uint32_t irq_state = __get_MSTATUS();

    /* 扫描和消费就绪标志必须原子化，避免tick落在选择与清除之间。 */
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

    /* 执行期间到点只累计overrun，不把同一任务再次排队。这样显式丢弃
     * 已经过期的release，避免超时的最高优先级任务连续追赶并饿死其余任务。 */
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

uint32_t C_Task_GetTotalOverrun(void)
{
    uint8_t i;
    uint32_t total = 0U;
    uint32_t irq_state = __get_MSTATUS();

    __disable_irq();
    for (i = 0U; i < s_task_count; i++) total += s_tasks[i].overrun;
    __set_MSTATUS(irq_state);
    return total;
}

void C_Task_ClearOverrun(void)
{
    uint8_t i;
    uint32_t irq_state = __get_MSTATUS();

    __disable_irq();
    for (i = 0U; i < s_task_count; i++) s_tasks[i].overrun = 0U;
    __set_MSTATUS(irq_state);
}
