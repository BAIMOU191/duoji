#ifndef __C_TASK_SCHEDULER_H__
#define __C_TASK_SCHEDULER_H__

#include <stdint.h>
#include <stddef.h>


#define TASK_ARRAY_COUNT(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))


/* 协作式优先级任务调度器，1ms时基由中断驱动。
 * 用 TASK_DEF 建任务表，C_TASK_INIT 整表注册；事件型(period=0)靠置run触发。
 */

typedef struct {
    volatile uint8_t  run;      /* 就绪标志，置1等待调度 */
    volatile uint8_t  active;   /* 正在执行；期间到期不再排队 */
    uint16_t timer;             /* 周期倒计时(tick)，归零置run */
    uint16_t period;            /* 周期(tick)；0=仅事件 */
    uint8_t  prio;              /* 优先级，0最高 */
    void   (*task_hook)(void);  /* 任务入口 */
} Task_t;

#define TASK_DEF(func, period_tick, priority) \
    { 0, 0, 0, (period_tick), (priority), (func) }

#define C_TASK_INIT(tasks) C_Task_Init((tasks), TASK_ARRAY_COUNT(tasks))

void C_Task_Init(Task_t *tasks,
                 uint8_t task_count);     /* 注册任务表并初始化计时 */
void C_Task_Tick_Update(void);             /* 时基更新，中断中调用 */
void C_Task_Run(void);                    /* 主循环调用，跑最高优先级就绪任务 */
volatile uint8_t *C_Task_EventFlag(void (*task_hook)(void)); /* 取就绪标志，置1触发；未注册返回NULL */

#endif /* __C_TASK_SCHEDULER_H__ */
