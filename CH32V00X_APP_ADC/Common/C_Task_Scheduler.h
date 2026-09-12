#ifndef __C_TASK_SCHEDULER_H__
#define __C_TASK_SCHEDULER_H__

#include <stdint.h>
#include <stddef.h>


/* 数组元素个数。调度器只吃"数组+长度"，不假设调用方的变量叫什么名字。 */
#define TASK_ARRAY_COUNT(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))


/* 协作式优先级任务调度器，1ms时基由外部中断驱动。
 * Common层不拥有任务表；应用层在初始化时显式传入，避免反向依赖。 */

/* 任务描述结构。任务运行期间再次到期只累计overrun，不重复排队。 */
typedef struct {
    volatile uint8_t  run;      /* 就绪标志：置1后等待被调度               */
    volatile uint8_t  active;   /* 正在执行；期间到期只记超时，不再次排队  */
    uint16_t timer;             /* 周期倒计时(tick)，归零时置run           */
    uint16_t period;            /* 执行周期(tick)；0=不周期触发，只靠事件  */
    volatile uint16_t overrun;  /* 到期时上一次还没跑完的累计次数(诊断用)  */
    uint8_t  prio;              /* 优先级，0最高                           */
    void   (*task_hook)(void);  /* 任务入口函数                            */
} Task_t;

#define TASK_DEF(func, period_tick, priority) \
    { 0, 0, 0, (period_tick), 0, (priority), (func) } /* 构造任务描述 */

void C_Task_Init(Task_t *tasks,
                 uint8_t task_count);     /* 注册任务表并初始化计时器 */
void C_Task_Tick_Update(void);             /* 时基更新，由外部中断调用 */
void C_Task_Run(void);                    /* 主循环调用，执行最高优先级就绪任务 */
/* 以下两个只供诊断/压测使用，正常固件不调用(未引用时会被--gc-sections丢掉)。 */
uint32_t C_Task_GetTotalOverrun(void);     /* 读取所有任务累计错过的release */
void C_Task_ClearOverrun(void);            /* 原子清零超时统计 */

#endif /* __C_TASK_SCHEDULER_H__ */
