/* 调度器审计：表长度自动推导、按入口函数绑定事件、事件型任务只靠标志触发、优先级、换表顺序后绑定不跑偏 */
#include <stdio.h>
#include <stdint.h>

/* 主机上单线程，临界区空实现即可 */
static inline uint32_t __get_MSTATUS(void) { return 0U; }
static inline void __set_MSTATUS(uint32_t v) { (void)v; }
static inline void __disable_irq(void) {}
#include "C_Task_Scheduler.c"

static int failures;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

static int n_fast, n_slow, n_event, order[8], n_order;
static void fast(void)   { n_fast++;  order[n_order++ % 8] = 1; }
static void slow(void)   { n_slow++;  order[n_order++ % 8] = 2; }
static void event(void)  { n_event++; }
static void unused(void) {}

static void run_all(void) { for (int i = 0; i < 8; i++) C_Task_Run(); }

int main(void)
{
    static Task_t a[] = { TASK_DEF(fast, 1, 0), TASK_DEF(slow, 3, 1), TASK_DEF(event, 0, 2) };
    static Task_t b[] = { TASK_DEF(event, 0, 2), TASK_DEF(slow, 3, 1), TASK_DEF(fast, 1, 0) };

    C_TASK_INIT(a);
    CHECK(s_task_count == 3);
    CHECK(C_Task_EventFlag(event)  == &a[2].run);
    CHECK(C_Task_EventFlag(fast)   == &a[0].run);
    CHECK(C_Task_EventFlag(unused) == NULL);
    CHECK(C_Task_EventFlag(NULL)   == NULL);

    /* 事件型任务：时基不唤醒，置标志才跑且只跑一次 */
    for (int t = 0; t < 30; t++) { C_Task_Tick_Update(); run_all(); }
    CHECK(n_event == 0);
    CHECK(n_fast == 30 && n_slow == 10);
    *C_Task_EventFlag(event) = 1U;
    run_all(); CHECK(n_event == 1);
    run_all(); CHECK(n_event == 1);

    /* 同时就绪时高优先级先跑 */
    n_order = 0;
    for (int t = 0; t < 3; t++) C_Task_Tick_Update();
    run_all();
    CHECK(n_order == 2 && order[0] == 1 && order[1] == 2);

    /* 换表顺序后，按函数取到的仍是同一个任务 */
    C_TASK_INIT(b);
    CHECK(C_Task_EventFlag(event) == &b[0].run);
    CHECK(C_Task_EventFlag(fast)  == &b[2].run);
    n_event = 0; *C_Task_EventFlag(event) = 1U; run_all();
    CHECK(n_event == 1);

    printf("sched failures=%d\n", failures);
    return failures ? 1 : 0;
}
