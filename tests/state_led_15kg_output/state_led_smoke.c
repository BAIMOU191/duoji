/* Exercise the production task with cached-state and hardware mocks. */
#include <assert.h>
#include <stdlib.h>
#include "A_Tasks.c"

static uint16_t fault;
static uint8_t work, stalled, rgb[3];
static unsigned sends, feeds;

void A_Servo_Control(void) { assert(0); }
void A_Uart_Process(void) { assert(0); }
void IAP_Service(void) { assert(0); }
void A_Protect_Task(void) { assert(0); }
uint8_t A_Config_SaveTask(void) { assert(0); return 0; }
void C_Task_Init(Task_t *tasks, uint8_t count)
{
    unsigned i, found = 0;
    for (i = 0; i < count; i++)
        if (tasks[i].task_hook == Task_State)
        { assert(tasks[i].period == 20 && tasks[i].prio == 3); found++; }
    assert(found == 1);
}
volatile uint8_t *C_Task_EventFlag(void (*hook)(void))
{ assert(hook == Task_ConfigSave); return &s_tasks[4].run; }
void A_Config_BindSaveEvent(volatile uint8_t *event)
{ assert(event == &s_tasks[4].run); }

uint16_t A_Protect_Fault(void) { return fault; }
uint8_t A_Servo_GetWorkState(void) { return fault ? SERVO_WORK_FAULT : work; }
uint8_t A_Protect_Stalled(void) { return stalled; }
void D_WS2812_Send(uint8_t r, uint8_t g, uint8_t b)
{ rgb[0] = r; rgb[1] = g; rgb[2] = b; sends++; }
void D_IWDG_Feed(void) { feeds++; }

static void color(uint8_t r, uint8_t g, uint8_t b)
{ assert(rgb[0] == r && rgb[1] == g && rgb[2] == b); }

static void steady(uint16_t f, uint8_t state, uint8_t r, uint8_t g, uint8_t b)
{
    unsigned i;
    fault = f; work = state;
    for (i = 0; i < 60; i++) { Task_State(); color(r, g, b); }
}

static void blink(uint16_t f, uint8_t state, unsigned half,
                  uint8_t r, uint8_t g, uint8_t b)
{
    unsigned i;
    fault = f; work = state;
    for (i = 0; i < 2 * half; i++)
    {
        Task_State();
        if (i < half) color(r, g, b); else color(0, 0, 0);
    }
    Task_State(); color(r, g, b);
}

int main(void)
{
    unsigned i;
    uint8_t first[3], previous[3];
    A_Tasks_Init();
    assert(TASK_STATE_PERIOD_MS == 20U);
    steady(PROT_FAULT_OVERTEMP, SERVO_WORK_IDLE, 255, 80, 0);
    steady(PROT_FAULT_OVERCUR, SERVO_WORK_IDLE, 255, 0, 255);
    blink(PROT_FAULT_OVERVOLT, SERVO_WORK_IDLE, 25, 255, 0, 0);
    steady(0, SERVO_WORK_PAUSED, 0, 0, 255);
    blink(PROT_FAULT_UNDERVOLT, SERVO_WORK_IDLE, 25, 255, 0, 0);
    steady(0, SERVO_WORK_PAUSED, 0, 0, 255);
    blink(PROT_FAULT_VOLT_INVALID, SERVO_WORK_IDLE, 25, 255, 0, 0);
    blink(0, SERVO_WORK_FAULT, 25, 0, 255, 0);
    steady(0, SERVO_WORK_PAUSED, 0, 0, 255);
    steady(0, SERVO_WORK_RELEASE_LOW, 0, 0, 255);
    steady(0, SERVO_WORK_RELEASE_HIGH, 0, 0, 255);
    stalled = 1;
    steady(0, SERVO_WORK_RELEASE_LOW, 0, 0, 255);
    steady(0, SERVO_WORK_PAUSED, 0, 0, 255);
    /* A higher-priority fault must start lit, even in another mode's dark half. */
    fault = PROT_FAULT_UNDERVOLT;
    Task_State();
    for (i = 0; i < 25; i++) Task_State();
    color(0, 0, 0);
    steady(PROT_FAULT_OVERTEMP | PROT_FAULT_OVERCUR | PROT_FAULT_UNDERVOLT,
           SERVO_WORK_RELEASE_LOW, 255, 80, 0);
    steady(PROT_FAULT_OVERCUR | PROT_FAULT_UNDERVOLT,
           SERVO_WORK_RELEASE_LOW, 255, 0, 255);
    blink(PROT_FAULT_UNDERVOLT, SERVO_WORK_RELEASE_LOW, 25, 255, 0, 0);
    steady(0, SERVO_WORK_RELEASE_LOW, 0, 0, 255);
    /* A new action overrides historical stall; the full rainbow wraps smoothly. */
    work = SERVO_WORK_MOVING;
    Task_State();
    for (i = 0; i < 3; i++) first[i] = previous[i] = rgb[i];
    for (i = 0; i < 306; i++)
    {
        unsigned channel;
        Task_State();
        assert(rgb[0] || rgb[1] || rgb[2]);
        for (channel = 0; channel < 3; channel++)
        {
            assert(abs((int)rgb[channel] - previous[channel]) <= 5);
            previous[channel] = rgb[channel];
        }
    }
    color(first[0], first[1], first[2]);
    /* Internal recovery and historical stall must not add user-facing modes. */
    work = SERVO_WORK_RECOVERING;
    Task_State();
    assert(rgb[0] || rgb[1] || rgb[2]);
    work = SERVO_WORK_IDLE;
    assert(StateLed_Select() == STATE_LED_RAINBOW);
    assert(feeds == sends);
    printf("PASS: status/fault colors, priority, blink timing, transitions, stall semantics, rainbow wrap; %u watchdog feeds\n", feeds);
    return 0;
}
