/* 实际标定任务 + IAP 协议 + 调度器联测，硬件改为探针。 */
#define main iap_protocol_audit_main
#include "iap_audit.c"
#undef main

static int motor_sets, motor_releases, watchdog_feeds, uart_services, log_calls;
static uint8_t motor_release_high;
static uint8_t rx_queue[128];
static unsigned rx_read, rx_write;

int16_t D_Motor_Set(int16_t pwm) { motor_sets++; return pwm; }
void D_Motor_Release(uint8_t high)
{
    motor_releases++;
    motor_release_high = high;
}
void D_IWDG_Feed(void) { watchdog_feeds++; }
void D_UART_Service(void) { uart_services++; }
uint8_t D_UART1_Rx_Get(uint8_t *byte)
{
    if (rx_read == rx_write) return 0U;
    *byte = rx_queue[rx_read++];
    return 1U;
}
uint8_t D_ADC_Encoder_Read(uint16_t *value) { *value = 2000U * 16U; return 1U; }
uint16_t D_MT6701_Read_Raw(void) { return 8000U; }
static int capture_printf(const char *fmt, ...) { (void)fmt; log_calls++; return 0; }

#define printf capture_printf
#define s_tasks calib_tasks
#include "A_Calib_Min.c"
#undef s_tasks
#undef printf

static inline uint32_t __get_MSTATUS(void) { return 0U; }
static inline void __set_MSTATUS(uint32_t v) { (void)v; }
static inline void __disable_irq(void) {}
#include "C_Task_Scheduler.c"

static void step_ms(void)
{
    C_Task_Tick_Update();
    for (int i = 0; i < 4; i++) C_Task_Run();
}

static void setup_calib(void)
{
    reset_state();
    IAP_Init();
    A_Calib_Min_Init();
    A_Calib_Min_Tasks_Init();
    motor_sets = motor_releases = watchdog_feeds = uart_services = log_calls = 0;
    rx_read = rx_write = 0;
    CHECK(s_task_count == 3U);
    CHECK(C_Task_EventFlag(Task_CalibMinIAP) != NULL);
}

static void queue_frame(uint8_t cmd, const uint8_t *data, uint8_t len, int bad_crc)
{
    CHECK(11U + len <= sizeof(rx_queue));
    rx_read = 0U;
    rx_write = 11U + len;
    rx_queue[0] = 0xAA; rx_queue[1] = 0x55; rx_queue[2] = cmd;
    rx_queue[3] = len; rx_queue[4] = 0U;
    memcpy(rx_queue + 5, data, len);
    put32(rx_queue + 5U + len, ref_crc32(rx_queue + 2, 3U + len) ^ (bad_crc ? 1U : 0U));
    rx_queue[9U + len] = 0x55; rx_queue[10U + len] = 0xAA;
    tx_len = 0;
}

static void check_calib_rx(void)
{
    uint8_t id = 7U;
    uint8_t identity[8];
    setup_calib();
    queue_frame(IAP_CMD_QUERY, &id, 1U, 0);
    step_ms();
    CHECK(rx_read == rx_write && tx_len == 20 && tx[2] == 0xA0 && tx[5] == 0U);
    memcpy(identity, tx + 6, sizeof(identity));
    CHECK(identity[0] == 7U && identity[4] == FW_TORQUE);
    CHECK(motor_releases == 0 && !s_iap_stopped);

    queue_frame(IAP_CMD_ENTER, identity, 8U, 1); /* 坏CRC */
    step_ms();
    CHECK(tx_len == 0 && !s_iap_stopped);
    identity[0] = 8U;
    queue_frame(IAP_CMD_ENTER, identity, 8U, 0); /* 别人的ID */
    step_ms();
    CHECK(tx_len == 0 && !s_iap_stopped);
    identity[0] = 255U;
    queue_frame(IAP_CMD_ENTER, identity, 8U, 0); /* 广播不能进入升级 */
    step_ms();
    CHECK(tx_len == 0 && !s_iap_stopped);
    identity[0] = 7U; identity[4] ^= 1U;
    queue_frame(IAP_CMD_ENTER, identity, 8U, 0); /* 硬件不符 */
    step_ms();
    CHECK(tx_len == 12 && tx[5] == IAP_ST_MISMATCH && !s_iap_stopped);
    CHECK(motor_releases == 0 && release_calls == 0 && reset_calls == 0);

    /* 普通运动命令被消费但不执行，也不妨碍下一条IAP查询。 */
    memcpy(rx_queue, "#007P1500T1000!", 14);
    rx_read = 0U; rx_write = 14U; tx_len = 0;
    step_ms();
    CHECK(rx_read == rx_write && tx_len == 0 && !s_iap_stopped);
    queue_frame(IAP_CMD_QUERY, &id, 1U, 0);
    step_ms();
    CHECK(tx_len == 20 && tx[5] == 0U);
}

static void check_calib_stop(uint8_t phase, uint8_t fail)
{
    const uint8_t identity[8] = {7U, FW_SERVO_TYPE, FW_HW_VERSION, FW_VOLTAGE,
        FW_TORQUE, SERVO_VERSION_MAJOR, SERVO_VERSION_MINOR, SERVO_VERSION_PATCH};
    int sets_before, logs_before;
    setup_calib();
    s_cm.phase = phase;
    s_cm.fail = fail;
    s_cm.step = ST_RUN;
    D_Motor_Set(900);
    tx_idle = 0U; /* 模拟低波特率，应答需要等待多个标定拍 */
    queue_frame(IAP_CMD_ENTER, identity, 8U, 0);
    *C_Task_EventFlag(Task_CalibMinIAP) = 1U;
    C_Task_Run();
    CHECK(tx_len == 12 && tx[2] == 0xA1 && tx[5] == 0U);
    CHECK(s_iap_stopped && motor_releases == 1 && motor_release_high == 0U);
    CHECK(release_calls == 0); /* 不调用未初始化的闭环舵机 */
    CHECK(reset_calls == 0);
    sets_before = motor_sets; logs_before = log_calls;
    for (int t = 0; t < 60; t++) step_ms();
    CHECK(motor_sets == sets_before && log_calls == logs_before);
    CHECK(reset_calls == 0 && watchdog_feeds > 0 && uart_services >= 60);
    tx_idle = 1U;
    step_ms();
    CHECK(reset_calls == 1 && reset_mode == Start_Mode_BOOT && clear_calls == 1);
    A_Calib_Min_Init();
    CHECK(!s_iap_stopped && s_cm.phase == PH_BOOT);
}

static void check_report_waits_for_reply(void)
{
    setup_calib();
    s_cm.phase = PH_T1_LINE;
    s_cm.fail = 4U; /* 本拍由运行转入故障报告，也不能插入IAP应答 */
    tx_idle = 0U;
    step_ms();
    CHECK(log_calls == 0 && s_cm.phase == PH_REPORT);
    step_ms();
    CHECK(log_calls == 0 && s_cm.phase == PH_REPORT);
    tx_idle = 1U;
    step_ms();
    CHECK(log_calls > 0 && s_cm.phase == PH_DONE);
}

int main(void)
{
    (void)iap_protocol_audit_main(0, NULL); /* 原有正常模式协议用例也必须通过 */
    check_calib_rx();
    check_calib_stop(PH_BOOT, 0U);
    check_calib_stop(PH_T1_LINE, 0U);
    check_calib_stop(PH_REPORT, 4U);
    check_calib_stop(PH_DONE, 0U);
    check_report_waits_for_reply();
    printf("calib_iap failures=%d\n", failures);
    return failures ? 1 : 0;
}
