/* A_Debug.c —— VOFA+ FireWater 诊断输出帧 servo:ch0..ch16(整数)。
 * 通道: 0目标 1参考 2实测位置(cdeg); 3参考 4观测速度(cdeg/s); 5 PWM(±2400);
 * 6前馈 7反馈 8积分 9起转 10输出 11扰动(mV); 12饱和 13状态 14flags 15故障;
 * 16电流(mA)。位置/速度角增大为正; PWM/电流硬件方向为正。
 * 仅串口输入模式可输出；半双工发送期间收不到命令；禁止直接printf以免阻塞1ms控制。
 */
#include "A_Debug.h"

#if TASK_FIREWATER_ENABLE
#include "A_Servo.h"
#include "A_Protect.h"
#include "A_UartCmd.h"
#include "D_uart.h"
#include "iap.h"
#include <stdio.h>

void A_Debug_Task(void)
{
    ServoDebugTelemetry_t data;
    char frame[RING_BUF_SIZE];
    int len;

    if (IAP_UpgradePending() || !D_UART_Tx_Idle() || !A_Uart_DebugReady()) return;
    A_Servo_GetDebugTelemetry(&data);
    if (data.input_source != SERVO_INPUT_TX) return; /* PWM输入共线，不能接管PC0 */
    len = snprintf(frame, sizeof(frame),
        "servo:%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%ld,%u,%u,%u,%u,%d\r\n",
        (long)data.target_cdeg, (long)data.ref_cdeg, (long)data.actual_cdeg,
        (long)data.ref_cdps, (long)data.actual_cdps, (int)data.pwm,
        (int)data.ff_mv, (int)data.fb_mv, (int)data.i_mv, (int)data.stick_mv,
        (int)data.out_mv, (long)data.dist_mv, (unsigned)data.saturated,
        (unsigned)data.state, (unsigned)data.flags, (unsigned)A_Protect_Fault(), (int)data.cur_ma);
    /* 队列仅127字节；超长丢整帧，不发截断CSV */
    if (len > 0 && (unsigned)len < sizeof(frame))
        (void)D_UART1_Tx_Write((const uint8_t *)frame, (uint8_t)len);
}
#endif
