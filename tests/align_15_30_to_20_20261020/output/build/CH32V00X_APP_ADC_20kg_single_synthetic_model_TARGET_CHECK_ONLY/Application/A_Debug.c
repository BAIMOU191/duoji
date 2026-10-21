/* A_Debug.c —— VOFA+ FireWater 诊断输出: servo:ch0,ch1,...,ch17\r\n
 * 通道顺序(整数，避免浮点printf)：
 *  0 目标位置(cdeg)       1 轨迹参考位置(cdeg)  2 实测位置(cdeg)
 *  3 参考速度(cdeg/s)     4 观测器速度(cdeg/s)  5 实际PWM(物理方向，±2400)
 *  6 前馈(mV)            7 反馈(mV)            8 积分(mV)
 *  9 起转补偿(mV)       10 控制输出(mV)       11 扰动估计(mV)
 * 12 输出饱和(0/1)      13 控制器状态(0运动 1收尾 2保持 3没闭环)  14 有效标志(flags)
 * 15 保护故障位掩码     16 绕组电流(mA，与PWM同号=在出力；flags bit3=0 时测不到)
 * 17 电位器ADC原始均值(Q4计数，未换算)；卸力时 2/17 照常刷新，可手转扫描
 * 位置/速度以角度增大为正；PWM、电流以硬件方向为正，比较符号时注意MOTOR_SIGN。
 * bit0=闭环本拍更新，bit1=最近位置采样有效，bit2=观测器粘住预测，bit3=电流本块测得到。
 * USART1/PC0使用配置波特率(默认115200，8N1)，仅串口输入模式可输出；
 * PWM输入时驱动已释放共线PC0，本任务不会重新接管。半双工发送期间收不到命令，
 * 高频命令调试可增大周期或关闭打印。禁止直接printf，以免阻塞1ms控制任务。
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
        "servo:%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%ld,%u,%u,%u,%u,%d,%u\r\n",
        (long)data.target_cdeg, (long)data.ref_cdeg, (long)data.actual_cdeg,
        (long)data.ref_cdps, (long)data.actual_cdps, (int)data.pwm,
        (int)data.ff_mv, (int)data.fb_mv, (int)data.i_mv, (int)data.stick_mv,
        (int)data.out_mv, (long)data.dist_mv, (unsigned)data.saturated,
        (unsigned)data.state, (unsigned)data.flags, (unsigned)A_Protect_Fault(), (int)data.cur_ma,
        (unsigned)data.pot_q4);
    /* 环形队列最多容纳127字节；超长/队列不足丢整帧，不能发截断的CSV。 */
    if (len > 0 && (unsigned)len < sizeof(frame))
        (void)D_UART1_Tx_Write((const uint8_t *)frame, (uint8_t)len);
}
#endif
