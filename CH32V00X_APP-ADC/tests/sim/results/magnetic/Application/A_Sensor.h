#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"   /* D_ADC_ENCODER_SCALE：编码器读数的定点小数位 */

/* A_Sensor.h 传感器聚合：编码器/电压/温度/电流读取与换算 */

#define ENCODER_MODE    1       /*电位器adc模式*/

/* 反馈坐标是不是圆周坐标。磁编码器整圈可测所以回绕；电位器是有限行程且读数
 * 可以为负，必须按线性带符号处理。A_Parameter.h 的 CFG_WRAP_RANGE_CDEG 要跟
 * 它一致，A_Servo.c 里有编译期断言。 */
#if ENCODER_MODE == 1
#define ENCODER_IS_CIRCULAR 1
#else
#define ENCODER_IS_CIRCULAR 0
#endif

/* 轴能不能转进"测不到角度"的区间。电位器的抽头转出碳膜就完全没有反馈，
 * 只能开环转出来；磁编码器整圈都可测，读失败就是真坏了。A_Servo 用它决定
 * 编码器失效时是开环脱困还是直接停机。 */
#if ENCODER_MODE == 1
#define ENCODER_HAS_DEADZONE 0
#else
#define ENCODER_HAS_DEADZONE 1
#endif

/* 编码器读取失败返回值。取65535是因为它落在两种模式的量程之外：磁编码
 * 是[0,35999]，电位器是[ENCODER_POT_ANGLE_MIN, ENCODER_POT_ANGLE_MAX]。
 * 定成int32_t才能和带符号的角度直接比较，不会触发有符号/无符号混比。 */
#define ENCODER_ANGLE_ERROR  ((int32_t)0xFFFF)

/* ==================== 电位器编码器标定(ENCODER_MODE=0) ====================
 *
 * 电位器两端各有一段死区：碳膜端头之外转轴还能继续走，但抽头电压不再变化，
 * 所以行程两端不是0和4095，得用实测值。下面两个是转到机械行程两端时读到的
 * ADC计数，换电位器或改分压电阻后重新实测这两个数即可，别的地方不用动。
 *
 *   ENCODER_POT_ADC_MIN -> 0度
 *   ENCODER_POT_ADC_MAX -> ENCODER_POT_SPAN_CDEG
 *
 * 输出是厘度。标定区间 [0, 27000] 正好是 A_Servo 那张 span 表里270度行程
 * 的那一档，但量程还向两端各外扩了10度(见下)，读数可以为负、也可以超过
 * 27000。A_Servo、观测器和位置控制器在电位器模式均使用带符号线性坐标；
 * CFG_WRAP_RANGE_CDEG必须为0，编译期断言防止再次混用。 */
/* 这两个是**整数ADC计数**，即你拿量角器实测时读到的数，不带Q4小数位。
 * 换算时再乘 D_ADC_ENCODER_SCALE 对齐，标定值本身保持人能核对的形式。 */
#define ENCODER_POT_ADC_MIN   350U   /* 0度对应的实测ADC计数 */
#define ENCODER_POT_ADC_MAX   3750U  /* 满行程对应的实测ADC计数 */
#define ENCODER_POT_SPAN_CDEG 27000U /* 满行程角度(厘度)，270.00度 */

#define ENCODER_POT_ADC_SPAN  (ENCODER_POT_ADC_MAX - ENCODER_POT_ADC_MIN)

/* 上面两个标定点只定义斜率，量程再按同一斜率向两端各外扩一段，让轴走到
 * 标定区间之外时还能读出角度而不是一上来就撞死在端点上：
 *
 *   -10.00度 <- ADC约224      280.00度 <- ADC约3876
 *
 * 两端都还在0~4095之内，所以这段外扩是电位器真能给出的电压，不是外推。
 * 超出这个范围才钳位——那之外才是碳膜端头的电气死区，轴还在动但抽头电压
 * 已经不变，钳住至少保证角度单调。 */
#define ENCODER_POT_MARGIN_CDEG 1000 /* 两端各外扩的角度(厘度)，10.00度 */

/* 抽头脱离碳膜时，下拉电阻把ADC输入钳到地，实测读数=4。轨道上最低的合法
 * 读数是-10度对应的约224，中间隔着一大段谁都不会落在的无人区，门限取100，
 * 两边各留几倍裕度。
 *
 * 这条判据是确定性的，这也是它值得为之加一颗电阻的原因：没有下拉时抽头
 * 悬空读到的是漏电流和采样电容残荷的合成，实测约1008——而1008按标定换算
 * 恰好是52.25度，是270度行程里一个完全合法的位置。光凭ADC值区分不了"轴在
 * 死区"和"轴真的停在52度"，上电时更是无从判断该不该脱困。 */
#define ENCODER_POT_ADC_FLOAT 100U   /* 低于它=抽头悬空=在死区里(整数计数) */

/* 换算分子 (raw_q4 - MIN*SCALE) * SPAN_CDEG 必须放得进 int32。写成除法形式
 * 是为了判据本身不会先溢出。换更大量程的电位器(比如整圈36000厘度)时这里
 * 会直接编译报错，而不是静默算出负数。 */
typedef char guard_pot_cdeg_numerator_overflow[
    (((4095L - (long)ENCODER_POT_ADC_MIN) * (long)D_ADC_ENCODER_SCALE)
     < (2147483647L / (long)ENCODER_POT_SPAN_CDEG)) ? 1 : -1];
#define ENCODER_POT_ANGLE_MIN (-(int32_t)ENCODER_POT_MARGIN_CDEG)
#define ENCODER_POT_ANGLE_MAX ((int32_t)ENCODER_POT_SPAN_CDEG \
                             + (int32_t)ENCODER_POT_MARGIN_CDEG)

#define ADC_VDD_MV          3300U  /* ADC参考电压(毫伏) */
#define ADC_FULL_SCALE      4096U  /* ADC满量程计数 */

/* 电压换算：V_bat = ADC × 3300mV × 49 / (4096 × 10) */
#define VOLTAGE_DIV_NUM     49U    /* 分压比分子 */
#define VOLTAGE_DIV_DEN     10U    /* 分压比分母 */

/* 电流检测参数：采样电阻60mΩ接在AT8236的ISEN与地之间，OPA1正输入直接
 * 搭在ISEN节点，单端PGA增益8x。
 *   ADC满量程 = 3300mV/8/60mΩ = 6.875A，1 LSB ≈ 1.68mA
 *   AT8236自身过流点 ITRIP = VREF/(10*R) = 3.3/0.6 = 5.5A，在量程之内
 * 只有PWM驱动段的电流流过该电阻：两路全高的刹车段电流在ISEN节点进出
 * 相消，读数为零，所以短路制动电流在硬件上不可见。 */
#define CURRENT_SENSE_MOHM  60U     /* 采样电阻(毫欧) */
#define CURRENT_SENSE_GAIN   8U     /* OPA1单端PGA增益 */

/* OPA工作在 8x + PGA基准VB=VDD/4：零电流时输出不是0而是约760mV
 * (实测945个ADC计数)，信号骑在这个台阶上。台阶由上电零点标定测出并
 * 在 D_ADC_Current_Read 里扣掉，所以对上层透明，但有两个后果：
 *   可上报的最大电流 = (4096-945) * 1.678mA = 5.29A，正好在AT8236
 *   自身过流点5.5A之下，斩波之前的范围全部可见；
 *   下面的满量程常量仍按整个4096算，只用于计数到毫安的比例换算。 */

/* 满量程电流(mA)，编译期常量 */
#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM))

/* 单次读取编码器角度，返回厘度；失败返回ENCODER_ANGLE_ERROR。
 * 磁编码模式量程 [0, 35999]，电位器模式量程可以为负，见上面的外扩说明：
 * [ENCODER_POT_ANGLE_MIN, ENCODER_POT_ANGLE_MAX] = [-1000, 28000]。 */
int32_t  A_Encoder_Read(void);
uint16_t A_Voltage_Read(void);        /* 读取电池电压，返回厘伏(V×100) */
int16_t  A_Temperature_Read(void);    /* 读取温度，返回0.1°C，失败返回-32768 */
uint8_t  A_Current_Read(uint16_t *current_ma); /* 读取电流并返回采样有效标志 */

#endif /* __A_SENSOR_H__ */
