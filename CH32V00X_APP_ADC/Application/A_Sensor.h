#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"       /* D_ADC_ENCODER_ENABLE：本板的角度传感器接在哪 */
#include "A_Parameter.h" /* CDEG_RANGE */

/*
 * A_Sensor.h —— 传感器聚合层：编码器/电压/温度/电流的读取与单位换算
 *
 * 上层只认业务单位(厘度/厘伏/0.1摄氏度/毫安)，换硬件只改本文件的常量。
 * 编码器模式由 D_adc.h 的 D_ADC_ENCODER_ENABLE 派生，全工程只有那一个开关。
 *
 * 电位器标定：碳膜两端各有一段死区，转轴还能走但抽头电压不再变化，所以行程
 * 两端要用实测ADC计数而不是0和4095。换电位器或改分压电阻后只需重测下面两个
 * 端点计数，其余全部派生。量程再按同一斜率向两端各外扩 ENCODER_POT_MARGIN_CDEG，
 * 那一段电位器仍在线性区，读数是真的，所以**角度可以为负**；更外面才钳位。
 * 抽头脱离碳膜时下拉电阻把读数钳到地(实测4)，与最低合法读数之间隔着一大段
 * 无人区，据此可以确定性地判定"在死区里"——没有下拉时悬空读数约1008，按标定
 * 恰好是52度，是行程内一个完全合法的位置，光凭ADC值区分不了。
 */

#if D_ADC_ENCODER_ENABLE
#define ENCODER_MODE 0 /** 编码器模式：0=电位器ADC，1=MT6701磁编码 */
#else
#define ENCODER_MODE 1 /** 编码器模式：0=电位器ADC，1=MT6701磁编码 */
#endif

/* 反馈坐标是不是圆周坐标。磁编码器整圈可测所以回绕；电位器是有限行程且读数
 * 可以为负，必须按线性带符号处理。A_Parameter.h 的 CFG_WRAP_RANGE_CDEG 要跟
 * 它一致，A_Servo.c 里有编译期断言。 */
#if ENCODER_MODE == 1
#define ENCODER_IS_CIRCULAR 1  /** 1=反馈坐标整圈回绕 */
#else
#define ENCODER_IS_CIRCULAR 0  /** 0=反馈坐标为带符号线性行程 */
#endif

/* 轴能不能转进"测不到角度"的区间。电位器抽头转出碳膜就完全没有反馈，只能
 * 开环转出来；磁编码器整圈都可测，读失败就是真坏了。A_Servo 用它决定编码器
 * 失效时是开环脱困还是直接停机。 */
#if ENCODER_MODE == 1
#define ENCODER_HAS_DEADZONE 0 /** 0=整圈可测，读失败即器件故障 */
#else
#define ENCODER_HAS_DEADZONE 1 /** 1=存在测不到角度的区间，需开环脱困 */
#endif

#define ENCODER_ANGLE_ERROR ((int32_t)0xFFFF) /** 读取失败返回值，落在两种模式量程之外 */

#if ENCODER_MODE == 0
#define ENCODER_POT_ADC_MIN   350U   /** 0度对应的实测ADC计数(整数，不带Q4小数位) */
#define ENCODER_POT_ADC_MAX   3750U  /** 满行程对应的实测ADC计数(整数，不带Q4小数位) */
#define ENCODER_POT_SPAN_CDEG 27000U /** 满行程角度(厘度)，270.00度 */
#define ENCODER_POT_ADC_SPAN  (ENCODER_POT_ADC_MAX - ENCODER_POT_ADC_MIN) /** 标定区间的ADC计数跨度 */
#define ENCODER_POT_MARGIN_CDEG 1000 /** 两端各外扩的角度(厘度)，对应ADC约224和3876 */
#define ENCODER_POT_ADC_FLOAT 100U   /** 低于它=抽头悬空=在死区里(整数计数) */
#define ENCODER_POT_ANGLE_MIN (-(int32_t)ENCODER_POT_MARGIN_CDEG) /** 可读角度下限，厘度，可为负 */
#define ENCODER_POT_ANGLE_MAX ((int32_t)ENCODER_POT_SPAN_CDEG \
                             + (int32_t)ENCODER_POT_MARGIN_CDEG) /** 可读角度上限，厘度 */
/* 换算分子 (raw_q4 - MIN*SCALE) * SPAN_CDEG 必须放得进 int32。写成除法形式是为了
 * 判据本身不会先溢出；换更大量程的电位器时这里会直接编译报错而不是静默算出负数。 */
typedef char guard_pot_cdeg_numerator_overflow[
    (((4095L - (long)ENCODER_POT_ADC_MIN) * (long)D_ADC_ENCODER_SCALE)
     < (2147483647L / (long)ENCODER_POT_SPAN_CDEG)) ? 1 : -1];
#endif

/* 编码器能给出角度的范围，两种模式都要提供：磁编码器整圈可测，电位器是外扩后的
 * 有限行程。脱困状态机和行程求交只认这三个，不直接碰某一种传感器的标定常数。 */
#if ENCODER_MODE == 0
#define ENCODER_ANGLE_LO ENCODER_POT_ANGLE_MIN /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI ENCODER_POT_ANGLE_MAX /** 可读角度上限，厘度 */
#else
#define ENCODER_ANGLE_LO ((int32_t)0)                /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI ((int32_t)CDEG_RANGE - 1)   /** 可读角度上限，厘度 */
#endif
#define ENCODER_ANGLE_MID ((ENCODER_ANGLE_LO + ENCODER_ANGLE_HI) / 2) /** 可读量程中点，厘度 */

#define ADC_VDD_MV      3300U /** ADC参考电压(毫伏) */
#define ADC_FULL_SCALE  4096U /** ADC满量程计数 */
#define VOLTAGE_DIV_NUM 49U   /** 分压比分子，V_bat = ADC*3300mV*49/(4096*10) */
#define VOLTAGE_DIV_DEN 10U   /** 分压比分母 */

/* 电流检测：60mΩ采样电阻接在AT8236的ISEN与地之间，OPA1正输入搭在ISEN节点，
 * 单端PGA增益8x，满量程 3300mV/8/60mΩ = 6.875A，1 LSB ≈ 1.68mA。AT8236自身的
 * 斩波点 ITRIP = 5.5A 在量程之内。只有PWM驱动段的电流流过该电阻，两路全高的
 * 刹车段电流在ISEN节点进出相消，所以短路制动电流在硬件上不可见。
 * OPA的PGA基准 VB=VDD/4，零电流时输出约760mV(实测945个计数)，这个台阶由上电
 * 零点标定测出并在 D_ADC_Current_Read 里扣掉，所以可上报的最大电流是
 * (4096-945)*1.678mA = 5.29A，正好在硬件斩波点之下。 */
#define CURRENT_SENSE_MOHM 60U /** 采样电阻(毫欧) */
#define CURRENT_SENSE_GAIN  8U /** OPA1单端PGA增益 */
#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM)) /** 满量程电流(mA)，编译期常量 */

int32_t  A_Encoder_Read(void);                 /* 读编码器角度，厘度；失败返回ENCODER_ANGLE_ERROR */
uint16_t A_Voltage_Read(void);                 /* 读电池电压，返回厘伏(V×100) */
int16_t  A_Temperature_Read(void);             /* 读温度，返回0.1摄氏度；失败返回-32768 */
uint8_t  A_Current_Read(uint16_t *current_ma); /* 读电流(mA)，返回采样有效标志 */

#endif /* __A_SENSOR_H__ */
