#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"       /* D_ADC_ENCODER_ENABLE：本板的角度传感器接在哪 */
#include "A_Parameter.h" /* CDEG_RANGE */

/*
 * A_Sensor.h —— 传感器聚合层：编码器/电压/温度/电流读数换算为业务单位
 * 编码器模式由 D_adc.h 的 D_ADC_ENCODER_ENABLE 派生。
 * 电位器按实测端点计数线性标定，两端各外扩 ENCODER_POT_MARGIN_CDEG(读数真实，角度可为负)；
 * 抽头脱离碳膜时下拉电阻把读数钳到地，据此判定"在死区里"。
 */

#if D_ADC_ENCODER_ENABLE
#define ENCODER_MODE 0 /** 编码器模式：0=电位器ADC，1=MT6701磁编码 */
#else
#define ENCODER_MODE 1 /** 编码器模式：0=电位器ADC，1=MT6701磁编码 */
#endif

/* 反馈坐标是否整圈回绕，须与 CFG_WRAP_RANGE_CDEG 一致(A_Servo.c 有编译期断言) */
#if ENCODER_MODE == 1
#define ENCODER_IS_CIRCULAR 1  /** 1=反馈坐标整圈回绕 */
#else
#define ENCODER_IS_CIRCULAR 0  /** 0=反馈坐标为带符号线性行程 */
#endif

/* 是否存在测不到角度的区间：有则编码器失效时开环脱困，无则直接停机 */
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
/* 换算分子须放得进 int32，写成除法形式避免判据本身溢出 */
typedef char guard_pot_cdeg_numerator_overflow[
    (((4095L - (long)ENCODER_POT_ADC_MIN) * (long)D_ADC_ENCODER_SCALE)
     < (2147483647L / (long)ENCODER_POT_SPAN_CDEG)) ? 1 : -1];
#endif

/* 编码器可读角度范围，脱困状态机与行程求交只认这三个 */
#if ENCODER_MODE == 0
#define ENCODER_ANGLE_LO ENCODER_POT_ANGLE_MIN /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI ENCODER_POT_ANGLE_MAX /** 可读角度上限，厘度 */
#else
#define ENCODER_ANGLE_LO ((int32_t)0)                /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI ((int32_t)CDEG_RANGE - 1)   /** 可读角度上限，厘度 */
#endif
#define ENCODER_ANGLE_MID ((ENCODER_ANGLE_LO + ENCODER_ANGLE_HI) / 2) /** 可读量程中点，厘度 */

/* 允许主动转入的区间：电位器外扩段只用来兜端点超调和脱困判断，轨迹范围收死在标称行程[0, SPAN] */
#if ENCODER_MODE == 0
#define ENCODER_TRAVEL_LO ((int32_t)0)                          /** 可主动转入的角度下限，厘度 */
#define ENCODER_TRAVEL_HI ((int32_t)ENCODER_POT_SPAN_CDEG)      /** 可主动转入的角度上限，厘度 */
#else
#define ENCODER_TRAVEL_LO ENCODER_ANGLE_LO                      /** 可主动转入的角度下限，厘度 */
#define ENCODER_TRAVEL_HI ENCODER_ANGLE_HI                      /** 可主动转入的角度上限，厘度 */
#endif
#define ENCODER_TRAVEL_SPAN (ENCODER_TRAVEL_HI - ENCODER_TRAVEL_LO) /** 可主动转入的行程长度，厘度 */

#define ADC_VDD_MV      3300U /** ADC参考电压(毫伏) */
#define ADC_FULL_SCALE  4096U /** ADC满量程计数 */
#define VOLTAGE_DIV_NUM 49U   /** 分压比分子，V_bat = ADC*3300mV*49/(4096*10) */
#define VOLTAGE_DIV_DEN 10U   /** 分压比分母 */

/* 电流检测：60mΩ采样电阻 + OPA1 单端8x，满量程6.875A，1LSB≈1.68mA；只有驱动段电流流过采样电阻。
 * PGA基准VB=VDD/4 的零点台阶(约945计数)由上电标定扣除，可上报最大5.29A(低于AT8236斩波点5.5A)。 */
#define CURRENT_SENSE_MOHM 60U /** 采样电阻(毫欧) */
#define CURRENT_SENSE_GAIN  8U /** OPA1单端PGA增益 */
#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM)) /** 满量程电流(mA)，编译期常量 */

int32_t  A_Encoder_Read(void);                 /* 读编码器角度，厘度；失败返回ENCODER_ANGLE_ERROR */
uint16_t A_Voltage_Read(void);                 /* 读电池电压，返回厘伏(V×100) */
int16_t  A_Temperature_Read(void);             /* 读温度，返回0.1摄氏度；失败返回-32768 */
uint8_t  A_Current_Read(uint16_t *current_ma); /* 读电流(mA)，总是写值；返回值=是否采在驱动段 */

#endif /* __A_SENSOR_H__ */
