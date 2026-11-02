#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"    /* 角度传感器是接在ADC上的电位器 */
#include "A_Config.h" /* 一圈的厘度量程CDEG_RANGE */

/*
 * A_Sensor.h —— 传感器聚合层：编码器/电压/温度/电流读数换算为业务单位。
 * 电位器只支持接ADC：带符号线性行程(不回绕)，抽头脱离碳膜为死区；
 * 悬空时下拉钳地，据此判定死区。两端各外扩 ENCODER_POT_MARGIN_CDEG，角度可为负。
 */

#define ENCODER_ANGLE_ERROR ((int32_t)0xFFFF) /** 读取失败返回值，落在量程之外 */

#define ENCODER_POT_ADC_MIN   350U   /** 0度对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_ADC_MAX   3750U  /** 满行程对应的实测ADC计数(整数，不带Q4) */
#define ENCODER_POT_SPAN_CDEG 27000U /** 满行程角度(厘度)，270.00度 */
#define ENCODER_POT_ADC_SPAN  (ENCODER_POT_ADC_MAX - ENCODER_POT_ADC_MIN) /** 标定区间ADC计数跨度 */
#define ENCODER_POT_MARGIN_CDEG 1000 /** 两端各外扩角度(厘度) */
#define ENCODER_POT_ADC_FLOAT 100U   /** 低于它=抽头悬空=在死区里(整数计数) */
#define ENCODER_POT_ANGLE_MIN (-(int32_t)ENCODER_POT_MARGIN_CDEG) /** 可读角度下限，厘度，可为负 */
#define ENCODER_POT_ANGLE_MAX ((int32_t)ENCODER_POT_SPAN_CDEG \
                             + (int32_t)ENCODER_POT_MARGIN_CDEG) /** 可读角度上限，厘度 */
/* 换算分子须放得进int32，写成除法形式避免判据本身溢出 */
typedef char guard_pot_cdeg_numerator_overflow[
    (((4095L - (long)ENCODER_POT_ADC_MIN) * (long)D_ADC_ENCODER_SCALE)
     < (2147483647L / (long)ENCODER_POT_SPAN_CDEG)) ? 1 : -1];

/* 编码器可读角度范围，脱困状态机与行程求交只认这三个 */
#define ENCODER_ANGLE_LO ENCODER_POT_ANGLE_MIN /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI ENCODER_POT_ANGLE_MAX /** 可读角度上限，厘度 */
#define ENCODER_ANGLE_MID ((ENCODER_ANGLE_LO + ENCODER_ANGLE_HI) / 2) /** 可读量程中点，厘度 */

/* 可主动转入区间：外扩段只兜端点超调和脱困，轨迹收死在标称行程[0,SPAN] */
#define ENCODER_TRAVEL_LO ((int32_t)0)                          /** 可主动转入角度下限，厘度 */
#define ENCODER_TRAVEL_HI ((int32_t)ENCODER_POT_SPAN_CDEG)      /** 可主动转入角度上限，厘度 */
#define ENCODER_TRAVEL_SPAN (ENCODER_TRAVEL_HI - ENCODER_TRAVEL_LO) /** 可主动转入行程长度，厘度 */

#define ADC_VDD_MV      3300U /** ADC参考电压(mV) */
#define ADC_FULL_SCALE  4096U /** ADC满量程计数 */
#define VOLTAGE_DIV_NUM 49U   /** 分压比分子，V_bat = ADC*3300mV*49/(4096*10) */
#define VOLTAGE_DIV_DEN 10U   /** 分压比分母 */

/* 电流检测：60mΩ采样电阻+OPA1单端8x，满量程6.875A，1LSB≈1.68mA；只有驱动段电流流过采样电阻。
 * 正向可上报最大5.29A(低于AT8236斩波点5.5A)，倒拖回馈也可测(约-1.6A以内)。 */
#define CURRENT_SENSE_MOHM 60U /** 采样电阻(毫欧) */
#define CURRENT_SENSE_GAIN  8U /** OPA1单端PGA增益 */
#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM)) /** 满量程电流(mA)，编译期常量 */

int32_t  A_Encoder_Read(void);                 /* 读编码器角度，厘度；失败返回ENCODER_ANGLE_ERROR */
uint16_t A_Voltage_Read(void);                 /* 读电池电压(约4ms低通)，厘伏(V×100) */
uint16_t A_Voltage_ReadRaw(void);              /* 电池电压单点读数，厘伏；含开关尖峰，只给回馈钳位 */
int16_t  A_Temperature_Read(void);             /* 读温度，0.1℃；失败返回-32768 */
uint8_t  A_Current_ReadSigned(int32_t *current_ma); /* 绕组电流(mA)，与施加电压同向为正；1=本块测得到 */
uint8_t  A_Current_Read(uint16_t *current_ma);      /* 电流大小(mA)，供保护与上报；返回值同上 */

#endif /* __A_SENSOR_H__ */
