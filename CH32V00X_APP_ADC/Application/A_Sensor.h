#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"    /* 角度传感器是接在ADC上的电位器 */
#include "A_Config.h" /* 板级常量(分压比、采样电阻)、电位器标定与设计常量(App_Model.h) */

/*
 * A_Sensor.h —— 传感器聚合层：编码器/电压/温度/电流读数换算为业务单位(厘度、厘伏、0.1℃、mA)。
 * 电位器标定与设计常量在 App_Model.h，分压比、采样电阻在 A_Config.h 第2节；本文件只放由它们推出的量程与编码器几何。
 * 电位器只支持接ADC：带符号线性行程(不回绕)，两端各外扩 ENCODER_POT_MARGIN_CDEG，角度可为负。
 */

#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM)) /** 电流检测满量程(mA) */

#define ENCODER_ANGLE_ERROR ((int32_t)0xFFFF) /** 读取失败返回值，落在量程之外 */
#define ENCODER_POT_ADC_SPAN (ENCODER_POT_ADC_MAX - ENCODER_POT_ADC_MIN) /** 标定区间ADC计数跨度 */

/* 编码器可读角度范围，脱困状态机与行程求交只认这三个 */
#define ENCODER_ANGLE_LO  (-(int32_t)ENCODER_POT_MARGIN_CDEG)                                /** 可读角度下限，厘度，可为负 */
#define ENCODER_ANGLE_HI  ((int32_t)ENCODER_POT_SPAN_CDEG + (int32_t)ENCODER_POT_MARGIN_CDEG) /** 可读角度上限，厘度 */
#define ENCODER_ANGLE_MID ((ENCODER_ANGLE_LO + ENCODER_ANGLE_HI) / 2)                        /** 可读量程中点，厘度 */

/* 可主动转入区间：外扩段只兜端点超调和脱困，轨迹收死在标称行程[0,SPAN] */
#define ENCODER_TRAVEL_LO ((int32_t)0)                          /** 可主动转入角度下限，厘度 */
#define ENCODER_TRAVEL_HI ((int32_t)ENCODER_POT_SPAN_CDEG)      /** 可主动转入角度上限，厘度 */

/* 标定固件(A_Calib_Min)的工作扇区：电位器取可读全程 */
#define CAL_SECTOR_LO     ENCODER_ANGLE_LO
#define CAL_SECTOR_HI     ENCODER_ANGLE_HI
#define CAL_SECTOR_SPAN   ((int32_t)ENCODER_POT_SPAN_CDEG)   /** 阶跃起点、斜坡位置按标称行程取 */

int32_t  A_Encoder_Read(void);                 /* 读编码器角度，厘度；失败返回ENCODER_ANGLE_ERROR */
int32_t  A_Encoder_ReadRaw(void);              /* 诊断：编码器原始读数(电位器ADC块均值，Q4计数)，未就绪返回-1 */
uint16_t A_Voltage_Read(void);                 /* 读电池电压(约4ms低通)，厘伏(V×100) */
uint16_t A_Voltage_ReadRaw(void);              /* 电池电压单点读数，厘伏；含开关尖峰，只给回馈钳位 */
int16_t  A_Temperature_Read(void);             /* 读温度，0.1℃；失败返回-32768 */
uint8_t  A_Current_ReadSigned(int32_t *current_ma); /* 绕组电流(mA)，硬件方向(正=正PWM方向)，与PWM同号=出力；1=本块测得到 */

#endif /* __A_SENSOR_H__ */
