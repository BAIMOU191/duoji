#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"    /* 电压和电流的ADC接口 */
#include "A_Config.h" /* 板级常量(分压比、采样电阻)、整圈量程 CDEG_RANGE */

/*
 * A_Sensor.h —— 传感器聚合层：编码器/电压/温度/电流读数换算为业务单位(厘度、厘伏、0.1℃、mA)。
 * 磁编码版：角度只来自 MT6701，整圈圆周坐标，没有电位器量程、抽头悬空与死区脱困。
 * 板级常量在 A_Config.h 第2节；本文件只放由它们推出的量程与编码器几何。
 */

#define CURRENT_FULL_SCALE_MA \
    (((uint32_t)ADC_VDD_MV * 1000UL) / ((uint32_t)CURRENT_SENSE_GAIN * CURRENT_SENSE_MOHM)) /** 电流检测满量程(mA) */

#define ENCODER_ANGLE_ERROR ((int32_t)0xFFFF) /** 读取失败返回值，落在量程之外 */

/* 编码器可读角度范围，行程求交与校准只认这几个 */
#define ENCODER_ANGLE_LO  ((int32_t)0)                                /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI  ((int32_t)CDEG_RANGE - 1)                   /** 可读角度上限，厘度 */
#define ENCODER_ANGLE_MID ((ENCODER_ANGLE_LO + ENCODER_ANGLE_HI) / 2) /** 可读量程中点，厘度 */

/* 整圈角度均可测，校准锚点允许使用整个量程 */
#define ENCODER_TRAVEL_LO ENCODER_ANGLE_LO   /** 可主动转入角度下限，厘度 */
#define ENCODER_TRAVEL_HI ENCODER_ANGLE_HI   /** 可主动转入角度上限，厘度 */

/* 标定固件(A_Calib_Min)的工作扇区：取270度，记录位移不跨零点、不溢出int16 */
#define CAL_SECTOR_LO     ENCODER_ANGLE_LO   /** 扇区下限，厘度 */
#define CAL_SECTOR_SPAN   27000              /** 扇区长度，厘度 */
#define CAL_SECTOR_HI     (CAL_SECTOR_LO + CAL_SECTOR_SPAN)

int32_t  A_Encoder_Read(void);                 /* 读编码器角度，厘度；失败返回ENCODER_ANGLE_ERROR */
int32_t  A_Encoder_ReadRaw(void);              /* 诊断：编码器原始读数(MT6701 14位计数)，失败返回-1 */
uint16_t A_Voltage_Read(void);                 /* 读电池电压(约4ms低通)，厘伏(V×100) */
uint16_t A_Voltage_ReadRaw(void);              /* 电池电压单点读数，厘伏；含开关尖峰，只给回馈钳位 */
int16_t  A_Temperature_Read(void);             /* 读温度，0.1℃；失败返回-32768 */
uint8_t  A_Current_ReadSigned(int32_t *current_ma); /* 绕组电流(mA)，硬件方向(正=正PWM方向)，与PWM同号=出力；1=本块测得到 */

#endif /* __A_SENSOR_H__ */
