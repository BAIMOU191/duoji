#ifndef __A_SENSOR_H__
#define __A_SENSOR_H__

#include <stdint.h>
#include "D_adc.h"       /* 电压和电流的ADC接口 */
#include "A_Config.h" /* 一圈的厘度量程CDEG_RANGE */

/* MT6701磁编码反馈为整圈圆周坐标；电压、温度、电流换算单位与ADC版一致。
 * 纯磁编码版：角度只来自 MT6701，没有电位器量程、抽头悬空与死区脱困。 */

/* 反馈坐标是否整圈回绕，须与 CFG_WRAP_RANGE_CDEG 一致(A_Servo.c 有编译期断言) */
#define ENCODER_IS_CIRCULAR 1  /** 1=反馈坐标整圈回绕 */

/* 是否存在测不到角度的区间：有则编码器失效时开环脱困，无则直接停机 */
#define ENCODER_HAS_DEADZONE 0 /** 0=整圈可测，读失败即器件故障 */

#define ENCODER_ANGLE_ERROR ((int32_t)0xFFFF) /** 读取失败返回值，落在两种模式量程之外 */


/* 编码器可读角度范围，脱困状态机与行程求交只认这三个 */
#define ENCODER_ANGLE_LO ((int32_t)0)                /** 可读角度下限，厘度 */
#define ENCODER_ANGLE_HI ((int32_t)CDEG_RANGE - 1)   /** 可读角度上限，厘度 */
#define ENCODER_ANGLE_MID ((ENCODER_ANGLE_LO + ENCODER_ANGLE_HI) / 2) /** 可读量程中点，厘度 */

/* 整圈角度均可测，校准锚点允许使用整个量程 */
#define ENCODER_TRAVEL_LO ENCODER_ANGLE_LO                      /** 可主动转入角度下限，厘度 */
#define ENCODER_TRAVEL_HI ENCODER_ANGLE_HI                      /** 可主动转入角度上限，厘度 */
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
