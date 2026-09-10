#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct {volatile uint16_t STAR1,STAR2;} I2C_TypeDef;
typedef struct {volatile uint32_t BSHR,BCR;} GPIO_TypeDef;
typedef int GPIOMode_TypeDef;
typedef struct {uint32_t GPIO_Pin,GPIO_Mode,GPIO_Speed;} GPIO_InitTypeDef;
typedef struct {uint32_t I2C_ClockSpeed,I2C_Mode,I2C_DutyCycle,I2C_OwnAddress1,I2C_Ack,I2C_AcknowledgedAddress;} I2C_InitTypeDef;
extern I2C_TypeDef fake_i2c;
extern GPIO_TypeDef fake_gpio;
#define I2C1 (&fake_i2c)
#define GPIOC (&fake_gpio)
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define Bit_RESET 0
#define I2C_STAR1_BERR 256
#define I2C_STAR1_ARLO 512
#define I2C_STAR1_AF 1024
#define I2C_STAR1_OVR 2048
#define I2C_STAR1_SB 1
#define I2C_STAR1_ADDR 2
#define I2C_STAR1_BTF 4
#define I2C_STAR1_RXNE 64
#define I2C_STAR2_BUSY 2
#define I2C_FLAG_BUSY 2
#define I2C_FLAG_TXE 128
#define GPIO_Pin_1 2
#define GPIO_Pin_2 4
#define GPIO_Speed_30MHz 3
#define GPIO_Mode_Out_OD 1
#define GPIO_Mode_AF_OD 2
#define RCC_PB2Periph_GPIOC 1
#define RCC_PB2Periph_AFIO 2
#define RCC_PB1Periph_I2C1 1
#define I2C_Mode_I2C 0
#define I2C_DutyCycle_16_9 1
#define I2C_DutyCycle_2 0
#define I2C_Ack_Enable 1
#define I2C_AcknowledgedAddress_7bit 0
#define I2C_NACKPosition_Current 0
#define I2C_NACKPosition_Next 1
#define I2C_Direction_Transmitter 0
#define I2C_Direction_Receiver 1
#define I2C_EVENT_MASTER_MODE_SELECT 1
#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 2
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED 4
void I2C_Init(I2C_TypeDef*,I2C_InitTypeDef*);
void I2C_NACKPositionConfig(I2C_TypeDef*,int);
void I2C_Cmd(I2C_TypeDef*,int);
void I2C_AcknowledgeConfig(I2C_TypeDef*,int);
void I2C_SoftwareResetCmd(I2C_TypeDef*,int);
void I2C_GenerateSTART(I2C_TypeDef*,int);
void I2C_GenerateSTOP(I2C_TypeDef*,int);
void I2C_Send7bitAddress(I2C_TypeDef*,uint8_t,int);
void I2C_SendData(I2C_TypeDef*,uint8_t);
uint8_t I2C_ReceiveData(I2C_TypeDef*);
int I2C_GetFlagStatus(I2C_TypeDef*,int);
int I2C_CheckEvent(I2C_TypeDef*,int);
void GPIO_Init(GPIO_TypeDef*,GPIO_InitTypeDef*);
int GPIO_ReadInputDataBit(GPIO_TypeDef*,int);
void RCC_PB1PeriphClockCmd(int,int);
void RCC_PB2PeriphClockCmd(int,int);
uint32_t __get_MSTATUS(void);
void __set_MSTATUS(uint32_t);
void __disable_irq(void);
void __NOP(void);
uint16_t test_read_star2(I2C_TypeDef*);
