/* Production driver with one register-read hook, modeling the documented ADDR/ACK contract. */
#include <stdio.h>
#include "D_i2c.h"
I2C_TypeDef fake_i2c;
GPIO_TypeDef fake_gpio;
static int ack,pos,receiver,received,absent,irq_enabled=1;
static int ack_order_bad,critical_bad,failures;
static uint32_t bus_hz;
#define CHECK(x) do {if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);failures++;}}while(0)
void I2C_Init(I2C_TypeDef *p,I2C_InitTypeDef *c) {
    uint32_t divisor=c->I2C_DutyCycle==I2C_DutyCycle_16_9?25:3;
    uint32_t ccr=48000000/(divisor*c->I2C_ClockSpeed);
    bus_hz=48000000/(divisor*ccr);p->STAR1=p->STAR2=0;ack=c->I2C_Ack;pos=0;
}
void I2C_NACKPositionConfig(I2C_TypeDef *p,int v) {(void)p;pos=v;}
void I2C_Cmd(I2C_TypeDef *p,int v) {(void)p;(void)v;}
void I2C_AcknowledgeConfig(I2C_TypeDef *p,int v) {
    if(receiver && !v && pos && (p->STAR1&I2C_STAR1_ADDR))ack_order_bad++;
    if(receiver && !v && irq_enabled)critical_bad++;
    ack=v;
}
uint16_t test_read_star2(I2C_TypeDef *p) {
    if(p->STAR1&I2C_STAR1_ADDR) {
        if(receiver) {
            if(pos && !ack)p->STAR1=I2C_STAR1_RXNE; /* Premature NACK: no second byte/BTF. */
            else p->STAR1=I2C_STAR1_RXNE|I2C_STAR1_BTF;
        }else p->STAR1=0;
    }
    return p->STAR2;
}
void I2C_SoftwareResetCmd(I2C_TypeDef *p,int v) {if(v){p->STAR1=p->STAR2=0;receiver=0;}}
void I2C_GenerateSTART(I2C_TypeDef *p,int v) {(void)v;p->STAR1=I2C_STAR1_SB;p->STAR2=I2C_STAR2_BUSY;receiver=0;}
void I2C_GenerateSTOP(I2C_TypeDef *p,int v) {(void)v;p->STAR2=0;}
void I2C_Send7bitAddress(I2C_TypeDef *p,uint8_t a,int d) {(void)a;receiver=d;received=0;p->STAR1=absent?I2C_STAR1_AF:I2C_STAR1_ADDR;}
void I2C_SendData(I2C_TypeDef *p,uint8_t d) {(void)d;p->STAR1=I2C_STAR1_BTF;}
uint8_t I2C_ReceiveData(I2C_TypeDef *p) {(void)p;return (uint8_t)(0x19+received++);}
int I2C_GetFlagStatus(I2C_TypeDef *p,int f) {return f==I2C_FLAG_BUSY?(p->STAR2&I2C_STAR2_BUSY):1;}
int I2C_CheckEvent(I2C_TypeDef *p,int f) {(void)p;(void)f;return !absent;}
void GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *c) {(void)p;(void)c;}
int GPIO_ReadInputDataBit(GPIO_TypeDef *p,int pin) {(void)p;(void)pin;return 1;}
void RCC_PB1PeriphClockCmd(int a,int b) {(void)a;(void)b;}
void RCC_PB2PeriphClockCmd(int a,int b) {(void)a;(void)b;}
uint32_t __get_MSTATUS(void) {return (uint32_t)irq_enabled;}
void __set_MSTATUS(uint32_t v) {irq_enabled=(int)v;}
void __disable_irq(void) {irq_enabled=0;}
void __NOP(void) {}
int main(void) {
    uint8_t b[2];D_Bus_I2C_Init();printf("modeled bus frequency=%luHz\n",(unsigned long)bus_hz);CHECK(bus_hz==400000);
    CHECK(D_I2C_Read(0x48,0,b,2)==0);CHECK(received==2);CHECK(!ack_order_bad);CHECK(!critical_bad);CHECK(irq_enabled);
    CHECK(D_I2C_Read(0x48,0,b,1)==0);CHECK(received==1);
    CHECK(D_I2C_Read(0x48,0,b,2)==0);CHECK(received==2);CHECK(ack && !pos);
    absent=1;CHECK(D_I2C_Read(0x48,0,b,2)==1);CHECK(irq_enabled);
    absent=0;CHECK(D_I2C_Read(0x48,0,b,2)==0);CHECK(received==2);
    CHECK(D_I2C_Read(0x48,0,0,2)==1);CHECK(D_I2C_Read(0x48,0,b,0)==1);
    printf("I2C contract failures=%d\n",failures);return failures?1:0;
}
