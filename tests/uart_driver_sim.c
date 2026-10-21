/* Actual D_uart.c + C_Ring_Buf.c, with a minimal register/IRQ model.
 * Verifies software receive windows, not electrical bus arbitration. */
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct {uint32_t USART_BaudRate,USART_WordLength,USART_StopBits,USART_Parity,USART_HardwareFlowControl,USART_Mode;} USART_InitTypeDef;
typedef struct {uint32_t GPIO_Pin,GPIO_Speed,GPIO_Mode;} GPIO_InitTypeDef;
typedef struct {uint32_t NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd;} NVIC_InitTypeDef;
enum {DISABLE=0,RESET=0,ENABLE=1,USART_IT_RXNE=1,USART_IT_TXE=2,USART_IT_TC=4,USART_FLAG_RXNE=1,USART_FLAG_TC=4,
      USART_WordLength_8b=8,USART_StopBits_1=1,USART_Parity_No=0,USART_HardwareFlowControl_None=0,USART_Mode_Tx=1,USART_Mode_Rx=2,
      RCC_PB2Periph_GPIOC=1,RCC_PB2Periph_AFIO=2,RCC_PB2Periph_USART1=4,GPIO_PartialRemap3_USART1=0,
      GPIO_Pin_0=1,GPIO_Speed_30MHz=30,GPIO_Mode_AF_PP=1,GPIO_Mode_IN_FLOATING=2,USART1_IRQn=1};
#define USART1 ((void *)1)
#define GPIOC ((void *)2)
static unsigned irq_mask,flags,rx_data,send_count,receive_count;
static uint32_t __get_MSTATUS(void) {return 0;}
static void __disable_irq(void) {}
static void __set_MSTATUS(uint32_t n) {(void)n;}
static void USART_Cmd(void *u,int en) {(void)u;(void)en;}
static void USART_Init(void *u,USART_InitTypeDef *c) {(void)u;(void)c;}
static void USART_HalfDuplexCmd(void *u,int en) {(void)u;(void)en;}
static void USART_ITConfig(void *u,int bits,int en) {(void)u;if(en)irq_mask|=bits;else irq_mask&=~bits;}
static int USART_GetITStatus(void *u,int bits) {(void)u;return !!(flags&irq_mask&bits);}
static int USART_GetFlagStatus(void *u,int bits) {(void)u;return !!(flags&bits);}
static uint16_t USART_ReceiveData(void *u) {(void)u;receive_count++;flags&=~USART_FLAG_RXNE;return rx_data;}
static void USART_SendData(void *u,uint16_t data) {(void)u;send_count++;flags&=~USART_FLAG_TC;flags|=USART_IT_TXE;rx_data=data;flags|=USART_FLAG_RXNE;}
static void RCC_PB2PeriphClockCmd(unsigned c,int en) {(void)c;(void)en;}
static void GPIO_PinRemapConfig(unsigned c,int en) {(void)c;(void)en;}
static void GPIO_Init(void *g,GPIO_InitTypeDef *c) {(void)g;(void)c;}
static void NVIC_Init(NVIC_InitTypeDef *c) {(void)c;}
static void NVIC_EnableIRQ(int n) {(void)n;}
static void NVIC_DisableIRQ(int n) {(void)n;}
#include "../CH32V00X_APP_ADC_30kg/Common/C_Ring_Buf.c"
#include "../CH32V00X_APP_ADC_30kg/Drivers/D_uart.c"
static void receive_byte(uint8_t b) {rx_data=b;flags|=USART_FLAG_RXNE;D_UART1_ISR();}
static void finish_tx(void) {
    unsigned guard=300;
    while(irq_mask&USART_IT_TXE) {D_UART1_ISR();assert(--guard);}
    flags|=USART_FLAG_TC;D_UART1_ISR();
    assert(D_UART_Tx_Idle());
}
int main(void) {
    D_USART1_Cfg(115200);uint8_t b;
    receive_byte('#');assert(D_UART1_Rx_Get(&b)&&b=='#');
    uint8_t reply[]={0xaa,0x66,0,0x83,1,0,0,0x66,0xaa};
    assert(D_UART1_Tx_Write(reply,sizeof(reply)));
    assert(!(irq_mask&USART_IT_RXNE));
    receive_byte('#'); /* Host starts next command before reply completes. */
    assert(!D_UART1_Rx_Get(&b));finish_tx();assert(!D_UART1_Rx_Get(&b));
    receive_byte('#');assert(D_UART1_Rx_Get(&b)&&b=='#');
    printf("PASS confirmed: actual driver discards host byte during TX; receives after TC. 9-byte ACK window %.3fms at 115200, %.3fms at 9600.\n",90.*1000/115200,90.*1000/9600);
    D_USART1_Cfg(115200);flags=0;irq_mask=USART_IT_RXNE;
    for(unsigned i=0;i<200;i++) receive_byte((uint8_t)i);
    unsigned n=0;while(D_UART1_Rx_Get(&b)) n++;
    assert(n==127);printf("PASS confirmed: 200 incoming bytes without service retain %u, discard %u.\n",n,200-n);
    return 0;
}
