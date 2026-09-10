#ifndef __C_RING_BUF_H__
#define __C_RING_BUF_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * C_Ring_Buf —— 单生产者/单消费者环形缓冲区，无硬件依赖
 *
 * 无锁的前提是**严格一对一**：生产者只写head，消费者只写tail，两边各自
 * 只读对方的指针。head/tail都是uint8_t，RISC-V上单字节读写天然原子，
 * 所以ISR和任务之间不需要关中断也不需要临界区。
 *
 * 一旦出现两个生产者(或两个消费者)，这个前提就没了，必须自己加互斥。
 * 缓冲区满时拒绝新字节而不是覆盖旧数据：协议帧宁可整帧丢失，也不能
 * 被截断成一条语义错误的帧。
 */

#define RING_BUF_SIZE  128  /* 容量(字节)，必须能被uint8_t下标绕回覆盖 */

typedef struct {
    uint8_t          buf[RING_BUF_SIZE]; /* 数据区                       */
    volatile uint8_t head;               /* 写指针，只由生产者修改       */
    volatile uint8_t tail;               /* 读指针，只由消费者修改       */
} RingBuf_t;

void    C_Ring_Buf_Init(RingBuf_t *rb);                /* 清空缓冲区                   */
bool    C_Ring_Buf_Put(RingBuf_t *rb, uint8_t data);   /* 写1字节，满时返回false       */
bool    C_Ring_Buf_Get(RingBuf_t *rb, uint8_t *data);  /* 读1字节，空时返回false       */
uint8_t C_Ring_Buf_Get_Count(RingBuf_t *rb);           /* 当前已存字节数               */

#endif /* __C_RING_BUF_H__ */
