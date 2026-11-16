#ifndef __C_RING_BUF_H__
#define __C_RING_BUF_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * C_Ring_Buf —— 单生产者/单消费者无锁环形缓冲区。
 * 只写head/只写tail，单字节原子，ISR与任务间无需临界区；满时整帧拒绝不截断。
 */

#define RING_BUF_SIZE  128  /* 容量(字节)，须能被uint8_t下标绕回 */

typedef struct {
    uint8_t          buf[RING_BUF_SIZE]; /* 数据区 */
    volatile uint8_t head;               /* 写指针，只由生产者改 */
    volatile uint8_t tail;               /* 读指针，只由消费者改 */
} RingBuf_t;

void    C_Ring_Buf_Init(RingBuf_t *rb);                /* 清空 */
bool    C_Ring_Buf_Put(RingBuf_t *rb, uint8_t data);   /* 写1字节，满返回false */
bool    C_Ring_Buf_Get(RingBuf_t *rb, uint8_t *data);  /* 读1字节，空返回false */
uint8_t C_Ring_Buf_Get_Count(RingBuf_t *rb);           /* 当前字节数 */

#endif /* __C_RING_BUF_H__ */
