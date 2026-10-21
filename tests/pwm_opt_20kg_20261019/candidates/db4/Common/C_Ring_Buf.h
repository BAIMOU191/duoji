#ifndef __C_RING_BUF_H__
#define __C_RING_BUF_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * C_Ring_Buf —— 单生产者/单消费者无锁环形缓冲区
 * 生产者只写head、消费者只写tail，单字节读写原子，ISR与任务间无需临界区；多生产者须自行互斥。
 * 满时拒绝新字节，协议帧宁可整帧丢失也不被截断。
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
