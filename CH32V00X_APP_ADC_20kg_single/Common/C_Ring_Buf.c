/* C_Ring_Buf.c 单生产者/单消费者环形缓冲区，无硬件依赖。 */

#include "C_Ring_Buf.h"

/* 初始化，头尾指针清零 */
void C_Ring_Buf_Init(RingBuf_t *rb)
{
    if (rb == 0) return;
    rb->head = 0;
    rb->tail = 0;
}

/* 写1字节，满时拒绝(不覆盖旧数据)，返回false */
bool C_Ring_Buf_Put(RingBuf_t *rb, uint8_t data)
{
    uint8_t next;

    if (rb == 0) return false;
    next = (uint8_t)(((uint16_t)rb->head + 1U) % RING_BUF_SIZE);

    /* 生产者只写head，不碰消费者拥有的tail */
    if (next == rb->tail) return false;

    rb->buf[rb->head] = data;
    rb->head = next;
    return true;
}

/* 读1字节，空时返回false */
bool C_Ring_Buf_Get(RingBuf_t *rb, uint8_t *data)
{
    if (rb == 0 || data == 0) return false;
    if (rb->head == rb->tail) return false;

    *data    = rb->buf[rb->tail];
    rb->tail = (uint8_t)(((uint16_t)rb->tail + 1U)
                       % RING_BUF_SIZE);
    return true;
}

/* 当前已存字节数 */
uint8_t C_Ring_Buf_Get_Count(RingBuf_t *rb)
{
    if (rb == 0) return 0;
    if (rb->head >= rb->tail)
        return (uint8_t)(rb->head - rb->tail);
    else
        return (uint8_t)((RING_BUF_SIZE - rb->tail) + rb->head);
}
