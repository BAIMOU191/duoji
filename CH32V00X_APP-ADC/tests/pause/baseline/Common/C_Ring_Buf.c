/* C_Ring_Buf.c 单生产者/单消费者环形缓冲区，无硬件依赖。 */

#include "C_Ring_Buf.h"

/*
 * @fn      C_Ring_Buf_Init
 * @brief   初始化缓冲区，头尾指针清零
 * @param   rb 缓冲区实例
 * @return  无
 */
void C_Ring_Buf_Init(RingBuf_t *rb)
{
    if (rb == 0) return;
    rb->head = 0;
    rb->tail = 0;
}

/*
 * @fn      C_Ring_Buf_Put
 * @brief   写入1字节，缓冲区满时拒绝写入
 * @param   rb   缓冲区实例
 * @param   data 待写入字节
 * @return  true=写入成功，false=参数无效或缓冲区已满
 */
bool C_Ring_Buf_Put(RingBuf_t *rb, uint8_t data)
{
    uint8_t next;

    if (rb == 0) return false;
    next = (uint8_t)(((uint16_t)rb->head + 1U) % RING_BUF_SIZE);

    /* ISR生产者不能改任务消费者拥有的tail，否则两端会发生读写竞争。 */
    if (next == rb->tail) return false;

    rb->buf[rb->head] = data;
    rb->head = next;
    return true;
}

/*
 * @fn      C_Ring_Buf_Get
 * @brief   读出1字节，缓冲区空时返回false
 * @param   rb   缓冲区实例
 * @param   data 输出数据指针
 * @return  true=成功，false=缓冲区空
 */
bool C_Ring_Buf_Get(RingBuf_t *rb, uint8_t *data)
{
    if (rb == 0 || data == 0) return false;
    if (rb->head == rb->tail) return false;

    *data    = rb->buf[rb->tail];
    rb->tail = (uint8_t)(((uint16_t)rb->tail + 1U)
                       % RING_BUF_SIZE);
    return true;
}

/*
 * @fn      C_Ring_Buf_Get_Count
 * @brief   获取当前已存字节数
 * @param   rb 缓冲区实例
 * @return  已存字节数
 */
uint8_t C_Ring_Buf_Get_Count(RingBuf_t *rb)
{
    if (rb == 0) return 0;
    if (rb->head >= rb->tail)
        return (uint8_t)(rb->head - rb->tail);
    else
        return (uint8_t)((RING_BUF_SIZE - rb->tail) + rb->head);
}
