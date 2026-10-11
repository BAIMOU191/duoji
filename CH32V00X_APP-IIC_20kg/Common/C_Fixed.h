/* C_Fixed.h 定点小工具，各算法文件共用。本芯片没有除法指令，这里都不用除法 */
#ifndef __C_FIXED_H__
#define __C_FIXED_H__

#include <stdint.h>

static inline int32_t Fx_Abs32(int32_t v) { return (v < 0) ? -v : v; }

/* 圆周差折到最短路径，每拍位移须小于半圈；range 可为 Q8 量程。 */
static inline int32_t Fx_WrapDiff(int32_t d, int32_t range)
{
    if (d > range / 2) d -= range;
    if (d < -range / 2) d += range;
    return d;
}

/* 对称限幅到 ±lim */
static inline int32_t Fx_Clamp32(int32_t v, int32_t lim)
{
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

static inline int32_t Fx_ClampRange(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* 对称死区：越过门限后只保留超出部分。 */
static inline int32_t Fx_Deadband(int32_t v, int32_t db)
{
    return (v > db) ? v - db : ((v < -db) ? v + db : 0);
}

/* floor(num * 2^15 / den)，要求 0 <= num < den < 2^16。商只有15位，逐位试商15轮，与除法逐位相同 */
static inline int32_t Fx_RatioQ15(uint32_t num, uint32_t den)
{
    uint32_t q = 0U;
    uint8_t  i;

    for (i = 0U; i < 15U; i++)
    {
        num <<= 1;
        q   <<= 1;
        if (num >= den) { num -= den; q |= 1U; }
    }
    return (int32_t)q;
}

/* 整数平方根，向下取整 */
static inline uint32_t Fx_Isqrt(uint32_t x)
{
    uint32_t r = 0U, bit = 1UL << 30;

    while (bit > x) bit >>= 2;
    while (bit)
    {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

#endif /* __C_FIXED_H__ */
