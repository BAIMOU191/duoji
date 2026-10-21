/* C_Fixed.h 定点小工具，各算法文件共用。本芯片没有除法指令，这里都不用除法 */
#ifndef __C_FIXED_H__
#define __C_FIXED_H__

#include <stdint.h>

static inline int32_t Fx_Abs32(int32_t v) { return (v < 0) ? -v : v; }
static inline int32_t Fx_Sign32(int32_t v) { return (v > 0) - (v < 0); }

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

/* (a*b) >> s，按幅值四舍五入，正负对称；s>=1 */
static inline int32_t Fx_MulShift(int32_t a, int32_t b, uint8_t s)
{
    int64_t p = (int64_t)a * b;
    int64_t m = ((p < 0 ? -p : p) + (1LL << (s - 1U))) >> s;
    return (int32_t)(p < 0 ? -m : m);
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

/* 64位整数平方根：先按4的幂缩进32位再开方，结果最多丢掉被移出的低位精度 */
static inline uint32_t Fx_Isqrt64(uint64_t x)
{
    uint8_t shift = 0U;

    while (x >> 32) { x >>= 2; shift++; }
    return Fx_Isqrt((uint32_t)x) << shift;
}

#endif /* __C_FIXED_H__ */
