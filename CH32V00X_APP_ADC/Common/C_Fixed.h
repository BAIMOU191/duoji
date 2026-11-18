/* C_Fixed.h 定点小工具，各算法共用；本芯片只有硬件乘法、没有除法指令，此处不用除法。
 * 约定：Qn = 值×2^n 的整数；变量名带 _qN 后缀。定点数只在模块内部用，跨模块接口用整数物理单位(见 A_Config.h 第3节)。 */
#ifndef __C_FIXED_H__
#define __C_FIXED_H__

#include <stdint.h>

/* 常数除法 x/n(n≥2 为编译期常数)：-Os 下编译器会调库函数做软件除法，这里改成乘倒数取高32位(一条 mulhu)。
 * recip = FX_RECIP(n) = ceil(2^32/n)；x < 2^32/n 时与整除(向下取整)逐位相同，超出时最多大1。 */
#define FX_RECIP(n) ((uint32_t)((0x100000000ULL + (uint64_t)(n) - 1U) / (uint64_t)(n)))
static inline uint32_t Fx_UDivR(uint32_t x, uint32_t recip) { return (uint32_t)(((uint64_t)x * recip) >> 32); }
/* 有符号：按幅值整除再还原符号(向0取整)，与 C 的 / 一致 */
static inline int32_t Fx_DivR(int32_t x, uint32_t recip)
{
    uint32_t q = Fx_UDivR((uint32_t)((x < 0) ? -x : x), recip);
    return (x < 0) ? -(int32_t)q : (int32_t)q;
}

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

/* 对称死区：越过门限后只保留超出部分 */
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

/* 64位整数平方根，向下取整；超过32位时先右移，丢掉被移出的低位精度 */
static inline uint32_t Fx_Isqrt64(uint64_t x)
{
    uint8_t shift = 0U;

    while (x >> 32) { x >>= 2; shift++; }
    return Fx_Isqrt((uint32_t)x) << shift;
}

#endif /* __C_FIXED_H__ */
