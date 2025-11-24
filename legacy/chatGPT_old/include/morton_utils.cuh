#pragma once
#include "config.h"
#include "types.h"
#include <cstdint>

// 21 bits per as in 64-bit (theoretisch), we reserveren bovenste REGIO_PREFIX_BITS voor regio
// Layout: [ regio R bits ][ morton XYZ (interleave LSB-first) ]
// Bits per as (effectief): bepaal uit VOXEL_MIDDEN_BITS en beschikbare restbits.

__host__ __device__ inline u64 part1by2(u64 x)
{
    x &= 0x1fffffULL; // 21 bits
    x = (x | (x << 32)) & 0x1f00000000ffffULL;
    x = (x | (x << 16)) & 0x1f0000ff0000ffULL;
    x = (x | (x << 8)) & 0x100f00f00f00f00fULL;
    x = (x | (x << 4)) & 0x10c30c30c30c30c3ULL;
    x = (x | (x << 2)) & 0x1249249249249249ULL;
    return x;
}

__host__ __device__ inline u64 morton3D(u32 x, u32 y, u32 z)
{
    return part1by2(x) | (part1by2(y) << 1) | (part1by2(z) << 2);
}

struct Kwantisatie
{
    Vec3f bbMin;
    float schaal; // meters → ints (globaal, aspect behouden)
};

__host__ __device__ inline u32 clampu32(u64 v) { return (v > 0xffffffffULL) ? 0xffffffffu : (u32)v; }

// Maak morton sleutel met regio-prefix (R MSB’s)
__host__ __device__ inline u64 morton_met_regio(u32 xi, u32 yi, u32 zi, u32 regioCode)
{
    u64 m = morton3D(xi, yi, zi);
    return (((u64)regioCode) << (64 - REGIO_PREFIX_BITS)) | (m & (~0ULL >> REGIO_PREFIX_BITS));
}

// Eenvoudige kwantisatie float3 → ints (assume alles al naar [0,MAX_RANGE_METERS))
__host__ __device__ inline void kwantiseer_punt(const Vec3f& p, const Kwantisatie& Q, u32& xi, u32& yi, u32& zi)
{
    float X = (p.x - Q.bbMin.x) * Q.schaal;
    float Y = (p.y - Q.bbMin.y) * Q.schaal;
    float Z = (p.z - Q.bbMin.z) * Q.schaal;
    // round-to-nearest-even kan via lrintf, hier JBF: +0.5f en floor
    auto q = [](float v) -> u32 {
        float t = v + 0.5f;
        if(t < 0.f) t = 0.f;
        u64 w = (u64)(t);
        return clampu32(w);
    };
    xi = q(X);
    yi = q(Y);
    zi = q(Z);
}