#pragma once
#include "types.cuh"
#include <stdint.h>

// --------------------------------------------------------------
// CONFIG
// --------------------------------------------------------------

#ifndef REGIO_PREFIX_BITS
#define REGIO_PREFIX_BITS 12 // of jouw waarde
#endif

// Max bits voor 3D morton: 3 × 21 = 63 bits → genoeg.
// De regio-prefix pakt de top REGIO_PREFIX_BITS bits.
static constexpr uint64_t MORTON_MASK = (~0ULL >> REGIO_PREFIX_BITS);

// --------------------------------------------------------------
// Bit interleave helpers (21 bits → 63 bits)
// --------------------------------------------------------------

__host__ __device__ inline uint64_t part1by2(uint64_t x)
{
    x &= 0x1fffffULL; // 21 bits
    x = (x | (x << 32)) & 0x1f00000000ffffULL;
    x = (x | (x << 16)) & 0x1f0000ff0000ffULL;
    x = (x | (x << 8)) & 0x100f00f00f00f00fULL;
    x = (x | (x << 4)) & 0x10c30c30c30c30c3ULL;
    x = (x | (x << 2)) & 0x1249249249249249ULL;
    return x;
}

__host__ __device__ inline uint64_t morton3D(uint32_t x, uint32_t y, uint32_t z)
{
    return part1by2(x) | (part1by2(y) << 1) | (part1by2(z) << 2);
}

// --------------------------------------------------------------
// Kwantisatie-struct
// --------------------------------------------------------------

struct Kwantisatie
{
    float3 bbMin;
};

// Clamp naar 32-bit (voor veiligheid)
__host__ __device__ inline uint32_t clampu32(uint64_t v) { return (v > 0xffffffffULL) ? 0xffffffffu : (uint32_t)v; }

// --------------------------------------------------------------
// Float3 → kwantisatie naar integer 0..2^21
// --------------------------------------------------------------
__host__ __device__ inline void kwantiseer_punt(const Vec3f& p, const Kwantisatie& Q, uint32_t& xi, uint32_t& yi,
                                                uint32_t& zi)
{
    float X = (p.x - Q.bbMin.x) * KWANTISATIE_SCHAAL;
    float Y = (p.y - Q.bbMin.y) * KWANTISATIE_SCHAAL;
    float Z = (p.z - Q.bbMin.z) * KWANTISATIE_SCHAAL;

    auto quant = [](float v) -> uint32_t {
        float t = v + 0.5f; // round-to-nearest (simpel, snel)
        if(t < 0.f) t = 0.f;
        uint64_t w = (uint64_t)t; // floor
        return (w > 0xffffffffULL ? 0xffffffffu : (uint32_t)w);
    };

    xi = quant(X);
    yi = quant(Y);
    zi = quant(Z);
}

// --------------------------------------------------------------
// Combine morton + regio-prefix
// --------------------------------------------------------------

__host__ __device__ inline uint64_t morton_met_regio(uint32_t xi, uint32_t yi, uint32_t zi, uint32_t regioCode)
{
    uint64_t m = morton3D(xi, yi, zi);
    return (((uint64_t)regioCode) << (64 - REGIO_PREFIX_BITS)) | (m & MORTON_MASK);
}
