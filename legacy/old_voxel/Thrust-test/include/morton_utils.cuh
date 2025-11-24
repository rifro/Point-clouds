#pragma once
//
// morton_utils.cuh -- LSB-first 3D Morton encode/decode + composite key
//
#include <cstdint>
#include "math_utils.cuh"

namespace Bocari { namespace Cuda {

// expand 10 bits into 30 with 2 zeros between
__host__ __device__ __forceinline__
uint32_t expandBitsBy2(uint32_t v) {
    v &= 0x000003ffu;                 // 10 bits
    v = (v | (v << 16)) & 0x30000ffu;
    v = (v | (v << 8))  & 0x300f00fu;
    v = (v | (v << 4))  & 0x30c30c3u;
    v = (v | (v << 2))  & 0x9249249u;
    return v;
}

// Interleave lower 21 bits safely (we'll usually pass fewer)
__host__ __device__ __forceinline__
uint64_t morton3D(uint32_t x, uint32_t y, uint32_t z) {
    // Use 21-bit safe method by expanding lower 21 bits in chunks of 10+11
    uint64_t xx = expandBitsBy2(x);
    uint64_t yy = expandBitsBy2(y);
    uint64_t zz = expandBitsBy2(z);
    return (zz << 2) | (yy << 1) | xx; // LSB-first triplets
}

// Deinterleave back (only for small bit counts used; for debugging/labels)
__host__ __device__ __forceinline__
uint32_t compactBitsBy2(uint32_t v) {
    v &= 0x9249249u;
    v = (v ^ (v >> 2)) & 0x30c30c3u;
    v = (v ^ (v >> 4)) & 0x300f00fu;
    v = (v ^ (v >> 8)) & 0x30000ffu;
    v = (v ^ (v >> 16))& 0x000003ffu;
    return v;
}

__host__ __device__ __forceinline__
void morton3D_decode(uint64_t code, uint32_t& x, uint32_t& y, uint32_t& z) {
    x = compactBitsBy2(static_cast<uint32_t>(code));
    y = compactBitsBy2(static_cast<uint32_t>(code >> 1));
    z = compactBitsBy2(static_cast<uint32_t>(code >> 2));
}

// Build composite key: high bits = voxelMorton, low bits = fineMorton
__host__ __device__ __forceinline__
uint64_t makeCompositeKey(uint32_t xi, uint32_t yi, uint32_t zi,
                          const ConstsStruct& voxel, const ConstsStruct& sub) {
    const uint32_t coarseX = xi >> sub.bits;
    const uint32_t coarseY = yi >> sub.bits;
    const uint32_t coarseZ = zi >> sub.bits;
    const uint32_t fineX   = xi & sub.mask;
    const uint32_t fineY   = yi & sub.mask;
    const uint32_t fineZ   = zi & sub.mask;

    const uint64_t voxelMorton = morton3D(coarseX, coarseY, coarseZ);
    const uint64_t fineMorton  = morton3D(fineX, fineY, fineZ);
    const uint32_t shift = 3u * sub.bits;
    return (voxelMorton << shift) | fineMorton;
}

}} // namespace Bocari::Cuda
