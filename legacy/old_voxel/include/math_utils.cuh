#pragma once
//
// math_utils.cuh  -- __host__/__device__ helpers and run logs
//
#include <cstdint>
#include <cuda_runtime.h>

namespace Bocari
{
    namespace Cuda
    {

        using u32 = uint32_t;
        using u64 = uint64_t;

        // ───────────────────────────────────────────────────────────
        // Compile-time helpers
        // ───────────────────────────────────────────────────────────
        __host__ __device__ __forceinline__ constexpr u32 log2u(u32 n) noexcept
        {
            u32 bits = 0;
            while((1u << bits) < n) ++bits;
            return bits;
        }
        __host__ __device__ __forceinline__ constexpr u32 maskFor(u32 n) noexcept { return n - 1u; }

        // ───────────────────────────────────────────────────────────
        // Numeric helpers
        // ───────────────────────────────────────────────────────────
        __host__ __device__ __forceinline__ constexpr float  sqr(float x) noexcept { return x * x; }
        __host__ __device__ __forceinline__ constexpr double sqr(double x) noexcept { return x * x; }
        __host__ __device__ __forceinline__ constexpr u32    sqr(u32 x) noexcept { return x * x; }

        template <typename T> __host__ __device__ __forceinline__ constexpr T pow2(T x) noexcept { return x * x; }

        template <typename Vec3> __host__ __device__ __forceinline__ float length2(const Vec3& v) noexcept
        {
            return sqr(v.x) + sqr(v.y) + sqr(v.z);
        }

        // ───────────────────────────────────────────────────────────
        // Grid __constant__s bundle (keeps N, bits, mask consistent)
        // ───────────────────────────────────────────────────────────
        struct ConstsStruct
        {
            u32 N;    // cells per axis
            u32 bits; // log2(N)
            u32 mask; // N-1
        };

        template <u32 S> __host__ __device__ __forceinline__ constexpr ConstsStruct makeConsts() noexcept
        {
            return ConstsStruct{S, log2u(S), maskFor(S)};
        }

        // ───────────────────────────────────────────────────────────
        // Flatten / unflatten for regular 3D lattice with N^3 cells
        // ───────────────────────────────────────────────────────────
        template <u32 N> __host__ __device__ __forceinline__ u32 flatten3(u32 z, u32 y, u32 x) noexcept
        {
            constexpr u32 shift = log2u(N);
            return (z << (2 * shift)) | (y << shift) | x;
        }

        template <u32 N> __host__ __device__ __forceinline__ void unflatten3(u32 idx, u32& z, u32& y, u32& x) noexcept
        {
            constexpr u32 shift = log2u(N);
            constexpr u32 mask  = maskFor(N);
            x                   = idx & mask;
            y                   = (idx >> shift) & mask;
            z                   = (idx >> (2 * shift)) & mask;
        }

        // ───────────────────────────────────────────────────────────
        // GPU run logging
        // ───────────────────────────────────────────────────────────
        struct AxisCounts
        {
            u32 topBottom        = 0; // horizontals
            u32 northSouth       = 0; // vertical N-Z
            u32 eastWest         = 0; // vertical E-W
            u32 foldedTopBottom  = 0;
            u32 foldedNorthSouth = 0;
            u32 foldedEastWest   = 0;

            __host__ __device__ inline void reset()
            {
                topBottom = northSouth = eastWest = 0;
                foldedTopBottom = foldedNorthSouth = foldedEastWest = 0;
            }
        };

        struct GpuRunLog
        {
            // step flags
            bool frameWasIdeal   = false;
            bool planesDetected  = false;
            bool rotationApplied = false;
            bool downsampled     = false;
            bool completed       = false;

            AxisCounts planes;

            float  rotationAngleDeg = 0.0f;
            float3 rotationAxis     = {0.f, 0.f, 0.f};

            // counts for downsample result
            u32 downsampleOutCount = 0;

            __host__ __device__ inline void reset()
            {
                frameWasIdeal = planesDetected = rotationApplied = downsampled = completed = false;
                planes.reset();
                rotationAngleDeg   = 0.f;
                rotationAxis       = {0.f, 0.f, 0.f};
                downsampleOutCount = 0;
            }
        };

        // Simple quaternion container (w + xyz)
        struct Quat
        {
            float w{1.f}, x{0.f}, y{0.f}, z{0.f};
        };

    } // namespace Cuda
} // namespace Bocari
