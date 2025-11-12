#pragma once
#include "includes.h"
#include "point.h"
#include "strict.h"

namespace Bocari
{
    namespace Voxelprocessing
    {
        struct RangeData;

        // GridUnion definition. Provides both 3D [_3D[z][y][x]] and flat _1D[] access
        template <typename T, u32 N> union GridUnionT
        {
            static constexpr u32 SIZE = Math::pow3(N);

            T _3D[N][N][N];
            T _1D[SIZE + 1]; // One beyond end

            GridUnionT()
            {
                // Initialiseer laatste element van _1D array op 0/false, omdat dit ongeldig is voor counts.
                _1D[SIZE] = T{};
            }

            void initialize(const T& value = T{})
            {
                for(u32 i = 0; i <= SIZE; ++i) _1D[i] = value;
            }

            T& operator()(u32 z, u32 y, u32 x)
            {
                assert(z < N && y < N && x < N && "3D bounds exceeded");
                return _3D[z][y][x];
            }

            const T& operator()(u32 z, u32 y, u32 x) const
            {
                assert(z < N && y < N && x < N && "3D bounds exceeded");
                return _3D[z][y][x];
            }

            T& operator[](u32 index)
            {
                assert(index < SIZE + 1 && "1D bounds exceeded");
                return _1D[index];
            }

            const T& operator[](u32 index) const
            {
                assert(index < SIZE + 1 && "1D bounds exceeded");
                return _1D[index];
            }
        };

        // Alias for backward compatibility
        template <u32 N> using GridUnion = GridUnionT<u32, N>;

        constexpr u32 log2(u32 n)
        {
            u32 bits = 0;
            while((1 << bits) < n) ++bits;
            return bits;
            // return n <= 1 ? 0 : 1 + computeMadRangeprocessing::log2(n >> 1);
        }

        constexpr u32 maskFor(u32 n) { return n - 1u; }

        template <u32 N> inline u32 flatten3(u32 z, u32 y, u32 x)
        {
            constexpr u32 shift = Voxelprocessing::log2(N);
            return (U32(z) << (2 * shift)) | U32((y) << shift) | U32(x);
        }

        template <u32 N> inline void unflatten3(u32 index, u32& z, u32& y, u32& x)
        {
            constexpr u32 shift = Voxelprocessing::log2(N);
            constexpr u32 mask  = maskFor(N);
            x                   = index & mask;
            y                   = (index >> shift) & mask;
            z                   = (index >> (2 * shift)) & mask;
        }

        template <u32 Bins, typename GroupIndexFunc>
        void countIndices(const std::vector<Pointcloud::Point>& points, u32 begin, u32 end, std::span<u32> counts,
                          GroupIndexFunc groupOf)
        {
            static_assert(Bins >= Math::pow3(32));
            for(u32 i = begin; i < end; ++i)
            {
                int32_t bin = groupOf(points[i]);
                if(bin < 0 || U32(bin) >= Bins) bin = 0;
                counts[U32(bin)]++;
            }
        }

        template <u32 N>
        void buildStartIndices(const std::span<u32> counts, Voxelprocessing::GridUnion<N>& starts, u32 begin)
        {
            static_assert(N <= 64);
            constexpr u32 Bins = Math::pow3(32);
            u32           sum  = begin;
            for(u32 i = 0; i < Bins; ++i)
            {
                starts._1D[i] = sum;
                sum += counts[i];
            }
            starts._1D[Bins] = sum;
        }

        template <u32 N, typename GroupIndexFunc>
        void countingSort(std::vector<Pointcloud::Point>& points, GroupIndexFunc groupOf,
                          Voxelprocessing::GridUnion<N>& starts)
        {
            static_assert(N <= 64);
            constexpr u32         Bins = Math::pow3(N);
            std::array<u32, Bins> counts{};

            countIndices<Bins>(points, 0, U32(points.size()), std::span{counts}, groupOf);
            buildStartIndices<N>(std::span{counts}, starts, 0);

            // Stable reorder
            std::vector<Pointcloud::Point> tmp;
            tmp.resize(points.size());

            std::array<u32, Bins + 1> writePos{};
            std::copy(&starts._1D[0], &starts._1D[Bins + 1], writePos.begin());

            for(const auto& p : points)
            {
                u32 bin = groupOf(p);
                if(bin >= Bins) bin = Bins - 1;
                u32 dest  = writePos[bin]++;
                tmp[dest] = p;
            }

            points.swap(tmp);
        }

        template <u32 N> void buildCumSum(const GridUnion<N>& counts, GridUnion<N>& cumSum)
        {
            cumSum.initialize(0);
            for(u32 z = 0; z < N; ++z)
            {
                for(u32 y = 0; y < N; ++y)
                {
                    for(u32 x = 0; x < N; ++x)
                    {
                        int32_t v = counts(z, y, x);
                        if(z > 0) v += cumSum(z - 1, y, x);
                        if(y > 0) v += cumSum(z, y - 1, x);
                        if(x > 0) v += cumSum(z, y, x - 1);
                        if(z > 0 && y > 0) v -= cumSum(z - 1, y - 1, x);
                        if(z > 0 && x > 0) v -= cumSum(z - 1, y, x - 1);
                        if(y > 0 && x > 0) v -= cumSum(z, y - 1, x - 1);
                        if(z > 0 && y > 0 && x > 0) v += cumSum(z - 1, y - 1, x - 1);
                        cumSum(z, y, x) = v;
                    }
                }
            }
        }

        template <u32 N>
        u32 countPointsInBox(const GridUnion<N>& cumSum, u32 x1, u32 y1, u32 z1, u32 x2, u32 y2, u32 z2)
        {
            if(x1 > x2) std::swap(x1, x2);
            if(y1 > y2) std::swap(y1, y2);
            if(z1 > z2) std::swap(z1, z2);
            auto constexpr mask = maskFor(N);
            x1 &= mask;
            x2 &= mask;
            y1 &= mask;
            y2 &= mask;
            z1 &= mask;
            z2 &= mask;
            if(x1 > x2 || y1 > y2 || z1 > z2) return 0;
            auto get = [&](u32 x, u32 y, u32 z) -> u32 { return cumSum(z, y, x); };
            u32  A   = get(x2, y2, z2);
            u32  B   = (x1 > 0) ? get(x1 - 1, y2, z2) : 0;
            u32  C   = (y1 > 0) ? get(x2, y1 - 1, z2) : 0;
            u32  D   = (z1 > 0) ? get(x2, y2, z1 - 1) : 0;
            u32  E   = (x1 > 0 && y1 > 0) ? get(x1 - 1, y1 - 1, z2) : 0;
            u32  F   = (x1 > 0 && z1 > 0) ? get(x1 - 1, y2, z1 - 1) : 0;
            u32  G   = (y1 > 0 && z1 > 0) ? get(x2, y1 - 1, z1 - 1) : 0;
            u32  H   = (x1 > 0 && y1 > 0 && z1 > 0) ? get(x1 - 1, y1 - 1, z1 - 1) : 0;
            return A - B - C - D + E + F + G - H;
        }
    } // namespace Voxelprocessing
} // namespace Bocari
