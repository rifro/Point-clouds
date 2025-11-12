#pragma once
#include "grid_utils.h"
#include "includes.h"
#include "point.h"
#include "strict.h"

// Debug macro
// clang-format off
#define _DEBUG
#define VOXEL_DEBUG

#ifdef VOXEL_DEBUG
   #define VOXEL_DBG(x) do { x; } while(0)
#else
   #define VOXEL_DBG(x) do { } while(0)
#endif
// clang-format on

namespace Bocari
{
    namespace Math
    {
        using Vec3f = Vec3T<float>;
        using Vec3d = Vec3T<double>;
    } // namespace Math
} // namespace Bocari

namespace Bocari
{
    namespace Voxelprocessing
    {
        // Constants
        constexpr u32    c_radix                = 1024u;             // 10 bits per component for radix sort
        constexpr u32    c_voxel                = 32u;               // top-level grid dimension (5 bits: 2^5=32)
        constexpr u32    c_subvoxel             = 32u;               // sub-voxel grid dimension (5 bits: 2^5=32)
        constexpr u32    c_bins_per_group       = c_radix / c_voxel; // Number of bins per group (32)
        constexpr double c_slice_to_voxel_scale = static_cast<double>(c_radix / c_voxel); // 1024/32 = 32.0

        // DensityRanges structure
        struct MediumVoxelRange
        {
            u32 minCount;
            u32 maxCount;

            // Extra stats for debugging/auditing:
            double p25_value;
            double p50_value;
            double p75_value;
        };

        class VoxelManager;
        class Voxel;
        class SubVoxel;

        struct OnlinePCA
        {
            int32_t        n = 0;
            Math::Vec3d    mean{0.0, 0.0, 0.0};
            Math::Matrix3d M2; // covariance accumulator

            void           reset();
            void           update(const Math::Vec3d& x);
            Math::Matrix3d covariance() const;
        };

        class Voxel
        {
        public:
            Voxel() = default;
            Voxel(u32 index_1D, const VoxelManager* manager, Voxelprocessing::GridUnion<c_subvoxel>& subVoxelStarts);

            u32      start() const;
            u32      count() const;
            void     getIndices(u32& x, u32& y, u32& z) const;
            void     ensureSubvoxelStartsComputed() const;
            SubVoxel getSubVoxel(u32 x, u32 y, u32 z) const;
            void     buildSubVoxelCumSum() const;
            bool     hasSubVoxelCumSum() const { return m_subvoxelCumSumComputed; }

            bool detectPlane();

        private:
            u32           m_index_1D = 0;
            u32           m_x = 0u, m_y = 0u, m_z = 0u;
            VoxelManager* m_manager                = nullptr;
            mutable bool  m_subvoxelStartsComputed = false;
            mutable bool  m_subvoxelCumSumComputed = false;

        public:
            mutable Voxelprocessing::GridUnion<c_subvoxel>* m_subvoxelStarts = nullptr;
            mutable Voxelprocessing::GridUnion<c_subvoxel>  m_subvoxelCumSum;

            friend class VoxelManager;
        };

        struct RangeData
        {
            Math::Vec3d range;
            Math::Vec3d invRange;

            void computeVoxelSizeData(const Math::Vec3d& bbMax, u32 c)
            {
                double f = 1.0 / static_cast<double>(c);
                range    = bbMax * f;
                invRange = {1.0 / range.x, 1.0 / range.y, 1.0 / range.z};
            }
        };

        class SubVoxel
        {
        public:
            SubVoxel() = default;
            SubVoxel(const Voxel& voxel, u32 x, u32 y, u32 z);
            SubVoxel(u32 index_1D, u32 start, u32 count, const Math::Vec3d& bbMin);

            void getIndices(u32& x, u32& y, u32& z) const
            {
                x = m_x;
                y = m_y;
                z = m_z;
            }

            u32 getIndex_1D() const { return m_index_1D; }

            u32                start() const { return m_start; }
            u32                count() const { return m_count; }
            const Math::Vec3d& bbMin() const { return m_bbMin; }

        private:
            u32         m_index_1D = 0u;
            u32         m_x = 0u, m_y = 0u, m_z = 0u;
            u32         m_start = 0u;
            u32         m_count = 0u;
            Math::Vec3d m_bbMin{0.0, 0.0, 0.0};
        };

        struct NormalizationResult
        {
            double      scale;
            bool        needsScaling;
            Math::Vec3d bbMinOrg;
            Math::Vec3d bbMaxTranslatedScaled;
        };

        class VoxelManager
        {
        public:
            VoxelManager();
            NormalizationResult computeNormalizationParameters(ccPointCloud* cloud);
            void                voxelize(ccPointCloud* cloud);
            void                getVoxel(u32 x, u32 y, u32 z, Voxel& out,
                                         Voxelprocessing::GridUnion<c_subvoxel>& subVoxelStarts) const;
            void                getVoxelById(u32 flatIndex, Voxel& out, GridUnion<c_subvoxel>& subVoxelStarts) const;
            bool                isMediumDensityVoxel(u32 index) const;
            u32                 countPointsInVoxelBox(const Voxel& voxel1, const Voxel& voxel2) const;
            u32                 countPointsInSubVoxelBox(const Voxel& voxel, const SubVoxel& subvoxel1,
                                                         const SubVoxel& subvoxel2) const;

            void buildVoxelCumSum() const;
            bool hasVoxelCumSum() const { return m_voxelCumSumComputed; }
            void initMediumVoxel();

            // Public members for direct access
            Voxelprocessing::NormalizationResult        m_normResult{1.0, false, Math::Vec3d{0.0, 0.0, 0.0},
                                                              Math::Vec3d{0.0, 0.0, 0.0}};
            Voxelprocessing::RangeData                  m_sliceRange;
            mutable Voxelprocessing::GridUnion<c_voxel> m_voxelCounts;
            mutable Voxelprocessing::GridUnion<c_voxel> m_voxelStarts;
            mutable Voxelprocessing::GridUnion<c_voxel> m_voxelCumSum;
            std::vector<Pointcloud::Point>              groupedPoints;
            MediumVoxelRange                            m_mediumVoxelRange; // Medium density.
            static VoxelManager*                        s_pInstance;

        private:
            const double maxAllowedSize       = 1000.0;
            const double maxAllowedScaledSize = 100.0;

            void computeVoxelMediumRange();

        private:
            mutable bool m_voxelStartsComputed = false;
            mutable bool m_voxelCumSumComputed = false;
        };

        double normalizePointCloud(ccPointCloud* cloud, Math::Vec3d& bbMin, Math::Vec3d& bbMax);

    } // namespace Voxelprocessing
} // namespace Bocari
