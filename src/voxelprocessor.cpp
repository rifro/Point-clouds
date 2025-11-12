#include "voxelprocessor.h"
#include <random>

namespace Bocari
{
    namespace Voxelprocessing
    {
        inline u32 computeIndexComponent(double coord, double inv)
        {
            constexpr double eps  = 1e-4;
            constexpr u32    mask = maskFor(Voxelprocessing::c_radix);
            double           v    = coord * inv;
            assert(v > -eps);
            assert(v - Voxelprocessing::c_radix < eps); // Edge case could land in bin 0, but OK.
            if(v < 0.0) v = 0.0;
            return U32(v) & mask;
        }

        VoxelManager* VoxelManager::s_pInstance = nullptr;

        // --- Voxel Implementation ---
        Voxel::Voxel(u32 index_1D, const VoxelManager* manager, Voxelprocessing::GridUnion<c_subvoxel>& subVoxelStarts)
            : m_index_1D(index_1D), m_manager(const_cast<VoxelManager*>(manager)), m_subvoxelStarts(&subVoxelStarts)
        {
            Voxelprocessing::unflatten3<c_voxel>(m_index_1D, m_z, m_y, m_x);
        }

        u32 Voxel::start() const
        {
            assert(m_manager != nullptr);
            return m_manager->m_voxelStarts._1D[m_index_1D];
        }

        u32 Voxel::count() const
        {
            assert(m_manager != nullptr);
            assert(m_manager->m_voxelStarts._1D[m_index_1D + 1] >= m_manager->m_voxelStarts._1D[m_index_1D]);
            return m_manager->m_voxelStarts._1D[m_index_1D + 1] - m_manager->m_voxelStarts._1D[m_index_1D];
        }

        void Voxel::getIndices(u32& x, u32& y, u32& z) const
        {
            x = m_x;
            y = m_y;
            z = m_z;
        }

        void Voxel::ensureSubvoxelStartsComputed() const
        {
            if(m_subvoxelStartsComputed) return;

            u32 b = start();
            u32 e = b + count();
            if(b >= e)
            {
                m_subvoxelStarts->initialize(e);
                m_subvoxelStartsComputed = true;
                return;
            }

            const Voxelprocessing::RangeData& sliceRangeData = m_manager->m_sliceRange;

            auto getSubvoxelIndex = [&](const Pointcloud::Point& p) {
                u32 x = computeIndexComponent(p.m_point.x, sliceRangeData.invRange.x);
                u32 y = computeIndexComponent(p.m_point.y, sliceRangeData.invRange.y);
                u32 z = computeIndexComponent(p.m_point.z, sliceRangeData.invRange.z);

                // Shift out LSB bits for subvoxels, so we keep MSB bits for voxel
                constexpr u32 shift = Voxelprocessing::log2(c_subvoxel);
                assert((x >> shift) == m_x);
                assert((y >> shift) == m_y);
                assert((z >> shift) == m_z);

                // LSB bits give subvoxel coordinate within parent voxel
                constexpr u32 mask = maskFor(c_subvoxel);
                return flatten3<c_subvoxel>(z & mask, y & mask, x & mask);
            };

            constexpr u32         Bins = Math::pow3(c_subvoxel);
            std::array<u32, Bins> counts{};
            countIndices<Bins>(m_manager->groupedPoints, b, e, counts, getSubvoxelIndex);
            buildStartIndices<c_subvoxel>(counts, *m_subvoxelStarts, b);

            VOXEL_DBG({
                for(u32 i = b; i < e; ++i)
                {
                    u32 g = getSubvoxelIndex(m_manager->groupedPoints[i]);
                    if(!(m_subvoxelStarts->_1D[g] <= i && i < m_subvoxelStarts->_1D[g + 1]))
                    {
                        std::cerr << "Assertion failed at i=" << i << " g=" << g << " range=["
                                  << m_subvoxelStarts->_1D[g] << "," << m_subvoxelStarts->_1D[g + 1]
                                  << "),  total=" << count() << std::endl;
                    }
                    assert(m_subvoxelStarts->_1D[g] <= i && i < m_subvoxelStarts->_1D[g + 1]);
                }
            });

            m_subvoxelStartsComputed = true;
        }

        // Calculates subvoxelCumSum if necessary, relative to Voxel.
        void Voxel::buildSubVoxelCumSum() const
        {
            if(m_subvoxelCumSumComputed) return;

            ensureSubvoxelStartsComputed();
            Voxelprocessing::GridUnion<c_subvoxel> subVoxelCounts;
            for(u32 i = 0; i < Math::pow3(c_subvoxel); ++i)
            {
                subVoxelCounts._1D[i] = m_subvoxelStarts->_1D[i + 1] - m_subvoxelStarts->_1D[i];
            }
            buildCumSum<c_subvoxel>(subVoxelCounts, m_subvoxelCumSum);
            m_subvoxelCumSumComputed = true;

            VOXEL_DBG({
                u32 voxelCount = count();
                if(voxelCount > 0)
                {
                    SubVoxel subvoxel1(*this, 0, 0, 0);
                    SubVoxel subvoxel2(*this, c_subvoxel - 1, c_subvoxel - 1, c_subvoxel - 1);
                    u32      subTotal = m_manager->countPointsInSubVoxelBox(*this, subvoxel1, subvoxel2);
                    assert(subTotal == voxelCount);
                }
            });
        }

        bool Voxel::detectPlane()
        {
            u32 cnt = count();
            if(cnt < 100) return false;
            u32                             tries   = 5;
            u32                             count40 = (cnt * 13) >> 5; // ca cnt*0.4
            u32                             count60 = (cnt * 77) >> 7; // ca cnt*0.6
            u32                             step    = (count60 - count40) / tries;
            u32                             s       = start();
            std::vector<Pointcloud::Point>& points  = m_manager->groupedPoints;

            u32       confirmation = 0;
            float     eps          = 0.01f;
            const u32 threshold    = 60; // 900 dots/m^2 -> 81 punten per 30x30 cm.
            u32       _e           = s + cnt - 1;

            for(u32 b = s; b < s + tries; ++b)
            {
                for(u32 e = _e; e > _e - tries; --e)
                {
                    for(u32 m = s + count40; m < count60; m += step)
                    {
                        bool ok;
                        auto n = Math::Vec3f::normalFrom(points[b].m_point, points[e].m_point, points[m].m_point, ok);
                        if(!ok) continue;

                        float d = n.dot(points[b].m_point);

                        for(u32 x = s; x < s + cnt; ++x)
                        {
                            if(fabsf(n.dot(points[x].m_point) - d) < eps)
                            {
                                ++confirmation;
                                if(confirmation > threshold)
                                {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }

            return false;
        }

        SubVoxel Voxel::getSubVoxel(u32 x, u32 y, u32 z) const
        {
            ensureSubvoxelStartsComputed();
            return SubVoxel(*this, x, y, z);
        }

        // --- SubVoxel Implementation ---
        SubVoxel::SubVoxel(const Voxel& voxel, u32 x, u32 y, u32 z)
        {
            voxel.ensureSubvoxelStartsComputed();
            m_x        = x;
            m_y        = y;
            m_z        = z;
            m_index_1D = flatten3<c_subvoxel>(z, y, x);
            m_start    = voxel.m_subvoxelStarts->_1D[m_index_1D];
            m_count    = voxel.m_subvoxelStarts->_1D[m_index_1D + 1] - m_start;
        }

        SubVoxel::SubVoxel(u32 index_1D, u32 start, u32 count, const Math::Vec3d& bbMin)
            : m_index_1D(index_1D), m_start(start), m_count(count), m_bbMin(bbMin)
        {
            Voxelprocessing::unflatten3<c_subvoxel>(m_index_1D, m_z, m_y, m_x);
        }

        // --- OnlinePCA Implementation ---
        void OnlinePCA::reset()
        {
            n    = 0;
            mean = Math::Vec3d(0.0, 0.0, 0.0);
            M2.zero();
        }

        void OnlinePCA::update(const Math::Vec3d& v)
        {
            n++;
            Math::Vec3d delta = v - mean;
            mean += delta / static_cast<double>(n);
            Math::Vec3d delta2 = v - mean;
            M2 += outer_product(delta, delta2);
        }

        Math::Matrix3d OnlinePCA::covariance() const
        {
            if(n < 2) return Math::Matrix3d::zeroMatrix();
            return M2 / static_cast<double>(n - 1);
        }

        // --- VoxelManager Implementation ---
        VoxelManager::VoxelManager()
        {
            s_pInstance = this;
            m_voxelStarts.initialize(0);
            m_voxelCumSum.initialize(0);
            groupedPoints.clear();
        }

        void VoxelManager::getVoxel(u32 x, u32 y, u32 z, Voxel& out,
                                    Voxelprocessing::GridUnion<c_subvoxel>& subVoxelStarts) const
        {
            u32 index_1D = flatten3<c_voxel>(z, y, x);
            out          = Voxel(index_1D, const_cast<VoxelManager*>(this), subVoxelStarts);
        }

        void VoxelManager::getVoxelById(u32 flatIndex, Voxel& out,
                                        Voxelprocessing::GridUnion<c_subvoxel>& subVoxelStarts) const
        {
            out = Voxel(flatIndex, const_cast<VoxelManager*>(this), subVoxelStarts);
        }

        void VoxelManager::voxelize(ccPointCloud* cloud)
        {
            assert(cloud != nullptr && cloud->size() > 0);

            // Bereken normalisatie parameters
            m_normResult = computeNormalizationParameters(cloud);

            // De cloud wordt opgesplitst in c_radix (1024) slices, voor 10 bits sortering in elke richting.
            m_sliceRange.computeVoxelSizeData(m_normResult.bbMaxTranslatedScaled, c_radix);

            m_voxelStarts._1D[0] = 0;

            // Subsample en normaliseer in groupedPoints
            u32 totalPoints = cloud->size();
            groupedPoints.clear();
            groupedPoints.reserve(totalPoints);

            const double eps = 1e-4;
            for(u32 i = 0; i < totalPoints; ++i)
            {
                const CCVector3* p = cloud->getPoint(i);
                assert(p != nullptr);
                Math::Vec3d point(*p);

                // Translatie (altijd)
                point -= m_normResult.bbMinOrg;
                assert(point.x < maxAllowedSize + eps);
                assert(point.y < maxAllowedSize + eps);
                assert(point.z < maxAllowedSize + eps);

                // Schaling (alleen als nodig - gebruik de flag!)
                if(m_normResult.needsScaling)
                {
                    point *= m_normResult.scale;
                    assert(point.x < maxAllowedScaledSize + eps);
                    assert(point.y < maxAllowedScaledSize + eps);
                    assert(point.z < maxAllowedScaledSize + eps);
                }

                // groupedPoints.emplace_back(Pointcloud::Point(i32(i), Math::Vec3f(point)));
                groupedPoints.emplace_back(i32(i), Math::Vec3f(point));
            }

            auto getX = [&](const Pointcloud::Point& p) {
                return computeIndexComponent(static_cast<double>(p.m_point.x), m_sliceRange.invRange.x);
            };
            auto getY = [&](const Pointcloud::Point& p) {
                return computeIndexComponent(static_cast<double>(p.m_point.y), m_sliceRange.invRange.y);
            };
            auto getZ = [&](const Pointcloud::Point& p) {
                return computeIndexComponent(static_cast<double>(p.m_point.z), m_sliceRange.invRange.z);
            };

            auto getVoxelIndex = [&](const Pointcloud::Point& p) {
                u32 x = getX(p);
                u32 y = getY(p);
                u32 z = getZ(p);
                // Shift out LSB bits for subvoxels, so we keep MSB bits for voxel
                constexpr u32 shift = Voxelprocessing::log2(c_subvoxel);
                return flatten3<c_voxel>(z >> shift, y >> shift, x >> shift);
            };

            auto getSubvoxelIndex = [&](const Pointcloud::Point& p) {
                u32 x = getX(p);
                u32 y = getY(p);
                u32 z = getZ(p);

                // LSB bits give subvoxel coordinate within parent voxel
                constexpr u32 mask = maskFor(c_subvoxel);
                return flatten3<c_subvoxel>(z & mask, y & mask, x & mask);
            };

            GridUnion<c_subvoxel> startSubVoxel;
            countingSort<c_subvoxel>(groupedPoints, getSubvoxelIndex, startSubVoxel);
            countingSort<c_voxel>(groupedPoints, getVoxelIndex, m_voxelStarts);

            // Herbereken voxelCounts uit startVoxel
            u32 Bins = Math::pow3(c_voxel);
            for(u32 i = 0; i < Bins; ++i)
            {
                m_voxelCounts[i] = m_voxelStarts._1D[i + 1] - m_voxelStarts._1D[i];
            }
            m_voxelCounts[Bins] = 0;
            computeVoxelMediumRange();

#ifdef _DEBUG
            u32 sum = 0;
            for(u32 i = 0; i < Bins; ++i) sum += m_voxelCounts._1D[i];
            assert(sum == groupedPoints.size());
#endif
            VOXEL_DBG({
                for(u32 i = 0; i < groupedPoints.size(); ++i)
                {
                    u32 g = getVoxelIndex(groupedPoints[i]);
                    if(!(m_voxelStarts._1D[g] <= i && i < m_voxelStarts._1D[g + 1]))
                    {
                        std::cerr << "Assertion failed at i=" << i << " g=" << g << " range=[" << m_voxelStarts._1D[g]
                                  << "," << m_voxelStarts._1D[g + 1] << "), total=" << groupedPoints.size()
                                  << std::endl;
                    }
                    assert(m_voxelStarts._1D[g] <= i && i < m_voxelStarts._1D[g + 1]);
                }
                std::cout << "Voxelization complete. Total points: " << groupedPoints.size() << std::endl;
                std::cout << "Scale: " << m_normResult.scale << std::endl;
            });
        }

        void VoxelManager::computeVoxelMediumRange()
        {
            constexpr u32         Bins = Math::pow3(c_voxel);
            std::array<u32, Bins> sortedCounts;
            std::copy(m_voxelCounts._1D,        // Bron startadres
                      m_voxelCounts._1D + Bins, // Bron eindadres (start + Bins elementen)
                      sortedCounts.begin()      // Bestemming startadres
            );

            std::sort(sortedCounts.begin(), sortedCounts.end());

            // Find first index j where count > 0 (exclude zero bins)
            u32       low            = 0;
            const u32 noiseThreshold = 10;
            while(low < Bins && sortedCounts[low] < noiseThreshold)
                ++low; // Less than 10 points per slice is considered noise.

            if(low >= Bins)
            {
                // All bins are zero
                m_mediumVoxelRange.minCount  = 0;
                m_mediumVoxelRange.maxCount  = 0;
                m_mediumVoxelRange.p25_value = 0.0;
                m_mediumVoxelRange.p50_value = 0.0;
                m_mediumVoxelRange.p75_value = 0.0;
                return;
            }

            // Q1, Q2, Q3 quartile indices within j..Bins
            u32 p50_index = (Bins + low) / 2;
            u32 p25_index = (low + p50_index) / 2;
            u32 p75_index = (Bins + p50_index) / 2;

            // Store percentile values in m_madRange
            m_mediumVoxelRange.p25_value = static_cast<double>(sortedCounts[p25_index]);
            m_mediumVoxelRange.p50_value = static_cast<double>(sortedCounts[p50_index]);
            m_mediumVoxelRange.p75_value = static_cast<double>(sortedCounts[p75_index]);

            const double alpha   = 0.2;
            double       lower_d = m_mediumVoxelRange.p25_value * (1 - alpha);
            double       upper_d = m_mediumVoxelRange.p75_value * (1 + alpha);

            // Store results (use integers for min/max counts)
            m_mediumVoxelRange.minCount = U32(std::floor(lower_d));
            m_mediumVoxelRange.maxCount = U32(std::ceil(upper_d));

            VOXEL_DBG({
                std::cout << "VoxelRange: j=" << low << " p25_index=" << p25_index << " p50_index=" << p50_index
                          << " p75_index=" << p75_index << " p25=" << m_mediumVoxelRange.p25_value
                          << " p50=" << m_mediumVoxelRange.p50_value << " p75=" << m_mediumVoxelRange.p75_value
                          << " => [" << m_mediumVoxelRange.minCount << ", " << m_mediumVoxelRange.maxCount << "]"
                          << std::endl;
            });
        }

        // Fast check using actual voxel count
        bool VoxelManager::isMediumDensityVoxel(u32 index) const
        {
            u32 count = m_voxelCounts._1D[index];
            return (count >= m_mediumVoxelRange.minCount) && (count <= m_mediumVoxelRange.maxCount);
            // return count > 4 && count < 25;
        }

        void VoxelManager::buildVoxelCumSum() const
        {
            if(m_voxelCumSumComputed) return;
            // Reconstruct counts from starts
            Voxelprocessing::GridUnion<c_voxel> voxelCounts;
            voxelCounts.initialize(0);
            for(u32 i = 0; i < Math::pow3(c_voxel); ++i)
            {
                voxelCounts._1D[i] = m_voxelStarts._1D[i + 1] - m_voxelStarts._1D[i];
            }
            buildCumSum<c_voxel>(voxelCounts, m_voxelCumSum);
            m_voxelCumSumComputed = true;

            VOXEL_DBG({
                Voxelprocessing::GridUnion<c_subvoxel> subVoxelStarts;
                Voxel                                  v1(0, this, subVoxelStarts);
                Voxel                                  v2(Math::pow3(c_voxel) - 1, this, subVoxelStarts);
                u32                                    totalPoints = countPointsInVoxelBox(v1, v2);
                assert(totalPoints == U32(groupedPoints.size()));
            });
        }

        u32 VoxelManager::countPointsInVoxelBox(const Voxel& voxel1, const Voxel& voxel2) const
        {
            return countPointsInBox<c_voxel>(m_voxelCumSum, voxel1.m_x, voxel1.m_y, voxel1.m_z, voxel2.m_x, voxel2.m_y,
                                             voxel2.m_z);
        }

        u32 VoxelManager::countPointsInSubVoxelBox(const Voxel& voxel, const SubVoxel& subvoxel1,
                                                   const SubVoxel& subvoxel2) const
        {
            if(!voxel.hasSubVoxelCumSum())
            {
                voxel.buildSubVoxelCumSum();
            }
            u32 sx1, sy1, sz1, sx2, sy2, sz2;
            subvoxel1.getIndices(sx1, sy1, sz1);
            subvoxel2.getIndices(sx2, sy2, sz2);
            return countPointsInBox<c_subvoxel>(voxel.m_subvoxelCumSum, sx1, sy1, sz1, sx2, sy2, sz2);
        }

        NormalizationResult VoxelManager::computeNormalizationParameters(ccPointCloud* cloud)
        {
            assert(cloud != nullptr && cloud->size() > 0);

            CCVector3 minCC, maxCC;
            cloud->getBoundingBox(minCC, maxCC);
            NormalizationResult result{1.0, false, Math::Vec3d(minCC), Math::Vec3d(maxCC)};
            result.bbMaxTranslatedScaled -= result.bbMinOrg;

            Math::Vec3d& bboxSize        = result.bbMaxTranslatedScaled;
            double       maxBoxDimension = std::max({bboxSize.x, bboxSize.y, bboxSize.z});
            result.needsScaling          = (maxBoxDimension > maxAllowedSize);

            if(result.needsScaling)
            {
                result.scale = maxAllowedScaledSize /
                               maxBoxDimension; // if scaling, coordinates will be 100.0 max, else 1000.0 max
                result.bbMaxTranslatedScaled = result.bbMaxTranslatedScaled * result.scale;
            }

            return result;
        }
    } // namespace Voxelprocessing

} // namespace Bocari
