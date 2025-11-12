#include "point.h"
namespace Bocari
{
    namespace Math
    {
        // Roteer een Pointcloud::Point (in-place versie)
        template <typename T> void Quaternion<T>::rotateInPlace(Pointcloud::Point& p) const
        {
            // Gebruik double precisie voor berekeningen, zelfs als punt float gebruikt
            Quaternion<T> p_quat(static_cast<T>(0), static_cast<T>(p.m_point.x), static_cast<T>(p.m_point.y),
                                 static_cast<T>(p.m_point.z));
            Quaternion<T> q_inv     = this->inverse();
            Quaternion<T> rotated_p = (*this) * p_quat * q_inv;

            // Converteer terug naar float
            p.m_point.x = static_cast<float>(rotated_p.x);
            p.m_point.y = static_cast<float>(rotated_p.y);
            p.m_point.z = static_cast<float>(rotated_p.z);
        }

        // Roteer een span van Points in-place
        template <typename T> void Quaternion<T>::rotateSpanInPlace(std::span<Pointcloud::Point> points) const
        {
            for(auto& p : points)
            {
                rotateInPlace(p);
            }
        }
    } // namespace Math

    namespace Pointcloud
    {
        // Converteer een span van onze Points naar een ccPointCloud
        ccPointCloud* to_ccPointCloud(std::span<const Pointcloud::Point> points)
        {
            if(points.empty()) return nullptr;

            ccPointCloud* result = new ccPointCloud();
            if(!result->reserve(U32(points.size())))
            {
                delete result;
                return nullptr;
            }

            // Voeg alle punten toe één voor één
            for(const auto& p : points)
            {
                CCVector3 ccPoint{p.m_point.x, p.m_point.y, p.m_point.z};
                result->addPoint(ccPoint);
            }

            return result;
        }

        // Converteer een ccPointCloud naar een vector van onze Points
        std::vector<Pointcloud::Point> from_ccPointCloud(const ccPointCloud* cloud, u32 step = 1)
        {
            std::vector<Pointcloud::Point> result;

            if(!cloud || cloud->size() == 0)
            {
                return result;
            }

            result.reserve(cloud->size());

            for(u32 i = 0; i < cloud->size(); i += step)
            {
                const CCVector3* p = cloud->getPoint(i);
                result.emplace_back(i32(i), Math::Vec3f{p->x, p->y, p->z});
            }

            return result;
        }

        // Helper: roteer een ccPointCloud met een quaternion
        ccPointCloud* createRotatedPointCloud(const Math::Quaternion<double>& quat, const ccPointCloud* original)
        {
            if(!original || original->size() == 0) return nullptr;

            // Converteer naar onze Points
            auto ourPoints = from_ccPointCloud(original);

            // Roteer de punten in-place
            quat.rotateSpanInPlace(std::span<Pointcloud::Point>(ourPoints));

            // Converteer terug naar ccPointCloud
            return to_ccPointCloud(std::span<const Pointcloud::Point>(ourPoints));
        }

    } // namespace Pointcloud
} // namespace Bocari