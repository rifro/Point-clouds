// include/autofit_impl.h
#pragma once
#include "autofit.h"
#include "includes.h"
#include "strict.h"

struct OptimizationParams
{
    float   startAngle;
    float   endAngle;
    float   step;
    int     strips;
    QString phaseName;
};

struct OptimizationResult
{
    float                         bestAngle    = 0.0f;
    float                         bestVariance = -1.0f;
    std::unique_ptr<ccPointCloud> bestTransformedCloud;
};

struct AxisAccess {
    char name;  // 'x', 'y', 'z'
    std::vector<float> data;  // Owned coords
    u32 V;  // Subdivisions
    std::function<float(const CCVector3*)> getter;  // Initial fill only
};

class AutofitImpl
{
public:
    explicit AutofitImpl();

    void performFit(ccMainAppInterface* app, ccPointCloud* selectedCloud); // Implementatie zonder Qt-signalen/slots
    void doOptimizeFrame1();
    std::tuple<float, float> doOptimizeFrame2();

private:
    ccMainAppInterface* m_app           = nullptr;
    ccPointCloud*       m_selectedCloud = nullptr;
    // Subsampling methodes
    std::unique_ptr<ccPointCloud> voxelSubsample(ccPointCloud* inputCloud, int32_t pointCount);
    std::unique_ptr<ccPointCloud> spatialSubsample(ccPointCloud* inputCloud, PointCoordinateType minDistance);
    std::unique_ptr<ccPointCloud> cloneReferenceCloud(CCCoreLib::ReferenceCloud* ref);


private:
    // Helpers (inline or separate .cpp)
    void make_square_method(const ccPointCloud& cloud, array<AxisAccess, 3>& axes);
    void h_merge_soa_to_float3(const array<vector<float>*, 3>& soa, vector<float3>& out);
    void h_split_float3_to_soa(DeviceBuffer<float3>& d_in, DeviceBuffer<float>& d_out_x, DeviceBuffer<float>& d_out_y, DeviceBuffer<float>& d_out_z, size_t N);
    void h_rotatie_quat_apply(DeviceBuffer<float3>& d_pts, const Quat& q, size_t N);
    // Stubs (expand with kernels)
    void h_voxeliseer_morton(DeviceBuffer<float3>& d_pts, DeviceBuffer<uint64_t>& d_keys, DeviceBuffer<uint32_t>& d_index, DeviceBuffer<float3>& d_pts_sorted, DeviceBuffer<u32>& d_voxelBereiken, size_t N, const MortonCfg& cfg);
    void h_ruis_boeren(DeviceBuffer<float3>& d_pts_sorted, DeviceBuffer<u8>& d_labels);
    void h_vlak_vote_current_frame(DeviceBuffer<float3>& d_pts_sorted, DeviceBuffer<u8>& d_labels, KandidaatAssen& assen);
    void h_vlak_vote_bruteforce(DeviceBuffer<float3>& d_pts_sorted, DeviceBuffer<u8>& d_labels, KandidaatAssen& assen);
    void h_buis_vote_bruteforce(DeviceBuffer<float3>& d_pts_sorted, DeviceBuffer<u8>& d_labels, KandidaatAssen& assen);
    void h_fine_slabs_cyl_label(DeviceBuffer<float3>& d_pts_sorted, DeviceBuffer<u8>& d_labels);
    Quat maak_quat_naar_ideaal_frame(const KandidaatAssen& assen);  // Bocari func stub
    float3 quat_to_euler(const Quat& q);  // Debug helper
};
