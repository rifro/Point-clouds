#include "autofit_impl.h"
#include "assen_hernoemen.h"
#include "axis.h"
#include "includes.h"
#include "voxelprocessor.h"

using namespace CCCoreLib;
using namespace Bocari::Voxelprocessing;
// using CCCoreLib::DegreesToRadians;

AutofitImpl::AutofitImpl() {}

void AutofitImpl::performFit(ccMainAppInterface* app, ccPointCloud* selectedCloud)
{
    m_app           = app;
    m_selectedCloud = selectedCloud;
    //[[maybe_unused]] auto res = doOptimizeFrame1();
    doOptimizeFrame1();
}

void AutofitImpl::doOptimizeFrame1()
{
    using namespace std::chrono;
    using namespace Bocari;
    if(!m_selectedCloud || m_selectedCloud->size() == 0)
    {
        std::cout << "Geen point cloud of lege cloud" << std::endl;
        return;
    }

    /*
    const unsigned int THRESHOLD_COUNT = 100000;
    dbg("Applying voxel subsampling");
    std::unique_ptr<ccPointCloud> voxelSubsampledCloud = AutofitImpl::voxelSubsample(m_selectedCloud, THRESHOLD_COUNT);

    if(voxelSubsampledCloud)
    {
        m_selectedCloud = voxelSubsampledCloud.get();
    } else
    {
        return;
    }uint32
    */

    // Start timing
    auto startTime = high_resolution_clock::now();

    VoxelManager voxelManager;

    // Stap 1: Verwerk de complete cloud naar voxels
    std::cout << "Verwerken van " << m_selectedCloud->size() << " punten..." << std::endl;
    voxelManager.voxelize(m_selectedCloud);

    // Eind timing voor voxelization
    auto endTime          = high_resolution_clock::now();
    auto voxelizeDuration = duration_cast<milliseconds>(endTime - startTime);

    // Convert naar minuten, seconden, milliseconden
    // Correcte en duidelijke manier om de resterende tijd te berekenen
    auto mins = duration_cast<minutes>(voxelizeDuration);
    auto secs = duration_cast<seconds>(voxelizeDuration - mins);
    auto ms   = duration_cast<milliseconds>(voxelizeDuration - mins - secs);

    std::cout << "Voxelization tijd: " << mins.count() << " min, " << secs.count() << " sec, " << ms.count() << " ms"
              << std::endl;

    // Stap 2: Selecteer specifieke voxel (z, y, x volgorde!)
    const uint8_t         targetZ = 1, targetY = 2, targetX = 3;
    Voxel                 voxel;
    GridUnion<c_subvoxel> subVoxelStarts;
    voxelManager.getVoxel(targetZ, targetY, targetX, voxel, subVoxelStarts);

    if(voxel.count() == 0)
    {
        std::cout << "Voxel (" << static_cast<int>(targetZ) << "," << static_cast<int>(targetY) << ","
                  << static_cast<int>(targetX) << ") bevat geen punten" << std::endl;
        // return;
    }

    voxelManager.buildVoxelCumSum();

    auto constexpr mask = maskFor(c_voxel);
    // Example: count points in (0,0,0)-(x,y,z)
    u32 magic = voxelManager.m_voxelCumSum(mask, mask, mask);

    std::cout << "Voxel (" << targetZ << "," << targetY << "," << targetX << ") bevat " << magic << " punten"
              << std::endl;

    assert(magic == voxelManager.groupedPoints.size());

    // Converteer naar ccPointCloud
    // RR!!! Hier
    bool b = false;
    for(u32 i = 0; i < U32(voxelManager.groupedPoints.size()); ++i)
    {
        GridUnion<c_subvoxel> subvoxelStarts;
        Voxel                 v;
        voxelManager.getVoxelById(i, v, subvoxelStarts);
        b |= v.detectPlane();
    }

    std::cout << b;

    return;

    ccPointCloud* ccCloud = new ccPointCloud();
    if(!ccCloud)
    {
        std::cout << "Error: Failed to create rotated sampled cloud" << std::endl;
        return;
    }

    if(!ccCloud->reserve(U32(voxelManager.groupedPoints.size())))
    {
        delete ccCloud;
        return;
    }

    // Stap 1: Voeg scalar field toe
    std::cout << "Adding points to cloud..." << std::endl;
    constexpr u32 Bin = Math::pow3(c_voxel);
    // Voeg eerst alle punten toe
    for(u32 v = 0; v < Bin; ++v)
    {
        for(u32 i = 0; i < voxelManager.m_voxelCounts[v]; ++i)
        {
            u32                j = voxelManager.m_voxelStarts[v] + i;
            Pointcloud::Point& p = voxelManager.groupedPoints[j];
            CCVector3          ccPoint{p.m_point.x, p.m_point.y, p.m_point.z};
            ccCloud->addPoint(ccPoint);
        }
    }

    std::cout << "Cloud size: " << ccCloud->size() << std::endl;

    // Stap 2: Nu pas de scalar field maken en toevoegen
    std::cout << "Creating scalar field..." << std::endl;
    ccScalarField* sf = new ccScalarField("ColoredPoints");

    // BELANGRIJK: Reserveer de exacte grootte
    sf->reserve(ccCloud->size());

    // Stap 3: Vul de scalar field
    auto odd = [](u32 x) -> bool { return (x & 1) != 0; };

    u32 pointIndex = 0;
    for(u32 v = 0; v < Bin; ++v)
    {
        if(!voxelManager.isMediumDensityVoxel(v)) continue;
        u32 x, y, z;
        Bocari::Voxelprocessing::unflatten3<c_voxel>(v, z, y, x);

        bool odds = odd(x) ^ odd(y) ^ odd(z);

        for(u32 i = 0; i < voxelManager.m_voxelCounts[v]; ++i)
        {
            float value = odds ? 1.0f : 0.0f;
            sf->addElement(value); // Gebruik addElement in plaats van setValue!

            if(pointIndex < 10)
            {
                std::cout << "Added SF[" << pointIndex << "] = " << value << std::endl;
            }

            pointIndex++;
        }
    }

    std::cout << "SF final size: " << sf->size() << std::endl;

    // Stap 4: Voeg scalar field toe aan cloud
    int sfIndex = ccCloud->addScalarField(sf);
    std::cout << "Scalar field index: " << sfIndex << std::endl;

    if(sfIndex < 0)
    {
        std::cout << "Failed to add scalar field to cloud!" << std::endl;
        sf->release();
        delete ccCloud;
        return;
    }

    // Stap 5: Compute min/max
    sf->computeMinAndMax();
    std::cout << "Scalar field range: " << sf->getMin() << " to " << sf->getMax() << std::endl;

    // Stap 6: Configureer display
    ccCloud->setCurrentScalarField(sfIndex);
    ccCloud->setCurrentDisplayedScalarField(sfIndex);
    ccCloud->showSF(true);
    ccCloud->setPointSize(2.0f);

    ccScalarField* currentSF = static_cast<ccScalarField*>(ccCloud->getScalarField(sfIndex));
    if(currentSF)
    {
        currentSF->setMinDisplayed(0.0f);
        currentSF->setMaxDisplayed(1.0f);
        currentSF->setSaturationStart(0.0f);
        currentSF->setSaturationStop(1.0f);

        ccColorScale::Shared colorScale = ccColorScale::Create("Checkerboard");
        if(colorScale)
        {
            colorScale->insert(ccColorScaleElement(0.0, Qt::red), false);
            colorScale->insert(ccColorScaleElement(1.0, Qt::blue), false);
            colorScale->update();
            currentSF->setColorScale(colorScale);
            std::cout << "Color scale configured" << std::endl;
        }
    }

    ccCloud->showSFColorsScale(true);
    ccCloud->prepareDisplayForRefresh();

    // Voeg toe aan CC
    ccHObject* parent = m_selectedCloud->getParent();
    if(parent)
    {
        parent->addChild(ccCloud);
    }

    m_app->addToDB(ccCloud);
    m_app->setSelectedInDB(ccCloud, true);
    m_app->refreshAll();

    std::cout << "Voxel checkerboard cloud created successfully!" << std::endl;
}

// Full square method: Fill data, sort on V descending for perm, adjust for square aspect <=1:2
void AutofitImpl::make_square_method(const ccPointCloud& cloud, array<AxisAccess, 3>& axes) {
    CCVector3 bbMin, bbMax;
    cloud.getBoundingBox(bbMin, bbMax);
    const size_t N = cloud.size();
    const float L[3] = {bbMax.x - bbMin.x, bbMax.y - bbMin.y, bbMax.z - bbMin.z};
    float maxL = *max_element(L, L + 3);
    if (maxL <= 0) return;  // Degenerate

    // Target total voxels ~1024 (adaptive on BB, for perf: coarse grid without overflow)
    const uint32_t target_total = 1024;
    uint32_t V_init[3];
    for (int i = 0; i < 3; ++i) {
        V_init[i] = max(1u, static_cast<uint32_t>(target_total * (L[i] / maxL)));  // Scale to dims
    }

    // Initial array with getters
    array<AxisAccess, 3> init_axes;
    init_axes[0] = {'x', {}, V_init[0], [](const auto* p) { return p->x; }};
    init_axes[1] = {'y', {}, V_init[1], [](const auto* p) { return p->y; }};
    init_axes[2] = {'z', {}, V_init[2], [](const auto* p) { return p->z; }};

    // Fill owned data (batch read)
    for (auto& ax : init_axes) {
        ax.data.resize(N);
        for (size_t i = 0; i < N; ++i) {
            const CCVector3* p = cloud.getPoint(i);
            ax.data[i] = ax.getter(p);
        }
    }

    // Sort array on V descending (Vx long, Vz short) – perm forward
    sort(init_axes.begin(), init_axes.end(), [](const auto& a, const auto& b) { return a.V > b.V; });

    // Square adjustment: Ensure Vy ≈ Vz (aspect <=1:2), recalc Vx for total ~target
    uint32_t target_doorsnede = static_cast<uint32_t>(sqrt(init_axes[1].V * init_axes[2].V));
    if (init_axes[1].V > init_axes[2].V * 2) init_axes[1].V = init_axes[2].V * 2;  // Cap aspect
    init_axes[0].V = max(1u, target_total / target_doorsnede);  // Adjust slices

    // Assign to output axes (sorted order)
    axes = init_axes;

    // Report #regions = Vx (slices along long)
    cout << "Square method: Vx=" << axes[0].V << " Vy=" << axes[1].V << " Vz=" << axes[2].V
         << " (#regions=" << axes[0].V << ", aspect " << static_cast<float>(axes[1].V) / axes[2].V << ":1)" << endl;
}

void AutofitImpl::doOptimizeFrame2() {
    if (!m_selectedCloud || m_selectedCloud->size() == 0) {
        std::cout << "[Autofit] Geen point cloud of leeg.\n";
        return;
    }

    const size_t N = m_selectedCloud->size();

    // 1) Vierkant-methode: Fill & sort axes on V (owned data, no pointers)
    array<AxisAccess, 3> axes;
    make_square_method(*m_selectedCloud, axes);  // Fills axes[0..2].data with permuted coords

    // Num regions from Vx
    const uint32_t num_regions = axes[0].V;

    std::cout << "[A] Morton-sort en permutatie (" << num_regions << " regions)... " << std::flush;

    // 2) Upload owned SoA data as float3 (merge on host for coalesced upload)
    vector<float3> h_pts(N);
    for (size_t i = 0; i < N; ++i) {
        h_pts[i] = make_float3(axes[0].data[i], axes[1].data[i], axes[2].data[i]);
    }

    DeviceBuffer<float3> d_pts(N);
    d_pts.upload(h_pts.data(), N);

    DeviceBuffer<uint64_t> d_keys(N);
    DeviceBuffer<uint32_t> d_index(N);
    DeviceBuffer<float3> d_pts_sorted(N);
    DeviceBuffer<u8> d_labels(N); d_labels.memset(0);

    // 3) Morton sort (voxel contiguous)
    h_voxeliseer_morton(d_pts, d_keys, d_index, d_pts_sorted, d_voxelBereiken, N, mortonCfg);

    std::cout << "gereed: points contiguous per voxel/regio" << std::endl;
    std::cout << "[B] Boeren 2-buren ruisfilter..." << std::endl;

    // Boeren filter
    h_ruis_boeren(d_pts_sorted, d_labels);

    std::cout << "klaar: ruis gelabeld" << std::endl;

    // 4) Vlak-stemming current frame
    std::cout << "[C] Vlak-stemming (brute, minimale support)..." << std::endl;

    KandidaatAssen assen{};
    h_vlak_vote_current_frame(d_pts_sorted, d_labels, assen);

    if (!assen.heeftMinstensEenAs()) {
        std::cout << "geen vlak in huidig frame. Brute planes…" << std::endl;
        h_vlak_vote_bruteforce(d_pts_sorted, d_labels, assen);
    }

    if (!assen.heeftMinstensEenAs()) {
        std::cout << "geen vlak gevonden. Brute buizen (annulus/wedge)…" << std::endl;
        h_buis_vote_bruteforce(d_pts_sorted, d_labels, assen);
    }

    if (!assen.heeftMinstensEenAs()) {
        std::cout << "[X] Geen eerste as te vinden (vlakken/buizen). Stop." << std::endl;
        return;
    }

    std::cout << "eerste as gevonden (kandidaat #1)." << std::endl;

    // 5) Orthonormaliseer + quat
    Quat q = maak_quat_naar_ideaal_frame(assen);

    std::cout << "[Autofit] Roteer naar ideaal frame (quaternion)…" << std::endl;

    h_rotatie_quat_apply(d_pts_sorted, q, N);

    // 6) Fine slabs labeling
    h_fine_slabs_cyl_label(d_pts_sorted, d_labels);

    // 7) Split to SoA on GPU, download
    DeviceBuffer<float> d_x_out(N), d_y_out(N), d_z_out(N);
    h_split_float3_to_soa(d_pts_sorted, d_x_out, d_y_out, d_z_out, N);

    vector<float> rx(N), ry(N), rz(N);
    d_x_out.download(rx.data(), N);
    d_y_out.download(ry.data(), N);
    d_z_out.download(rz.data(), N);

    // 8) Back-permute: Sort axes on name ascending ('x'<'y'<'z') for orig order
    array<AxisAccess, 3> back_axes;
    back_axes[0] = {axes[0].name, move(axes[0].data), axes[0].V, {}};
    back_axes[1] = {axes[1].name, move(axes[1].data), axes[1].V, {}};
    back_axes[2] = {axes[2].name, move(axes[2].data), axes[2].V, {}};
    sort(back_axes.begin(), back_axes.end(), [](const auto& a, const auto& b) { return a.name < b.name; });  // Orig order

    // Batch write to CC (safe, no mutable in loop)
    for (size_t i = 0; i < N; ++i) {
        CCVector3* p = m_selectedCloud->getPointMutable(static_cast<unsigned>(i));
        if (!p) continue;
        p->x = back_axes[0].data[i];  // Orig 'x' data
        p->y = back_axes[1].data[i];
        p->z = back_axes[2].data[i];
    }

    std::cout << "[Autofit] Klaar: punten teruggezet in originele as-namen.\n";

    // Debug rot: #ifdef DEBUG_ROT
    // float3 euler = quat_to_euler(q);
    // m_selectedCloud->rotate(CCVector3(1,0,0), euler.x * M_PI / 180.0f);  // Degrees to rad
    // ... Y/Z
    // #endif
}

// Helper to convert ReferenceCloud (from CloudSamplingTools) to
// unique_ptr<ccPointCloud>
std::unique_ptr<ccPointCloud> convertReferenceCloudToCCPointCloud(ReferenceCloud* refCloud)
{
    if(!refCloud || refCloud->size() == 0) return nullptr;

    auto cloud = std::make_unique<ccPointCloud>();

    if(!cloud->reserve(refCloud->size())) return nullptr;

    for(unsigned i = 0; i < refCloud->size(); ++i)
    {
        const CCVector3* point = refCloud->getPoint(i);
        cloud->addPoint(*point);
    }

    cloud->setName("ConvertedCloud");
    delete refCloud; // ReferenceCloud is dynamically allocated by CloudCompare
                     // tools and needs to be deleted
    return cloud;
}

std::unique_ptr<ccPointCloud> AutofitImpl::voxelSubsample(ccPointCloud* inputCloud, int32_t pointCount)
{
    if(!inputCloud) return nullptr;

    auto subsampledCloud = CCCoreLib::CloudSamplingTools::subsampleCloudWithOctree(
        inputCloud, pointCount, CCCoreLib::CloudSamplingTools::SUBSAMPLING_CELL_METHOD::RANDOM_POINT, nullptr, nullptr);

    return convertReferenceCloudToCCPointCloud(subsampledCloud);
}

std::unique_ptr<ccPointCloud> AutofitImpl::spatialSubsample(ccPointCloud* inputCloud, PointCoordinateType minDistance)
{
    if(!inputCloud) return nullptr;

    CloudSamplingTools::SFModulationParams modParams;
    auto subsampledCloud = CloudSamplingTools::resampleCloudSpatially(inputCloud, minDistance, modParams);

    return convertReferenceCloudToCCPointCloud(subsampledCloud);
}
