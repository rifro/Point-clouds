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

void AutofitImpl::doOptimizeFrame2()
{
    if(!m_selectedCloud || m_selectedCloud->size() == 0)
    {
        std::cout << "Geen point cloud of lege cloud\n";
        return;
    }

    CCVector3 bbMin, bbMax;
    m_selectedCloud->getBoundingBox(bbMin, bbMax);

    // Bepaal Sx, Sy, Sz op basis van jouw "vierkant-houden" + regio’s.
    // (Hier placeholder; hang dit aan jouw regioneringslogica.)
    uint32_t Sx = /*...*/ 64;
    uint32_t Sy = /*...*/ 64;
    uint32_t Sz = /*...*/ 32;

    // Maak accessor-lijst met huidige asvolgorde
    std::array<Toegang, 3> ax = {{{'x', [](auto* p) { return p->x; }, Sx},
                                  {'y', [](auto* p) { return p->y; }, Sy},
                                  {'z', [](auto* p) { return p->z; }, Sz}}};

    // Forceer Sx ≥ Sy ≥ Sz en onthoud welke as waarheen verhuisde
    orden_axes_op_S(ax); // ax[0] → newX, ax[1] → newY, ax[2] → newZ

    // Bouw host-buffers (SoA + labels)
    const size_t        N = m_selectedCloud->size();
    std::vector<float3> punten;
    punten.reserve(N);

    // vertaal/scale → hier enkel copy, echte translatie/scale kan op GPU in init-kernel
    for(size_t i = 0; i < N; ++i)
    {
        const CCVector3* pt = m_selectedCloud->getPoint(static_cast<unsigned>(i));
        if(!pt) continue;
        float X = ax[0].get(pt);
        float Y = ax[1].get(pt);
        float Z = ax[2].get(pt);
        punten.push_back(make_float3(X, Y, Z));
    }

    // Labels initialiseren
    enum Label : uint32_t { LABEL_GEEN = 0, LABEL_RUIS = 1, LABEL_VLAK = 2, LABEL_BUIS = 3 };
    std::vector<uint32_t> labels(punten.size(), LABEL_GEEN);

    // Device allocs/kopie
    float3*   d_punten = nullptr;
    uint32_t* d_labels = nullptr;
    cudaMalloc((void**)&d_punten, punten.size() * sizeof(float3));
    cudaMalloc((void**)&d_labels, punten.size() * sizeof(uint32_t));
    cudaMemcpy(d_punten, punten.data(), punten.size() * sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_labels, labels.data(), labels.size() * sizeof(uint32_t), cudaMemcpyHostToDevice);

    // Daarna: roep jouw GPU-pipeline (ruisfilter → vlak-quickcheck → wedge) aan.
    // (Zie aanroep-skel onderaan.)
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
