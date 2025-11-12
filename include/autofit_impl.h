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
};
