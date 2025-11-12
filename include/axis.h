#pragma once

#include "includes.h"
#include "strict.h" // Assuming strict.h is also needed for includes.h context
// #include <QDebug>   // Required for qDebug

/*
./C++/CloudCompare/libs/qCC_db/src/ccTorus.cpp
./C++/CloudCompare/libs/qCC_db/include/ccBox.h
./C++/CloudCompare/libs/qCC_db/include/ccHObject.h
./C++/CloudCompare/libs/qCC_db/include/ccGenericMesh.h
./C++/CloudCompare/libs/qCC_db/include/ccFacet.h
./C++/CloudCompare/libs/qCC_db/include/ccClipBox.h
./C++/CloudCompare/libs/qCC_db/include/ccQuadric.h
./C++/CloudCompare/libs/qCC_db/include/ccDrawableObject.h
./C++/CloudCompare/libs/qCC_db/include/ccPlanarEntityInterface.h
./C++/CloudCompare/libs/qCC_db/include/ccCoordinateSystem.h
./C++/CloudCompare/libs/qCC_db/include/ccCone.h
./C++/CloudCompare/libs/qCC_db/include/ccKdTree.h
./C++/CloudCompare/libs/qCC_db/src/ccAdvancedTypes.cpp
./C++/CloudCompare/libs/qCC_db/src/ccRasterGrid.cpp
./C++/CloudCompare/libs/qCC_db/src/ccPlanarEntityInterface.cpp
./C++/CloudCompare/libs/qCC_db/src/ccExtru.cpp
./C++/CloudCompare/libs/qCC_db/src/ccPlane.cpp
./C++/CloudCompare/libs/qCC_db/src/ccBBox.cpp
./C++/CloudCompare/libs/qCC_db/src/ccKdTree.cpp// --- AxisAccessor Functor
Struct (Onveranderd) ---
*/

// using CCCoreLib::DegreesToRadians; // If this is a macro or global function, it needs to be defined or included.
// Assuming DegreesToRadians is a function defined elsewhere or in includes.h
namespace CCCoreLib
{
    float DegreesToRadians(float degrees); // Placeholder if not explicitly included
}

const PointCoordinateType zero = 1e-9f;
// Inline downcast helper
inline std::unique_ptr<ccPointCloud> safeDowncastToPointCloud(ccGenericPointCloud* rawCloud)
{
    // Perform a dynamic_cast to safely check the type at runtime
    assert(dynamic_cast<ccPointCloud*>(rawCloud) != nullptr &&
           "Failed to downcast ccGenericPointCloud to ccPointCloud. Object is "
           "not a point cloud.");
    return std::unique_ptr<ccPointCloud>{static_cast<ccPointCloud*>(rawCloud)};
}

inline void mkDir(const QString& path)
{
    QDir dir(path);
    if(!dir.exists())
    {
        std::cout << "Directory does not exist. Creating: " << path.toStdString() << std::endl;
        if(!dir.mkpath("."))
        {
            std::cerr << "Error creating directory" << std::endl;
            return;
        }
    }

    // 3. Verwijder alle bestaande plaatjes met Qt
    std::cout << "Deleting existing images in directory." << std::endl;
    QFileInfoList fileList = dir.entryInfoList(QDir::Files);
    for(const QFileInfo& fileInfo : fileList)
    {
        dir.remove(fileInfo.fileName());
        std::cout << "Deleted: " << fileInfo.fileName().toStdString() << std::endl;
    }
}

// --- Functor Structs for Coordinate Accessors ---
// Deze structs zijn stateless en zullen door de compiler geoptimaliseerd worden
// (zero overhead). Ze bevatten ook een static constexpr char voor de conceptuele
// naam van de as.
struct GetXCoord
{
    inline PointCoordinateType operator()(const CCVector3& v) const { return v.x; }
    static constexpr char      name = 'x';
};

struct GetYCoord
{
    inline PointCoordinateType operator()(const CCVector3& v) const { return v.y; }
    static constexpr char      name = 'y';
};

struct GetZCoord
{
    inline PointCoordinateType operator()(const CCVector3& v) const { return v.z; }
    static constexpr char      name = 'z';
};

template <typename T>
concept HasStaticConstexprCharName = requires {
    // Checkt of T::name een geldige expressie is en exact het type 'char' heeft.
    { T::name } -> std::same_as<const char&>;
};

// --- Global Helper Functions ---
inline size_t getBinnedIndex(PointCoordinateType coord, PointCoordinateType minCoord, PointCoordinateType range,
                             size_t numBins)
{
    if(numBins == 0) return 0; // Geen bins, dus altijd 0

    // Expliciete clamping om binnen het bereik [minCoord, minCoord + range) te
    // blijven
    if(coord < minCoord) return 0;
    // if (coord >= (minCoord + range)) return numBins - 1;

    PointCoordinateType ratio = (coord - minCoord) / range;
    size_t              index = static_cast<size_t>(std::floor(ratio * static_cast<float>(numBins)));
    // Finale clamp om te verzekeren dat de index binnen [0, numBins - 1] valt
    return std::min(index, numBins - 1);
}

// --- PixelMapper Class (ALLES INLINE IN HEADER) ---
class PixelMapper
{
public:
    PixelMapper(PointCoordinateType minCoord, PointCoordinateType maxCoord, size_t numPixels)
        : m_minCoord(minCoord), m_numPixels(numPixels), m_range(maxCoord - minCoord)
    {
        assert(numPixels != 0 && "numPixels cannot be zero for PixelMapper.");

        assert(m_range > static_cast<PointCoordinateType>(zero) && "PixelMapper range must be positive.");
    }

    inline size_t getPixelIndex(PointCoordinateType coord) const
    {
        return getBinnedIndex(coord, m_minCoord, m_range, m_numPixels);
    }

    inline size_t getNumPixels() const { return m_numPixels; }

private:
    PointCoordinateType m_minCoord;
    size_t              m_numPixels;
    PointCoordinateType m_range;
};

// --- Strip Class - ALLES INLINE IN HEADER) ---
class Strip
{
public:
    Strip(PointCoordinateType minCoord, PointCoordinateType maxCoord, size_t numStrips)
        : m_minCoord(minCoord), m_numStrips(numStrips), m_range(maxCoord - minCoord)
    {
        assert(numStrips != 0 && "numStrips cannot be zero.");
        assert(m_range > static_cast<PointCoordinateType>(zero) && "Strip range must be positive.");
        m_pointCountPerStrip.resize(m_numStrips, 0);
    }

    inline void addPoint(PointCoordinateType coord)
    {
        size_t index = getBinnedIndex(coord, m_minCoord, m_range, m_numStrips);
        if(index < m_numStrips)
        {
            m_pointCountPerStrip[index]++;
        }
    }

    // Deze functie berekent nu de variantie van de *tellingen* in alle strips
    inline float getVarianceOfCounts() const
    {
        if(m_numStrips < 2)
        {
            return 0.0f; // Niet genoeg strips om variantie te berekenen
        }

        // Bereken gemiddelde van strip tellingen
        uint64_t totalPoints = 0;
        for(Bocari::u32 count : m_pointCountPerStrip)
        {
            totalPoints += count;
        }

        double mean = static_cast<double>(totalPoints) / static_cast<double>(m_numStrips);

        // Bereken som van gekwadrateerde verschillen van het gemiddelde
        double sumSqDiff = 0.0;
        for(Bocari::u32 count : m_pointCountPerStrip)
        {
            double diff = static_cast<double>(count) - mean;
            sumSqDiff += (diff * diff);
        }

        // Variantie (gedeeld door N, geen sample maar volledige set).
        return static_cast<float>(sumSqDiff / static_cast<double>(m_numStrips));
    }

private:
    PointCoordinateType m_minCoord;
    size_t              m_numStrips;
    PointCoordinateType m_range; // Bereik van coördinaten
    std::vector<size_t> m_pointCountPerStrip;
};

// --- TEMPLATE Axis Class (ALLES INLINE IN HEADER) ---
template <typename AccessorFunctor> class Axis
{
public:
    // Constructor accepteert nu geen 'roleChar' meer, die komt van de functor
    Axis(const CCVector3& minSceneBounds, const CCVector3& maxSceneBounds, size_t numStrips, size_t numPixels)
        : m_strip(AccessorFunctor{}(minSceneBounds), AccessorFunctor{}(maxSceneBounds), numStrips),
          m_pixelMapper(AccessorFunctor{}(minSceneBounds), AccessorFunctor{}(maxSceneBounds), numPixels)
    {
    }

    inline void addPoint(const CCVector3& point) { m_strip.addPoint(AccessorFunctor{}(point)); }

    inline float getVariance() const { return m_strip.getVarianceOfCounts(); }

    inline size_t getPixelIndex(const CCVector3& point) const
    {
        return m_pixelMapper.getPixelIndex(AccessorFunctor{}(point));
    }

    inline size_t getNumPixels() const { return m_pixelMapper.getNumPixels(); }

    // Role char komt nu direct van de functor via de template parameter
    inline char getRoleChar() const { return AccessorFunctor::name; }

private:
    Strip       m_strip;
    PixelMapper m_pixelMapper;
};

// --- TEMPLATE Plane Class - Zoveel mogelijk INLINE IN HEADER ---
template <typename RotationAccessorFunctor, typename OtherAccessorFunctor> class Plane
{
public:
    // Constructor accepteert nu referenties naar Axis objecten, met hun eigen
    // functor templates
    Plane(Axis<RotationAccessorFunctor>& rotationAxis,
          Axis<OtherAccessorFunctor>&    otherAxis)

        : m_rotationAxis(rotationAxis),                                // Initialiseer referentie-leden
          m_otherAxis(otherAxis),                                      // Initialiseer referentie-leden
          m_outputImage(static_cast<int>(rotationAxis.getNumPixels()), // Breedte van de afbeelding
                        static_cast<int>(otherAxis.getNumPixels()),    // Hoogte van de afbeelding
                        QImage::Format_ARGB32)
    // QImage::Format_Mono) //RR!!!
    {
        m_outputImage.fill(Qt::white);
    }

    inline void addPoint(const CCVector3& point)
    {
        m_rotationAxis.addPoint(point);
        m_otherAxis.addPoint(point);

        size_t x = m_rotationAxis.getPixelIndex(point);
        size_t y = m_otherAxis.getPixelIndex(point);
        m_outputImage.setPixel(static_cast<int>(x), static_cast<int>(y), qRgb(255, 0, 0)); // Teken punt als zwart
    }

    // Bereken de totale variantie van de vlakprojectie
    inline float calcTotalVariance() const
    {
        // Nu roepen we de nieuwe getVariance op die de variantie van de counts
        // berekent
        return m_rotationAxis.getVariance() + m_otherAxis.getVariance();
    }

    // Role chars komen nu direct van de functor templates
    inline char          getRotationAxisRoleChar() const { return RotationAccessorFunctor::name; }
    inline char          getOtherAxisRoleChar() const { return OtherAccessorFunctor::name; }
    inline const QImage& getQImage() const { return m_outputImage; }

private:
    Axis<RotationAccessorFunctor>& m_rotationAxis; // Blijft een referentie
    Axis<OtherAccessorFunctor>&    m_otherAxis;    // Blijft een referentie
    QImage                         m_outputImage;
};

using TMatrixBuilder = std::function<void(ccGLMatrix&, float angleRad)>;

// --- AxisRotation Class (Declaration and implementation in header) ---
class AxisRotation
{
public:
    // Constructor die de contextgegevens ontvangt
    AxisRotation(ccPointCloud* cloud, TMatrixBuilder& matrixBuilder, ccMainAppInterface* app)
        : m_cloud(cloud), m_matrixBuilder(matrixBuilder), m_app(app)
    {
        if(!m_cloud)
        {
            dbg("AxisRotation::AxisRotation: cloud can't be null.");
        }
    }

#include <chrono> // De cruciale header

    template <typename RotationAccessorFunctor, typename OtherAccessorFunctor> float rotateAndAnalyzeCoarseMiddleFine()
    {
        // 1. Grove fase (Coarse)
        float bestCoarseAngle =
            analyzeRotationCase<RotationAccessorFunctor, OtherAccessorFunctor>("Coarse", -45.0f, +45.0f, 15.0f, 128);
        std::ostringstream ss_coarse;
        ss_coarse << "Phase: Coarse, Best Angle: " << bestCoarseAngle;
        dbg(ss_coarse.str().c_str());

        /*

        // 2. Midden fase (Middle)
        float bestMiddleAngle = analyzeRotationCase<RotationAccessorFunctor, OtherAccessorFunctor>(
            "Middle", -15.0f + bestCoarseAngle, +15.0f + bestCoarseAngle, 5.0f, 512);
        std::ostringstream ss_middle;
        ss_middle << "Phase: Middle, Best Angle: " << bestMiddleAngle;
        dbg(ss_middle.str().c_str());

        // 3. Fijne fase (Fine)
        float bestFineAngle = analyzeRotationCase<RotationAccessorFunctor, OtherAccessorFunctor>(
            "Fine", -5.0f + bestMiddleAngle, +5.0f + bestMiddleAngle, 1.0f, 1024);
        std::ostringstream ss_fine;
        ss_fine << "Phase: Fine, Best Angle: " << bestFineAngle;
        dbg(ss_fine.str().c_str());

        */
        // return bestFineAngle;
        return bestCoarseAngle;
    }

private:
    template <typename RotationAccessorFunctor, typename OtherAccessorFunctor>
    float analyzeRotationCase(const std::string& phaseName, float startDeg, float endDeg, float stepDeg,
                              unsigned numStrips)
    {
        // Kloon de m_cloud EENMALIG aan het begin van analyzeRotationCase.
        // De verdere rotaties en analyses vinden plaats op deze kloon, zodat m_cloud
        // (de member) onaangetast blijft voor de volgende fasen in
        // rotateAndAnalyzeCoarseMiddleFine.
        std::unique_ptr<ccPointCloud> workingCloud(safeDowncastToPointCloud(m_cloud->clone()));
        if(!workingCloud)
        {
            dbg("Failed to clone point cloud for analyzeRotationCase.");
            // Retourneer een veilige standaardwaarde (bijvoorbeeld de start hoek) bij
            // falen
            return startDeg;
        }

        // Struct om het beste resultaat van de iteraties op te slaan
        struct BestIterationResult
        {
            float bestVariance = -std::numeric_limits<float>::infinity();
            float bestAngle    = 0.0f;
        } bestResult;

        const float epsilon   = 0.001f;
        const float stopAngle = endDeg + epsilon;

        bestResult.bestAngle = startDeg;

        if(!m_cloud)
        {
            dbg("Error: Internal cloud pointer is null in analyzeRotationCase.");
            return bestResult.bestAngle;
        }

        // Matrix voor de incrementele stap-rotatie
        ccGLMatrix stepRotMatrix;
        m_matrixBuilder(stepRotMatrix, CCCoreLib::DegreesToRadians(stepDeg));

        // Matrix voor de initiële rotatie naar startDeg
        ccGLMatrix initialRotMatrix;
        m_matrixBuilder(initialRotMatrix, CCCoreLib::DegreesToRadians(startDeg));

        // Pas de initiële rotatie één keer toe op de workingCloud om bij startDeg te
        // beginnen
        workingCloud->applyRigidTransformation(initialRotMatrix);
        float angle = startDeg; // Begin direct bij de starthoek

        bool firstIteration = true; // Vlag om de eerste iteratie te markeren (geen
                                    // incrementele rotatie nodig)

        // Typedefs voor de accessor functors om de code leesbaarder te maken
        using ZAccessorFunctor = GetZCoord;

        // Typedefs voor de Axis klassen met specifieke accessor functors
        using RotationAxis = Axis<RotationAccessorFunctor>;
        using OtherAxis    = Axis<OtherAccessorFunctor>;
        using Z_Axis       = Axis<ZAccessorFunctor>;
        // De lus doorloopt alle stappen, beginnend bij de (reeds bereikte) starthoek.
        // De incrementele rotatie wordt *voorafgaand* aan de analyse van elke stap
        // toegepast, behalve voor de allereerste meting op 'startDeg'.
        int sort = 0;
        for(; angle <= stopAngle; angle += stepDeg)
        {
            ++sort;
            if(!firstIteration)
            {
                // Pas de incrementele transformatie toe op de workingCloud, alleen na de
                // eerste iteratie
                workingCloud->applyRigidTransformation(stepRotMatrix);
            }
            firstIteration = false; // Na de eerste iteratie, zet de vlag op false

            // Haal de bounding box van de getransformeerde puntenwolk op
            CCVector3 bbMin, bbMax;
            workingCloud->getBoundingBox(bbMin, bbMax);

            // Bereken de bereiken (ranges) langs de verschillende assen
            PointCoordinateType rotRange   = RotationAccessorFunctor{}(bbMax)-RotationAccessorFunctor{}(bbMin);
            PointCoordinateType otherRange = OtherAccessorFunctor{}(bbMax)-OtherAccessorFunctor{}(bbMin);
            PointCoordinateType zRange     = ZAccessorFunctor{}(bbMax)-ZAccessorFunctor{}(bbMin);

            // Bepaal de maximale range om de scaling factor te berekenen
            PointCoordinateType maxOverallRange = std::max({rotRange, otherRange, zRange});
            if(maxOverallRange <= zero)
            { // Voorkom delen door nul of zeer kleine waardes
                maxOverallRange = 1.0f;
            }

            // Bereken de schaalfactor om de punten in een vaste pixelruimte te passen
            float scalingFactor = static_cast<float>(AxisRotation::MAX_LONGEST_SIDE_PIXELS) / maxOverallRange;

            // Bereken de afmetingen in pixels voor elke as
            size_t rotPixels   = static_cast<size_t>(rotRange * scalingFactor);
            size_t otherPixels = static_cast<size_t>(otherRange * scalingFactor);
            size_t zPixels     = static_cast<size_t>(zRange * scalingFactor);

            // Zorg ervoor dat de afmetingen minimaal 1 pixel zijn om fouten te
            // voorkomen
            if(rotPixels == 0) rotPixels = 1;
            if(otherPixels == 0) otherPixels = 1;
            if(zPixels == 0) zPixels = 1;

            // Construeer nieuwe Axis instanties per lus-iteratie met de berekende
            // parameters Plane 1: Rotatie-as en de 'Other' as
            RotationAxis                                         rotationAxis1(bbMin, bbMax, numStrips, rotPixels);
            OtherAxis                                            otherAxis1(bbMin, bbMax, numStrips, otherPixels);
            Plane<RotationAccessorFunctor, OtherAccessorFunctor> plane_RotAxis_OtherAxis(rotationAxis1, otherAxis1);

            // Plane 2: Rotatie-as en de Z-as
            RotationAxis                                     rotationAxis2(bbMin, bbMax, numStrips, rotPixels);
            Z_Axis                                           zAxis2(bbMin, bbMax, numStrips, zPixels);
            Plane<RotationAccessorFunctor, ZAccessorFunctor> plane_RotAxis_zAxis(rotationAxis2, zAxis2);

            // Plane 3: OtherAxis and the Z-axis
            OtherAxis                                     otherAxis3(bbMin, bbMax, numStrips, otherPixels);
            Z_Axis                                        zAxis3(bbMin, bbMax, numStrips, zPixels);
            Plane<OtherAccessorFunctor, ZAccessorFunctor> plane_OtherAxis_zAxis(otherAxis3, zAxis3);

            // Gebruik één stringstream voor alle initiële debug-informatie
            std::ostringstream ss; // Gebruik ostringstream voor gemakkelijker samenvoegen

            ss << "--- Debugging workingCloud in analyzeRotationCase ---" << std::endl;

            if(!workingCloud)
            {
                ss << "Error: workingCloud is nullptr!" << std::endl;
                dbg(ss.str().c_str()); // Roep dbg hier al aan bij een fatale fout
                return 0.0f;
            }

            ss << "workingCloud current size: " << workingCloud->size() << std::endl;

            // Roep dbg één keer aan voor alle verzamelde initiële debug-informatie
            dbg(ss.str().c_str());
            for(unsigned i = 0; i < workingCloud->size(); ++i)
            {
                const CCVector3* pt = workingCloud->getPoint(i);
                if(!pt)
                {
                    continue;
                }
                plane_RotAxis_OtherAxis.addPoint(*pt);
                plane_RotAxis_zAxis.addPoint(*pt);
                plane_OtherAxis_zAxis.addPoint(*pt);
            }

            // Bereken de totale variantie score voor de huidige hoek
            auto score = plane_RotAxis_OtherAxis.calcTotalVariance() + plane_RotAxis_zAxis.calcTotalVariance() +
                         plane_OtherAxis_zAxis.calcTotalVariance();

            // Toon de huidige geaccumuleerde hoek en de berekende variantie in de
            // console De 'phaseName' parameter wordt hier gebruikt voor contextuele
            // logging
            dbg(std::format("[{}] Angle: {:.1f} deg, Total Variance: {:.4f}\n", phaseName, angle, score).c_str());

            // Sla de visualisaties van de vlakken op
            QString imageSavePath =
                QString("/workdir/projects/C++/PluginAutoFit/images/%1/").arg(RotationAccessorFunctor::name);

            logAndSavePlane(plane_RotAxis_OtherAxis, phaseName, sort, angle, score, imageSavePath);
            logAndSavePlane(plane_RotAxis_zAxis, phaseName, sort, angle, score, imageSavePath);
            logAndSavePlane(plane_OtherAxis_zAxis, phaseName, sort, angle, score, imageSavePath);
            dbg("--------------------------------\n");

            // Update het beste resultaat als de huidige score beter is
            if(score > bestResult.bestVariance)
            { // We zoeken naar de maximale variantie
                bestResult.bestVariance = score;
                bestResult.bestAngle    = angle;
            }
        }

        // Retourneer de best gevonden absolute hoek voor deze analysefase
        return bestResult.bestAngle;
    }

    ccPointCloud*   m_cloud;         // Niet-eigendomsreferentie naar de cloud
    TMatrixBuilder& m_matrixBuilder; // Referentie naar de matrix builder functie

    ccMainAppInterface*   m_app;
    const static unsigned MAX_LONGEST_SIDE_PIXELS = 1000; // Voorbeeldwaarde
};

// Functie om de breedte en hoogte van een plane te loggen
// 'T' staat hier voor het specifieke type van de Plane objecten die je
// doorgeeft (bijv. Plane<int>, Plane<double> etc.)
template <typename T> void logPlaneInfo(const T& plane)
{
    std::cout << std::format("Plane {}{} (Width: {} Height: {})\n", plane.getRotationAxisRoleChar(),
                             plane.getOtherAxisRoleChar(), plane.getQImage().width(), plane.getQImage().height());
}

std::string angleToSortableString(float angle) { return std::format("{:02.0f}", angle); }

// Functie om een afbeelding van een plane op te slaan
// 'T' staat hier voor het specifieke type van de Plane objecten die je
// doorgeeft.
template <typename T>
bool savePlaneImage(const T& plane, const std::string& phaseName, int sort, float angle, float score,
                    const QString& imageSaveDirectory)
{
    // Bestandsnaam genereren
    // Usage in your Qt code:
    std::string baseFileName =
        std::format("{}_{}{}_{:03}_{:02.0f}_{:.2f}.png", phaseName, plane.getRotationAxisRoleChar(),
                    plane.getOtherAxisRoleChar(), sort, angle, score);

    QString fullPath = imageSaveDirectory + QString::fromStdString(baseFileName);

    // Afbeelding opslaan
    if(plane.getQImage().save(fullPath))
    {
        std::cout << std::format("Saved image: {}\n", fullPath.toStdString());
        return true;
    } else
    {
        std::cerr << std::format("Failed to save image: {}\n", fullPath.toStdString());
        return false;
    }
}

template <typename T>
void logAndSavePlane(const T& plane, const std::string& phaseName, int sort, float angle, float score,
                     const QString& imageSaveDirectory)
{
    logPlaneInfo(plane);
    savePlaneImage(plane, phaseName, sort, angle, score, imageSaveDirectory);
}

// --- Explicit Template Instantiations ---
// These instantiations force the compiler to generate code for these specific
// template specializations, allowing debuggers to set breakpoints.

// Instantiations for Axis class (used by Plane)
template class __attribute__((used)) Axis<GetXCoord>;
template class __attribute__((used)) Axis<GetYCoord>;
template class __attribute__((used)) Axis<GetZCoord>;

// Instantiations for Plane class
// Based on AxisRotation::analyzeRotationCase, the following Plane types are created:
// Plane<RotationAccessorFunctor, OtherAccessorFunctor>
// Plane<RotationAccessorFunctor, ZAccessorFunctor> (where ZAccessorFunctor is GetZCoord)
template class __attribute__((
    used)) Plane<GetXCoord, GetYCoord>; // For Plane_RotAxis_OtherAxis when Rot is X, Other is Y
template class __attribute__((
    used)) Plane<GetYCoord, GetXCoord>; // For Plane_RotAxis_OtherAxis when Rot is Y, Other is X

// Instantiations for logPlaneInfo, savePlaneImage, logAndSavePlane
// These are instantiated for the specific Plane types generated above.
template void __attribute__((used)) logPlaneInfo<Plane<GetXCoord, GetYCoord>>(const Plane<GetXCoord, GetYCoord>& plane);
template bool __attribute__((used))
savePlaneImage<Plane<GetXCoord, GetYCoord>>(const Plane<GetXCoord, GetYCoord>& plane, const std::string& phaseName,
                                            int sort, float angle, float score, const QString& imageSaveDirectory);
template void __attribute__((used))
logAndSavePlane<Plane<GetXCoord, GetYCoord>>(const Plane<GetXCoord, GetYCoord>& plane, const std::string& phaseName,
                                             int sort, float angle, float score, const QString& imageSaveDirectory);

template void __attribute__((used)) logPlaneInfo<Plane<GetYCoord, GetXCoord>>(const Plane<GetYCoord, GetXCoord>& plane);
template bool __attribute__((used))
savePlaneImage<Plane<GetYCoord, GetXCoord>>(const Plane<GetYCoord, GetXCoord>& plane, const std::string& phaseName,
                                            int sort, float angle, float score, const QString& imageSaveDirectory);
template void __attribute__((used))
logAndSavePlane<Plane<GetYCoord, GetXCoord>>(const Plane<GetYCoord, GetXCoord>& plane, const std::string& phaseName,
                                             int sort, float angle, float score, const QString& imageSaveDirectory);

template void __attribute__((used)) logPlaneInfo<Plane<GetXCoord, GetZCoord>>(const Plane<GetXCoord, GetZCoord>& plane);
template bool __attribute__((used))
savePlaneImage<Plane<GetXCoord, GetZCoord>>(const Plane<GetXCoord, GetZCoord>& plane, const std::string& phaseName,
                                            int sort, float angle, float score, const QString& imageSaveDirectory);
template void __attribute__((used))
logAndSavePlane<Plane<GetXCoord, GetZCoord>>(const Plane<GetXCoord, GetZCoord>& plane, const std::string& phaseName,
                                             int sort, float angle, float score, const QString& imageSaveDirectory);

template void __attribute__((used)) logPlaneInfo<Plane<GetYCoord, GetZCoord>>(const Plane<GetYCoord, GetZCoord>& plane);
template bool __attribute__((used))
savePlaneImage<Plane<GetYCoord, GetZCoord>>(const Plane<GetYCoord, GetZCoord>& plane, const std::string& phaseName,
                                            int sort, float angle, float score, const QString& imageSaveDirectory);
template void __attribute__((used))
logAndSavePlane<Plane<GetYCoord, GetZCoord>>(const Plane<GetYCoord, GetZCoord>& plane, const std::string& phaseName,
                                             int sort, float angle, float score, const QString& imageSaveDirectory);

// Instantiations for AxisRotation::analyzeRotationCase
// These are called from rotateAndAnalyzeCoarseMiddleFine.
template float __attribute__((used))
AxisRotation::analyzeRotationCase<GetXCoord, GetYCoord>(const std::string& phaseName, float startDeg, float endDeg,
                                                        float stepDeg, unsigned numStrips);
template float __attribute__((used))
AxisRotation::analyzeRotationCase<GetYCoord, GetXCoord>(const std::string& phaseName, float startDeg, float endDeg,
                                                        float stepDeg, unsigned numStrips);

// Instantiations for AxisRotation::rotateAndAnalyzeCoarseMiddleFine
// You had these already, keeping them.
template float __attribute__((used)) AxisRotation::rotateAndAnalyzeCoarseMiddleFine<GetXCoord, GetYCoord>();
template float __attribute__((used)) AxisRotation::rotateAndAnalyzeCoarseMiddleFine<GetYCoord, GetXCoord>();
