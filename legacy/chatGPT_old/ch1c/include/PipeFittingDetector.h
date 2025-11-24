#pragma once

#include <cstdint>
#include <vector>
#include <span>

#include "pipetypes.h"

namespace Bocari::PipeFitting {

using Bocari::u8;
using Bocari::u16;
using Bocari::u32;
using Bocari::i32;
using Bocari::Point3mm;
using Bocari::Pipe;
using Bocari::PipeType;
using Bocari::pipeType;
using Bocari::isHyp;
using Bocari::makeTF;

// Optionele vlak-informatie (bijv. vloer/muur-planes) in 3mm grid
struct Plane
{
    u8  axis;   // 0=X,1=Y,2=Z
    i32 coord;  // in 3mm units
};

// Output-node (elbow, tee, cross, reducer, gapfill, blind)
// 44 bytes, GPU-vriendelijk
struct Fitting
{
    u8        type_and_flags = 0; // [7:hyp] [6-0: PipeType]

    Point3mm  pos{};              // center van de fitting (node)
    Point3mm  elbow_center{};     // alleen bij Elbow: torus center
    Point3mm  elbow_corner{};     // alleen bij Elbow: snijpunt assen

    u16       buisStraal_mm = 0;          // grootste buis op deze node
    u16       torusStraal_mm = 0;         // geschatte bochtradius (GPU verfijnt)
    u16       reducer_to_buisStraal_mm=0; // bij Reducer: doel-straal

    u8        direction_count : 3 {0};    // 0–4
    u8        elbow_plane     : 2 {0};    // 0=XZ, 1=XY, 2=YZ
    u8        reserved        : 3 {0};

    // 4 richtingen × 3 bits = 12 bits → past in u16
    // per richting: 2 bits as (0..2) + 1 bit sign
    u16       packed_directions = 0;

    // helpers
    void setHyp(bool h) noexcept     { type_and_flags = makeTF(type(), h); }
    void setType(PipeType t) noexcept{ type_and_flags = makeTF(t, isHyp()); }
    bool     isHyp() const noexcept  { return Bocari::isHyp(type_and_flags); }
    PipeType type()  const noexcept  { return pipeType(type_and_flags); }
};

// Hoofdklasse: CPU-topologie op 3mm-grid
class PipeFittingDetector
{
public:
    // pipes: rechte buizen (3mm grid), rechtstreeks uit jouw GPU seeds
    // planes: optionele globale vlakken (vloer/muren)
    std::vector<Fitting> process(
        std::span<const Pipe>  pipes,
        std::span<const Plane> planes        = {},
        int max_gap_fill_mm                  = 5000,
        int max_extension_mm                 = 2000
    );

private:
    struct Segment;

    void snap_to_planes(std::vector<Segment>& segs,
                        std::span<const Plane> planes,
                        int max_ext_mm);

    void merge_overlapping(std::vector<Segment>& segs);

    void fill_gaps(std::vector<Segment>& segs,
                   int max_gap_fill_mm);

    void cluster_nodes(const std::vector<Segment>& segs,
                       std::vector<Fitting>& fittings);

    u8 determine_elbow_plane(u8 axis1, u8 axis2) const noexcept;

    Point3mm calculate_torus_center(const Segment* s1,
                                    const Segment* s2,
                                    u16 buisStraal_mm) const;
};

} // namespace Bocari::PipeFitting
