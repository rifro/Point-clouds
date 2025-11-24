#pragma once
#include <cstdint>
#include <vector>
#include <span>
#include "pipetypes.h"  // Bocari::Pipe, Plane

namespace Bocari::PipeFitting {

class PipeFittingDetector {
public:
    std::vector<Pipe> process(  // Unified output as Pipe
        std::span<const Pipe> pipes,
        std::span<const Plane> planes = {},
        int max_gap_fill_mm = 1000,
        int max_extension_mm = 2000
    );

private:
    struct Segment;
    void snap_to_planes(std::vector<Segment>& segs, std::span<const Plane> planes, int max_ext_mm);
    void merge_overlapping(std::vector<Segment>& segs);
    void fill_gaps(std::vector<Segment>& segs, int max_gap_mm);
    std::vector<Pipe> cluster_nodes(const std::vector<Segment>& segs);
    u8 determine_elbow_plane(u8 axis1, u8 axis2) const noexcept;
    Point3mm calculate_torus_center(const Segment* s1, const Segment* s2, u16 buisStraal_mm) const;
};

} // Bocari::PipeFitting
