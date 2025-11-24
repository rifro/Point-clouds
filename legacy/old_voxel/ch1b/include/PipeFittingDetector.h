#pragma once

#include <cstdint>
#include <vector>
#include <span>
#include <array>

namespace Bocari::PipeFitting {

struct Point3mm { int32_t x, y, z; };   // 1 unit = 3 mm

struct InputPipe {
    Point3mm start;
    Point3mm end;
    uint16_t buisStraal_mm = 0;
};

struct Plane {
    uint8_t axis;      // 0=X, 1=Y, 2=Z
    int32_t coord;     // 3 mm units
};

// 44 bytes – GPU-vriendelijk
struct Fitting {
    uint8_t  type_and_flags = 0;           // [7:hypothetical] [6-0: jouw enum]
    uint8_t  direction_count : 3;
    uint8_t  elbow_plane     : 2;           // 0=XZ, 1=XY, 2=YZ (alleen Elbow)
    uint8_t  _pad            : 3;

    uint16_t packed_directions = 0;         // 4× (axis<<1|sign) → 12 bits
    uint16_t buisStraal_mm = 0;             // grootste straal op deze node
    uint16_t torusStraal_mm = 0;            // ruwe schatting → GPU verfijnt
    uint16_t reducer_to_buisStraal_mm = 0;  // 0 = geen reducer

    Point3mm pos;                           // node centrum
    Point3mm elbow_center;                  // torus center (alleen Elbow)

    void set_hypothetical(bool h) noexcept { if (h) type_and_flags |= 0x80; else type_and_flags &= 0x7F; }
    void set_type(uint8_t t)       noexcept { type_and_flags = (type_and_flags & 0x80) | (t & 0x7F); }
    bool is_hypothetical()   const noexcept { return type_and_flags & 0x80; }
    uint8_t type()           const noexcept { return type_and_flags & 0x7F; }
};

class PipeFittingDetector {
public:
    std::vector<Fitting> process(
        std::span<const InputPipe> pipes,
        std::span<const Plane> planes = {},
        int max_gap_fill_mm = 5000,
        int max_extension_mm = 2000
    );
};

} // namespace Bocari::PipeFitting
