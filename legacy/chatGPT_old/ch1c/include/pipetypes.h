#pragma once
#include <cstdint>

namespace Bocari {

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using i32 = std::int32_t;

// 3mm grid point: 1 unit = 3 mm
struct Point3mm
{
    i32 x{};
    i32 y{};
    i32 z{};

    bool operator==(const Point3mm&) const = default;
};

// Gemeenschappelijke type voor alles wat we willen labelen
// (punten, pipes, fittings)
enum PipeType : u8
{
    PT_Unlabeled = 0,
    PT_Noise     = 1,
    PT_Plane     = 2,
    PT_Pipe      = 3,
    PT_Elbow     = 4,
    PT_Tee       = 5,
    PT_Cross     = 6,
    PT_Reducer   = 7,
    PT_GapFill   = 8, // hypothetische buis (topologisch afgeleid)
    PT_Blind     = 9  // doodlopend knooppunt
    // ruimte zat voor uitbreiding
};

// Hoogste bit = "hypothetical" vlag
constexpr u8 PT_HYP = 0x80;

inline PipeType pipeType(u8 tf) noexcept
{
    return static_cast<PipeType>(tf & 0x7F);
}
inline bool isHyp(u8 tf) noexcept
{
    return (tf & PT_HYP) != 0;
}
inline u8 makeTF(PipeType t, bool hyp) noexcept
{
    return (static_cast<u8>(t) & 0x7F) | (hyp ? PT_HYP : 0);
}

// Rechte buis in 3mm grid
// - start/end: 3mm grid
// - buisStraal_mm: straal in mm
// - axisIndex: 0=X,1=Y,2=Z in ideale frame
// - confidence: 0..255 (optioneel)
// - clusterId: voor latere labeling
struct Pipe
{
    Point3mm start;
    Point3mm end;
    u16      buisStraal_mm{};
    u8       axisIndex{};   // 0,1,2
    u8       confidence{};  // 0..255
    u32      clusterId{};   // 0 = nog geen cluster
};

} // namespace Bocari
