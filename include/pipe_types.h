#pragma once
#include <cstdint>

namespace Bocari {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using i32 = std::int32_t;

struct Point3mm { i32 x, y, z; bool operator==(const Point3mm&) const = default; };

enum PipeType : u8 {
    Unlabeled = 0,
    Noise = 1,
    Plane = 2,
    Pipe = 3,
    Elbow = 4,
    Tee = 5,
    Cross = 6,
    Reducer = 7,
    Blind = 8
};

constexpr u8 Hypothetical = 0x80;  // Bit 7
constexpr u8 Confirmed = 0x40;  // Bit 6

inline PipeType pipeType(u8 tf) noexcept { return static_cast<PipeType>(tf & 0x3F); }
inline bool isHyp(u8 tf) noexcept { return (tf & Hypothetical) != 0; }
inline bool isConfirmed(u8 tf) noexcept { return (tf & Confirmed) != 0; }
inline u8 makeTF(PipeType t, bool hyp, bool conf = false) noexcept {
    return (static_cast<u8>(t) & 0x3F) | (hyp ? Hypothetical : 0) | (conf ? Confirmed : 0);
}

struct Plane { u8 axis; i32 coord; };  // 3mm units

struct Pipe {
    Point3mm start;
    Point3mm end;
    u16 buisStraal_mm{};
    u8 axisIndex{};  // 0-2
    u8 confidence{};  // 0-255
    u32 clusterId{};  // Oplopend per object
    u8 type = Pipe;
    u8 flags = 0;  // Hyp/Confirmed
};

} // Bocari
