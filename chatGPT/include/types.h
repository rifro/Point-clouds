// Minimale types/structs voor orkestratie (sluit aan op eerdere parts)
#pragma once
#include <cstdint>
#include <cmath>

using u8  = uint8_t;
using u32 = uint32_t;
using u64 = uint64_t;

struct Vec3f { float x,y,z; };
struct Quatf { float w,x,y,z; };

struct StemKeyVal {
u64   key;
float value;
};

// Helpers (declaraties) — implementatie in wiskunde_utils.cuh
inline Vec3f make_vec3f(float x,float y,float z){ return Vec3f{x,y,z}; }