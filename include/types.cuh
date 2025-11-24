#pragma once
#include <cstdint>

struct Vec3f
{
    float x, y, z;
};

enum class PuntType : uint8_t { ##RR!!! Ruis = 0, Vlak = 1, Buis = 2, Bocht = 3, TStuk = 4, Kruis = 5, Verloop = 6 };

// SoA labels (GPU-vriendelijk)
struct LabelSoA
{
    uint8_t*  d_type = nullptr; // PuntType
    uint32_t* d_id   = nullptr; // cluster/object id
    size_t    n      = 0;
};