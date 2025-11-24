#pragma once

// We werken altijd in millimeters.
// Dus: ints = round( (p - bbMin) * 1000 )
static constexpr float KWANTISATIE_SCHAAL = 1000.0f;  // meter → mm
static constexpr int   KWANTISATIE_MAX_BITS = 21;     // 0..2^21 (2M mm ≈ 2000m)

