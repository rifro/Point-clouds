#pragma once
#include <stdint.h>
#include "types.cuh"

namespace Tuning {
// Afstandsdrempels voor snelle heuristiek (vlak check) per voxel/triangle:
constexpr float EPS_NEAR2 = 1e-6f;   // "dichtbij": d^2 ≤ EPS_NEAR2  (bv. 1 mm → 0.001^2)
constexpr float TAU_FAR2  = 0.04f;   // "ver":      d^2 ≥ TAU_FAR2   (bv. 20 mm → 0.02^2)

// Waarom NEAR "eps" en FAR "tau"?
// - "eps" gebruiken we traditioneel voor "kleine" toleranties (nabij het vlak).
// - "tau" (τ) als "grotere drempel" om duidelijk te maken dat dit geen mini-tolerantie is.


    static constexpr int   MIN_NEAR  = 6;         // min support
    static constexpr int   MAX_FAR   = 2;

// Buisstraalband (annulus) voor annulus/wedge scan:
constexpr float R_MIN     = 0.025f;  // 2.5 cm  (kleinste te verwachten buis)
constexpr float R_MAX     = 0.40f;   // 40  cm  (typisch plafond/utility)
                                      // (kan naar 1.0f als jouw data dat vereist)

// Halve-cirkel sectoren (zodat +v en -v samenvallen voor as-richting):
constexpr int   SECTORS_HALF = 24;   // 24 sectoren in [0, π). Fijner → nauwkeuriger, duurder.

    // globale octa-grid voor richtingkeys
    static constexpr u32 OCT_NX = 64;
    static constexpr u32 OCT_NY = 64;

    // LRU-lite slots (nu niet gebruikt in deze minimale set)
    static constexpr int LRU_K = 8;
}

enum PuntLabel : u8 { LABEL_NONE=0, LABEL_RUIS, LABEL_VLAK, LABEL_BUIS };
enum VoxelKlasse : u8 { VOX_GEEN=0, VOX_VLAK_SEED, VOX_BUIS_KAND, VOX_RUIS };

struct VoxelBereik { u32 start; u32 eind; };  // [start,eind)

struct KeyCount { u32 key; u32 count; };
