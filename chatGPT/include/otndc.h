#pragma once
#include <cuda_runtime.h>
#include <stdint.h>

// === Resultaat voor hoofdassen ===
struct AssenResultaat {
    float v[3][3];   // 3 assen (unit)
    float score[3];  // sterkte per as (aantal gewogen stemmen)
    int   dimensie;  // 1D/2D/3D (hier: 1 of 2 of 3 als we later uitbreiden)
};

// === Config (OTNDC) ===
// Namen NL; geen magic numbers.
struct OTConfig {
    // Boeren-ring (ruisfilter)
    float binnenStraal2;   // min ring (kwadraat)
    float buitenStraal2;   // max ring (kwadraat)

    // Paar-acceptatie (orthogonaliteit)
    // Voor "hoek >= 20°" gebruiken we |dot| <= cos(70°) ~= 0.342.
    float maxDotOrtho;     // max |dot(n1,n2)| om kruispaar toe te laten (bv 0.34)

    // Warmup/minimum
    int   warmupMin;       // minimum stemmen om een richting te houden (host-reduce)

    // Kernel/block
    int   blokGrootte;     // threads per blok (bv 256)
    int   partnerZoekRadius; // max offset in tile om partner te zoeken (bv 32)
};

// Interne slot-accu voor as-samples (globaal).
struct OTAsAccu {
    float3 som;    // vectorsom van a-samples
    int    count;  // aantal (gewogen) stemmen
};

// Run via paren → schat een top-as richting (v[0])
// x/y/z: arrays met punten (getransleerde cloud: stralen ≈ p/|p|)
// N: aantal punten
// out: vult v[0], score[0], dimensie (>=1 als er steun is)
void runOTNDC_pairs(
    const float* x, const float* y, const float* z, int N,
    AssenResultaat* out,
    const OTConfig& cfg);
