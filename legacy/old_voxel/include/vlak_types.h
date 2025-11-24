#pragma once
#include <cstdint>
#include <cuda_runtime.h>

namespace jbf {

enum Label : uint8_t {
    ONGELABELD = 0,
    RUIS       = 1,
    VLAK       = 2,
    // later: BUIS=3, BOCHT=4, T_STUK=5, ...
};

struct PlaneHyp {
    float3 n;            // unit normaal
    float  d;            // vlakvergelijking: n·x = d
    uint32_t count;      // #kept punten (na slab + trim)
    float    rms;        // kwaliteit (optioneel gate)
    uint32_t areaHint;   // ruwe maat voor oppervlak (projectie/bbox/extent)
};

// Tuning (kan later naar __constant__ als runtime zelden wijzigt)
struct VlakTuning {
    float cosParallelMax = 0.9f;        // τ‖ voor u-keuze
    float slabEps        = 0.005f;      // fine slab half-thickness (m)
    float maxRefineDeg   = 2.0f;        // clamp n* deviate
    float edgeMin        = 0.03f;       // 3 cm, min driehoekszijde (m)
    float edgeMax        = 0.30f;       // 30 cm, max driehoekszijde (m)
    float hoekMaxDeg     = 10.0f; // zaad-Δhoek tov doel-normaal
    float ruisStraal     = 0.003f; // 3 mm (boeren 2e buur)
    int   kTrim          = 3;           // Trim-K
    uint32_t minSupport  = 200;         // minimale punten voor accept
    float maxRms         = 0.0035f;     // maximale RMS voor accept
};

} // namespace jbf
