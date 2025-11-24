#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include "include/config.h"
#include "include/wiskunde_utils.cuh"

// Helper
static inline float clampf(float x, float a, float b) {
    return std::max(a, std::min(b, x));
}

// ------------------------------------------------------------
// __host__ visuele check — rotatie-voorspeller
// ------------------------------------------------------------
namespace __host__vis {

static inline void print_frame_nudge_preview(const Vec3f& asX,
                                             const Vec3f& asY,
                                             const Vec3f& asZ)
{
    Vec3f Z{0,0,1};
    Vec3f v = cross(Z, asZ);
    float c = dot(Z, asZ);
    float ang = std::acos( std::clamp(c, -1.0f, 1.0f) );
    Quat qz = (std::fabs(ang) < 1e-6f) ? Quat{1,0,0,0}
                                      : quat_from_axis_angle(v, ang);

    Vec3f X{1,0,0};
    Vec3f Xp = quat_rotate(qz, X);
    Vec3f t  = cross(Xp, asX);
    float c2 = dot(Xp, asX);
    float ang2 = std::acos( std::clamp(c2, -1.0f, 1.0f) );
    Quat qx = (std::fabs(ang2) < 1e-6f) ? Quat{1,0,0,0}
                                       : quat_from_axis_angle(t, ang2);

    Quat q = quat_mul(qx, qz);

    float yawDeg=0.f, pitchDeg=0.f;
    quat_to_ZY_angles(q, yawDeg, pitchDeg);

    float clip = cfg::get().stemmen.clipKleinGraads;
    clip_small_degrees(clip, yawDeg);
    clip_small_degrees(clip, pitchDeg);

    float maxLock = cfg::get().stemmen.maxLockGraads;
    bool reject   = (std::fabs(yawDeg) > maxLock ||
                     std::fabs(pitchDeg) > maxLock);

    std::cout << "[FrameCheck] Rotaties (Z,Y): yaw="
              << yawDeg << "°, pitch=" << pitchDeg << "°";
    if (reject) {
        std::cout << " [REJECT >" << maxLock << "°]";
    }
    std::cout << "\n";
}

} // namespace __host__vis

// ------------------------------------------------------------
// Struct for logging
// ------------------------------------------------------------
struct StatBlok {
    uint32_t hitsTotaal   = 0;
    uint32_t hitsNieuw    = 0;
    uint32_t nearCount    = 0;
    uint32_t okCount      = 0;
    uint32_t slotCount    = 0;
    float    dHoekDeg     = 0.f;
    float    varCos       = 0.f;
    bool     locked       = false;
};

static inline void log_status(const StatBlok& s)
{
    std::cout << "hits+=" << s.hitsNieuw
              << " total=" << s.hitsTotaal
              << " near=" << s.nearCount
              << " ok=" << s.okCount
              << " slots=" << s.slotCount
              << " dHoek=" << std::fixed << std::setprecision(2)
              << s.dHoekDeg << "°"
              << " varCos=" << std::setprecision(4) << s.varCos
              << " " << (s.locked ? "LOCKED" : "-")
              << std::endl;
}

// ------------------------------------------------------------
// Dummy orchestration loop — compiles now
// When real __device__ stats exist, replace dummy block
// ------------------------------------------------------------
void h_run_orchestratie_logging_loop()
{
    StatBlok acc;
    uint32_t intervalHits = 0;

    // Dummy loop just once for test
    {
        uint32_t newlyAccepted = ORCH_LOG_INTERVAL;
        uint32_t nearIn        = ORCH_LOG_INTERVAL * 3 / 4;
        uint32_t okIn          = ORCH_LOG_INTERVAL * 1 / 4;
        uint32_t activeSlots   = 3;
        float dHoekDegSample   = 0.18f;
        float varCosSample     = 0.0017f;
        bool lockedNow         = true;

        intervalHits       += newlyAccepted;
        acc.hitsTotaal     += newlyAccepted;
        acc.nearCount      += nearIn;
        acc.okCount        += okIn;
        acc.slotCount       = activeSlots;
        acc.dHoekDeg        = dHoekDegSample;
        acc.varCos          = varCosSample;
        acc.locked          = lockedNow;
        acc.hitsNieuw       = intervalHits;

        log_status(acc);

        // ==== FINAL axes preview ====
        // 🔥 Dit is nu gewoon identity frame
        Vec3f asX{1,0,0}, asY{0,1,0}, asZ{0,0,1};
        __host__vis::print_frame_nudge_preview(asX, asY, asZ);
    }
}

