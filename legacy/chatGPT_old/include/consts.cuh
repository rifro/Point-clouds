#pragma once
#include <cuda_runtime.h>

// ============================================================
// Globale (tuneable) __constant__s voor orkestratie & logging
// ============================================================

// Log iedere N "hits" (votes die Gate A passeren)
#ifndef ORCH_LOG_INTERVAL
#define ORCH_LOG_INTERVAL 1024u
#endif

// 1 = ook naar log.txt schrijven (append), 0 = alleen stdout
#ifndef ORCH_LOG_TO_FILE
#define ORCH_LOG_TO_FILE 1
#endif

// Format: vaste precisie voor hoeken/varianties
#ifndef ORCH_LOG_PREC_DEG
#define ORCH_LOG_PREC_DEG 3
#endif

#ifndef ORCH_LOG_PREC_VAR
#define ORCH_LOG_PREC_VAR 4
#endif

*Globale __constant__en voor JBF -
    pipeline.*Waarden in meters.Houd rekening met schaal(__host__ kan schalen als scene > 1km).*/ namespace Consts
{
    // ---------- LIDAR / ring ----------
    __host__ __device__ constexpr float LIDAR_HALF_ERROR   = 0.001f;                           // 1 mm
    __host__ __device__ constexpr float R1_CLUSTER         = 0.004f;                           // 4 mm
    __host__ __device__ constexpr float R2_RING            = 0.012f;                           // 12 mm
    __host__ __device__ constexpr float R2_HALF_CLAIM      = 0.006f;                           // 6 mm
    __host__ __device__ constexpr float R2_HALF_CLAIM_SAFE = R2_HALF_CLAIM - LIDAR_HALF_ERROR; // claim-marge

    // ---------- slab ----------
    __host__ __device__ constexpr float SLAB_TOL         = 0.003f; // 3 mm | vlak-inliers
    __host__ __device__ constexpr int   SLAB_MIN_INLIERS = 4;

    // ---------- chaos veto ----------
    __host__ __device__ constexpr float CHAOS_RANGE_MIN  = 0.008f; // 8 mm
    __host__ __device__ constexpr int   CHAOS_INLIER_MAX = 3;      // <4 => zwak

    // ---------- stemmen / LRU ----------
    __host__ __device__ constexpr int   LRU_SLOTS       = 8;
    __host__ __device__ constexpr int   WARMUP_SEEDS    = 200;
    __host__ __device__ constexpr float ORTHO_COS_MAX   = 0.2f;  // |dot| < 0.2 ~ 11°
    __host__ __device__ constexpr float SIMILAR_COS_MIN = 0.98f; // bijna dezelfde richting
    __host__ __device__ constexpr float EVICT_RATIO     = 5.0f;  // score[i]*5 < maxScore => evict
}