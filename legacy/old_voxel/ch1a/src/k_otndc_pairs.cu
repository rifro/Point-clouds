#include <cuda_runtime.h>
#include <math_constants.h>
#include "otndc.h"

// ============ device helpers ============
__device__ __forceinline__ float3 make3(float x,float y,float z){
    return make_float3(x,y,z);
}

__device__ __forceinline__ float dot3(float3 a,float3 b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

__device__ __forceinline__ float3 cross3(float3 a,float3 b){
    return make3(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x
    );
}

__device__ __forceinline__ float3 norm3(float3 v){
    float n2 = v.x*v.x + v.y*v.y + v.z*v.z;
    if(n2 < 1e-20f) return make3(0,0,0);
    float inv = rsqrtf(n2);
    return make3(v.x*inv, v.y*inv, v.z*inv);
}

// sin(theta)^16 helper: neem dot = |cos(theta)|, dan:
// sin^2 = 1 - dot^2;
// sin^4 = (sin^2)^2;
// sin^8 = (sin^4)^2;
// sin^16 = (sin^8)^2;
__device__ __forceinline__ float sin16_from_abs_cos(float absCos){
    float c2 = absCos * absCos;
    float s2 = fmaxf(0.0f, 1.0f - c2);
    float s4 = s2 * s2;
    float s8 = s4 * s4;
    float s16 = s8 * s8;
    return s16;
}

// ============ kernel ============
//
// Invoer: dNormals (triangle-normals), lengte N.
// Idee:
//  * elke thread pakt n_i
//  * in gedeeld geheugen tile we normals
//  * zoek binnen +-partnerZoekRadius een partner n_j
//    met |dot(n_i, n_j)| <= maxDotOrtho en |dot| zo klein mogelijk
//  * bereken a = normalize(n_i x n_j)
//  * gewicht w = base * sin(theta)^16
//      - base = 1.0 voor vlak-burst
//      - base = 0.5 voor "rest" (buis / noisy)
//  * atomics naar globale OTAsAccu
extern "C" __global__
void k_otndc_pairs(
    const Vec3f* __restrict__ normals,
    std::uint32_t N,
    float maxDotOrtho,
    int   partnerZoekRadius,
    float vlakBurstCosMin,
    OTAsAccu* __restrict__ gAccu)
{
    extern __shared__ float sMem[]; // 3 * blockDim.x floats
    float* sx = sMem;
    float* sy = sx + blockDim.x;
    float* sz = sy + blockDim.x;

    const int gid  = blockIdx.x * blockDim.x + threadIdx.x;
    const int lane = threadIdx.x;

    // Laad deze normal in registers
    float3 n = make3(0,0,0);
    if(gid < (int)N){
        Vec3f v = normals[gid];
        n = make3(v.x, v.y, v.z);
    }

    // Normaliseer, anders kloppen dot en cross-hoeken niet
    n = norm3(n);

    // Tile in shared memory
    sx[lane] = n.x;
    sy[lane] = n.y;
    sz[lane] = n.z;
    __syncthreads();

    if(gid >= (int)N) return;

    // Bepaal vlak-burst heuristiek mbv vorige normal in globale volgorde
    bool isVlakBurst = false;
    if(gid > 0){
        // vorige normal in blok (als binnen zelfde block)
        float3 prev = n;
        if(lane > 0){
            prev = make3(sx[lane-1], sy[lane-1], sz[lane-1]);
        } else {
            // lane == 0 → vorige komt uit vorig block; globale read
            Vec3f vp = normals[gid-1];
            prev = norm3(make3(vp.x,vp.y,vp.z));
        }
        float dprev = fabsf(dot3(n, prev));
        if(dprev >= vlakBurstCosMin) {
            isVlakBurst = true;
        }
    }

    // Zoek partner in lokale omgeving binnen block
    float bestAbsDot = 1.0f;
    float3 bestPartner = make3(0,0,0);
    const int B = blockDim.x;

    // lokale index range
    int start = max(0, lane - partnerZoekRadius);
    int end   = min(B-1, lane + partnerZoekRadius);

    for(int j = start; j <= end; ++j){
        if(j == lane) continue;
        float3 m = make3(sx[j], sy[j], sz[j]);
        float d  = dot3(n, m);
        float ad = fabsf(d);
        if(ad <= maxDotOrtho && ad < bestAbsDot){
            bestAbsDot   = ad;
            bestPartner  = m;
        }
    }

    // Geen geschikte partner gevonden
    if(bestAbsDot > maxDotOrtho) return;

    // As-sample
    float3 a = norm3(cross3(n, bestPartner));
    float3 zero = make3(0,0,0);
    if(a.x == 0.f && a.y == 0.f && a.z == 0.f) return;

    // Gewicht: base * sin(theta)^16
    float sin16 = sin16_from_abs_cos(bestAbsDot);
    if(sin16 <= 1e-6f) return;

    float base = isVlakBurst ? 1.0f : 0.5f;
    float w = base * sin16;

    // Atomics naar globale accumulator
    atomicAdd(&gAccu->somAs.x, a.x * w);
    atomicAdd(&gAccu->somAs.y, a.y * w);
    atomicAdd(&gAccu->somAs.z, a.z * w);
    atomicAdd(&gAccu->gewichtSom, w);
    atomicAdd(&gAccu->paarCount, 1);
}
