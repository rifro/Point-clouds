#include "otndc.h"
#include <math_constants.h>

// ---- device helpers ----
__device__ __forceinline__ float3 make3(float x,float y,float z){
    return make_float3(x,y,z);
}

__device__ __forceinline__ float dot3(float3 a,float3 b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

__device__ __forceinline__ float3 norm3(float3 v){
    float n2 = v.x*v.x + v.y*v.y + v.z*v.z;
    if(n2 < 1e-20f) return make3(0,0,0);
    float inv = rsqrtf(n2);
    return make3(v.x*inv, v.y*inv, v.z*inv);
}

// ---- kernel: punten -> stralen, stemmen op direction-slots ----
// We gebruiken hier punten als stralen: p -> v = normalize(p).
// Boeren-ring filter: alleen punten met r^2 in [binnenStraalSqd, buitenStraalSqd].
__global__ void k_otndc_stralen(
    const Vec3f* __restrict__ punten,
    uint32_t N,
    float binnen2,
    float buiten2,
    OTSlot* __restrict__ slots,
    int maxSlots)
{
    const uint32_t gid = blockIdx.x * blockDim.x + threadIdx.x;
    if(gid >= N) return;

    Vec3f pH = punten[gid];
    float3 p = make3(pH.x, pH.y, pH.z);
    float r2 = dot3(p, p);
    if(r2 < binnen2 || r2 > buiten2) return;

    float3 v = norm3(p);
    if(v.x == 0.f && v.y == 0.f && v.z == 0.f) return;

    // Slot zoeken: of eerste lege, of beste dot-product.
    int best = -1;
    float bestDot = -1.f;

    for(int s=0; s<maxSlots; ++s){
        int c = atomicAdd(&slots[s].count, 0); // alleen lezen
        if(c == 0){
            // probeer dit slot te claimen
            int old = atomicCAS(&slots[s].count, 0, 1);
            if(old == 0){
                // wij zijn de eerste: zet som = v, count = 1
                atomicExch(&slots[s].som.x, v.x);
                atomicExch(&slots[s].som.y, v.y);
                atomicExch(&slots[s].som.z, v.z);
                return;
            }
            // anders is er net iemand anders eerder geweest -> doorzoeken
        }
        // slot bestaat: richtingsvergelijking
        float3 acc = slots[s].som;
        float3 dir = norm3(acc);
        float d = fabsf(dot3(v, dir));
        if(d > bestDot){
            bestDot = d;
            best = s;
        }
    }

    if(best < 0) return;

    // stem bij op beste slot
    atomicAdd(&slots[best].som.x, v.x);
    atomicAdd(&slots[best].som.y, v.y);
    atomicAdd(&slots[best].som.z, v.z);
    atomicAdd(&slots[best].count, 1);
}
