#include "otndc.h"
#include <cuda_runtime.h>
#include <math___constant__s.h>

////////////////////////////////////////////////////////////////////////////////
// __device__ helpers
////////////////////////////////////////////////////////////////////////////////
__device__ __forceinline__ float3 norm3(float3 v)
{
    float l = rsqrtf(v.x * v.x + v.y * v.y + v.z * v.z + 1e-20f);
    return make_float3(v.x * l, v.y * l, v.z * l);
}

__device__ __forceinline__ float dot3(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

__device__ __forceinline__ float3 cross3(float3 a, float3 b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

////////////////////////////////////////////////////////////////////////////////
// kernel: generate stralen voor boeren-ring + slot vote
////////////////////////////////////////////////////////////////////////////////
__global__ void k_oTNDCRays(const float* __restrict__ x, const float* __restrict__ y, const float* __restrict__ z,
                            int N, OTSlot* slots, int maxSlots, float binnen2, float buiten2, float minHoekCos)
{
    // per punt → straal
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;

    float3 p  = make_float3(x[i], y[i], z[i]);
    float  r2 = dot3(p, p);

    // ring filter
    if(r2 < binnen2 || r2 > buiten2) return;

    float3 v = norm3(p);

    // warm-up cross bootstrap (met vorige in warp)
    // simpel: only with thread-1 if exists
    if(threadIdx.x > 0)
    {
        float3 vp = v; // current
        // but we need 3 floats: do it properly
        float  px    = __shfl_up_sync(0xffffffff, v.x, 1);
        float  py    = __shfl_up_sync(0xffffffff, v.y, 1);
        float  pz    = __shfl_up_sync(0xffffffff, v.z, 1);
        float3 vprev = make_float3(px, py, pz);

        float dp = fabsf(dot3(vp, vprev));
        if(dp < minHoekCos)
        {
            float3 cr = norm3(cross3(vp, vprev));
            // vote cross to slot 0 always during warmup
            atomicAdd(&slots[0].som.x, cr.x);
            atomicAdd(&slots[0].som.y, cr.y);
            atomicAdd(&slots[0].som.z, cr.z);
            atomicAdd(&slots[0].count, 1);
        }
    }

    // vote v to best slot or empty slot
    int   best    = -1;
    float bestDot = 0.f;

    for(int s = 0; s < maxSlots; ++s)
    {
        int c = slots[s].count;
        if(c == 0)
        {
            best = s;
            break;
        }
        float3 acc = slots[s].som;
        float3 dir = norm3(acc);
        float  d   = fabsf(dot3(v, dir));
        if(d > bestDot)
        {
            bestDot = d;
            best    = s;
        }
    }
    if(best < 0) return;

    atomicAdd(&slots[best].som.x, v.x);
    atomicAdd(&slots[best].som.y, v.y);
    atomicAdd(&slots[best].som.z, v.z);
    atomicAdd(&slots[best].count, 1);
}
