#include "pairs_bucket.cuh"
#include <math___constant__s.h>

// ============ __device__ helpers ============
__device__ __forceinline__ float3 make3(float x, float y, float z) { return make_float3(x, y, z); }
__device__ __forceinline__ float  dot3(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
__device__ __forceinline__ float3 cross3(float3 a, float3 b)
{
    return make3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
__device__ __forceinline__ float3 norm3(float3 v)
{
    float n2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if(n2 < 1e-20f) return make3(0, 0, 0);
    float inv = rsqrtf(n2);
    return make3(v.x * inv, v.y * inv, v.z * inv);
}

// ============ kernel ============
// Per punt -> straal (v). In gedeeld geheugen een tile.
// Voor elk v: zoek binnen +- partnerZoekRadius in de tile 1 partner u
// met |dot(v,u)| <= maxDotOrtho; kies de beste (kleinste |dot|).
// Voeg a = normalize(cross(v,u)) bij aan globale accumulator met gewicht sin(theta).
extern "C" __global__ void k_otndc_pairs(const float* __restrict__ x, const float* __restrict__ y,
                                         const float* __restrict__ z, int N, float binnen2, float buiten2,
                                         float maxDotOrtho, int partnerZoekRadius, OTAsAccu* __restrict__ gAccu)
{
    extern __shared__ float sMem[]; // layout: vx[], vy[], vz[]
    float*                  sx = sMem;
    float*                  sy = sx + blockDim.x;
    float*                  sz = sy + blockDim.x;

    const int gid = blockIdx.x * blockDim.x + threadIdx.x;

    // Laad tile stralen (met ringfilter)
    float3 v = make3(0, 0, 0);
    if(gid < N)
    {
        float3 p  = make3(x[gid], y[gid], z[gid]);
        float  r2 = dot3(p, p);
        if(r2 >= binnen2 && r2 <= buiten2)
        {
            v = norm3(p); // straal = genormaliseerde plaatsvector
        }
    }
    sx[threadIdx.x] = v.x;
    sy[threadIdx.x] = v.y;
    sz[threadIdx.x] = v.z;
    __syncthreads();

    // Zoek één partner in de tile
    if(gid >= N) return;
    if(v.x == 0.f && v.y == 0.f && v.z == 0.f) return; // buiten ring

    float  bestAbsDot = 1e9f;
    float3 bestU      = make3(0, 0, 0);

    // Zoek rondom huidige thread (beperkt venster)
    const int t = threadIdx.x;
    const int L = max(0, t - partnerZoekRadius);
    const int R = min(blockDim.x - 1, t + partnerZoekRadius);

    for(int j = L; j <= R; ++j)
    {
        if(j == t) continue;
        float3 u = make3(sx[j], sy[j], sz[j]);
        if(u.x == 0.f && u.y == 0.f && u.z == 0.f) continue;

        float d = fabsf(dot3(v, u)); // |cos theta|
        if(d <= maxDotOrtho && d < bestAbsDot)
        {
            bestAbsDot = d;
            bestU      = u;
        }
    }

    if(bestAbsDot == 1e9f) return; // geen geschikte partner

    // Kruisproduct → as-sample
    float3 a  = cross3(v, bestU);
    float  a2 = dot3(a, a);
    if(a2 < 1e-20f) return;
    float inv = rsqrtf(a2);
    a.x *= inv;
    a.y *= inv;
    a.z *= inv;

    // Gewicht: sin(theta) = sqrt(1 - cos^2)
    float w = sqrtf(fmaxf(0.f, 1.f - bestAbsDot * bestAbsDot));

    // Atomics naar globale accu
    atomicAdd(&gAccu->som.x, a.x * w);
    atomicAdd(&gAccu->som.y, a.y * w);
    atomicAdd(&gAccu->som.z, a.z * w);
    atomicAdd(&gAccu->count, 1);
}
