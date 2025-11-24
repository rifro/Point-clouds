#include <cuda_runtime.h>
#include "vlak_types.h"
#include "math_hulpfuncties.cuh"
#include "planaire_regressie.cuh"

using namespace jbf;

extern "C" {

__global__ void k_bepaalSlabPunten(const float3* __restrict__ pts,
                                   const uint8_t* __restrict__ labels,
                                   uint32_t N,
                                   float3 n,
                                   float  d0,
                                   float  slabEps,
                                   uint32_t* __restrict__ outIdx,
                                   uint32_t  maxOut,
                                   uint32_t* __restrict__ outCount)
{
    // eenvoudige grid-stride scan
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    uint32_t stride = blockDim.x * gridDim.x;

    for (uint32_t i=tid; i<N; i+=stride) {
        if (labels[i] != ONGELABELD) continue;
        float s = d_dot3(n, pts[i]);
        if (fabsf(s - d0) <= slabEps) {
            uint32_t pos = atomicAdd(outCount, 1u);
            if (pos < maxOut) outIdx[pos] = i; // eenvoudige overrun bescherming
        }
    }
}

__global__ void k_bepaalD_TrimK(const float3* __restrict__ pts,
                                const uint32_t* __restrict__ idx,
                                uint32_t M,
                                float3 n_star,
                                int kTrim,
                                float* d_out,
                                float* rms_out,
                                uint32_t* kept_out)
{
    // Single-block kernel verwacht; voor eenvoud (kun je later parallen)
    float sum = 0.f; uint32_t cnt = 0;

    // kleine buffers top-k
    extern __shared__ float sh[]; // we gebruiken registers hieronder voor simpelheid
    float minK[8]; float maxK[8]; // neem kTrim<=8
    #pragma unroll
    for (int k=0;k<8;++k){ minK[k]= CUDART_INF_F; maxK[k]= -CUDART_INF_F; }

    for (uint32_t j=0; j<M; ++j) {
        float r = d_dot3(n_star, pts[idx[j]]);
        sum += r; ++cnt;

        // plaats in minK (kleinste)
        int mMin=0; for (int k=1;k<kTrim;++k) if (minK[k] > minK[mMin]) mMin=k;
        if (r < minK[mMin]) minK[mMin] = r;

        // plaats in maxK (grootste)
        int mMax=0; for (int k=1;k<kTrim;++k) if (maxK[k] < maxK[mMax]) mMax=k;
        if (r > maxK[mMax]) maxK[mMax] = r;
    }

    float sumMin=0.f, sumMax=0.f;
    for (int k=0;k<kTrim;++k){ sumMin+=minK[k]; sumMax+=maxK[k]; }
    uint32_t kept = (cnt > (uint32_t)(2*kTrim)) ? (cnt - 2*kTrim) : 0u;
    float d = (kept? (sum - sumMin - sumMax) / (float)kept : 0.f);

    // RMS (optioneel): tweede pass over kept
    float sse=0.f; uint32_t kc=0;
    for (uint32_t j=0; j<M; ++j) {
        float r = d_dot3(n_star, pts[idx[j]]);
        bool isMin=false, isMax=false;
        #pragma unroll
        for (int k=0;k<kTrim;++k){ if (fabsf(r - minK[k])<1e-12f) {isMin=true; break;} }
        #pragma unroll
        for (int k=0;k<kTrim;++k){ if (fabsf(r - maxK[k])<1e-12f) {isMax=true; break;} }
        if (isMin || isMax) continue;
        float e = r - d; sse += e*e; ++kc;
    }
    float rms = (kc? sqrtf(sse/(float)kc) : 0.f);

    if (threadIdx.x==0) {
        *d_out = d; *rms_out = rms; *kept_out = kept;
    }
}

} // extern "C"
