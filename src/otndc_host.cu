#include "otndc.h"
#include <algorithm>
#include <cmath>
#include <cuda_runtime.h> // voor CUDA launch macros
#include <iostream>
#include <vector>

static float3 to3(float x, float y, float z) { return make_float3(x, y, z); }

// forward decl of CUDA kernel
__global__ void k_oTNDCRays(const float* x, const float* y, const float* z, int N, OTSlot* slots, int maxSlots,
                            float binnen2, float buiten2, float minHoekCos);

void runOTNDC(const float* x, const float* y, const float* z, int N, AssenResultaat* out, const OTConfig& cfg)
{
    int blocks  = (N + 255) / 256;
    int threads = 256;

    int                 S = cfg.maxSlots;
    std::vector<OTSlot> hslots(S);
    OTSlot*             dslots;
    cudaMalloc(&dslots, S * sizeof(OTSlot));
    cudaMemset(dslots, 0, S * sizeof(OTSlot));

    k_oTNDCRays<<<blocks, threads>>>(x, y, z, N, dslots, S, cfg.binnenStraal2, cfg.buitenStraal2, cfg.minHoekCos);
    cudaDeviceSynchronize();

    cudaMemcpy(hslots.data(), dslots, S * sizeof(OTSlot), cudaMemcpyDeviceToHost);
    cudaFree(dslots);

    // reduce + pick top 3
    struct V
    {
        float3 v;
        float  score;
    };
    std::vector<V> vals;
    for(int i = 0; i < S; i++)
    {
        if(hslots[i].count < cfg.warmupMin) continue;
        float3 a   = to3(hslots[i].som.x, hslots[i].som.y, hslots[i].som.z);
        float  len = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        if(len < 1e-6) continue;
        vals.push_back({make_float3(a.x / len, a.y / len, a.z / len), (float)hslots[i].count});
    }

    // sort by score desc
    std::sort(vals.begin(), vals.end(), [](const V& A, const V& B) { return A.score > B.score; });

    int D         = std::min((int)vals.size(), 3);
    out->dimensie = D;
    for(int i = 0; i < D; i++)
    {
        out->v[i][0]  = vals[i].v.x;
        out->v[i][1]  = vals[i].v.y;
        out->v[i][2]  = vals[i].v.z;
        out->score[i] = vals[i].score;
    }
}
