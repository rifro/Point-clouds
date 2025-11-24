#include <cuda_runtime.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream> // alleen voor eventuele foutmeldingen/log, mag later weg

#include "otndc.h"

// ========================
// Kleine host-helpers
// ========================

static inline void cudaCheck(cudaError_t e, const char* where)
{
    if(e != cudaSuccess){
        std::cerr << "[CUDA] fout bij " << where << ": "
                  << cudaGetErrorString(e) << "\n";
        std::abort();
    }
}

static inline float3 make3(float x,float y,float z)
{
    return make_float3(x,y,z);
}

static inline float3 norm3_host(float3 v)
{
    float n2 = v.x*v.x + v.y*v.y + v.z*v.z;
    if(n2 < 1e-20f) return make3(0,0,0);
    float inv = 1.0f / std::sqrt(n2);
    return make3(v.x*inv, v.y*inv, v.z*inv);
}

// Kernel-declaratie (implementatie in k_otndc.cu)
__global__
void k_otndc_fromNormals(
    const Vec3f* __restrict__ normals,
    int N,
    OTConfig cfg,
    OTSlot* __restrict__ gSlots);

// ========================
// Device-variant
// ========================

void runOTNDC_fromNormals_device(
    const Vec3f* dNormals,
    int N,
    const OTConfig& cfg,
    AssenResultaat* out)
{
    if(!out){
        std::cerr << "[OTNDC] out == null in runOTNDC_fromNormals_device\n";
        return;
    }
    if(N <= 0){
        out->dimensie = 0;
        for(int i=0;i<3;i++){
            out->v[i][0] = out->v[i][1] = out->v[i][2] = 0.0f;
            out->score[i] = 0.0f;
        }
        return;
    }

    // Beperk slots tot harde kernel-limiet
    OTConfig cfgLocal = cfg;
    const int MAX_SLOTS = 16;
    if(cfgLocal.maxSlots <= 0 || cfgLocal.maxSlots > MAX_SLOTS){
        cfgLocal.maxSlots = 8; // veilige default
    }
    if(cfgLocal.sinPowK < 1) cfgLocal.sinPowK = 1;
    if(cfgLocal.sinPowK > 4) cfgLocal.sinPowK = 4;

    // Device buffer voor slots
    OTSlot* dSlots = nullptr;
    cudaCheck(cudaMalloc(&dSlots, cfgLocal.maxSlots * sizeof(OTSlot)), "malloc dSlots");
    cudaCheck(cudaMemset(dSlots, 0, cfgLocal.maxSlots * sizeof(OTSlot)), "memset dSlots");

    // Kernel-launch:
    // We gebruiken één block met B threads; elke thread loopt over N in strides.
    // Dat is simpel en voldoende voor eerste versie.
    const int B = 256;
    const dim3 grid(1);
    const dim3 block(B);

    k_otndc_fromNormals<<<grid, block>>>(
        dNormals,
        N,
        cfgLocal,
        dSlots);

    cudaCheck(cudaGetLastError(), "launch k_otndc_fromNormals");
    cudaCheck(cudaDeviceSynchronize(), "sync k_otndc_fromNormals");

    // Slots terughalen
    std::vector<OTSlot> hSlots(cfgLocal.maxSlots);
    cudaCheck(cudaMemcpy(hSlots.data(), dSlots,
                         cfgLocal.maxSlots * sizeof(OTSlot),
                         cudaMemcpyDeviceToHost),
              "memcpy dSlots->hSlots");

    cudaCheck(cudaFree(dSlots), "free dSlots");

    // ========================
    // Reduceren naar 1–3 assen
    // ========================

    struct AsInfo {
        float3 v;
        float  score;
    };
    std::vector<AsInfo> assen;
    assen.reserve(cfgLocal.maxSlots);

    const float epsLen2 = 1e-10f;

    for(int i=0;i<cfgLocal.maxSlots;i++){
        const OTSlot& s = hSlots[i];
        if(s.count < cfgLocal.minCountAxis) continue;
        float3 v = s.n;
        float n2 = v.x*v.x + v.y*v.y + v.z*v.z;
        if(n2 < epsLen2) continue;
        float3 dir = norm3_host(v);
        assen.push_back({dir, s.count});
    }

    // Sorteren op aflopende score
    std::sort(assen.begin(), assen.end(),
              [](const AsInfo& a, const AsInfo& b){
                  return a.score > b.score;
              });

    // Top 3 invullen
    int D = (int)assen.size();
    if(D > 3) D = 3;
    out->dimensie = D;

    for(int i=0;i<3;i++){
        out->v[i][0] = out->v[i][1] = out->v[i][2] = 0.0f;
        out->score[i] = 0.0f;
    }

    for(int i=0;i<D;i++){
        out->v[i][0] = assen[i].v.x;
        out->v[i][1] = assen[i].v.y;
        out->v[i][2] = assen[i].v.z;
        out->score[i] = assen[i].score;
    }
}

// ========================
// Host-variant (normals op host)
// ========================
//
// Eenvoudige wrapper: kopieert naar device, roept device-variant,
// en geeft daarna de device-buffer vrij.

void runOTNDC_fromNormals(
    const Vec3f* normals,
    int N,
    const OTConfig& cfg,
    AssenResultaat* out)
{
    if(!normals || N <= 0){
        if(out){
            out->dimensie = 0;
            for(int i=0;i<3;i++){
                out->v[i][0] = out->v[i][1] = out->v[i][2] = 0.0f;
                out->score[i] = 0.0f;
            }
        }
        return;
    }

    Vec3f* dNormals = nullptr;
    cudaCheck(cudaMalloc(&dNormals, N * sizeof(Vec3f)), "malloc dNormals");
    cudaCheck(cudaMemcpy(dNormals, normals,
                         N * sizeof(Vec3f),
                         cudaMemcpyHostToDevice),
              "memcpy normals H->D");

    runOTNDC_fromNormals_device(dNormals, N, cfg, out);

    cudaCheck(cudaFree(dNormals), "free dNormals");
}
