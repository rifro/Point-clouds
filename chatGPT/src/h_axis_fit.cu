#include "pairs_bucket.cuh"
#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

// small helper
static inline void cudaCheck(cudaError_t e, const char* msg){
    if(e!=cudaSuccess){
        std::cerr<<"CUDA fout: "<<msg<<": "<<cudaGetErrorString(e)<<"\n";
        std::abort();
    }
}
static inline float3 make3(float x,float y,float z){return make_float3(x,y,z);}

void runOTNDC_pairs(
    const float* x,const float* y,const float* z,int N,
    AssenResultaat* out,const OTConfig& cfg)
{
    // Globale accumulator
    OTAsAccu* dAccu = nullptr;
    cudaCheck(cudaMalloc(&dAccu, sizeof(OTAsAccu)), "malloc dAccu");
    cudaCheck(cudaMemset(dAccu, 0, sizeof(OTAsAccu)), "memset dAccu");

    // Kernelconfig
    const int B = (cfg.blokGrootte>0? cfg.blokGrootte:256);
    const int G = (N + B - 1)/B;
    const size_t shmem = 3 * B * sizeof(float); // sx,sy,sz

    // Launch
    k_otndc_pairs<<<G, B, shmem>>>(
        x,y,z,N,
        cfg.binnenStraal2, cfg.buitenStraal2,
        cfg.maxDotOrtho,
        cfg.partnerZoekRadius,
        dAccu);
    cudaCheck(cudaGetLastError(), "launch k_otndc_pairs");
    cudaCheck(cudaDeviceSynchronize(), "sync k_otndc_pairs");

    // Host reduce
    OTAsAccu hAccu{};
    cudaCheck(cudaMemcpy(&hAccu, dAccu, sizeof(OTAsAccu), cudaMemcpyDeviceToHost), "cpy accu");
    cudaCheck(cudaFree(dAccu), "free accu");

    // Normeer richting
    float len = std::sqrt(hAccu.som.x*hAccu.som.x + hAccu.som.y*hAccu.som.y + hAccu.som.z*hAccu.som.z);
    int dim = 0;
    if(hAccu.count >= cfg.warmupMin && len > 1e-8f){
        out->v[0][0] = hAccu.som.x / len;
        out->v[0][1] = hAccu.som.y / len;
        out->v[0][2] = hAccu.som.z / len;
        out->score[0] = static_cast<float>(hAccu.count);
        dim = 1;
    }
    out->dimensie = dim;

    // (Slots 2 en 3 blijven leeg in deze fase)
    for(int i=1;i<3;i++){ out->v[i][0]=out->v[i][1]=out->v[i][2]=0.f; out->score[i]=0.f; }
}
