#include <vector>
#include <cuda_runtime.h>
#include "vlak_types.h"
#include "voxel_params.h"

using namespace jbf;
using namespace grid;

// Helpers voor launches (kies je eigen block/grid config)
static inline dim3 blok(uint32_t n) { return dim3( min<uint32_t>(256,n) ); }
static inline dim3 rooster(uint32_t n) { return dim3( (n + 255)/256 ); }

// Kernels (extern "C" uit .cu’s)
extern __global__ void k_initLabels(uint8_t*, uint32_t, uint8_t);
extern __global__ void k_ruisBoerenTweedeBuur(const float3*, uint8_t*, uint32_t, VoxelRaster, VoxelIndexing, float);
struct ZaadDriehoek; // fwd
extern __global__ void k_vindDriehoekZadenLangsRichting(const float3*, const uint8_t*, uint32_t,
                                                        VoxelRaster, VoxelIndexing,
                                                        float3, float, float, float,
                                                        uint32_t, ZaadDriehoek*, uint32_t*);

// TODO: k_bepaalSlabPunten, k_planaireRegressie, k_bepaalD_TrimK zitten in jouw andere CU’s
extern __global__ void k_bepaalSlabPunten(const float3*, const uint8_t*, uint32_t,
                                          float3, float, float,
                                          uint32_t*, uint32_t, uint32_t*);
extern __global__ void k_bepaalD_TrimK(const float3*, const uint32_t*, uint32_t,
                                       float3, int, float*, float*, uint32_t*);
// d_planaireRegressie zit als __device__ helper; maak evt. wrapper-kernel als je die __device__-weg wil houden

// Orchestratie voor 1 richting
bool vindVlakkenLangsRichting(const float3* d_pts,
                              uint8_t* d_labels,
                              uint32_t N,
                              VoxelRaster vr,
                              VoxelIndexing vx,
                              float3 n_as,
                              const VlakTuning& tun,
                              std::vector<PlaneHyp>& uit)
{
    // 0) ruis-filter
    k_ruisBoerenTweedeBuur<<<vx.numVoxels, 128>>>(d_pts, d_labels, N, vr, vx, tun.ruisStraal);
    cudaDeviceSynchronize();

    // 1) zaden (strak: kleine driehoeken, bandpass, hoek tov n_as)
    uint32_t maxZaden = 1'000'000; // cap; tuning
    ZaadDriehoek* d_zaden = nullptr; cudaMalloc(&d_zaden, maxZaden * sizeof(ZaadDriehoek));
    uint32_t* d_zCount = nullptr;   cudaMalloc(&d_zCount, sizeof(uint32_t));
    cudaMemset(d_zCount, 0, sizeof(uint32_t));

    float cosHoekMin = cosf(tun.hoekMaxDeg * 3.1415926535f / 180.f);
    k_vindDriehoekZadenLangsRichting<<<vx.numVoxels, 128>>>(d_pts, d_labels, N, vr, vx,
                                                            n_as,
                                                            tun.edgeMin, tun.edgeMax,
                                                            cosHoekMin,
                                                            /*maxZadenPerVoxel*/ 64,
                                                            d_zaden, d_zCount);
    cudaDeviceSynchronize();

    uint32_t h_zCount=0; cudaMemcpy(&h_zCount, d_zCount, sizeof(uint32_t), cudaMemcpyDeviceToHost);
    if (h_zCount == 0) { cudaFree(d_zaden); cudaFree(d_zCount); return false; }

    // 2) per zaad: slabpunten rond d0 = n_as·centroid → planaireRegressie → d (Trim-K) → gates → output
    //    (hier: demonstratief 1 zaad pakken; jij maakt er een batched variant van)

    // buffers voor slab-gather
    const uint32_t MAX_IN_SLAG = 200000; // cap; maak dynamisch/batched als nodig
    uint32_t* d_idx = nullptr;   cudaMalloc(&d_idx, MAX_IN_SLAG * sizeof(uint32_t));
    uint32_t* d_cnt = nullptr;   cudaMalloc(&d_cnt, sizeof(uint32_t));

    // kopieer eerste zaad
    ZaadDriehoek hz; cudaMemcpy(&hz, d_zaden, sizeof(ZaadDriehoek), cudaMemcpyDeviceToHost);
    float d0 = hz.n.x * hz.c.x + hz.n.y * hz.c.y + hz.n.z * hz.c.z; // of n_as·ctd

    cudaMemset(d_cnt, 0, sizeof(uint32_t));
    k_bepaalSlabPunten<<<rooster(N), blok(N)>>>(d_pts, d_labels, N,
                                                n_as, d0, tun.slabEps,
                                                d_idx, MAX_IN_SLAG, d_cnt);
    cudaDeviceSynchronize();

    uint32_t M=0; cudaMemcpy(&M, d_cnt, sizeof(uint32_t), cudaMemcpyDeviceToHost);
    if (M >= 16) // minimale set
    {
        // 2b) planaire regressie (__device__ helper via wrapper-kernel of inline in aparte kernel)
        // — maak evt. een kleine wrapper: k_planaireRegressieWrapper(pts, idx, M, n_in, X,Y,Z, ...)
        // Voor nu: we nemen n_as als n_in en doen d-trimK; later vervang je n_as door n* uit regressie.
        float* d_d=nullptr, *d_rms=nullptr; uint32_t* d_kept=nullptr;
        cudaMalloc(&d_d, sizeof(float)); cudaMalloc(&d_rms, sizeof(float)); cudaMalloc(&d_kept, sizeof(uint32_t));

        k_bepaalD_TrimK<<<1, 1, 0>>>(d_pts, d_idx, M, n_as, tun.kTrim, d_d, d_rms, d_kept);
        cudaDeviceSynchronize();

        float h_d=0.f, h_rms=0.f; uint32_t h_kept=0;
        cudaMemcpy(&h_d, d_d, sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(&h_rms, d_rms, sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(&h_kept, d_kept, sizeof(uint32_t), cudaMemcpyDeviceToHost);

        if (h_kept >= tun.minSupport && h_rms <= tun.maxRms) {
            PlaneHyp ph; ph.n = n_as; ph.d = h_d; ph.count = h_kept; ph.rms = h_rms; ph.areaHint = 0;
            uit.push_back(ph);
            // TODO: label punten in slab als VLAK (aparte kernel die label set op basis van n_as & d)
        }

        cudaFree(d_d); cudaFree(d_rms); cudaFree(d_kept);
    }

    cudaFree(d_idx); cudaFree(d_cnt);
    cudaFree(d_zaden); cudaFree(d_zCount);
    return !uit.empty();
}

bool vindVlakkenHuidigFrame(const float3* d_pts, uint8_t* d_labels, uint32_t N,
                            VoxelRaster vr, VoxelIndexing vx,
                            const VlakTuning& tun,
                            std::vector<PlaneHyp>& uit)
{
    // init labels op ONGELABELD (éénmalig)
    k_initLabels<<<rooster(N), blok(N)>>>(d_labels, N, ONGELABELD);
    cudaDeviceSynchronize();

    bool ok=false;
    ok |= vindVlakkenLangsRichting(d_pts, d_labels, N, vr, vx, make_float3(0,0,1), tun, uit); // TB
    ok |= vindVlakkenLangsRichting(d_pts, d_labels, N, vr, vx, make_float3(0,1,0), tun, uit); // NZ
    ok |= vindVlakkenLangsRichting(d_pts, d_labels, N, vr, vx, make_float3(1,0,0), tun, uit); // OW
    return ok;
}
