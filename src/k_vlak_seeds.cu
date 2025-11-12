#include <cuda_runtime.h>
#include "vlak_types.h"
#include "voxel_params.h"
#include "math_utils.cuh"          // jouw device dot/cross/norm helpers (d_*)

using namespace jbf;
using namespace grid;

// Output-struct (compact) voor een driehoek-zaad
struct ZaadDriehoek {
    uint32_t i, j, k; // indices in d_pts
    float3   n;       // unit normaal van de driehoek
    float3   c;       // centroid (optioneel voor d0)
};

// NB: we limiteren zaden per voxel om combinatorische explosie te voorkomen.
__global__ void k_vindDriehoekZadenLangsRichting(const float3* __restrict__ pts,
                                                 const uint8_t* __restrict__ labels,
                                                 uint32_t N,
                                                 VoxelRaster vr,
                                                 VoxelIndexing vx,
                                                 float3 n_doel,           // (0,0,1) etc.
                                                 float edgeMin, float edgeMax,
                                                 float cosHoekMin,        // cos(maxΔhoek)
                                                 uint32_t maxZadenPerVoxel,
                                                 ZaadDriehoek* __restrict__ zaden,
                                                 uint32_t* __restrict__ zadenCount)
{
    uint32_t v = blockIdx.x; // 1 CTA per voxel (kan ook tiled)
    if (v >= vx.numVoxels) return;

    uint32_t begin = vx.d_voxelStarts[v];
    uint32_t eind  = vx.d_voxelStarts[v+1];
    if (begin + 2 >= eind) return;

    // decode v → (ix,iy,iz)
    uint32_t S = vr.S;
    uint32_t iz = v / (S*S);
    uint32_t iy = (v / S) % S;
    uint32_t ix = v % S;

    // neem buurt (3x3x3)
    int x0 = max<int>(0, ix-1), x1 = min<int>(S-1, ix+1);
    int y0 = max<int>(0, iy-1), y1 = min<int>(S-1, iy+1);
    int z0 = max<int>(0, iz-1), z1 = min<int>(S-1, iz+1);

    // verzamel lokale index-range lijst (optioneel: direct itereren zoals hieronder)
    uint32_t lokaalMax = 4096; // cap; pas aan of maak dynamisch
    __shared__ uint32_t lokIdx[4096];
    __shared__ uint32_t L;
    if (threadIdx.x == 0) L = 0;
    __syncthreads();

    for (int zz=z0; zz<=z1; ++zz)
    for (int yy=y0; yy<=y1; ++yy)
    for (int xx=x0; xx<=x1; ++xx)
    {
        uint32_t nb = flatten(xx,yy,zz,S,S,S);
        uint32_t nbB = vx.d_voxelStarts[nb];
        uint32_t nbE = vx.d_voxelStarts[nb+1];

        for (uint32_t t=nbB + threadIdx.x; t<nbE; t+=blockDim.x) {
            if (labels[t] != ONGELABELD) continue;
            uint32_t pos = atomicAdd(&L, 1u);
            if (pos < lokaalMax) lokIdx[pos] = t;
        }
        __syncthreads();
    }
    if (L < 3) return;

    // elke thread pakt een i en maakt enkele combinaties j,k in de buurt
    uint32_t seedsEmitted = 0;
    for (uint32_t a = threadIdx.x; a < L && seedsEmitted < maxZadenPerVoxel; a += blockDim.x)
    {
        uint32_t i = lokIdx[a];
        float3 Pi = pts[i];

        // kies beperkt aantal buren rondom a (bandbreedte beperken)
        const uint32_t KMAX = 16;
        for (uint32_t b = a+1; b < min(L, a+1+KMAX) && seedsEmitted < maxZadenPerVoxel; ++b)
        for (uint32_t c = b+1; c < min(L, b+1+KMAX) && seedsEmitted < maxZadenPerVoxel; ++c)
        {
            uint32_t j = lokIdx[b], k = lokIdx[c];
            if (labels[j] != ONGELABELD || labels[k] != ONGELABELD) continue;
            float3 Pj = pts[j], Pk = pts[k];

            // edge-lengtes
            float3 e1 = make_float3(Pj.x-Pi.x, Pj.y-Pi.y, Pj.z-Pi.z);
            float3 e2 = make_float3(Pk.x-Pi.x, Pk.y-Pi.y, Pk.z-Pi.z);
            float L1 = sqrtf(e1.x*e1.x + e1.y*e1.y + e1.z*e1.z);
            float L2 = sqrtf(e2.x*e2.x + e2.y*e2.y + e2.z*e2.z);
            float L3 = sqrtf((Pk.x-Pj.x)*(Pk.x-Pj.x) + (Pk.y-Pj.y)*(Pk.y-Pj.y) + (Pk.z-Pj.z)*(Pk.z-Pj.z));

            if (L1 < edgeMin || L2 < edgeMin || L3 < edgeMin) continue;
            if (L1 > edgeMax || L2 > edgeMax || L3 > edgeMax) continue;

            // triangle normaal
            float3 n = jbf::d_norm3( jbf::d_cross3(e1, e2) );
            float c = fabsf(jbf::d_dot3(n, n_doel));
            if (c < cosHoekMin) continue; // wijkt teveel af van doel-normaal

            // centroid
            float3 ctd = make_float3((Pi.x+Pj.x+Pk.x)/3.f,
                                     (Pi.y+Pj.y+Pk.y)/3.f,
                                     (Pi.z+Pj.z+Pk.z)/3.f);

            // schrijf uit
            uint32_t outPos = atomicAdd(zadenCount, 1u);
            zaden[outPos] = { i, j, k, n, ctd };
            ++seedsEmitted;
        }
    }
}
