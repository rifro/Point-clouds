#include <cuda_runtime.h>
#include <cub/cub.cuh>
#include "types.cuh"
#include "consts.cuh"
#include "plane_buis_utils.cuh"

// 3) annulus/wedge per voxel (VOX_BUIS_KAND): lokale sector-hist in shared → top-1 → global key emit
__global__ void k_buis_annulus_wedge(
    const Vec3f* __restrict__ punten,
    const VoxelBereik* __restrict__ voxels,
    const VoxelKlasse* __restrict__ klasse,
    int aantal_voxels,
    Vec3f n_zaad,
    KeyCount* __restrict__ out_emits,
    u32* __restrict__ out_size
){
    __shared__ u32 sect_hist[Tuning::SECTORS_HALF];
    for (int i = threadIdx.x; i < Tuning::SECTORS_HALF; i += blockDim.x) sect_hist[i] = 0;
    __syncthreads();

    Vec3f u, v; basis_uv_orthonormaal(norm(n_zaad), u, v);

    int vxl = blockIdx.x; // 1 voxel per block (simple mapping)
    if (vxl >= aantal_voxels) return;
    if (klasse[vxl] != VOX_BUIS_KAND) return;

    VoxelBereik br = voxels[vxl];
    __shared__ float sx, sy, sz; __shared__ int sc;
    if (threadIdx.x==0){ sx=sy=sz=0.0f; sc=0; }
    __syncthreads();

    // eenvoudige centroid
    for (u32 i = br.start + threadIdx.x; i < br.eind; i += blockDim.x){
        atomicAdd(&sx, punten[i].x);
        atomicAdd(&sy, punten[i].y);
        atomicAdd(&sz, punten[i].z);
        atomicAdd(&sc, 1);
    }
    __syncthreads();
    Vec3f c = {0,0,0};
    if (sc>0 && threadIdx.x==0){ c = maak_vec3(sx/sc, sy/sc, sz/sc); }
    __syncthreads();
    if (sc==0) return;

    // annulus stemmen
    for (u32 i = br.start + threadIdx.x; i < br.eind; i += blockDim.x){
        const Vec3f p = punten[i];
        if (!in_annulus(p, c, Tuning::R_MIN, Tuning::R_MAX)) continue;
        Vec3f d = minv(p, c);
        float theta = halve_cirkel_hoek(d, u, v);
        int s = (int)floorf(theta / (float)M_PI * Tuning::SECTORS_HALF);
        if (s < 0) s = 0; if (s >= Tuning::SECTORS_HALF) s = Tuning::SECTORS_HALF-1;
        atomicAdd(&sect_hist[s], 1u);
    }
    __syncthreads();

    // top-1 sector → map naar globale richting-key (octa)
    if (threadIdx.x == 0){
        u32 bestS = 0; u32 bestC = 0;
        for (int s=0; s<Tuning::SECTORS_HALF; ++s){
            if (sect_hist[s] > bestC){ bestC = sect_hist[s]; bestS = s; }
        }
        if (bestC > 0){
            float theta_hat = ( (bestS + 0.5f) / Tuning::SECTORS_HALF ) * (float)M_PI;
            Vec3f a_local = norm( plus( schaal(u, cosf(theta_hat)), schaal(v, sinf(theta_hat)) ) );
            u32 key = kwantiseer_octa_key(a_local, Tuning::OCT_NX, Tuning::OCT_NY);
            u32 idx = atomicAdd(out_size, 1u);
            out_emits[idx] = { key, bestC };
        }
    }
}
