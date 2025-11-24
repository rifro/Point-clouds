#include <cuda_runtime.h>
#include <cub/cub.cuh>
#include "types.cuh"
#include "consts.cuh"
#include "plane_buis_utils.cuh"

// 1) voxel-quickcheck: near/far tellen voor vlak- en buisindicatie
__global__ void k_classificeer_voxels_op_vlak_buis(
    const Vec3f* __restrict__ punten, 
    const VoxelBereik* __restrict__ voxels,
    int aantal_voxels,
    Vec3f test_normaal,
    float d0,
    VoxelKlasse* __restrict__ out_klasse
){
    int v = blockIdx.x * blockDim.x + threadIdx.x;
    if (v >= aantal_voxels) return;

    VoxelBereik br = voxels[v];
    int nearCount = 0;
    int farCount = 0;

    for (u32 i = br.start; i < br.eind; ++i){
        float a2 = afstand2_vlak(test_normaal, d0, punten[i]);
        if (a2 <= Tuning::EPS_NEAR2) nearCount++;
        else if (a2 >= Tuning::TAU_FAR2) farCount++;
    }

    if (nearCount >= Tuning::MIN_NEAR && farCount <= Tuning::MAX_FAR){
        out_klasse[v] = VOX_VLAK_SEED;
    } else if (nearCount < 2 && farCount > 4){
        out_klasse[v] = VOX_BUIS_KAND;
    } else if (nearCount==0 && farCount==0){
        out_klasse[v] = VOX_RUIS;
    } else {
        out_klasse[v] = VOX_GEEN;
    }
}

// 2) block-lokale stemmen naar compacte (key,count) records; 1 richting per pass
__global__ void k_stem_vlak_richting_block(
    const VoxelKlasse* __restrict__ klasse,
    int aantal_voxels,
    Vec3f vlak_normaal,
    KeyCount* __restrict__ out_emits,
    u32* __restrict__ out_size
){
    __shared__ u32 s_key;
    __shared__ u32 s_count;
    if (threadIdx.x == 0){
        u32 key = kwantiseer_octa_key(hemisfeer_fold(norm(vlak_normaal)), Tuning::OCT_NX, Tuning::OCT_NY);
        s_key = key;
        s_count = 0;
    }
    __syncthreads();

    int v = blockIdx.x * blockDim.x + threadIdx.x;
    if (v < aantal_voxels && klasse[v] == VOX_VLAK_SEED){
        atomicAdd(&s_count, 1u);
    }
    __syncthreads();

    if (threadIdx.x == 0 && s_count > 0){
        u32 idx = atomicAdd(out_size, 1u);
        out_emits[idx] = { s_key, s_count };
    }
}
