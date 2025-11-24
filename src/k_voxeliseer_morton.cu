#include <cub/cub.cuh>
#include <cuda_runtime.h>

#include "morton_utils.cuh"
#include "types.cuh"

__global__ void k_kwantiseer_en_key(const Vec3f* __restrict__ punten, uint32_t N, Kwantisatie Q, uint32_t regioCode,
                                    uint64_t* __restrict__ keys_out)
{
    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;

    uint32_t xi, yi, zi;
    kwantiseer_punt(punten[i], Q, xi, yi, zi);

    uint64_t key = morton_met_regio(xi, yi, zi, regioCode);
    keys_out[i]  = key;
}

__global__ void k_init_indices(u32* __restrict__ idx, u32 N)
{
    u32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;
    idx[i] = i;
}

__global__ void k_permuteer_punten(const Vec3f* __restrict__ punten_in, const u32* __restrict__ index_perm, u32 N,
                                   Vec3f* __restrict__ punten_out)
{
    u32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;

    u32 src       = index_perm[i];
    punten_out[i] = punten_in[src];
}

// Host wrapper:

void voxeliseer_morton(const Vec3f* d_points, u32 N, Kwantisatie Q, Vec3f* d_points_perm)
{
    // alloc keys
    u64* d_keys;
    cudaMalloc(&d_keys, N * sizeof(u64));

    u32* d_index_perm;
    cudaMalloc(&d_index_perm, N * sizeof(u32));

    dim3 bs(256);
    dim3 gs((N + bs.x - 1) / bs.x);

    k_kwantiseer_en_key<<<gs, bs>>>(d_points, N, Q, d_keys);
    k_init_indices<<<gs, bs>>>(d_index_perm, N);

    // Radix sort
    size_t temp_bytes = 0;
    void*  d_temp     = nullptr;
    cub::DeviceRadixSort::SortPairs(nullptr, temp_bytes, d_keys, d_keys, d_index_perm, d_index_perm, N);
    cudaMalloc(&d_temp, temp_bytes);
    cub::DeviceRadixSort::SortPairs(d_temp, temp_bytes, d_keys, d_keys, d_index_perm, d_index_perm, N);

    k_permuteer_punten<<<gs, bs>>>(d_points, d_index_perm, N, d_points_perm);

    cudaFree(d_keys);
    cudaFree(d_index_perm);
    cudaFree(d_temp);
}
