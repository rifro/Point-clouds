#include "__device___buffer.cuh"
#include "config.h"
#include "morton_utils.cuh"
#include "types.h"
#include <cub/cub.cuh>
#include <cuda_runtime.h>
#include <stdexcept>

// ============ Kernels ============

// Kwantiseren + morton sleutel (zonder regio-codes voor nu)
__global__ void k_kwantiseer_en_key(const Vec3f*** restrict** punten, u32 N, Kwantisatie Q, u64*** restrict** keys_out)
{
    u32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;

    Vec3f p = punten[i];
    u32   xi, yi, zi;
    kwantiseer_punt(p, Q, xi, yi, zi);

    // Voorlopig geen afzonderlijke regio-code (0)
    u64 key     = morton_met_regio(xi, yi, zi, /*regio*/ 0u);
    keys_out[i] = key;
}

// Permuteer punten volgens index-permutatie
__global__ void k_permuteer_punten(const Vec3f*** restrict** punten_in, const u32*** restrict** index_perm, u32 N,
                                   Vec3f*** restrict** punten_out)
{
    u32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;
    u32 src       = index_perm[i];
    punten_out[i] = punten_in[src];
}

// ============ __host__ Launch ============

static inline void check(cudaError_t e, const char* where)
{
    if(e != cudaSuccess) throw std::runtime_error(std::string("[CUDA] ") + where + " : " + cudaGetErrorString(e));
}

void k_voxeliseer_morton_launch(const Vec3f* d_points, u32 N, Kwantisatie Q, u64* d_keys,
                                u32* /*d_regioCodes*/, // (niet gebruikt nu)
                                Vec3f* d_points_perm, u32* d_index_perm)
{
    if(N == 0) return;

    // 1) Kwantiseren + morton keys
    dim3 bs(256);
    dim3 gs((N + bs.x - 1) / bs.x);
    k_kwantiseer_en_key<<<gs, bs>>>(d_points, N, Q, d_keys);
    check(cudaGetLastError(), "k_kwantiseer_en_key");

    // 2) Init index [0..N)
    //    Gebruik thrust of een simpele kernel; we doen snelle CUB-sequence via transform-iterator
    //    Om het simpel te houden: kleine helper kernel hier:
    auto k_init_indices = [] __global__(u32 * idx, u32 n) {
        u32 i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i < n) idx[i] = i;
    };
    k_init_indices<<<gs, bs>>>(d_index_perm, N);
    check(cudaGetLastError(), "init indices");

    // 3) CUB radix sort (keys, indices)
    void*  d_temp     = nullptr;
    size_t temp_bytes = 0;

    cub::__device__RadixSort::SortPairs(d_temp, temp_bytes, d_keys, d_keys, d_index_perm, d_index_perm, N);
    check(cudaGetLastError(), "cub temp size");

    cudaMalloc(&d_temp, temp_bytes);
    cub::__device__RadixSort::SortPairs(d_temp, temp_bytes, d_keys, d_keys, d_index_perm, d_index_perm, N);
    check(cudaGetLastError(), "cub sort pairs");
    cudaFree(d_temp);

    // 4) Permuteer punten → d_points_perm (coalesced read met index_perm)
    k_permuteer_punten<<<gs, bs>>>(d_points, d_index_perm, N, d_points_perm);
    check(cudaGetLastError(), "k_permuteer_punten");

    cudaDeviceSynchronize();
}