#pragma once
#include <cstdint>
#include <cuda_runtime.h>

namespace grid {

// voxel raster (kubisch binnen regio)
struct VoxelRaster {
    uint32_t S;           // voxels per as (bv. 64 of 128)
    float    h;           // voxel edge length (meters in geschaalde ruimte)
    uint32_t dimX, dimY, dimZ; // identiek aan S; expliciet voor leesbaarheid
};

// device-buffers (gemaakt door jouw sort/voxel fase)
struct VoxelIndexing {
    // Per-voxel start/eind in punten-array (Morton-geordend):
    // starts[v] .. starts[v+1]-1
    const uint32_t* d_voxelStarts; // size = numVoxels+1
    const uint32_t* d_pointToVoxel; // size = N (optioneel; snel checken)
    uint32_t        numVoxels;
};

__host__ __device__ inline
uint32_t flatten(uint32_t x, uint32_t y, uint32_t z,
                 uint32_t Sx, uint32_t Sy, uint32_t Sz)
{
    return (z * Sy + y) * Sx + x;
}

} // namespace grid

