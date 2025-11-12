#pragma once
//
// consts.cuh -- constexpr tables + device constant mirrors
//
#include "math_utils.cuh"

namespace Bocari { namespace Cuda {

// Pre-instantiated sets (compile-time)
constexpr ConstsStruct kVoxelTable[] = {
    makeConsts<64>(),   // index 0
    makeConsts<128>()   // index 1
};
constexpr int kVoxelCount = sizeof(kVoxelTable)/sizeof(kVoxelTable[0]);

constexpr ConstsStruct kSubVoxelTable[] = {
    makeConsts<8>(),    // 3 bits
    makeConsts<16>(),   // 4 bits
    makeConsts<32>(),   // 5 bits
    makeConsts<64>()    // 6 bits
};
constexpr int kSubVoxelCount = sizeof(kSubVoxelTable)/sizeof(kSubVoxelTable[0]);

// Device mirrors (optional; host can memcpy into these if desired)
__device__ __constant__ ConstsStruct d_voxelTable[kVoxelCount];
__device__ __constant__ ConstsStruct d_subVoxelTable[kSubVoxelCount];

inline void uploadConstsToDevice() {
    cudaMemcpyToSymbol(d_voxelTable,   kVoxelTable,   sizeof(kVoxelTable));
    cudaMemcpyToSymbol(d_subVoxelTable,kSubVoxelTable,sizeof(kSubVoxelTable));
}

}} // namespace Bocari::Cuda
