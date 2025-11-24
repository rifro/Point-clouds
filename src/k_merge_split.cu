#include "types.cuh"

// ====== Kernels ======

__global__ void k_merge_xyz_naar_vec3_kernel(const float*** restrict** x, const float*** restrict** y,
                                             const float*** restrict** z, Vec3f*** restrict** pts, uint32_t N)
{
    const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;
    pts[i] = Vec3f{x[i], y[i], z[i]};
}

__global__ void k_split_vec3_naar_xyz_kernel(const Vec3f*** restrict** pts, float*** restrict** x,
                                             float*** restrict** y, float*** restrict** z, uint32_t N)
{
    const uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= N) return;
    Vec3f p = pts[i];
    x[i]    = p.x;
    y[i]    = p.y;
    z[i]    = p.z;
}