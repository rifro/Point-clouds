#include "merge_split.cuh"
#include <cassert>

void h_merge_xyz_naar_vec3(const DeviceBuffer<float>& d_x, const DeviceBuffer<float>& d_y,
                           const DeviceBuffer<float>& d_z, DeviceBuffer<Vec3f>& d_pts)
{
    assert(d_x.n == d_y.n && d_y.n == d_z.n);
    const uint32_t N = (uint32_t)d_x.n;
    if(d_pts.n != N) d_pts.alloc(N);

    dim3 blk(256);
    dim3 grd((N + blk.x - 1u) / blk.x);
    k_merge_xyz_naar_vec3_kernel<<<grd, blk>>>(d_x.ptr, d_y.ptr, d_z.ptr, d_pts.ptr, N);
    cudaDeviceSynchronize();
}

void h_split_vec3_naar_xyz(const DeviceBuffer<Vec3f>& d_pts, DeviceBuffer<float>& d_x, DeviceBuffer<float>& d_y,
                           DeviceBuffer<float>& d_z)
{
    const uint32_t N = (uint32_t)d_pts.n;
    if(d_x.n != N) d_x.alloc(N);
    if(d_y.n != N) d_y.alloc(N);
    if(d_z.n != N) d_z.alloc(N);

    dim3 blk(256);
    dim3 grd((N + blk.x - 1u) / blk.x);
    k_split_vec3_naar_xyz_kernel<<<grd, blk>>>(d_pts.ptr, d_x.ptr, d_y.ptr, d_z.ptr, N);
    cudaDeviceSynchronize();
}