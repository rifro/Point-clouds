#include <cuda_runtime.h>
#include "types.h"
#include "wiskunde_utils.cuh"
#include "config.h"

__device__ __forceinline__ Vec3f d_quat_apply(const Quatf& q, const Vec3f& v)
{
    Vec3f qv = make_vec3f(q.x, q.y, q.z);
    Vec3f t  = mul( cross(qv, v), 2.0f );
    Vec3f vp = v + mul(t, q.w) + mul( cross(qv, t), 1.0f );
    return vp;
}

__global__ void k_rotate_quat(const Quatf q,
                              Vec3f* punten,
                              const u8* mask,
                              u32 N)
{
    u32 i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i>=N) return;
    if (mask && mask[i]==0) return;
    punten[i] = d_quat_apply(q, punten[i]);
}

extern "C"
void k_rotatie_quat_launch(const Quatf& q_norm,
                           Vec3f* d_punten,
                           const u8* d_mask,
                           u32 N)
{
    if (!d_punten || N==0) return;
    dim3 bs(256), gs((N+bs.x-1)/bs.x);
    k_rotate_quat<<<gs,bs>>>(q_norm, d_punten, d_mask, N);
    cudaDeviceSynchronize();
}