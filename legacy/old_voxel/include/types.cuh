#pragma once
// Nederlandse identifiers, behalve: seed, triangle, forest, tree
#include <cuda_runtime.h>
#include <math.h>
#include <stdint.h>

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

struct Vec3f
{
    float x, y, z;
};

__host__ __device__ inline Vec3f maak_vec3(float x, float y, float z) { return {x, y, z}; }
__host__ __device__ inline float dot(const Vec3f& a, const Vec3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
__host__ __device__ inline Vec3f cross(const Vec3f& a, const Vec3f& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
__host__ __device__ inline float lengte2(const Vec3f& a) { return dot(a, a); }
__host__ __device__ inline float lengte(const Vec3f& a) { return sqrtf(lengte2(a)); }
__host__ __device__ inline Vec3f norm(const Vec3f& a)
{
    float L = lengte(a);
    return L > 0 ? maak_vec3(a.x / L, a.y / L, a.z / L) : maak_vec3(0, 0, 0);
}
__host__ __device__ inline Vec3f plus(const Vec3f& a, const Vec3f& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
__host__ __device__ inline Vec3f minv(const Vec3f& a, const Vec3f& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
__host__ __device__ inline Vec3f schaal(const Vec3f& a, float s) { return {a.x * s, a.y * s, a.z * s}; }

// deterministische (u,v) ⟂ n
__host__ __device__ inline void basis_uv_orthonormaal(const Vec3f& n, Vec3f& u, Vec3f& v)
{
    Vec3f h = (fabsf(n.x) < 0.9f) ? maak_vec3(1, 0, 0) : maak_vec3(0, 1, 0);
    u       = norm(cross(h, n));
    v       = norm(cross(n, u));
}

// hemisfeer fold (n.z ≥ 0)
__host__ __device__ inline Vec3f hemisfeer_fold(const Vec3f& a) { return (a.z < 0.0f) ? schaal(a, -1.0f) : a; }

// simpele octahedral mapping naar [0,1]^2
__host__ __device__ inline void octa_map(const Vec3f& n, float& ox, float& oy)
{
    float ax = fabsf(n.x), ay = fabsf(n.y), az = fabsf(n.z);
    float s = ax + ay + az + 1e-20f;
    float x = n.x / s, y = n.y / s;
    if(n.z < 0)
    {
        float xo = (n.x >= 0) ? 1.0f - fabsf(y) : -1.0f + fabsf(y);
        float yo = (n.y >= 0) ? 1.0f - fabsf(x) : -1.0f + fabsf(x);
        x        = xo;
        y        = yo;
    }
    ox = x * 0.5f + 0.5f;
    oy = y * 0.5f + 0.5f;
}

__host__ __device__ inline u32 kwantiseer_octa_key(const Vec3f& dir, u32 Nx, u32 Ny)
{
    Vec3f f = hemisfeer_fold(norm(dir));
    float ox, oy;
    octa_map(f, ox, oy);
    u32 ix = (u32)fminf(fmaxf(ox * Nx, 0.0f), (float)(Nx - 1));
    u32 iy = (u32)fminf(fmaxf(oy * Ny, 0.0f), (float)(Ny - 1));
    return (iy * Nx) + ix; // 2D → 1D
}
