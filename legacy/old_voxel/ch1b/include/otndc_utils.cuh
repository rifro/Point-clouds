#pragma once
#include <cuda_runtime.h>
#include <math_constants.h>

// Kleine verzameling device/host helpers voor OTNDC

// ---------- vector helpers ----------

__host__ __device__ inline float3 make3hf(float x,float y,float z)
{
    return make_float3(x,y,z);
}

__host__ __device__ inline float dot3hf(const float3& a, const float3& b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

__host__ __device__ inline float3 norm3hf(const float3& v_in)
{
    float3 v = v_in;
    float n2 = v.x*v.x + v.y*v.y + v.z*v.z;
    if(n2 < 1e-20f) return make3hf(0,0,0);
    float inv = rsqrtf(n2);
    v.x *= inv; v.y *= inv; v.z *= inv;
    return v;
}

__host__ __device__ inline float pow2hf(float x)
{
    return x * x;
}

// ---------- sin^k(θ)-gewicht ----------
//
// Input: cosθ (dot-product tussen genormaliseerde n en as)
// sin²θ = 1 - cos²θ
// w = (sin²θ)^(sinPowK), sinPowK ∈ [1..4]
//
// sinPowK = 1 → sin²
//          2 → sin⁴
//          3 → sin⁸
//          4 → sin¹⁶
//

__host__ __device__ inline float orthoWeightFromDot(float cosTheta, int sinPowK)
{
    if(sinPowK < 1) sinPowK = 1;
    if(sinPowK > 4) sinPowK = 4;

    float cos2 = cosTheta * cosTheta;
    float sin2 = 1.0f - cos2;
    if(sin2 < 0.0f) sin2 = 0.0f;

    float w = sin2;
    for(int k=1; k<sinPowK; ++k)
    {
        w = pow2hf(w);
    }
    return w;
}

// ---------- EMA-nudge ----------
//
// Berekent nieuwe asrichting a' via een EMA in richting n:
//
//   beta   = alpha * weight
//   a'     = (1-beta)*a + beta*n
//   a'_norm = normalize(a')
//
// Als a of n te klein is, dan krijg je (0,0,0) terug.
//

__host__ __device__ inline float3 emaNudgeDirection(
    const float3& a_in,
    const float3& n_in,
    float alpha,
    float weight)
{
    float3 a = a_in;
    float3 n = n_in;

    // Beide vectoren moeten een beetje lengte hebben
    float a2 = a.x*a.x + a.y*a.y + a.z*a.z;
    float n2 = n.x*n.x + n.y*n.y + n.z*n.z;
    if(a2 < 1e-20f || n2 < 1e-20f)
        return make3hf(0,0,0);

    float beta = alpha * weight;
    if(beta > 1.0f) beta = 1.0f;
    float oneMinus = 1.0f - beta;

    float3 anew;
    anew.x = oneMinus * a.x + beta * n.x;
    anew.y = oneMinus * a.y + beta * n.y;
    anew.z = oneMinus * a.z + beta * n.z;

    return norm3hf(anew);
}

