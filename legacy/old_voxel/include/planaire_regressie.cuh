#pragma once
#include "math_hulpfuncties.cuh"

namespace jbf
{

    // Klein helper: 1D richtingsfit in vlak met trim van 3 grootste residuen.
    // Voor simpelheid nemen we hier een lineaire fit y = a x + b in lokaal 2D.
    struct RegressieUitkomst
    {
        float3 dir;  // richting in 3D (teruggeheven naar vlak)
        float  a, b; // optioneel, parameters in 2D
    };

    template <int K = 3>
    __device__ __forceinline__ RegressieUitkomst
    d_lineaireRegressieInVlak(const float3* __restrict__ pts, const uint32_t* __restrict__ idx, uint32_t M,
                              const float3& vlak_norm, // u-of-v normaal voor dit doorsnedevlak
                              const float3& tang,      // tangent-richting binnen het projectievlak (u of v)
                              const float3& n_current) // huidige n (voor 3D lift)
    {
        // Projecteer naar lokaal 2D: x = dot(p,tang), y = dot(p, n_current × tang)
        float3 ortho = d_norm3(d_cross3(n_current, tang));

        // 1e pass: residuen bepalen t.o.v. grove fit (we nemen in JBF-stijl eerst moment-statistiek)
        // Voor eenvoud: eerste fit via moment (∑x, ∑y, ∑xy, ∑x^2)
        float    Sx = 0.f, Sy = 0.f, Sxy = 0.f, Sxx = 0.f;
        uint32_t N = 0;
        for(uint32_t i = 0; i < M; ++i)
        {
            const float3 p = pts[idx[i]];
            float        x = jbf::d_dot3(p, tang);
            float        y = jbf::d_dot3(p, ortho);
            Sx += x;
            Sy += y;
            Sxy += x * y;
            Sxx += x * x;
            ++N;
        }
        float denom = fmaxf(1e-12f, (N * Sxx - Sx * Sx));
        float a     = (N * Sxy - Sx * Sy) / denom;
        float b     = (Sy - a * Sx) / fmaxf(1e-12f, N);

        // Zoek K grootste residuen (|y - (ax+b)|), markeer die.
        float worst[K];
        int   worstIdx[K];
#pragma unroll
        for(int k = 0; k < K; ++k)
        {
            worst[k]    = -1.f;
            worstIdx[k] = -1;
        }

        for(uint32_t i = 0; i < M; ++i)
        {
            const float3 p = pts[idx[i]];
            float        x = jbf::d_dot3(p, tang);
            float        y = jbf::d_dot3(p, ortho);
            float        r = fabsf(y - (a * x + b));
            // kleine top-K selectie
            int m = 0;
            for(int k = 1; k < K; ++k)
                if(worst[k] < worst[m]) m = k;
            if(r > worst[m])
            {
                worst[m]    = r;
                worstIdx[m] = i;
            }
        }

        // 2e pass: refit zonder die K outliers
        Sx = Sy = Sxy = Sxx = 0.f;
        N                   = 0;
        for(uint32_t i = 0; i < M; ++i)
        {
            bool skip = false;
#pragma unroll
            for(int k = 0; k < K; ++k)
                if(i == (uint32_t)worstIdx[k])
                {
                    skip = true;
                    break;
                }
            if(skip) continue;

            const float3 p = pts[idx[i]];
            float        x = jbf::d_dot3(p, tang);
            float        y = jbf::d_dot3(p, ortho);
            Sx += x;
            Sy += y;
            Sxy += x * y;
            Sxx += x * x;
            ++N;
        }
        denom = fmaxf(1e-12f, (N * Sxx - Sx * Sx));
        a     = (N * Sxy - Sx * Sy) / denom;
        b     = (Sy - a * Sx) / fmaxf(1e-12f, N);

        // Richting in 2D is langs toenemende x → 3D richting is 'tang' (y verandert met a*x)
        // We kunnen de 3D richting in vlak kiezen als unit(tang + a * ortho) maar dat maakt hem schuin.
        // Voor jouw doel: de *richting-as* binnen het doorsnedevlak is primair 'tang'.
        // We geven hier 'tang' terug als dominante richting in dit vlak.
        RegressieUitkomst out;
        out.dir = tang;
        out.a   = a;
        out.b   = b;
        return out;
    }

    __device__ __forceinline__ float3 d_planaireRegressie(const float3* __restrict__ pts,
                                                          const uint32_t* __restrict__ idx, uint32_t M,
                                                          const float3& n_in, const float3& X, const float3& Y,
                                                          const float3& Z, float cosParallelMax, float maxRefineDeg)
    {
        float3 u, v;
        jbf::d_bepaal_u_v_uit_n(n_in, X, Y, Z, cosParallelMax, u, v);

        // Regressie in U-doorsnede (vlak bepaald door n_in en u)
        auto ruU = d_lineaireRegressieInVlak(pts, idx, M, /*vlak_norm=*/n_in, /*tang=*/u, /*n=*/n_in);
        // Regressie in V-doorsnede
        auto ruV = d_lineaireRegressieInVlak(pts, idx, M, /*vlak_norm=*/n_in, /*tang=*/v, /*n=*/n_in);

        float3 n_star = jbf::d_norm3(jbf::d_cross3(ruU.dir, ruV.dir));
        return jbf::d_clamp_richting(n_in, n_star, maxRefineDeg);
    }

} // namespace jbf
