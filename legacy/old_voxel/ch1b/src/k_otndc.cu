#include <cuda_runtime.h>
#include <math_constants.h>

#include "otndc.h"
#include "otndc_utils.cuh"

// Eén block, shared slot-cache.
// Elke thread loopt in stride over normals en werkt shared slots bij.
// Daarna worden slots naar global gekopieerd.

__global__
void k_otndc_fromNormals(
    const Vec3f* __restrict__ normals,
    int N,
    OTConfig cfg,
    OTSlot* __restrict__ gSlots)
{
    const int MAX_SLOTS = 16;
    __shared__ OTSlot slots[MAX_SLOTS];

    const int tid = threadIdx.x;
    const int B   = blockDim.x;

    // Init shared slots
    if(tid < cfg.maxSlots)
    {
        slots[tid].n     = make3hf(0,0,0);
        slots[tid].count = 0.0f;
    }
    __syncthreads();

    const int sinK = cfg.sinPowK;

    for(int idx = tid; idx < N; idx += B)
    {
        Vec3f vn = normals[idx];
        float3 n = make3hf(vn.x, vn.y, vn.z);
        n = norm3hf(n);
        float n2 = n.x*n.x + n.y*n.y + n.z*n.z;
        if(n2 < 1e-20f) continue;

        // Beste slot zoeken
        int   bestIdx      = -1;
        float bestAbsDot   = 1e30f;
        int   emptyIdx     = -1;

        for(int s=0; s<cfg.maxSlots; ++s)
        {
            float3 a = slots[s].n;
            float len2 = a.x*a.x + a.y*a.y + a.z*a.z;
            if(len2 < 1e-20f)
            {
                if(emptyIdx < 0) emptyIdx = s;
                continue;
            }
            float d  = dot3hf(n, a);
            float ad = fabsf(d);
            if(ad < bestAbsDot)
            {
                bestAbsDot = ad;
                bestIdx    = s;
            }
        }

        if(bestIdx >= 0 && bestAbsDot <= cfg.maxDotAxis)
        {
            // Match met bestaand slot
            float3 a = slots[bestIdx].n;

            float cosv = dot3hf(n, a);
            float w    = orthoWeightFromDot(cosv, sinK); // sin^{2^k}(θ)

            slots[bestIdx].count += w;

            // EMA-nudge
            float3 anew = emaNudgeDirection(a, n, cfg.alpha, w);
            float a2 = anew.x*anew.x + anew.y*anew.y + anew.z*anew.z;
            if(a2 > 1e-20f)
            {
                slots[bestIdx].n = anew;
            }

            // Max 2 bubbleswap-steps omhoog
            int i = bestIdx;
            for(int step=0; step<2 && i>0; ++step)
            {
                if(slots[i].count > slots[i-1].count)
                {
                    OTSlot tmp = slots[i-1];
                    slots[i-1] = slots[i];
                    slots[i]   = tmp;
                    --i;
                }
                else
                    break;
            }
        }
        else
        {
            // Geen goede match → nieuw slot als er nog eentje leeg is
            if(emptyIdx >= 0)
            {
                slots[emptyIdx].n     = n;
                slots[emptyIdx].count = 1.0f;
            }
            // Slots vol en geen goede match → negeren
        }
    }

    __syncthreads();

    if(tid < cfg.maxSlots)
    {
        gSlots[tid] = slots[tid];
    }
}
