#include <cuda_runtime.h>
#include "vlak_types.h"
#include "voxel_params.h"

using namespace jbf;
using namespace grid;

__global__ void k_initLabels(uint8_t* __restrict__ labels, uint32_t N, uint8_t waarde)
{
    uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < N) labels[i] = waarde;
}

// tel buren in 3x3x3 voxels; stop vroeg zodra 2 gevonden
__global__ void k_ruisBoerenTweedeBuur(const float3* __restrict__ pts,
                                       uint8_t* __restrict__ labels,
                                       uint32_t N,
                                       VoxelRaster vr,
                                       VoxelIndexing vx,
                                       float r_keep /*≈ 1–3 mm*/)
{
    const float r2 = r_keep * r_keep;
    uint32_t v = blockIdx.x; // 1 CTA per voxel (of chunk-wise)
    if (v >= vx.numVoxels) return;

    uint32_t begin = vx.d_voxelStarts[v];
    uint32_t eind  = vx.d_voxelStarts[v+1];
    if (begin >= eind) return;

    // decode v → (ix,iy,iz)
    uint32_t S = vr.S;
    uint32_t iz = v / (S*S);
    uint32_t iy = (v / S) % S;
    uint32_t ix = v % S;

    // buur-voxel grenzen
    int x0 = max<int>(0, ix-1), x1 = min<int>(S-1, ix+1);
    int y0 = max<int>(0, iy-1), y1 = min<int>(S-1, iy+1);
    int z0 = max<int>(0, iz-1), z1 = min<int>(S-1, iz+1);

    // voor elk punt in deze voxel: zoek 2 buren
    for (uint32_t i = begin + threadIdx.x; i < eind; i += blockDim.x)
    {
        if (labels[i] != ONGELABELD) continue; // al gezet elders
        float3 p = pts[i];
        int found = 0;

        for (int zz=z0; zz<=z1 && found<2; ++zz)
        for (int yy=y0; yy<=y1 && found<2; ++yy)
        for (int xx=x0; xx<=x1 && found<2; ++xx)
        {
            uint32_t nb = flatten(xx,yy,zz,S,S,S);
            uint32_t nbB = vx.d_voxelStarts[nb];
            uint32_t nbE = vx.d_voxelStarts[nb+1];
            for (uint32_t j = nbB; j < nbE; ++j) {
                if (j == i) continue;
                float3 q = pts[j];
                float dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
                float d2 = dx*dx + dy*dy + dz*dz;
                if (d2 <= r2) {
                    ++found;
                    if (found >= 2) break;
                }
            }
        }

        if (found < 2) labels[i] = RUIS; // geen 2e buur → ruis
    }
}
