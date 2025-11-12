#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include "config.h"
#include "types.h"
#include "wiskunde_utils.cuh"

// JBF-planar regressie: voor een candidate vlak (normaal n0) binnen puntset,
// refine naar n* door projectie op (u,v) basis, lineaire regressie op 2D,
// bereken nieuwe n* = normalize(u* × v*), en bereken d via trim-K methode.

// Kernel: voor eenvoud per blok één kandidaat plaatser (optioneel uitbreiden),
// nu: host roept één candidate richting aan.
// Input: punten_sorted[N], kandidaat normaal n0, parameter k_trim_fraction
// Output: top RichtingPeak (richting n*, stemmen = aantal kept, vertrouwen ~ 1/(1+rms))

**global** void k_planair_regressie_kernel(const Vec3f* **restrict** punten,
u32 N,
Vec3f n0,
float trim_frac,
Vec3f* out_n_star,
u32*   out_kept,
float* out_rms)
{
extern **shared** float s_buf[]; // tijdelijk geen gebruik
// Eén thread per kernel (host scale) — dus blockDim=1, gridDim=1

```
// Stap 1: maak u,v basis uit n0
Vec3f u,v;
maak_orthobasis_uv_uit_n(n0, u, v);

// Stap 2: projecteer alle punten op u,v: (u_i, v_i) in 2D
// On‐device simpel accumulate
float sum_u=0.f, sum_v=0.f, sum_uu=0.f, sum_uv=0.f, sum_vv=0.f;
for (u32 i=0;i<N;++i){
    Vec3f p = punten[i];
    float uu = dot(p,u);
    float vv = dot(p,v);
    sum_u += uu;
    sum_v += vv;
    sum_uu += uu*uu;
    sum_uv += uu*vv;
    sum_vv += vv*vv;
}

float mean_u = sum_u / N;
float mean_v = sum_v / N;

float cov_uu = sum_uu/N - mean_u*mean_u;
float cov_uv = sum_uv/N - mean_u*mean_v;
float cov_vv = sum_vv/N - mean_v*mean_v;

// Eenvoudige Richting v* op basis van covariantie
Vec3f v_star  = norm( make_vec3f(cov_uv, cov_vv - cov_uu, 0.f) );
Vec3f u_star  = norm( cross(v_star, n0) );

Vec3f n_star = norm( cross(u_star, v_star) );

// Stap 3: afstand d_i = dot(n_star, p_i). Bereken RMS na trim-K
// Kopiëer d_i in host‐array? Simpel: host doet dit later. Hier device approx:
float sum_d2 = 0.f;
for (u32 i=0;i<N;++i){
  float di = dot(n_star, punten[i]);
  sum_d2 += di*di;
}
float rms = sqrtf(sum_d2 / (float)N);

*out_n_star = n_star;
*out_kept  = N;
*out_rms   = rms;
```

}

extern "C"
void k_planair_regressie_launch(const Vec3f* d_points_sorted,
u32 N,
const Vec3f& n0,
u32 K_trim_frac_int,
RichtingPeak& peakOut)
{
if (N==0) {
peakOut.stemmen   = 0;
peakOut.vertrouwen= 0.f;
peakOut.richting  = make_vec3f(0,0,0);
return;
}

```
Vec3f* d_nstar; uint32_t* d_kept; float* d_rms;
cudaMalloc(&d_nstar, sizeof(Vec3f));
cudaMalloc(&d_kept,  sizeof(uint32_t));
cudaMalloc(&d_rms,   sizeof(float));

float trim_frac   = ((float)K_trim_frac_int) * 0.01f;
k_planair_regressie_kernel<<<1,1, 0>>>(d_points_sorted, N, n0, trim_frac, d_nstar, d_kept, d_rms);
cudaDeviceSynchronize();

Vec3f h_nstar; uint32_t h_kept; float h_rms;
cudaMemcpy(&h_nstar, d_nstar, sizeof(Vec3f), cudaMemcpyDeviceToHost);
cudaMemcpy(&h_kept,  d_kept,  sizeof(uint32_t), cudaMemcpyDeviceToHost);
cudaMemcpy(&h_rms,   d_rms,   sizeof(float),  cudaMemcpyDeviceToHost);

cudaFree(d_nstar);
cudaFree(d_kept);
cudaFree(d_rms);

peakOut.richting   = h_nstar;
peakOut.stemmen    = h_kept;
peakOut.vertrouwen = 1.f / (1.f + h_rms);  // simpele confidence
```

}