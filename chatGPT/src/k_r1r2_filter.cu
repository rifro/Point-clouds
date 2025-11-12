#include &lt;cuda_runtime.h&gt;
#include "consts.cuh"
#include "types.cuh"

/*
k_r1r2_filter
-------------

In:  d_pts (Vec3f*), N
Out: d_mask_tri (uint8*)  : 1 = mini-triangle kandidaat (exclusief), 0 anders
d_norm_tri (float3*) : normale van (c,a,b) indien mask==1
d_inlier_slab (int*) : aantal slab-inliers binnen r2 (voor chaos veto)
d_is_chaos (uint8*)  : 1 = chaos, 0 = ok
NB: eenvoudige implementatie: per punt c zoeken we 2 dichtste in ring [r1…r2], dan exclusiviteitscheck.

ASCII:
r2
.-----.
/       \    ring [r1..r2], kies 2 dichtste {a,b}, check max(|c-a|,|c-b|) <= r2/2 - 1mm
|    c    |
\       /
'-----'
r1
*/

**device** inline float3 make3(const Vec3f& p){ return make_float3(p.x,p.y,p.z); }
**device** inline float  dot3(const float3 a,const float3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
**device** inline float3 cross3(const float3 a,const float3 b){return make_float3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);}
**device** inline float  len3(const float3 v){return sqrtf(dot3(v,v));}
**device** inline float3 norm3(const float3 v){float L=len3(v); return (L>0)?make_float3(v.x/L,v.y/L,v.z/L):make_float3(0,0,0);}

**global** void k_r1r2_filter(const Vec3f* **restrict** d_pts,
int N,
uint8_t* **restrict** d_mask_tri,
float3*  **restrict** d_norm_tri,
int*     **restrict** d_inlier_slab,
uint8_t* **restrict** d_is_chaos)
{
const int i = blockIdx.x*blockDim.x + threadIdx.x;
if (i>=N) return;

```
const float r1  = Consts::R1_CLUSTER;
const float r2  = Consts::R2_RING;
const float r2h = Consts::R2_HALF_CLAIM_SAFE;
const float slabTol = Consts::SLAB_TOL;

const float3 c = make3(d_pts[i]);

// eenvoudig: brute buurt (N) — in praktijk: per tile/voxel buurtlijst
// DEMO: we gaan lineair door (kan later getiled worden)
float best1=1e9f, best2=1e9f; int ia=-1, ib=-1;

for (int j=0;j&lt;N;j++){
    if (j==i) continue;
    const float3 q = make3(d_pts[j]);
    float d = len3(make_float3(q.x-c.x,q.y-c.y,q.z-c.z));
    if (d &lt; r1-Consts::LIDAR_HALF_ERROR) continue;
    if (d &gt; r2+Consts::LIDAR_HALF_ERROR) continue;

    if (d&lt;best1){ best2=best1; ib=ia; best1=d; ia=j; }
    else if (d&lt;best2){ best2=d; ib=j; }
}

// init outputs
d_mask_tri[i]=0; d_is_chaos[i]=0; d_inlier_slab[i]=0; d_norm_tri[i]=make_float3(0,0,0);

// slab inliers quick check (vlak vs chaos)
int slabCnt=0;
if (ia&gt;=0 &amp;&amp; ib&gt;=0)
{
    // normale van driehoek (c,a,b)
    const float3 a = make3(d_pts[ia]);
    const float3 b = make3(d_pts[ib]);
    const float3 n = norm3(cross3(make_float3(a.x-c.x,a.y-c.y,a.z-c.z),
                                  make_float3(b.x-c.x,b.y-c.y,b.z-c.z)));

    // slab door c met normaal n: |dot(n, p-c)| &lt;= slabTol
    for (int j=0;j&lt;N;j++){
        float3 q = make3(d_pts[j]);
        float3 qc = make_float3(q.x-c.x,q.y-c.y,q.z-c.z);
        float ad = fabsf(dot3(n,qc));
        if (ad &lt;= slabTol) slabCnt++;
    }

    d_inlier_slab[i] = slabCnt;

    // exclusiviteitscheck
    const bool triOk = (best1 &lt;= r2h) &amp;&amp; (best2 &lt;= r2h);
    if (triOk) { d_mask_tri[i]=1; d_norm_tri[i]=n; }
}

// chaos veto
// cheap 8mm-range + inliers&lt;4
// (range berekenen: hier approximatie via min/max in X/Y/Z; demo: subsample)
if (slabCnt &lt;= Consts::SLAB_MIN_INLIERS-1) {
    // heuristische cheap-check: markeer chaos indien geen triangle en weinig slab-inliers
    if (!d_mask_tri[i]) d_is_chaos[i]=1;
}
```

}