#include &lt;cuda_runtime.h&gt;
#include &lt;cooperative_groups.h&gt;
#include "consts.cuh"
#include "types.cuh"
#include "bdrm_lru.cuh"

namespace cg = cooperative_groups;

/*
k_direction_vote_bdrm
---------------------

Boeren-richtingstralenmethode:
- Neem normale n van mini-triangle (of fallback), translatie naar O is impliciet (we gebruiken alleen richting).
- LRU per block; stem op n.
- In warmup: maak max 2 cross-products met top-slots die bijna orthogonaal zijn; voeg toe als nieuw.
- Bij stem: als LRU al vector ~orthogonaal heeft, bump ook die.

ASCII:
O  ← alle stralen (normals) worden gezien als pijlen vanuit O.
We clusteren VLAKRICHTINGEN en leiden asrichtingen af via kruisproducten.
*/

**device** inline float  dot3f(const float3 a,const float3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
**device** inline float3 cross3f(const float3 a,const float3 b){return make_float3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);}
**device** inline float  len3f(const float3 v){return sqrtf(dot3f(v,v));}
**device** inline float3 norm3f(const float3 v){float L=len3f(v); return (L>0)?make_float3(v.x/L,v.y/L,v.z/L):make_float3(0,0,0);}

template <int K>
**global** void k_direction_vote_bdrm(const float3* **restrict** d_norm_tri,
const uint8_t* **restrict** d_ok_for_vote,
int N,
float3* **restrict** d_block_winners, // per block: top 3
int    maxWinnersPerBlock)
{
extern **shared** unsigned char shmem[];
auto* lru = reinterpret_cast<LruBlock<K>*>(shmem);
if (threadIdx.x==0) lru->init();
__syncthreads();

```
const int gid  = blockIdx.x*blockDim.x + threadIdx.x;
const int lane = threadIdx.x;

int warmupLeft = Consts::WARMUP_SEEDS;

if (gid &lt; N &amp;&amp; d_ok_for_vote[gid]) {
    float3 n = d_norm_tri[gid];
    // force hemisfeer-consistentie (optioneel): maak z&gt;=0 b.v.
    if (n.z &lt; 0.f) { n.x=-n.x; n.y=-n.y; n.z=-n.z; }
    n = norm3f(n);

    // stem op n
    __syncthreads();
    int idx = lru-&gt;add_or_bump(n, warmupLeft);
    __syncthreads();

    // orthogonale partner bump
    if (idx &gt;= 0) {
        #pragma unroll
        for (int i=0;i&lt;K;i++){
            if (!lru-&gt;items[i].used) continue;
            if (i==idx) continue;
            if (fabsf(dot3f(lru-&gt;items[i].dir, n)) &lt;= Consts::ORTHO_COS_MAX){
                lru-&gt;bump_and_bubble(i);
                break; // één partner is genoeg
            }
        }
    }

    // warmup cross-product (max 2 tegen top slots)
    if (warmupLeft &gt; 0) {
        // top 2
        for (int t=0;t&lt;2 &amp;&amp; t&lt;K; ++t){
            if (!lru-&gt;items[t].used) continue;
            float c = fabsf(dot3f(lru-&gt;items[t].dir, n));
            if (c &lt;= Consts::ORTHO_COS_MAX){
                float3 v = norm3f(cross3f(n, lru-&gt;items[t].dir));
                if (len3f(v) &gt; 0.5f) {
                    // voeg toe als niet gelijk
                    int sim = lru-&gt;find_similar(v);
                    if (sim &lt; 0) lru-&gt;add_or_bump(v, warmupLeft);
                }
            }
        }
        atomicSub(&amp;warmupLeft, 1);
    }
}
__syncthreads();

// schrijf block-winnaars (top M)
if (threadIdx.x==0) {
    const int base = blockIdx.x*maxWinnersPerBlock;
    const int M = (maxWinnersPerBlock&lt;K?maxWinnersPerBlock:K);
    for (int m=0;m&lt;M;m++){
        d_block_winners[base+m] = lru-&gt;items[m].used ? lru-&gt;items[m].dir : make_float3(0,0,0);
    }
}
```

}

// explicit instantiation
extern "C" void launch_direction_vote_bdrm(const float3* d_norms,
const uint8_t* d_voteMask,
int N,
float3* d_blockWinners,
int maxWinnersPerBlock,
cudaStream_t stream,
int* gridOut, int* blockOut)
{
const int block = 256;
const int grid  = (N + block - 1)/block;
size_t sh = sizeof(LruBlock<Consts::LRU_SLOTS>);
k_direction_vote_bdrm<Consts::LRU_SLOTS><<<grid,block,sh,stream>>>(
d_norms, d_voteMask, N, d_blockWinners, maxWinnersPerBlock
);
if (gridOut) *gridOut=grid;
if (blockOut)*blockOut=block;
}