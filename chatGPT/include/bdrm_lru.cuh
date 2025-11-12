#pragma once
#include &lt;cuda_runtime.h&gt;
#include "consts.cuh"

/**

* Boeren-richtingstralenmethode (BDRM) — LRU ringbuffer voor richtingen.
* Bewaart top-k richtingskandidaten binnen een CTA (block), met simpele bubble-swap na score++.
*
* ASCII (per block in shared):
* slot:   0        1        2 ... k-1
* dir:   v0       v1       v2     vk
* score: s0       s1       s2     sk
*
* Invariant: geen volledige sort; alleen lokale swap als score[i] > score[i-1].
  */

struct LruItem {
float3 dir;   // genormaliseerd
int    score; // stemmen
int    used;  // 0/1
};

template <int K>
struct LruBlock {
LruItem items[K];

```
__device__ void init() {
    #pragma unroll
    for (int i=0;i&lt;K;i++){ items[i].dir=make_float3(0,0,0); items[i].score=0; items[i].used=0; }
}

__device__ int find_similar(const float3 v) {
    #pragma unroll
    for (int i=0;i&lt;K;i++){
        if (!items[i].used) continue;
        float c = fabsf(items[i].dir.x*v.x + items[i].dir.y*v.y + items[i].dir.z*v.z);
        if (c &gt;= Consts::SIMILAR_COS_MIN) return i;
    }
    return -1;
}

__device__ int find_empty_or_evict() {
    // probeer leeg slot
    #pragma unroll
    for (int i=0;i&lt;K;i++) if (!items[i].used) return i;
    // evict agressief op ratio
    int maxIdx=0;
    for (int i=1;i&lt;K;i++) if (items[i].score &gt; items[maxIdx].score) maxIdx=i;
    const float mx = float(items[maxIdx].score);
    for (int i=0;i&lt;K;i++) {
        if (float(items[i].score) * Consts::EVICT_RATIO &lt; mx) return i;
    }
    // geen evict: pak laagste score
    int minIdx=0;
    for (int i=1;i&lt;K;i++) if (items[i].score &lt; items[minIdx].score) minIdx=i;
    return minIdx;
}

__device__ void bump_and_bubble(int idx) {
    items[idx].score += 1;
    // bubble 1 stap naar links indien nodig
    if (idx &gt; 0 &amp;&amp; items[idx].score &gt; items[idx-1].score) {
        LruItem tmp = items[idx-1];
        items[idx-1] = items[idx];
        items[idx]   = tmp;
    }
}

__device__ int add_or_bump(const float3 v, int warmupLeft) {
    int j = find_similar(v);
    if (j &gt;= 0) { bump_and_bubble(j); return j; }
    // alleen nieuwe richting toevoegen als nog warmup of evict toegestaan
    int i = find_empty_or_evict();
    items[i].dir   = v;
    items[i].score = (warmupLeft&gt;0 ? 1 : 1); // init score
    items[i].used  = 1;
    // geen bubble voor nieuw; wordt vanzelf omhoog gedrukt bij volgende hits
    return i;
}
```

};