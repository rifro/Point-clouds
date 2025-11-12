#include &lt;thrust/device_ptr.h&gt;
#include &lt;thrust/sort.h&gt;
#include &lt;thrust/tuple.h&gt;
#include &lt;thrust/iterator/zip_iterator.h&gt;
#include "types.cuh"

/**

* Sorteer labels op (type, clusterId, mortonKey) stabiel om spatial locality te bewaren.
* Aanname: d_morton bestaat (uint64_t*) parallel aan punten.
  */

extern "C" void sort_labels_by_type_cluster_morton(uint8_t* d_type,
uint32_t* d_cluster,
uint64_t* d_morton,
int N,
cudaStream_t stream)
{
using thrust::device_ptr;
using thrust::make_tuple;
using thrust::make_zip_iterator;

```
auto t0 = device_ptr&lt;uint8_t&gt;(d_type);
auto t1 = device_ptr&lt;uint32_t&gt;(d_cluster);
auto t2 = device_ptr&lt;uint64_t&gt;(d_morton);

auto zip_begin = make_zip_iterator(make_tuple(t0, t1, t2));
auto zip_end   = zip_begin + N;

thrust::stable_sort(thrust::cuda::par.on(stream), zip_begin, zip_end,
    [] __device__ (const thrust::tuple&lt;uint8_t,uint32_t,uint64_t&gt;&amp; a,
                   const thrust::tuple&lt;uint8_t,uint32_t,uint64_t&gt;&amp; b)
    {
        if (thrust::get&lt;0&gt;(a) &lt; thrust::get&lt;0&gt;(b)) return true;
        if (thrust::get&lt;0&gt;(a) &gt; thrust::get&lt;0&gt;(b)) return false;
        if (thrust::get&lt;1&gt;(a) &lt; thrust::get&lt;1&gt;(b)) return true;
        if (thrust::get&lt;1&gt;(a) &gt; thrust::get&lt;1&gt;(b)) return false;
        return thrust::get&lt;2&gt;(a) &lt; thrust::get&lt;2&gt;(b);
    }
);
```

}