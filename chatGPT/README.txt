Autofit GPU - JBF pipeline (NL identifiers)

Bouwen zonder CMake:
chmod +x nv
./nv

Inprikken in CloudCompare:

* Koppel h_do_optimize_frame3.cpp aan jouw plugin bronbestand waar AutofitImpl::doOptimizeFrame3() leeft.
* Roep h_do_optimize_frame3(...) aan met CCVector3/ccPointCloud adapter (zie include/cc_adapter.h).

Directory-indeling:
include/
config.h                ← globale drempels/constanten (EPS_NEAR2, TAU_FAR2, R_MIN, R_MAX, SECTORS_HALF, enz.)
types.h                 ← basis types/labels/AxisPermutatie
device_buffer.cuh       ← DeviceBuffer<T> (alloc/kopie/resize)
morton_utils.cuh        ← 64-bit Morton encode/decode + region-prefix
cc_adapter.h            ← dunne adapter naar CloudCompare types (pas include-pad aan)
src/
h_do_optimize_frame3.cpp← host-sturing (JBF flow) — roept kernels (worden in part02+ geleverd)
k_voxeliseer_morton.cu  ← quantize → morton keys → CUB radixsort → permute floats (komt in part02)
k_ruis_boeren.cu        ← 2-burenfilter (komt in part02)
k_vlak_vote.cu          ← zaad triangle → lokale support → richting-stem (komt in part02)
k_buis_vote.cu          ← annulus/wedge voor buisrichting (komt in part03)
k_reduce_groepen.cu     ← reduce-by-key voor (richting-keys) (komt in part03)
k_planair_regressie.cu  ← JBF u*/v* regressie → n* (komt in part03)
k_rotatie_quat.cu       ← roteren naar ideaal frame (komt in part04)
k_fine_slabs_cyl.cu     ← fine labeling na frame (komt in part04)

Buildscript:
nv  (nvcc, sm_89, C++20, extended-lambda)

Let op taalafspraken:

* NL identifiers & comments
* Uitzonderingen toegestaan: seed, triangle, tree, forest, vote, count, slab
* Kernels: prefix k_
* Device helpers: d_
* Host helpers: h_

Volgorde van werken:

1. ./nv bouwen
2. Integreer cc_adapter.h include-pad
3. Start met h_do_optimize_frame3.cpp (logt elke stap); kernels volgen in part02/03/04.