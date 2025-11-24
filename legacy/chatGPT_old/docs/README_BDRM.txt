GPU-Autofit — Boeren-richtingstralenmethode (BDRM)
==================================================

## Doel

Betrouwbare hoofdassen vinden in een pointcloud zonder SVD/PCA:
– geen hemisfeer-binning
– geen full sort per richting
– JBF (simpel, robuust, snel)

## Kernidee

• Raakvlak-normaal van buizen staat loodrecht op de as.
• Transleer alle “stralen” conceptueel naar O (we gebruiken alleen richting).
• Cluster die richtingen met een LRU per block en voeg in warm-up enkele kruisproducten toe.
• Vlakken stemmen direct hun normaal; buizen induceren via kruisen de asrichting.

## Pipeline (vroege fase)

1. k_r1r2_filter
   – Zoek 2 dichtste buren in ring [r1..r2]
   – Exclusieve driehoek check: max(|c-a|,|c-b|) ≤ r2/2 − 1mm
   – Normaal van mini-triangle + slab-inliers tellen
   – Chaos = geen tri & weinig slab-inliers

2. k_local_slab_test
   – Markeer ok_for_vote (geen chaos) — vlak en “curved” mogen beiden stemmen

3. k_direction_vote_bdrm
   – LRU per block (K=8)
   – Stem op n
   – Warm-up: kruisen met top-slots als ~orthogonaal → voeg ontbrekende assen toe
   – Eén bubble-swap per update (geen dure sort)
   – Output: top M winners per block → host combineert tot A*,B*,C*

## Labels

• In deze fase worden labels alleen geïnitialiseerd (Ruis).
• Na frame-lock volgt fine slab/cylinder labeling in ideaal frame (volgende stap).

## Build

./scripts/build_nv.sh

## Bestanden

include/consts.cuh             – toleranties
include/types.cuh              – Vec3f, PuntType
include/device_buffer.cuh      – eenvoudige device buffer
include/bdrm_lru.cuh           – LRU per block
src/k_r1r2_filter.cu           – ring + tri + slab inliers
src/k_local_slab_test.cu       – ok_for_vote
src/k_direction_vote_bdrm.cu   – LRU stemmen + warm-up kruisen
src/k_label_sort.cu            – presentatie sort (na labeling)
src/h_do_optimize_frame2.cpp   – host flow en meldingen