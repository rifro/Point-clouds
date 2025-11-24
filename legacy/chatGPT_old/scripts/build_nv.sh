#!/usr/bin/env bash
set -e
nvcc -std=c++17 -O3 -arch=sm_80 \
  -Iinclude \
  src/k_r1r2_filter.cu \
  src/k_local_slab_test.cu \
  src/k_direction_vote_bdrm.cu \
  src/k_label_sort.cu \
  src/h_do_optimize_frame2.cpp \
  -o build/autofit_demo
echo "Build OK → build/autofit_demo"