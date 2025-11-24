#pragma once
#include "otndc.h" // for OTAsAccu
#include <cuda_runtime.h>

// kernel forward declaration
extern "C" __global__ void k_otndc_pairs(
    const float* __restrict__ x,
    const float* __restrict__ y,
    const float* __restrict__ z,
    int N,
    float binnen2, float buiten2,
    float maxDotOrtho,
    int partnerZoekRadius,
    OTAsAccu* __restrict__ gAccu);
