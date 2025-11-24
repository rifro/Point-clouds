#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/execution_policy.h>

#include <cuda_runtime.h>
#include <iostream>
#include <vector>

int main() {
    // ──────────────────────────────────────────────
    // 1. Laat zien welke GPU actief is
    int dev = 0;
    cudaGetDevice(&dev);
    cudaDeviceProp prop{};
    cudaGetDeviceProperties(&prop, dev);
    std::cout << "Running on GPU: " << prop.name
              << " (sm_" << prop.major << prop.minor << ")\n";

    // ──────────────────────────────────────────────
    // 2. Maak hostdata
    thrust::host_vector<unsigned long long> h_keys = {42, 7, 19, 3, 88, 12};
    thrust::host_vector<int> h_vals = {420, 70, 190, 30, 880, 120};

    // 3. Kopieer naar device
    thrust::device_vector<unsigned long long> d_keys = h_keys;
    thrust::device_vector<int> d_vals = h_vals;

    // ──────────────────────────────────────────────
    // 4. Sorteren op GPU met expliciete policy
    //    (zo test je dat Thrust daadwerkelijk CUDA gebruikt)
    thrust::sort_by_key(thrust::cuda::par, d_keys.begin(), d_keys.end(), d_vals.begin());

    // 5. Kopieer terug naar host
    thrust::copy(d_keys.begin(), d_keys.end(), h_keys.begin());
    thrust::copy(d_vals.begin(), d_vals.end(), h_vals.begin());

    // ──────────────────────────────────────────────
    // 6. Toon resultaat
    std::cout << "Gesorteerd resultaat:\n";
    for (size_t i = 0; i < h_keys.size(); ++i)
        std::cout << "  " << h_keys[i] << " : " << h_vals[i] << "\n";

    // 7. Synchroniseer en controleer op fouten
    cudaDeviceSynchronize();
    if (cudaGetLastError() == cudaSuccess)
        std::cout << "\nCUDA sortering geslaagd ✅\n";
    else
        std::cerr << "\nFout bij sortering ❌\n";

    return 0;
}
