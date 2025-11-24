#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <iostream>

int main() {
    // Device data
    thrust::device_vector<unsigned long long> keys = {5, 2, 8, 1};
    thrust::device_vector<int> values = {50, 20, 80, 10};

    thrust::sort_by_key(keys.begin(), keys.end(), values.begin());

    thrust::host_vector<unsigned long long> h_keys = keys;
    thrust::host_vector<int> h_values = values;

    for (size_t i = 0; i < h_keys.size(); ++i)
        std::cout << h_keys[i] << " : " << h_values[i] << "\n";

    return 0;
}
