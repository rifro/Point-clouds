#pragma once
#include <cuda_runtime.h>
#include <stdexcept>

template <typename T> struct DeviceBuffer
{
    T*     ptr = nullptr;
    size_t n   = 0;

    DeviceBuffer() = default;
    explicit DeviceBuffer(size_t count) { alloc(count); }
    ~DeviceBuffer() { free(); }

    void alloc(size_t count)
    {
        free();
        n = count;
        if(n)
        {
            cudaError_t e = cudaMalloc(&ptr, n * sizeof(T));
            if(e != cudaSuccess) throw std::runtime_error("cudaMalloc failed");
        }
    }
    void free()
    {
        if(ptr) cudaFree(ptr);
        ptr = nullptr;
        n   = 0;
    }
    void upload(const T* h, size_t count)
    {
        if(count != n) throw std::runtime_error("upload size mismatch");
        if(count) cudaMemcpy(ptr, h, count * sizeof(T), cudaMemcpyHostToDevice);
    }
    void download(T* h, size_t count) const
    {
        if(count != n) throw std::runtime_error("download size mismatch");
        if(count) cudaMemcpy(h, ptr, count * sizeof(T), cudaMemcpyDeviceToHost);
    }
};