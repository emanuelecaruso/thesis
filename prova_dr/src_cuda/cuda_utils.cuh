#pragma once
#include <cuda_runtime.h>

inline void printCudaError(std::string error_name){
  cudaError_t err ;
  err = cudaGetLastError();
  cudaDeviceSynchronize();
  if (err != cudaSuccess){
    printf(error_name.c_str());
    printf(" Error: %s\n", cudaGetErrorString(err));
  }
}
