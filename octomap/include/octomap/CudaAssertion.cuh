#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>

#ifndef CUDA_ASSERTION_CUH
#define CUDA_ASSERTION_CUH

#define cudaCheckErrors(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}
#endif