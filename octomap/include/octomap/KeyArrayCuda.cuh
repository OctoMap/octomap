#ifndef KEY_ARRAY_CUDA_CUH
#define KEY_ARRAY_CUDA_CUH
#ifdef __CUDA_SUPPORT__
#include <cuda.h>
#include <cuda_runtime.h>
#include <octomap/AssertionCuda.cuh>
#include <octomap/OcTreeKey.h>
#include <octomap/HashSetCuda.cuh>

namespace octomap {

  class KeyArrayCuda {
  public:
    
    CUDA_CALLABLE KeyArrayCuda (
      const int& maxSize = 100) :
      maxSize(maxSize)
    {
    }

    CUDA_CALLABLE ~KeyArrayCuda () {
    }
    
    CUDA_CALLABLE KeyArrayCuda(const KeyArrayCuda& other)
    {
      ray = other.ray;
      last = other.last;
      maxSize = other.maxSize;
    }
    
    __host__ void allocateDevice() {
      cudaCheckErrors(cudaMallocManaged(&ray, maxSize * sizeof(OcTreeKey)))
    }

    __host__ void freeDevice() {
      cudaCheckErrors(cudaFree(ray));
    }

    __host__ void copyFromHost(const KeyArrayCuda& other) {
      assert (maxSize == other.sizeMax());
      cudaCheckErrors(cudaMemcpy(ray, other.ray, maxSize * sizeof(OcTreeKey), cudaMemcpyHostToDevice));
      last = other.last;
      maxSize = other.maxSize;
    }

    __host__ void copyFromDevice(const KeyArrayCuda& other) {
      assert (maxSize == other.sizeMax());
      cudaCheckErrors(cudaMemcpy(ray, other.ray, maxSize * sizeof(OcTreeKey), cudaMemcpyDeviceToHost));
      last = other.last;
    }

    CUDA_CALLABLE KeyArrayCuda& operator=(const KeyArrayCuda& other){
      ray = other.ray;
      last = other.last;
      maxSize = other.maxSize;
      return *this;
    }
    
    CUDA_CALLABLE void reset() {
      last = 0;
    }
    
    CUDA_CALLABLE void addKey(const OcTreeKey& k) {
      assert(last != maxSize);
      ray[last] = k;
      ++last;
    }

    __device__ void addKeyAtomic(const OcTreeKey& k);

    CUDA_CALLABLE const OcTreeKey* begin() { return ray; }
    CUDA_CALLABLE const OcTreeKey* end() { return &ray[last]; }

    CUDA_CALLABLE int size() const { return last; }
    CUDA_CALLABLE int sizeMax() const { return maxSize; }
    
  private:
    OcTreeKey* ray;
    int last;
    int maxSize;
  };

  using KeyRayCuda = KeyArrayCuda;
}


#endif
#endif