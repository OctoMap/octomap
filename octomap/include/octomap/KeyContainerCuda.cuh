#ifndef KEY_CONTAINDER_CUDA_CUH
#define KEY_CONTAINDER_CUDA_CUH
#ifdef __CUDA_SUPPORT__
#include <cuda.h>
#include <cuda_runtime.h>
#include <octomap/AssertionCuda.cuh>
#include <octomap/OcTreeKey.h>

namespace octomap {

  class KeyContainerCuda {
  public:
    
    CUDA_CALLABLE KeyContainerCuda (
      const int& maxSize = 100000) :
      maxSize(maxSize)
    {
      ray = new OcTreeKey[maxSize];
      reset();
    }

    CUDA_CALLABLE ~KeyContainerCuda () {
      delete ray;
    }
    
    CUDA_CALLABLE KeyContainerCuda(const KeyContainerCuda& other)
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

    __host__ void copyToDevice(const KeyContainerCuda& other) {
      assert (maxSize == other.sizeMax());
      cudaCheckErrors(cudaMemcpy(ray, other.ray, maxSize * sizeof(OcTreeKey), cudaMemcpyHostToDevice));
      last = other.last;
      maxSize = other.maxSize;
    }

    __host__ void copyToHost(const KeyContainerCuda& other) {
      assert (maxSize == other.sizeMax());
      cudaCheckErrors(cudaMemcpy(ray, other.ray, maxSize * sizeof(OcTreeKey), cudaMemcpyDeviceToHost));
      last = other.last;
    }

    CUDA_CALLABLE KeyContainerCuda& operator=(const KeyContainerCuda& other){
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

  using KeyRayCuda = KeyContainerCuda;
}
#endif
#endif