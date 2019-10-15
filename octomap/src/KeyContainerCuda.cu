#ifdef __CUDA_SUPPORT__
#include <octomap/KeyContainerCuda.cuh>

namespace octomap {
  __device__ void KeyContainerCuda::addKeyAtomic(const OcTreeKey& k) {
      int idx = atomicAdd(&last, 1);
      ray[idx] = k;
    }
}
#endif