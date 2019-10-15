#ifdef __CUDA_SUPPORT__
#include <octomap/KeyContainerCUDA.cuh>

namespace octomap {
  __device__ void KeyContainerCUDA::addKeyAtomic(const OcTreeKey& k) {
      int idx = atomicAdd(&last, 1);
      ray[idx] = k;
    }
}
#endif