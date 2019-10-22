#ifdef __CUDA_SUPPORT__
#include <octomap/KeyArrayCuda.cuh>

namespace octomap {
  __device__ void KeyArrayCuda::addKeyAtomic(const OcTreeKey& k) {
      int idx = atomicAdd(&last, 1);
      ray[idx] = k;
    }
}
#endif