#ifdef __CUDA_SUPPORT__
#include <octomap/KeyArrayCuda.cuh>

namespace octomap {
  int KeyRayConfig::max_ray_size_ = 100;

  template <typename T>
  __device__ void ArrayCuda<T>::addKeyAtomic(const T& k) {
      ray_[atomicAdd(&last_, 1)] = k;
    }

  __device__ unsigned UnsignedArrayCuda::addKeyAtomicAt(const unsigned& k, const unsigned& l) {
    return atomicCAS((unsigned*)&ray_[l], 0, k);
  }

  template class ArrayCuda<size_t>;
  template class ArrayCuda<OcTreeKey*>;
  template class ArrayCuda<KeyHash>;
}
#endif