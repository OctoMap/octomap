#ifdef __CUDA_SUPPORT__
#include <octomap/KeyArrayCuda.cuh>

namespace octomap {
  int KeyRayConfig::max_ray_size_ = 100;

  template <typename T>
  __device__ void ArrayCuda<T>::addKeyAtomic(const T& k) {
      ray_[atomicAdd(&last_, 1)] = k;
    }

  __device__ unsigned long long int KeyPtrArrayCuda::addKeyAtomicAt(const OcTreeKey* k, const int& l) {
    return atomicCAS((unsigned long long int*)&ray_[l], 0, (unsigned long long int)k);
  }

  template class ArrayCuda<size_t>;
  template class ArrayCuda<OcTreeKey*>;
  template class ArrayCuda<KeyHash>;
}
#endif