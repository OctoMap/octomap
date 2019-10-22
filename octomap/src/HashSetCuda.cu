#ifdef __CUDA_SUPPORT__
#include <octomap/HashSetCuda.cuh>

namespace octomap {
  __device__ volatile int sem = 0;

  __device__ void acquire_semaphore(volatile int *lock){
    while (atomicCAS((int *)lock, 0, 1) != 0);
  }

  __device__ void release_semaphore(volatile int *lock){
    *lock = 0;
    __threadfence();
  }

  __device__ void LockCuda::lock(void) {
    while(atomicCAS(mutex, 0, 1) != 0);
  }
  __device__ void LockCuda::unlock(void) {
    atomicExch(mutex, 0);
    __threadfence();
  }

  __device__ void HashSetCuda::insertDevice(const OcTreeKey& key, const int& value)
  {
    if (type_ == DEVICE) {
      size_t hash = OcTreeKey::KeyHash{}(key) % count_;
      // two different keys might have same hash
      __syncthreads();
      if (threadIdx.x == 0)
        acquire_semaphore(&sem);//pair_locks_[hash].lock();
        __syncthreads();
      if (!elements_[hash] || elements_[hash]->key_ != key) {
        auto location = &pool_[atomicAdd(&first_free_, 1)];
        location->key_ = key;
        location->value_ = max(location->value_, value); // gives preference to occupied over free
        location->next_ = elements_[hash];
        elements_[hash] = location;
      } else {
        elements_[hash]->value_ = max(elements_[hash]->value_, value);
      }
      __syncthreads();
      if (threadIdx.x == 0)
        release_semaphore(&sem);//pair_locks_[hash].unlock();
      __syncthreads();
    } else {
      printf(
        "Map not initialized to device. Cannot insert key: %i %i %i %i.\n",
        key[0], key[1], key[2], type_);
    }
  }
}
#endif