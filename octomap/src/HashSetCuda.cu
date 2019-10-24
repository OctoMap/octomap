#ifdef __CUDA_SUPPORT__
#include <octomap/HashSetCuda.cuh>

namespace octomap {
  __device__ void lock(unsigned int *lock){
    while (atomicCAS(lock, 0, 1) != 0);
  }

  __device__ void unlock(unsigned int *lock){
    atomicExch(lock, 0);
  }

  #define WARP_SIZE 32
  __device__ void HashSetCuda::insertDevice(const KeyRayCuda& ray, const int& start, const int& tid, const int& value)
  {
    for (int r = 0; r < ray.size(); ++r) {
      const auto& key = ray[r];
      size_t hash = OcTreeKey::KeyHash{}(key) % count_;
      auto location = &pool_[start + r];
      location->key_ = key;
      location->value_ = value;
      location->hash_ = hash;
      lock(&hash_locks_[hash]);
      /*if (elements_[hash] && key == elements_[hash]->key_) {
        // gives preference to occupied over free
        elements_[hash]->value_ = max(elements_[hash]->value_, value);
        unlock(&hash_locks_[hash]);
        continue;
      }*/
      // two different keys might have same hash
      location->next_ = elements_[hash];
      elements_[hash] = location;
      unlock(&hash_locks_[hash]);
    }
  }
}
#endif