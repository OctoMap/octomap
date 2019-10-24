#ifndef HASH_MAP_CUDA_CUH
#define HASH_MAP_CUDA_CUH
#ifdef __CUDA_SUPPORT__
#include <cuda.h>
#include <cuda_runtime.h>
#include <octomap/AssertionCuda.cuh>
#include <octomap/OcTreeKey.h>
#include <octomap/KeyArrayCuda.cuh>

/**
 * Based on CUDA by Example: 
 * An Introduction to General-Purpose GPU Programming
 * Book by Edward Kandrot and Jason Sanders
 */

namespace octomap {

  struct SetElement {
    OcTreeKey key_;
    SetElement *next_;
    size_t hash_;
    int value_ = {UNKNOWN};
  };

  class HashSetCuda {
  private:
    enum Type {
      HOST,
      DEVICE,
      NONE
    };
    int type_ = {NONE};
    size_t count_;
    size_t n_elements_;
    SetElement** elements_;
    SetElement* pool_;
    int first_free_;
  public:
    unsigned int* hash_locks_;
    
    __host__ HashSetCuda () : type_(NONE) {
    }
    __host__ ~HashSetCuda () 
    {
    }

    __host__ void initializeHost(const int& n_hash_elements, const int& n_elements) 
    {
      if (type_ == NONE || type_ == HOST) {
        count_ = n_hash_elements;
        n_elements_ = n_elements;
        elements_ = (SetElement**)calloc(n_hash_elements, sizeof(SetElement*));
        pool_ = (SetElement*)malloc(n_elements * sizeof(SetElement));
        first_free_ = 0;
        type_ = HOST;
      } else {
        printf("Map already initialized to device.");
      }
    }

    __host__ void initializeDevice(const int& n_hash_elements, const int& n_elements) 
    {
      if (type_ == NONE || type_ == DEVICE) {
        count_ = n_hash_elements;
        n_elements_ = n_elements;
        cudaCheckErrors(cudaMallocManaged(&elements_, n_hash_elements * sizeof(SetElement*)));
        cudaCheckErrors(cudaMemset(elements_, 0, n_hash_elements * sizeof(SetElement*)));
        cudaCheckErrors(cudaMallocManaged(&pool_, n_elements * sizeof(SetElement)));
        first_free_ = 0;
        cudaCheckErrors(cudaMallocManaged(&hash_locks_, n_hash_elements * sizeof(unsigned int)));
        cudaCheckErrors(cudaMemset(hash_locks_, 0, n_hash_elements * sizeof(unsigned int)));
        type_ = DEVICE;
      } else {
        printf("Map already initialized to host.");
      }
    }

    __host__ void freeMemory() {
      if (type_ == HOST) {
        free(elements_);
        free(pool_);
      } else if (type_ == DEVICE) {
        cudaCheckErrors(cudaFree(elements_));
        cudaCheckErrors(cudaFree(pool_));
      }
    }

    __host__ void insertHost(const OcTreeKey& key, const int& value)
    {
      if (type_ == HOST) {
        size_t hash = OcTreeKey::KeyHash{}(key) % count_;
        // two different keys might have same hash
        if (!elements_[hash] || elements_[hash]->key_ != key) { 
          auto location = &pool_[first_free_++];
          location->key_ = key;
          location->value_ = value;
          location->hash_ = hash;
          location->next_ = elements_[hash];
          elements_[hash] = location;
        }
      } else {
        printf(
          "Map not initialized to host. Cannot insert key: %i %i %i.\n",
          key[0], key[1], key[2]);
      }
    }

    __device__ void insertDevice(const KeyRayCuda& ray, const int& start, const int& tid, const int& value);

    CUDA_CALLABLE size_t count() const { return count_; }
    CUDA_CALLABLE const SetElement* element(const int& idx) const { return elements_[idx]; }
    CUDA_CALLABLE void resetElement(const int& idx) { elements_[idx] = nullptr; }

    CUDA_CALLABLE SetElement** elements() { return elements_; }
    CUDA_CALLABLE SetElement* pool() { return pool_; }
    CUDA_CALLABLE int& firstFree() { return first_free_; }

    __host__ void copyFromDevice(const HashSetCuda& other) {
      if (type_ == HOST) {
        assert (n_elements_ == other.n_elements_);
        assert (count_ == other.count_);
        cudaCheckErrors(cudaMemcpy(elements_, other.elements_, count_ * sizeof(SetElement*), cudaMemcpyDeviceToHost));
        cudaCheckErrors(cudaMemcpy(pool_, other.pool_, n_elements_ * sizeof(SetElement), cudaMemcpyDeviceToHost));
      } else {
        printf("Uninitialized map cannot copy data from device.");
      }

      // Fix pointer offsets
      for (size_t i = 0; i < count_; i++) {
        if (elements_[i] != NULL) {
          elements_[i] =
            (SetElement*)
              ((size_t)elements_[i] - (size_t)other.pool_ + (size_t)pool_);
        }
      }
      
      // Fix pointer offsets
      for (size_t i = 0; i < n_elements_; i++) {
        if (pool_[i].next_ != NULL) {
          pool_[i].next_ =
            (SetElement*)(
              (size_t)pool_[i].next_ - (size_t)other.pool_ + (size_t)pool_);
        }
      }
    }

    __host__ void copyElementsFromDevice(const HashSetCuda& other) {
      if (type_ == HOST) {
        assert (count_ == other.count_);
        cudaCheckErrors(cudaMemcpy(elements_, other.elements_, count_ * sizeof(SetElement*), cudaMemcpyDeviceToHost));
      } else {
        printf("Uninitialized map cannot copy data from device.");
      }

      // Fix pointer offsets
      for (size_t i = 0; i < count_; i++) {
        if (elements_[i] != NULL) {
          elements_[i] =
            (SetElement*)
              ((size_t)elements_[i] - (size_t)other.pool_ + (size_t)pool_);
        }
      }
    }
  };
}


#endif
#endif