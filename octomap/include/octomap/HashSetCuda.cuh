#ifndef HASH_MAP_CUDA_CUH
#define HASH_MAP_CUDA_CUH
#ifdef __CUDA_SUPPORT__
#include <cuda.h>
#include <cuda_runtime.h>
#include <octomap/AssertionCuda.cuh>
#include <octomap/OcTreeKey.h>

/**
 * Based on CUDA by Example: 
 * An Introduction to General-Purpose GPU Programming
 * Book by Edward Kandrot and Jason Sanders
 */

namespace octomap {
  struct LockCuda {
    int *mutex;
    __host__ LockCuda(void) {
      initializeDevice();
    }
    __host__ ~LockCuda(void) {
      //cudaCheckErrors(cudaFree(mutex));
    }
    __device__ void lock(void);
    __device__ void unlock(void);
    __host__ void initializeDevice() {
      int state = 0;
      cudaCheckErrors(cudaMallocManaged(&mutex, sizeof(int)));
      cudaCheckErrors(cudaMemcpy(mutex, &state, sizeof(int), cudaMemcpyHostToDevice));
    }
  };

  #define UNKNOWN -1
  #define FREE 0
  #define OCCUPIED 1

  struct SetElement {
    OcTreeKey key_;
    SetElement *next_;
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
    LockCuda* pair_locks_;
    
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
        //auto locks = new LockCuda[n_hash_elements];
        //cudaCheckErrors(cudaMallocManaged(&pair_locks_, n_hash_elements * sizeof(LockCuda)));
        //cudaCheckErrors(cudaMemcpy(pair_locks_, &locks[0], n_hash_elements * sizeof(LockCuda), cudaMemcpyHostToDevice));
        //delete[] locks;
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
          location->next_ = elements_[hash];
          elements_[hash] = location;
        }
      } else {
        printf(
          "Map not initialized to host. Cannot insert key: %i %i %i.\n",
          key[0], key[1], key[2]);
      }
    }

    __device__ void insertDevice(const OcTreeKey& key, const int& value);
    __host__ void reset() {
        // reset all map values
        while (first_free_ > 0) {
          pool_[--first_free_].value_ = UNKNOWN;
        }

        //cudaCheckErrors(cudaMallocManaged(&elements_, count_ * sizeof(SetElement*)));
        cudaCheckErrors(cudaMemset(elements_, 0, count_ * sizeof(SetElement*)));
        //cudaCheckErrors(cudaMallocManaged(&pool_, n_elements_ * sizeof(SetElement)));
    }

    CUDA_CALLABLE size_t count() const { return count_; }
    CUDA_CALLABLE const SetElement* element(const int& idx) const { return elements_[idx]; }
    CUDA_CALLABLE void resetElement(const int& idx) { elements_[idx] = nullptr; }

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