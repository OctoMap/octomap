#ifndef OCCUPANCY_OCTREE_BASE_CUH
#define OCCUPANCY_OCTREE_BASE_CUH
#include <cuda.h>
#include <cuda_runtime.h>
#include <nppi.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/KeyArrayCuda.cuh>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <algorithm>

using namespace octomap;

#ifdef __CUDA_SUPPORT__
#define N_BLOCKS 32
#define N_THREADS 256
#define MAX_HASHED_ELEMENTS 1e5
#define MAX_ELEMENTS 2e5

template <class NODE>
class OctomapUpdaterCuda 
{
public:
  OctomapUpdaterCuda(octomap::OccupancyOcTreeBase<NODE>* tree_base) :
    tree_base_(tree_base)
  {
    use_bbx_limit_ = tree_base_->bbxSet();
    res_ = tree_base_->getResolution();
    res_half_ = res_ * 0.5;
  }
  ~OctomapUpdaterCuda() {
    key_map_device_->freeMemory();
    for (int i = 0; i < n_rays_; ++i) {
      rays_device_[i].freeDevice();
    }
    cudaCheckErrors(cudaFree(rays_device_));
    cudaCheckErrors(cudaFree(key_map_device_));
    cudaCheckErrors(cudaFree(tree_base_device_));
  }

  void initialize();

  void computeUpdate(
    const octomap::Pointcloud& scan, 
    const octomap::point3d& origin,
    const double& maxrange,
    const bool& lazy_eval);

private:
  // tree base on host
  octomap::OccupancyOcTreeBase<NODE>* tree_base_;
  // copy of tree base on device
  octomap::OccupancyOcTreeBase<NODE>* tree_base_device_;
  // preallocation of ray containers for ray-casting
  KeyRayCuda* rays_device_;
  int n_rays_;
  // Hashset for occupied/free cells on device
  HashSetCuda* key_map_device_;
  // Hashset for occupied/free cells on host
  HashSetCuda key_map_host_;
  // Hashset for free cells on host
  HashSetCuda free_map_host_;
  bool use_bbx_limit_;
  double res_;
  double res_half_;
};
#endif
#endif