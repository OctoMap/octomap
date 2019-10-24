#ifndef OCCUPANCY_OCTREE_BASE_CUH
#define OCCUPANCY_OCTREE_BASE_CUH
#include <cuda.h>
#include <cuda_runtime.h>
#include <nppi.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/KeyArrayCuda.cuh>
#include <octomap/HashSetCuda.cuh>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <algorithm>

using namespace octomap;

#ifdef __CUDA_SUPPORT__

template <class NODE>
class OctomapUpdaterCuda 
{
public:
  OctomapUpdaterCuda(
    octomap::OccupancyOcTreeBase<NODE>* tree_base, 
    const double& max_range,
    const size_t& scan_size);

  ~OctomapUpdaterCuda();

  void initialize();

  void computeUpdate(
    const octomap::Pointcloud& scan, 
    const octomap::point3d& origin,
    const double& max_range);

private:
  // tree base on host
  octomap::OccupancyOcTreeBase<NODE>* tree_base_;
  // copy of tree base on device
  octomap::OccupancyOcTreeBase<NODE>* tree_base_device_;
  // preallocation of ray containers for ray-casting
  KeyRayCuda* rays_device_;
  int n_rays_;
  // Hashset for occupied/free cells on device
  ArrayCuda<KeyValue>* key_value_arr_device_;
  KeyPtrArrayCuda* free_hash_arr_device_;
  KeyPtrArrayCuda* occupied_hash_arr_device_;
  ArrayCuda<KeyHash>* free_hashes_device_;
  ArrayCuda<KeyHash>* occupied_hashes_device_;
  ArrayCuda<KeyHash>* free_hashes_host_;
  ArrayCuda<KeyHash>* occupied_hashes_host_;
  bool use_bbx_limit_;
  double res_;
  double res_half_;

  // make an array of points from the point cloud for device usage
  octomap::point3d* scan_device;
  size_t scan_size_;

  // Kernel settings
  int n_total_threads_;
  int n_blocks_;
  int n_threads_per_block_;

  // map size settings
  int ray_size_;
  int max_arr_elements_ = 5e6;
  int max_hash_elements_ = 2e5;
  int max_range_ = -1;
};
#endif
#endif