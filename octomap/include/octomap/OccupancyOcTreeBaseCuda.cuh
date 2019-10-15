#ifndef OCCUPANCY_OCTREE_BASE_CUH
#define OCCUPANCY_OCTREE_BASE_CUH
#include <cuda.h>
#include <cuda_runtime.h>
#include <nppi.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/KeyContainerCuda.cuh>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <algorithm>

using namespace octomap;

#ifdef __CUDA_SUPPORT__
template <class NODE>
void computeUpdateCuda(
  const octomap::Pointcloud& scan, const octomap::point3d& origin, octomap::KeySet& free_cells, 
  octomap::KeySet& occupied_cells, double maxrange, octomap::OccupancyOcTreeBase<NODE>* tree_base);
#endif

#endif