#include <stddef.h>
#include <octomap/OctomapUpdaterCuda.cuh>
#include <octomap/AssertionCuda.cuh>
#include <boost/chrono.hpp>
#ifdef __CUDA_SUPPORT__

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

template <class NODE,class I>
CUDA_CALLABLE bool computeRayKeysCuda(
  const point3d& origin, 
  const point3d& end, 
  KeyRayCuda& ray,
  const double& resolution,
  const double& resolution_half,
  OcTreeBaseImpl<NODE,I>* tree_base)
{
  ray.reset();

  OcTreeKey key_origin, key_end;
  if ( !tree_base->coordToKeyChecked(origin, key_origin) ||
        !tree_base->coordToKeyChecked(end, key_end) ) {
    return false;
  }

  if (key_origin == key_end)
    return true; // same tree cell, we're done.

  ray.addKey(key_origin);

  // Initialization phase -------------------------------------------------------

  point3d direction = (end - origin);
  float length = (float) direction.norm();
  direction /= length; // normalize vector

  int    step[3];
  double tMax[3];
  double tDelta[3];

  OcTreeKey current_key = key_origin;

  for(unsigned int i=0; i < 3; ++i) {
    // compute step direction
    if (direction(i) > 0.0) step[i] =  1;
    else if (direction(i) < 0.0)   step[i] = -1;
    else step[i] = 0;

    // compute tMax, tDelta
    if (step[i] != 0) {
      // corner point of voxel (in direction of ray)
      double voxelBorder = tree_base->keyToCoord(current_key[i]);
      voxelBorder += (float) (step[i] * resolution_half);

      tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
      tDelta[i] = resolution / fabs( direction(i) );
    }
    else {
      tMax[i] = NPP_MAXABS_64F;
      tDelta[i] = NPP_MAXABS_64F;
    }
  }

  // Incremental phase  ---------------------------------------------------------
  bool done = false;
  while (!done) {

    unsigned int dim;

    // find minimum tMax:
    if (tMax[0] < tMax[1]){
      if (tMax[0] < tMax[2]) dim = 0;
      else                   dim = 2;
    }
    else {
      if (tMax[1] < tMax[2]) dim = 1;
      else                   dim = 2;
    }

    // advance in direction "dim"
    current_key[dim] += step[dim];
    tMax[dim] += tDelta[dim];

    assert (current_key[dim] < 2*tree_base->tree_max_val);

    // reached endpoint, key equv?
    if (current_key == key_end) {
      done = true;
      break;
    }
    else {

      // reached endpoint world coords?
      // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
      double dist_from_origin = fmin(fmin(tMax[0], tMax[1]), tMax[2]);
      // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
      // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
      if (dist_from_origin > length) {
        done = true;
        break;
      }

      else {  // continue to add freespace cells
        ray.addKey(current_key);
      }
    }
    assert ( ray.size() < ray.sizeMax() - 1);

  } // end while

  return true;
}

template <class NODE>
__global__ void computeUpdateNoBBxRanged(
  octomap::point3d origin,
  octomap::point3d* points,
  KeyRayCuda* rays,
  UnsignedArrayCuda* free_hash_arr_device,
  UnsignedArrayCuda* occupied_hash_arr_device,
  ArrayCuda<KeyHash>* free_hashes_device,
  ArrayCuda<KeyHash>* occupied_hashes_device,
  size_t size,
  size_t max_hash_elements,
  double max_range,
  double resolution,
  double resolution_half,
  OccupancyOcTreeBase<NODE>* tree_base)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = tid; i < size; i += stride) 
  {
    auto& p = points[i];
    auto& ray = rays[tid];
    if ((p - origin).norm() <= max_range) { // is not max_range meas.
      // free cells
      if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
        for (auto it = ray.begin(); it != ray.end(); ++it) {
          size_t hash = OcTreeKey::KeyHash{}(*it) % max_hash_elements;
          if (free_hash_arr_device->addKeyAtomicAt((unsigned)1, hash) == 0)
            free_hashes_device->addKeyAtomic(KeyHash(*it, hash));
        }
      }
      
      // occupied endpoint
      OcTreeKey key;
      if (tree_base->coordToKeyChecked(p, key)) {
        size_t hash = OcTreeKey::KeyHash{}(key) % max_hash_elements;
        if (occupied_hash_arr_device->addKeyAtomicAt((unsigned)1, hash) == 0) {}
          occupied_hashes_device->addKeyAtomic(KeyHash(key, hash));
      }
    } else { // user set a max_range and length is above
      point3d direction = (p - origin).normalized ();
      point3d new_end = origin + direction * (float) max_range;
      // free cells
      if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
        for (auto it = ray.begin(); it != ray.end(); ++it) {
          size_t hash = OcTreeKey::KeyHash{}(*it) % max_hash_elements;
          if (free_hash_arr_device->addKeyAtomicAt((unsigned)1, hash) == 0)
            free_hashes_device->addKeyAtomic(KeyHash(*it, hash));
        }
      }
    } // end if max_range
  }
}


template <class NODE>
__global__ void computeUpdateNoBBxNoRange(
  octomap::point3d origin,
  octomap::point3d* points,
  KeyRayCuda* rays,
  UnsignedArrayCuda* free_hash_arr_device,
  UnsignedArrayCuda* occupied_hash_arr_device,
  ArrayCuda<KeyHash>* free_hashes_device,
  ArrayCuda<KeyHash>* occupied_hashes_device,
  size_t size,
  size_t max_hash_elements,
  double resolution,
  double resolution_half,
  OccupancyOcTreeBase<NODE>* tree_base)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = tid; i < size; i += stride) 
  {
    auto& p = points[i];
    auto& ray = rays[tid];
    // free cells
    if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
      for (auto it = ray.begin(); it != ray.end(); ++it) {
        size_t hash = OcTreeKey::KeyHash{}(*it) % max_hash_elements;
        if (free_hash_arr_device->addKeyAtomicAt((unsigned)1, hash) == 0)
          free_hashes_device->addKeyAtomic(KeyHash(*it, hash));
      }
    }
    
    // occupied endpoint
    OcTreeKey key;
    if (tree_base->coordToKeyChecked(p, key)) {
      size_t hash = OcTreeKey::KeyHash{}(key) % max_hash_elements;
      if (occupied_hash_arr_device->addKeyAtomicAt((unsigned)1, hash) == 0) {}
        occupied_hashes_device->addKeyAtomic(KeyHash(key, hash));
    }
  }
}

__global__ void reset(
  UnsignedArrayCuda* arr_device,
  ArrayCuda<KeyHash>* hashes_device,
  size_t size
)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = tid; i < size; i += stride) 
  {
    arr_device->addKey(0, hashes_device->get(i).hash_);
  }
}

void verifyTable(const HashSetCuda &table ) {
  int free_cells = 0, occ = 0, count = 0;
  for (size_t i = 0; i < table.count(); i++) {
    auto current = table.element(i);
    while (current != NULL) {
      if (current->value_ == 0) 
        ++free_cells;
      else if (current->value_ == 1) 
        ++occ;
      ++count;
      current = current->next_;
    }
  }
  printf( "%d elements found in hash table with %d free and %d occupied.\n", count, free_cells, occ);
}

template <class NODE>
OctomapUpdaterCuda<NODE>::OctomapUpdaterCuda(
  octomap::OccupancyOcTreeBase<NODE>* tree_base, 
  const double& max_range,
  const size_t& scan_size) :
  tree_base_(tree_base),
  max_range_(max_range)
{
  // must be known priorly for preallocation of memory
  scan_size_ = scan_size;
  use_bbx_limit_ = tree_base_->bbxSet();
  res_ = tree_base_->getResolution();
  res_half_ = res_ * 0.5;
  if (max_range < 10.0) {
    ray_size_ = 10.0 / res_;
  } else if (max_range < 20.0) {
    ray_size_ = 20.0 / res_;
  }
  std::cout << "ray_size_:" << ray_size_ << std::endl;
  KeyRayConfig::setMaxSize(ray_size_);

  std::cout << "Setting up cuda updater for the scan size:" << scan_size_ << std::endl;
  std::cout << "scan_size_:" << scan_size_ << std::endl;
  int warp_size = 32;
  n_total_threads_ = std::ceil(scan_size_ / (float) warp_size) * warp_size;
  n_threads_per_block_ = min(n_total_threads_, 256);
  n_blocks_ = n_total_threads_ / n_threads_per_block_;
  std::cout << "Total number of threads: " << n_total_threads_ << std::endl;
  std::cout << "Total number of blocks: " << n_blocks_ << std::endl;
  std::cout << "Threads per block: " << n_threads_per_block_ << std::endl;
  n_rays_ = n_total_threads_;
  std::cout << 
    "Memory used by rays:" << 
    n_rays_ * 
    (ray_size_ * sizeof(OcTreeKey) + sizeof(KeyArrayCuda)) 
    << " bytes. " << std::endl;

}

template <class NODE>
OctomapUpdaterCuda<NODE>::~OctomapUpdaterCuda() {
  for (int i = 0; i < n_rays_; ++i) {
    rays_device_[i].freeDevice();
  }
  cudaCheckErrors(cudaFree(rays_device_));
  cudaCheckErrors(cudaFree(tree_base_device_));
  cudaCheckErrors(cudaFree(scan_device));
}

template <class NODE>
void OctomapUpdaterCuda<NODE>::initialize() {
  // make a copy of tree for device usage - need a better way ? or idk
  cudaCheckErrors(cudaMallocManaged(&tree_base_device_, sizeof(octomap::OccupancyOcTreeBase<NODE>)));
  cudaCheckErrors(cudaMemcpy(tree_base_device_, tree_base_, sizeof(octomap::OccupancyOcTreeBase<NODE>), cudaMemcpyHostToDevice));

  // rays
  auto rays_host = new KeyRayCuda[n_rays_];
  for (int i = 0; i < n_rays_; ++i) {
    rays_host[i].allocateDevice();
  }
  cudaCheckErrors(cudaMallocManaged(&rays_device_, n_rays_ * sizeof(KeyRayCuda)));
  cudaCheckErrors(cudaMemcpy(rays_device_, &rays_host[0], n_rays_ * sizeof(KeyRayCuda), cudaMemcpyHostToDevice));
  delete[] rays_host;

  UnsignedArrayCuda free_hash_arr_host(max_hash_elements_);
  UnsignedArrayCuda occupied_hash_arr_host(max_hash_elements_);
  free_hash_arr_host.allocateDevice();
  occupied_hash_arr_host.allocateDevice();
  
  cudaCheckErrors(cudaMallocManaged(&free_hash_arr_device_, sizeof(UnsignedArrayCuda)));
  cudaCheckErrors(cudaMemcpy(free_hash_arr_device_, &free_hash_arr_host, sizeof(UnsignedArrayCuda), cudaMemcpyHostToDevice));
  cudaCheckErrors(cudaMallocManaged(&occupied_hash_arr_device_, sizeof(UnsignedArrayCuda)));
  cudaCheckErrors(cudaMemcpy(occupied_hash_arr_device_, &occupied_hash_arr_host, sizeof(UnsignedArrayCuda), cudaMemcpyHostToDevice));

  ArrayCuda<KeyHash> free_hashes(max_hash_elements_);
  ArrayCuda<KeyHash> occupied_hashes(max_hash_elements_);
  free_hashes.allocateDevice();
  occupied_hashes.allocateDevice();
  cudaCheckErrors(cudaMallocManaged(&free_hashes_device_, sizeof(ArrayCuda<KeyHash>)));
  cudaCheckErrors(cudaMemcpy(free_hashes_device_, &free_hashes, sizeof(ArrayCuda<KeyHash>), cudaMemcpyHostToDevice));
  cudaCheckErrors(cudaMallocManaged(&occupied_hashes_device_, sizeof(ArrayCuda<KeyHash>)));
  cudaCheckErrors(cudaMemcpy(occupied_hashes_device_, &occupied_hashes, sizeof(ArrayCuda<KeyHash>), cudaMemcpyHostToDevice));

  // initialized scan points
  cudaCheckErrors(cudaMallocManaged(&scan_device, scan_size_ * sizeof(octomap::point3d)));
}

template <class NODE>
void OctomapUpdaterCuda<NODE>::computeUpdate(
  const octomap::Pointcloud& scan, 
  const octomap::point3d& origin,
  const double& max_range)
{
  auto t_start = boost::chrono::high_resolution_clock::now();
  // total number of rays or points
  assert(scan_size_ == scan.size());
  cudaCheckErrors(cudaMemcpy(scan_device, &scan[0], scan_size_ * sizeof(octomap::point3d), cudaMemcpyHostToDevice));

  if (!use_bbx_limit_) { // no BBX specified
    if ((max_range < 0.0)) { // is not max_range meas.)
      computeUpdateNoBBxNoRange<NODE><<<n_blocks_, n_threads_per_block_>>>( 
        origin, 
        scan_device,
        rays_device_,
        free_hash_arr_device_,
        occupied_hash_arr_device_,
        free_hashes_device_,
        occupied_hashes_device_,
        scan_size_,
        max_hash_elements_,
        res_,
        res_half_,
        tree_base_device_); 
    } else {
      computeUpdateNoBBxRanged<NODE><<<n_blocks_, n_threads_per_block_>>>( 
        origin, 
        scan_device,
        rays_device_,
        free_hash_arr_device_,
        occupied_hash_arr_device_,
        free_hashes_device_,
        occupied_hashes_device_,
        scan_size_,
        max_hash_elements_,
        max_range,
        res_,
        res_half_,
        tree_base_device_);
    }
  }
  cudaCheckErrors(cudaDeviceSynchronize());
  for (int i = 0; i < occupied_hashes_device_->size(); ++i) {
    tree_base_->updateNode(occupied_hashes_device_->get(i).key_, false, false);
  }
  for (int i = 0; i < occupied_hashes_device_->size(); ++i) {
    tree_base_->updateNode(occupied_hashes_device_->get(i).key_, true, false);
  }
  reset<<<n_blocks_, n_threads_per_block_>>>(free_hash_arr_device_, free_hashes_device_, free_hashes_device_->size());
  reset<<<n_blocks_, n_threads_per_block_>>>(occupied_hash_arr_device_, occupied_hashes_device_, occupied_hashes_device_->size());
  free_hashes_device_->reset();
  occupied_hashes_device_->reset();
}
template class OctomapUpdaterCuda<OcTreeNode>;
template class OctomapUpdaterCuda<OcTreeNodeStamped>;
template class OctomapUpdaterCuda<ColorOcTreeNode>;
#endif