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
  ArrayCuda<KeyValue>* key_value_arr,
  size_t size,
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
    int start = 0;
    int last_ray_size = 0;
    if ((p - origin).norm() <= max_range) { // is not max_range meas.
      // free cells
      if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
        start = atomicAdd(key_value_arr->last(), ray.size()+1); //for occupied
        last_ray_size = ray.size();
        for (int r = 0; r < last_ray_size; ++r) {
          key_value_arr->addKey(KeyValue(ray[r], 0), r + start);
        }
      } else {
        start = atomicAdd(key_value_arr->last(), 1);
      }
      
      // occupied endpoint
      OcTreeKey key;
      if (tree_base->coordToKeyChecked(p, key)) {
        key_value_arr->addKey(KeyValue(key, 1), last_ray_size + start);
      }
    } else { // user set a max_range and length is above
      point3d direction = (p - origin).normalized ();
      point3d new_end = origin + direction * (float) max_range;
      if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
        start = atomicAdd(key_value_arr->last(), ray.size()); //for occupied
        last_ray_size = ray.size();
        for (int r = 0; r < last_ray_size; ++r) {
          key_value_arr->addKey(KeyValue(ray[r], 0), r + start);
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
  ArrayCuda<KeyValue>* key_value_arr,
  size_t size,
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
    int start = 0;
    int last_ray_size = 0;
    // free cells
    if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
      start = atomicAdd(key_value_arr->last(), ray.size()+1); //for occupied
      last_ray_size = ray.size();
      for (int r = 0; r < last_ray_size; ++r) {
        key_value_arr->addKey(KeyValue(ray[r], 0), r + start);
      }
    } else {
      start = atomicAdd(key_value_arr->last(), 1);
    }
    
    // occupied endpoint
    OcTreeKey key;
    if (tree_base->coordToKeyChecked(p, key)) {
      key_value_arr->addKey(KeyValue(key, 1), last_ray_size + start);
    }
  }
}

__global__ void clean(
  ArrayCuda<KeyValue>* key_value_arr_device,
  KeyPtrArrayCuda* free_hash_arr_device,
  KeyPtrArrayCuda* occupied_hash_arr_device,
  ArrayCuda<KeyHash>* free_hashes_device,
  ArrayCuda<KeyHash>* occupied_hashes_device,
  size_t size,
  int max_hash_elements
)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = tid; i < size; i += stride) 
  {
    auto& key_value = key_value_arr_device->get(i);
    size_t hash = OcTreeKey::KeyHash{}(key_value.key_) % max_hash_elements;
    if (key_value.value_ == 0) {
      if (((OcTreeKey*)free_hash_arr_device->addKeyAtomicAt(&key_value.key_, hash)) == nullptr)
        free_hashes_device->addKeyAtomic(KeyHash(key_value.key_, hash));
    } else if (key_value.value_ == 1) {
      if (((OcTreeKey*)occupied_hash_arr_device->addKeyAtomicAt(&key_value.key_, hash)) == nullptr)
        occupied_hashes_device->addKeyAtomic(KeyHash(key_value.key_, hash));
    }
  }
}

__global__ void reset(
  KeyPtrArrayCuda* arr_device,
  ArrayCuda<KeyHash>* hashes_device,
  size_t size
)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = tid; i < size; i += stride) 
  {
    arr_device->addKey(nullptr, hashes_device->get(i).hash_);
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
  ray_size_ = max_range < 0 ? 20.0 / res_ : max_range / res_; // defaults to 10m
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
  key_value_arr_device_->freeDevice();
  for (int i = 0; i < n_rays_; ++i) {
    rays_device_[i].freeDevice();
  }
  cudaCheckErrors(cudaFree(rays_device_));
  cudaCheckErrors(cudaFree(key_value_arr_device_));
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

  ArrayCuda<KeyValue> key_value_arr_host(max_arr_elements_);
  key_value_arr_host.allocateDevice();
  cudaCheckErrors(cudaMallocManaged(&key_value_arr_device_, sizeof(ArrayCuda<KeyValue>)));
  cudaCheckErrors(cudaMemcpy(key_value_arr_device_, &key_value_arr_host, sizeof(ArrayCuda<KeyValue>), cudaMemcpyHostToDevice));

  KeyPtrArrayCuda free_hash_arr_host(max_hash_elements_);
  KeyPtrArrayCuda occupied_hash_arr_host(max_hash_elements_);
  free_hash_arr_host.allocateDevice();
  occupied_hash_arr_host.allocateDevice();
  
  cudaCheckErrors(cudaMallocManaged(&free_hash_arr_device_, sizeof(KeyPtrArrayCuda)));
  cudaCheckErrors(cudaMemcpy(free_hash_arr_device_, &free_hash_arr_host, sizeof(KeyPtrArrayCuda), cudaMemcpyHostToDevice));
  cudaCheckErrors(cudaMallocManaged(&occupied_hash_arr_device_, sizeof(KeyPtrArrayCuda)));
  cudaCheckErrors(cudaMemcpy(occupied_hash_arr_device_, &occupied_hash_arr_host, sizeof(KeyPtrArrayCuda), cudaMemcpyHostToDevice));

  ArrayCuda<KeyHash> free_hashes(max_hash_elements_);
  ArrayCuda<KeyHash> occupied_hashes(max_hash_elements_);
  free_hashes.allocateDevice();
  occupied_hashes.allocateDevice();
  cudaCheckErrors(cudaMallocManaged(&free_hashes_device_, sizeof(ArrayCuda<KeyHash>)));
  cudaCheckErrors(cudaMemcpy(free_hashes_device_, &free_hashes, sizeof(ArrayCuda<KeyHash>), cudaMemcpyHostToDevice));
  cudaCheckErrors(cudaMallocManaged(&occupied_hashes_device_, sizeof(ArrayCuda<KeyHash>)));
  cudaCheckErrors(cudaMemcpy(occupied_hashes_device_, &occupied_hashes, sizeof(ArrayCuda<KeyHash>), cudaMemcpyHostToDevice));

  free_hashes_host_ = new ArrayCuda<KeyHash>(max_hash_elements_);
  occupied_hashes_host_ = new ArrayCuda<KeyHash>(max_hash_elements_);

  free_hashes_host_->allocateHost();
  occupied_hashes_host_->allocateHost();

  // initialized scan points
  cudaCheckErrors(cudaMallocManaged(&scan_device, scan_size_ * sizeof(octomap::point3d)));
}

template <class NODE>
void OctomapUpdaterCuda<NODE>::computeUpdate(
  const octomap::Pointcloud& scan, 
  const octomap::point3d& origin,
  const double& max_range)
{
  // total number of rays or points
  assert(scan_size_ == scan.size());
  cudaCheckErrors(cudaMemcpy(scan_device, &scan[0], scan_size_ * sizeof(octomap::point3d), cudaMemcpyHostToDevice));

  if (!use_bbx_limit_) { // no BBX specified
    if ((max_range < 0.0)) { // is not max_range meas.)
      computeUpdateNoBBxNoRange<NODE><<<n_blocks_, n_threads_per_block_>>>( 
        origin, 
        scan_device,
        rays_device_,
        key_value_arr_device_,
        scan_size_,
        res_,
        res_half_,
        tree_base_device_); 
    } else {
      computeUpdateNoBBxRanged<NODE><<<n_blocks_, n_threads_per_block_>>>( 
        origin, 
        scan_device,
        rays_device_,
        key_value_arr_device_,
        scan_size_,
        max_range,
        res_,
        res_half_,
        tree_base_device_);
    }
  }
  cudaCheckErrors(cudaDeviceSynchronize());
  clean<<<n_blocks_, n_threads_per_block_>>>(
    key_value_arr_device_, 
    free_hash_arr_device_,
    occupied_hash_arr_device_,
    free_hashes_device_,
    occupied_hashes_device_,
    key_value_arr_device_->size(),
    max_hash_elements_);
  cudaCheckErrors(cudaDeviceSynchronize());
  free_hashes_host_->copyFromDevice(*free_hashes_device_);
  occupied_hashes_host_->copyFromDevice(*occupied_hashes_device_);
  for (int i = 0; i < free_hashes_host_->size(); ++i) {
    tree_base_->updateNode(free_hashes_host_->get(i).key_, false, false);
  }
  for (int i = 0; i < occupied_hashes_host_->size(); ++i) {
    tree_base_->updateNode(occupied_hashes_host_->get(i).key_, true, false);
  }
  reset<<<n_blocks_, n_threads_per_block_>>>(free_hash_arr_device_, free_hashes_device_, free_hashes_device_->size());
  reset<<<n_blocks_, n_threads_per_block_>>>(occupied_hash_arr_device_, occupied_hashes_device_, occupied_hashes_device_->size());
  free_hashes_device_->reset();
  occupied_hashes_device_->reset();
  key_value_arr_device_->reset();
}
template class OctomapUpdaterCuda<OcTreeNode>;
template class OctomapUpdaterCuda<OcTreeNodeStamped>;
template class OctomapUpdaterCuda<ColorOcTreeNode>;
#endif