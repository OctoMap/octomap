#include <octomap/OctomapUpdaterCuda.cuh>
#include <octomap/AssertionCuda.cuh>
#include <boost/chrono.hpp>
#ifdef __CUDA_SUPPORT__

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

#define MAX_RAY_SIZE 128

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
__global__ void computeUpdateKernel(
  octomap::point3d origin,
  octomap::point3d* points,
  KeyRayCuda* rays,
  HashSetCuda* key_map,
  size_t size,
  double maxrange,
  bool use_bbx_limit,
  double resolution,
  double resolution_half,
  OccupancyOcTreeBase<NODE>* tree_base)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = idx; i < size; i += stride) 
  //for (int i =0; i < 1; i ++) 
  {
    auto& p = points[i];
    auto& ray = rays[idx];
    if (!use_bbx_limit) { // no BBX specified
      if ((maxrange < 0.0) || ((p - origin).norm() <= maxrange) ) { // is not maxrange meas.
        // free cells
        if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
          for (auto rp = ray.begin(); rp != ray.end(); ++rp) {
            key_map->insertDevice(*rp, FREE);
          }
        }
        // occupied endpoint
        OcTreeKey key;
        if (tree_base->coordToKeyChecked(p, key)) {
          key_map->insertDevice(key, OCCUPIED);
        }
      } else { // user set a maxrange and length is above
        point3d direction = (p - origin).normalized ();
        point3d new_end = origin + direction * (float) maxrange;
        if (computeRayKeysCuda(origin, new_end, ray, resolution, resolution_half, tree_base)) { // *ray
          for (auto rp = ray.begin(); rp != ray.end(); ++rp) {
            key_map->insertDevice(*rp, FREE);
          }
        }
      } // end if maxrange
    } else { // BBX was set
      // endpoint in bbx and not maxrange?
      if ( tree_base->inBBX(p) && ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) )  {
        // occupied endpoint
        OcTreeKey key;
        if (tree_base->coordToKeyChecked(p, key)){
          key_map->insertDevice(key, OCCUPIED);
        }

        // update freespace, break as soon as bbx limit is reached
        if (computeRayKeysCuda(origin, p, ray, resolution, resolution_half, tree_base)) {
          for(auto rp=ray.end(); rp != ray.begin(); rp--) {
            if (!tree_base->inBBX(*rp)) {
              key_map->insertDevice(*rp, FREE);
            }
            else break;
          }
        } // end if compute ray
      } // end if in BBX and not maxrange
    } // end bbx case
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
void OctomapUpdaterCuda<NODE>::initialize() {
  // make a copy of tree for device usage - need a better way ? or idk
  cudaCheckErrors(cudaMallocManaged(&tree_base_device_, sizeof(octomap::OccupancyOcTreeBase<NODE>)));
  cudaCheckErrors(cudaMemcpy(tree_base_device_, tree_base_, sizeof(octomap::OccupancyOcTreeBase<NODE>), cudaMemcpyHostToDevice));

  // rays
  n_rays_ = N_BLOCKS * N_THREADS;
  auto rays_host = new KeyRayCuda[n_rays_];
  for (int i = 0; i < N_BLOCKS * N_THREADS; ++i) {
    rays_host[i].allocateDevice();
  }
  cudaCheckErrors(cudaMallocManaged(&rays_device_, n_rays_ * sizeof(KeyRayCuda)));
  cudaCheckErrors(cudaMemcpy(rays_device_, &rays_host[0], n_rays_ * sizeof(KeyRayCuda), cudaMemcpyHostToDevice));
  delete[] rays_host;

  // Make a temporary container for occupied cells
  HashSetCuda key_map_temp_host;
  //HashSetCuda free_map_host;

  key_map_temp_host.initializeDevice(MAX_HASHED_ELEMENTS, MAX_ELEMENTS);
  //free_map_host.initializeDevice(MAX_HASHED_ELEMENTS, MAX_ELEMENTS);

  cudaCheckErrors(cudaMallocManaged(&key_map_device_, sizeof(HashSetCuda)));
  //cudaCheckErrors(cudaMallocManaged(&free_map_device_, sizeof(HashSetCuda)));
  cudaCheckErrors(cudaMemcpy(key_map_device_, &key_map_temp_host, sizeof(HashSetCuda), cudaMemcpyHostToDevice));
  //cudaCheckErrors(cudaMemcpy(free_map_device_, &free_map_host, sizeof(HashSetCuda), cudaMemcpyHostToDevice));

  // copy from device to host
  key_map_host_.initializeHost(MAX_HASHED_ELEMENTS, MAX_ELEMENTS);
  //free_map_host_.initializeHost(MAX_HASHED_ELEMENTS, MAX_ELEMENTS);
}

template <class NODE>
void OctomapUpdaterCuda<NODE>::computeUpdate(
  const octomap::Pointcloud& scan, 
  const octomap::point3d& origin,
  const double& maxrange,
  const bool& lazy_eval)
{
  //auto t_start = boost::chrono::high_resolution_clock::now();
  // total number of rays or points
  auto scan_size = scan.size();

  // make an array of points from the point cloud for device usage
  octomap::point3d* scan_device;
  cudaCheckErrors(cudaMallocManaged(&scan_device, scan_size * sizeof(octomap::point3d)));
  cudaCheckErrors(cudaMemcpy(scan_device, &scan[0], scan_size * sizeof(octomap::point3d), cudaMemcpyHostToDevice));

  // Reset the map
  key_map_device_->reset();
  computeUpdateKernel<NODE><<<6, 32>>>( 
    origin, 
    scan_device,
    rays_device_,
    key_map_device_,
    scan_size, 
    maxrange, 
    use_bbx_limit_,
    res_,
    res_half_,
    tree_base_device_); 
  cudaCheckErrors(cudaDeviceSynchronize());
  
  for (size_t i = 0; i < key_map_device_->count(); i++) {
    auto current = key_map_device_->element(i);
    while (current != NULL) {
      tree_base_->updateNode(current->key_, static_cast<bool>(current->value_), lazy_eval);
      current = current->next_;
    }
  }
  // free memory  
  cudaCheckErrors(cudaFree(scan_device));
  //std::cout << "time:" << boost::chrono::duration<double>(boost::chrono::high_resolution_clock::now() - t_start).count() << std::endl;
}
template class OctomapUpdaterCuda<OcTreeNode>;
template class OctomapUpdaterCuda<OcTreeNodeStamped>;
template class OctomapUpdaterCuda<ColorOcTreeNode>;
#endif