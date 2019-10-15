#include <octomap/OccupancyOcTreeBaseCUDA.cuh>
#include <octomap/CUDAAssertion.h>
#ifdef __CUDA_SUPPORT__

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#else
#define CUDA_CALLABLE
#endif

template <class NODE,class I>
CUDA_CALLABLE bool computeRayKeysCUDA(
  const point3d& origin, 
  const point3d& end, 
  KeyRayCUDA& ray,
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
  //printf ("direction: %f, %f, %f\n", direction.x(), direction.y(), direction.z());
  //printf ("tMax: %f, %f, %f\n", tMax[0], tMax[1], tMax[2]);
  //printf ("tDelta: %f, %f, %f\n", tDelta[0], tDelta[1], tDelta[2]);

  int i = 0;
  // Incremental phase  ---------------------------------------------------------
  bool done = false;
  while (!done) {
    //printf ("i: %i\n", i);

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
        //printf("length: %f\n", length);
        done = true;
        break;
      }

      else {  // continue to add freespace cells
      //printf ("adding key.");
        ray.addKey(current_key);
      }
    }
    ++i;
    assert ( ray.size() < ray.sizeMax() - 1);

  } // end while

  return true;
}

#define MAX_RAY_SIZE 100
template <class NODE>
__global__ void kernel(
  octomap::point3d origin,
  octomap::point3d* points,
  KeyContainerCUDA* occupied_cells,
  KeyContainerCUDA* free_cells,
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
  {
    auto& p = points[i];
    KeyRayCUDA ray(MAX_RAY_SIZE);
    if (!use_bbx_limit) { // no BBX specified
      if ((maxrange < 0.0) || ((p - origin).norm() <= maxrange) ) { // is not maxrange meas.
        // free cells
        if (computeRayKeysCUDA(origin, p, ray, resolution, resolution_half, tree_base)) {
          for (auto rp = ray.begin(); rp != ray.end(); ++rp) {
            free_cells->addKeyAtomic(*rp);
          }
        }
        // occupied endpoint
        OcTreeKey key;
        if (tree_base->coordToKeyChecked(p, key)) {
          occupied_cells->addKeyAtomic(key);
        }
      } else { // user set a maxrange and length is above
        point3d direction = (p - origin).normalized ();
        point3d new_end = origin + direction * (float) maxrange;
        if (computeRayKeysCUDA(origin, new_end, ray, resolution, resolution_half, tree_base)) { // *ray
          for (auto rp = ray.begin(); rp != ray.end(); ++rp) {
            free_cells->addKeyAtomic(*rp);
          }
        }
      } // end if maxrange
    } else { // BBX was set
      // endpoint in bbx and not maxrange?
      if ( tree_base->inBBX(p) && ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) )  {
        // occupied endpoint
        OcTreeKey key;
        if (tree_base->coordToKeyChecked(p, key)){
          occupied_cells->addKeyAtomic(key);
        }

        // update freespace, break as soon as bbx limit is reached
        if (computeRayKeysCUDA(origin, p, ray, resolution, resolution_half, tree_base)) {
          for(auto rp=ray.end(); rp != ray.begin(); rp--) {
            if (!tree_base->inBBX(*rp)) {
              free_cells->addKeyAtomic(*rp);
            }
            else break;
          }
        } // end if compute ray
      } // end if in BBX and not maxrange
    } // end bbx case
  }
}

template <class NODE>
void computeUpdateCUDA(
  const octomap::Pointcloud& scan, const octomap::point3d& origin, octomap::KeySet& free_cells, 
  octomap::KeySet& occupied_cells, double maxrange, octomap::OccupancyOcTreeBase<NODE>* tree_base)
{
  // total number of rays or points
  auto scan_size = scan.size();
  // make a copy of tree for device usage - need a better way ? or idk
  octomap::OccupancyOcTreeBase<NODE>* tree_base_device;
  cudaCheckErrors(cudaMallocManaged(&tree_base_device, sizeof(octomap::OccupancyOcTreeBase<NODE>)));
  cudaCheckErrors(cudaMemcpy(tree_base_device, tree_base, sizeof(octomap::OccupancyOcTreeBase<NODE>), cudaMemcpyHostToDevice));

  // make an array of points from the point cloud for device usage
  octomap::point3d* scan_device;
  cudaCheckErrors(cudaMallocManaged(&scan_device, scan_size * sizeof(octomap::point3d)));
	cudaCheckErrors(cudaMemcpy(scan_device, &scan[0], scan_size * sizeof(octomap::point3d), cudaMemcpyHostToDevice));


  // Make a container for occupied cells
  static const auto free_cells_arr_size = 1000000;
  auto occupied_cells_host = KeyContainerCUDA();
  auto free_cells_host = KeyContainerCUDA(free_cells_arr_size);
  KeyContainerCUDA* occupied_cells_device;
  KeyContainerCUDA* free_cells_device;
  cudaCheckErrors(cudaMallocManaged(&occupied_cells_device, sizeof(KeyContainerCUDA)));
  cudaCheckErrors(cudaMallocManaged(&free_cells_device, sizeof(KeyContainerCUDA)));
  cudaCheckErrors(cudaMemcpy(occupied_cells_device, &occupied_cells_host, sizeof(KeyContainerCUDA), cudaMemcpyHostToDevice));
  cudaCheckErrors(cudaMemcpy(free_cells_device, &free_cells_host, sizeof(KeyContainerCUDA), cudaMemcpyHostToDevice));
  occupied_cells_device->allocateDevice();
  free_cells_device->allocateDevice();

  occupied_cells_device->copyToDevice(occupied_cells_host);
  free_cells_device->copyToDevice(free_cells_host);
  bool use_bbx_limit = tree_base->bbxSet();
  auto resolution = tree_base->getResolution();
  auto resolution_half = resolution * 0.5;
  cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
  cudaEventRecord(start);
  kernel<NODE><<<8, 256>>>(
    origin, 
    scan_device, 
    occupied_cells_device, 
    free_cells_device,
    scan_size, 
    maxrange, 
    use_bbx_limit,
    resolution,
    resolution_half,
    tree_base_device);
  cudaEventRecord(stop);
  cudaDeviceSynchronize();
  cudaEventSynchronize(stop);

  printf("free_cells_device size: %i\n", free_cells_device->size());
  printf("occupied_cells_device size: %i\n", occupied_cells_device->size());

  // copy from device to host
  occupied_cells_host.copyToHost(*occupied_cells_device);
  free_cells_host.copyToHost(*free_cells_device);

  printf("free_cells_host size: %i\n", free_cells_host.size());
  printf("occupied_cells_host size: %i\n", occupied_cells_host.size());

  float ms = 0;
	cudaEventElapsedTime(&ms, start, stop);
	std::cout << "Elapsed time:" << ms << std::endl;

  // free memory  
  cudaFree(scan_device);
  //occupied_cells_device->freeDevice();
  //free_cells_device->freeDevice();
  cudaFree(occupied_cells_device);
  cudaFree(free_cells_device);
  cudaFree(tree_base_device);
}
template void computeUpdateCUDA(
  const Pointcloud& scan, const point3d& origin, KeySet& free_cells, 
  KeySet& occupied_cells, double maxrange, OccupancyOcTreeBase<OcTreeNode>*);
template void computeUpdateCUDA(
  const Pointcloud& scan, const point3d& origin, KeySet& free_cells, 
  KeySet& occupied_cells, double maxrange, OccupancyOcTreeBase<OcTreeNodeStamped>*);
template void computeUpdateCUDA(
  const Pointcloud& scan, const point3d& origin, KeySet& free_cells, 
  KeySet& occupied_cells, double maxrange, OccupancyOcTreeBase<ColorOcTreeNode>*);
#endif