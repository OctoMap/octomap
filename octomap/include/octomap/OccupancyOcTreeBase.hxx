/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bitset>
#include <algorithm>

#include <octomap/MCTables.h>

namespace octomap {

  template <class NODE>
  OccupancyOcTreeBase<NODE>::OccupancyOcTreeBase(double resolution)
    : OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>(resolution), use_bbx_limit(false), use_change_detection(false)
  {

  }
  
  template <class NODE>
  OccupancyOcTreeBase<NODE>::OccupancyOcTreeBase(double resolution, unsigned int tree_depth, unsigned int tree_max_val)
    : OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>(resolution, tree_depth, tree_max_val), use_bbx_limit(false), use_change_detection(false)
  {

  }  

  template <class NODE>
  OccupancyOcTreeBase<NODE>::~OccupancyOcTreeBase(){
  }

  template <class NODE>
  OccupancyOcTreeBase<NODE>::OccupancyOcTreeBase(const OccupancyOcTreeBase<NODE>& rhs) :
  OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>(rhs), use_bbx_limit(rhs.use_bbx_limit),
    bbx_min(rhs.bbx_min), bbx_max(rhs.bbx_max),
    bbx_min_key(rhs.bbx_min_key), bbx_max_key(rhs.bbx_max_key),
    use_change_detection(rhs.use_change_detection), changed_keys(rhs.changed_keys)
  {
    this->clamping_thres_min = rhs.clamping_thres_min;
    this->clamping_thres_max = rhs.clamping_thres_max;
    this->prob_hit_log = rhs.prob_hit_log;
    this->prob_miss_log = rhs.prob_miss_log;
    this->occ_prob_thres_log = rhs.occ_prob_thres_log;


  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertPointCloud(const ScanNode& scan, double maxrange, bool lazy_eval, bool discretize) {
    // performs transformation to data and sensor origin first
    Pointcloud& cloud = *(scan.scan);
    pose6d frame_origin = scan.pose;
    point3d sensor_origin = frame_origin.inv().transform(scan.pose.trans());
    insertPointCloud(cloud, sensor_origin, frame_origin, maxrange, lazy_eval, discretize);
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                                             double maxrange, bool lazy_eval, bool discretize) {

    KeySet free_cells, occupied_cells;
    if (discretize)
      computeDiscreteUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);
    else
      computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);

    // insert data into tree  -----------------------
    for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
      updateNode(*it, false, lazy_eval);
    }
    for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
      updateNode(*it, true, lazy_eval);
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertPointCloud(const Pointcloud& pc, const point3d& sensor_origin, const pose6d& frame_origin,
                                             double maxrange, bool lazy_eval, bool discretize) {
    // performs transformation to data and sensor origin first
    Pointcloud transformed_scan (pc);
    transformed_scan.transform(frame_origin);
    point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
    insertPointCloud(transformed_scan, transformed_sensor_origin, maxrange, lazy_eval, discretize);
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertPointCloudRays(const Pointcloud& pc, const point3d& origin, double maxrange, bool lazy_eval) {
    if (pc.size() < 1)
      return;

#ifdef _OPENMP
    omp_set_num_threads(this->keyrays.size());
    #pragma omp parallel for
#endif
    for (int i = 0; i < (int)pc.size(); ++i) {
      const point3d& p = pc[i];
      unsigned threadIdx = 0;
#ifdef _OPENMP
      threadIdx = omp_get_thread_num();
#endif
      KeyRay* keyray = &(this->keyrays.at(threadIdx));

      if (this->computeRayKeys(origin, p, *keyray)){
#ifdef _OPENMP
        #pragma omp critical
#endif
        {
          for(KeyRay::iterator it=keyray->begin(); it != keyray->end(); it++) {
            updateNode(*it, false, lazy_eval); // insert freespace measurement
          }
          updateNode(p, true, lazy_eval); // update endpoint to be occupied
        }
      }

    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::computeDiscreteUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                                                KeySet& free_cells, KeySet& occupied_cells,
                                                double maxrange)
 {
   Pointcloud discretePC;
   discretePC.reserve(scan.size());
   KeySet endpoints;

   for (int i = 0; i < (int)scan.size(); ++i) {
     OcTreeKey k = this->coordToKey(scan[i]);
     std::pair<KeySet::iterator,bool> ret = endpoints.insert(k);
     if (ret.second){ // insertion took place => k was not in set
       discretePC.push_back(this->keyToCoord(k));
     }
   }

   computeUpdate(discretePC, origin, free_cells, occupied_cells, maxrange);
 }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::computeUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                                                KeySet& free_cells, KeySet& occupied_cells,
                                                double maxrange)
  {



#ifdef _OPENMP
    omp_set_num_threads(this->keyrays.size());
    #pragma omp parallel for schedule(guided)
#endif
    for (int i = 0; i < (int)scan.size(); ++i) {
      const point3d& p = scan[i];
      unsigned threadIdx = 0;
#ifdef _OPENMP
      threadIdx = omp_get_thread_num();
#endif
      KeyRay* keyray = &(this->keyrays.at(threadIdx));


      if (!use_bbx_limit) { // no BBX specified
        if ((maxrange < 0.0) || ((p - origin).norm() <= maxrange) ) { // is not maxrange meas.
          // free cells
          if (this->computeRayKeys(origin, p, *keyray)){
#ifdef _OPENMP
            #pragma omp critical (free_insert)
#endif
            {
              free_cells.insert(keyray->begin(), keyray->end());
            }
          }
          // occupied endpoint
          OcTreeKey key;
          if (this->coordToKeyChecked(p, key)){
#ifdef _OPENMP
            #pragma omp critical (occupied_insert)
#endif
            {
              occupied_cells.insert(key);
            }
          }
        } else { // user set a maxrange and length is above
          point3d direction = (p - origin).normalized ();
          point3d new_end = origin + direction * (float) maxrange;
          if (this->computeRayKeys(origin, new_end, *keyray)){
#ifdef _OPENMP
            #pragma omp critical (free_insert)
#endif
            {
              free_cells.insert(keyray->begin(), keyray->end());
            }
          }
        } // end if maxrange
      } else { // BBX was set
        // endpoint in bbx and not maxrange?
        if ( inBBX(p) && ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) )  {

          // occupied endpoint
          OcTreeKey key;
          if (this->coordToKeyChecked(p, key)){
#ifdef _OPENMP
            #pragma omp critical (occupied_insert)
#endif
            {
              occupied_cells.insert(key);
            }
          }

          // update freespace, break as soon as bbx limit is reached
          if (this->computeRayKeys(origin, p, *keyray)){
            for(KeyRay::reverse_iterator rit=keyray->rbegin(); rit != keyray->rend(); rit++) {
              if (inBBX(*rit)) {
#ifdef _OPENMP
                #pragma omp critical (free_insert)
#endif
                {
                  free_cells.insert(*rit);
                }
              }
              else break;
            }
          } // end if compute ray
        } // end if in BBX and not maxrange
      } // end bbx case

    } // end for all points, end of parallel OMP loop

    // prefer occupied cells over free ones (and make sets disjunct)
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ){
      if (occupied_cells.find(*it) != occupied_cells.end()){
        it = free_cells.erase(it);
      } else {
        ++it;
      }
    }
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::setNodeValue(const OcTreeKey& key, float log_odds_value, bool lazy_eval) {
    // clamp log odds within range:
    log_odds_value = std::min(std::max(log_odds_value, this->clamping_thres_min), this->clamping_thres_max);

    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new NODE();
      this->tree_size++;
      createdRoot = true;
    }

    return setNodeValueRecurs(this->root, createdRoot, key, 0, log_odds_value, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::setNodeValue(const point3d& value, float log_odds_value, bool lazy_eval) {
    OcTreeKey key;
    if (!this->coordToKeyChecked(value, key))
      return NULL;

    return setNodeValue(key, log_odds_value, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::setNodeValue(double x, double y, double z, float log_odds_value, bool lazy_eval) {
    OcTreeKey key;
    if (!this->coordToKeyChecked(x, y, z, key))
      return NULL;

    return setNodeValue(key, log_odds_value, lazy_eval);
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const OcTreeKey& key, float log_odds_update, bool lazy_eval) {
    // early abort (no change will happen).
    // may cause an overhead in some configuration, but more often helps
    NODE* leaf = this->search(key);
    // no change: node already at threshold
    if (leaf
        && ((log_odds_update >= 0 && leaf->getLogOdds() >= this->clamping_thres_max)
        || ( log_odds_update <= 0 && leaf->getLogOdds() <= this->clamping_thres_min)))
    {
      return leaf;
    }

    bool createdRoot = false;
    if (this->root == NULL){
      this->root = new NODE();
      this->tree_size++;
      createdRoot = true;
    }

    return updateNodeRecurs(this->root, createdRoot, key, 0, log_odds_update, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const point3d& value, float log_odds_update, bool lazy_eval) {
    OcTreeKey key;
    if (!this->coordToKeyChecked(value, key))
      return NULL;

    return updateNode(key, log_odds_update, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval) {
    OcTreeKey key;
    if (!this->coordToKeyChecked(x, y, z, key))
      return NULL;

    return updateNode(key, log_odds_update, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval) {
    float logOdds = this->prob_miss_log;
    if (occupied)
      logOdds = this->prob_hit_log;

    return updateNode(key, logOdds, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const point3d& value, bool occupied, bool lazy_eval) {
    OcTreeKey key;
    if (!this->coordToKeyChecked(value, key))
      return NULL;
    return updateNode(key, occupied, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(double x, double y, double z, bool occupied, bool lazy_eval) {
    OcTreeKey key;
    if (!this->coordToKeyChecked(x, y, z, key))
      return NULL;
    return updateNode(key, occupied, lazy_eval);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                                                    unsigned int depth, const float& log_odds_update, bool lazy_eval) {
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth) {
      unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
      if (!node->childExists(pos)) {
        // child does not exist, but maybe it's a pruned node?
        if ((!node->hasChildren()) && !node_just_created ) {
          // current node does not have children AND it is not a new node 
          // -> expand pruned node
          node->expandNode();
          this->tree_size+=8;
          this->size_changed = true;
        }
        else {
          // not a pruned node, create requested child
          node->createChild(pos);
          this->tree_size++;
          this->size_changed = true;
          created_node = true;
        }
      }

      if (lazy_eval)
        return updateNodeRecurs(node->getChild(pos), created_node, key, depth+1, log_odds_update, lazy_eval);
      else {
        NODE* retval = updateNodeRecurs(node->getChild(pos), created_node, key, depth+1, log_odds_update, lazy_eval);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (node->pruneNode()){
          this->tree_size -= 8;
          // return pointer to current parent (pruned), the just updated node no longer exists
          retval = node;
        } else{
          node->updateOccupancyChildren();
        }

        return retval;
      }
    }

    // at last level, update node, end of recursion
    else {
      if (use_change_detection) {
        bool occBefore = this->isNodeOccupied(node);
        updateNodeLogOdds(node, log_odds_update); 

        if (node_just_created){  // new node
          changed_keys.insert(std::pair<OcTreeKey,bool>(key, true));
        } else if (occBefore != this->isNodeOccupied(node)) {  // occupancy changed, track it
          KeyBoolMap::iterator it = changed_keys.find(key);
          if (it == changed_keys.end())
            changed_keys.insert(std::pair<OcTreeKey,bool>(key, false));
          else if (it->second == false)
            changed_keys.erase(it);
        }
      } else {
        updateNodeLogOdds(node, log_odds_update); 
      }
      return node;
    }
  }
  
  // TODO: mostly copy of updateNodeRecurs => merge code or general tree modifier / traversal
  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::setNodeValueRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                                                    unsigned int depth, const float& log_odds_value, bool lazy_eval) {
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth) {
      unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
      if (!node->childExists(pos)) {
        // child does not exist, but maybe it's a pruned node?
        if ((!node->hasChildren()) && !node_just_created ) {
          // current node does not have children AND it is not a new node
          // -> expand pruned node
          node->expandNode();
          this->tree_size+=8;
          this->size_changed = true;
        }
        else {
          // not a pruned node, create requested child
          node->createChild(pos);
          this->tree_size++;
          this->size_changed = true;
          created_node = true;
        }
      }

      if (lazy_eval)
        return setNodeValueRecurs(node->getChild(pos), created_node, key, depth+1, log_odds_value, lazy_eval);
      else {
        NODE* retval = setNodeValueRecurs(node->getChild(pos), created_node, key, depth+1, log_odds_value, lazy_eval);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (node->pruneNode()){
          this->tree_size -= 8;
          // return pointer to current parent (pruned), the just updated node no longer exists
          retval = node;
        } else{
          node->updateOccupancyChildren();
        }

        return retval;
      }
    }

    // at last level, update node, end of recursion
    else {
      if (use_change_detection) {
        bool occBefore = this->isNodeOccupied(node);
        node->setLogOdds(log_odds_value);

        if (node_just_created){  // new node
          changed_keys.insert(std::pair<OcTreeKey,bool>(key, true));
        } else if (occBefore != this->isNodeOccupied(node)) {  // occupancy changed, track it
          KeyBoolMap::iterator it = changed_keys.find(key);
          if (it == changed_keys.end())
            changed_keys.insert(std::pair<OcTreeKey,bool>(key, false));
          else if (it->second == false)
            changed_keys.erase(it);
        }
      } else {
        node->setLogOdds(log_odds_value);
      }
      return node;
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::updateInnerOccupancy(){
    if (this->root)
      this->updateInnerOccupancyRecurs(this->root, 0);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::updateInnerOccupancyRecurs(NODE* node, unsigned int depth){
    assert(node);

    // only recurse and update for inner nodes:
    if (node->hasChildren()){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {
            updateInnerOccupancyRecurs(node->getChild(i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::toMaxLikelihood() {
    if (this->root == NULL)
      return;

    // convert bottom up
    for (unsigned int depth=this->tree_depth; depth>0; depth--) {
      toMaxLikelihoodRecurs(this->root, 0, depth);
    }

    // convert root
    nodeToMaxLikelihood(this->root);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::toMaxLikelihoodRecurs(NODE* node, unsigned int depth,
      unsigned int max_depth) {

    assert(node);

    if (depth < max_depth) {
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          toMaxLikelihoodRecurs(node->getChild(i), depth+1, max_depth);
        }
      }
    }

    else { // max level reached
      nodeToMaxLikelihood(node);
    }
  }
  
  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::getNormals(const point3d& point, std::vector<point3d>& normals,
                                             bool unknownStatus) const {
    normals.clear();

    OcTreeKey init_key;
    if ( !OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>::coordToKeyChecked(point, init_key) ) {
      OCTOMAP_WARNING_STR("Voxel out of bounds");
      return false;
    }

    // OCTOMAP_WARNING("Normal for %f, %f, %f\n", point.x(), point.y(), point.z());

    int vertex_values[8];

    OcTreeKey current_key;
    NODE* current_node;

    // There is 8 neighbouring sets
    // The current cube can be at any of the 8 vertex
    int x_index[4][4] = {{1, 1, 0, 0}, {1, 1, 0, 0}, {0, 0 -1, -1}, {0, 0 -1, -1}};
    int y_index[4][4] = {{1, 0, 0, 1}, {0, -1, -1, 0}, {0, -1, -1, 0}, {1, 0, 0, 1}};
    int z_index[2][2] = {{0, 1}, {-1, 0}};

    // Iterate over the 8 neighboring sets
    for(int m = 0; m < 2; ++m){
      for(int l = 0; l < 4; ++l){
        
        int k = 0;
        // Iterate over the cubes
        for(int j = 0; j < 2; ++j){
          for(int i = 0; i < 4; ++i){
            current_key[0] = init_key[0] + x_index[l][i];
            current_key[1] = init_key[1] + y_index[l][i];
            current_key[2] = init_key[2] + z_index[m][j];
            current_node = this->search(current_key);

            if(current_node){
              vertex_values[k] = this->isNodeOccupied(current_node);

              // point3d coord = this->keyToCoord(current_key);
              // OCTOMAP_WARNING_STR("vertex " << k << " at " << coord << "; value " << vertex_values[k]);
            }else{
              // Occupancy of unknown cells
              vertex_values[k] = unknownStatus;
            }
            ++k;
          }
        }

        int cube_index = 0;
        if (vertex_values[0]) cube_index |= 1;
        if (vertex_values[1]) cube_index |= 2;
        if (vertex_values[2]) cube_index |= 4;
        if (vertex_values[3]) cube_index |= 8;
        if (vertex_values[4]) cube_index |= 16;
        if (vertex_values[5]) cube_index |= 32;
        if (vertex_values[6]) cube_index |= 64;
        if (vertex_values[7]) cube_index |= 128;

        // OCTOMAP_WARNING_STR("cubde_index: " << cube_index);

        // All vertices are occupied or free resulting in no normal
        if (edgeTable[cube_index] == 0)
          return true;

        // No interpolation is done yet, we use vertexList in <MCTables.h>.
        for(int i = 0; triTable[cube_index][i] != -1; i += 3){
          point3d p1 = vertexList[triTable[cube_index][i  ]];
          point3d p2 = vertexList[triTable[cube_index][i+1]];
          point3d p3 = vertexList[triTable[cube_index][i+2]];
          point3d v1 = p2 - p1;
          point3d v2 = p3 - p1;

          // OCTOMAP_WARNING("Vertex p1 %f, %f, %f\n", p1.x(), p1.y(), p1.z());
          // OCTOMAP_WARNING("Vertex p2 %f, %f, %f\n", p2.x(), p2.y(), p2.z());
          // OCTOMAP_WARNING("Vertex p3 %f, %f, %f\n", p3.x(), p3.y(), p3.z());

          // Right hand side cross product to retrieve the normal in the good
          // direction (pointing to the free nodes).
          normals.push_back(v1.cross(v2).normalize());
        }
      }
    }

    return true;
  }
  
  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::castRay(const point3d& origin, const point3d& directionP, point3d& end, 
                                          bool ignoreUnknown, double maxRange) const {

    /// ----------  see OcTreeBase::computeRayKeys  -----------

    // Initialization phase -------------------------------------------------------
    OcTreeKey current_key;
    if ( !OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>::coordToKeyChecked(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }

    NODE* startingNode = this->search(current_key);
    if (startingNode){
      if (this->isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        // (need to convert from key, since origin does not need to be a voxel center)
        end = this->keyToCoord(current_key);
        return true;
      }
    } else if(!ignoreUnknown){
      end = this->keyToCoord(current_key);
      return false;
    }

    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

    int step[3]; 
    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      // compute step direction
      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      // compute tMax, tDelta
      if (step[i] != 0) {
        // corner point of voxel (in direction of ray)
        double voxelBorder = this->keyToCoord(current_key[i]);
        voxelBorder += double(step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0){
    	OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
    	return false;
    }

    // for speedup:
    double maxrange_sq = maxRange *maxRange;

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

      // check for overflow:
      if ((step[dim] < 0 && current_key[dim] == 0)
    		  || (step[dim] > 0 && current_key[dim] == 2* this->tree_max_val-1))
      {
        OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
        // return border point nevertheless:
        end = this->keyToCoord(current_key);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];


      // generate world coords from key
      end = this->keyToCoord(current_key);

      // check for maxrange:
      if (max_range_set){
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq)
          return false;

      }

      NODE* currentNode = this->search(current_key);
      if (currentNode){
        if (this->isNodeOccupied(currentNode)) {
          done = true;
          break;
        }
        // otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
        return false;
      }
    } // end while

    return true;
  }

  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::getRayIntersection (const point3d& origin, const point3d& direction, const point3d& center,
                 point3d& intersection, double delta/*=0.0*/) const {
    // We only need three normals for the six planes
    octomap::point3d normalX(1, 0, 0);
    octomap::point3d normalY(0, 1, 0);
    octomap::point3d normalZ(0, 0, 1);

    // One point on each plane, let them be the center for simplicity
    octomap::point3d pointXNeg(center(0) - float(this->resolution / 2.0), center(1), center(2));
    octomap::point3d pointXPos(center(0) + float(this->resolution / 2.0), center(1), center(2));
    octomap::point3d pointYNeg(center(0), center(1) - float(this->resolution / 2.0), center(2));
    octomap::point3d pointYPos(center(0), center(1) + float(this->resolution / 2.0), center(2));
    octomap::point3d pointZNeg(center(0), center(1), center(2) - float(this->resolution / 2.0));
    octomap::point3d pointZPos(center(0), center(1), center(2) + float(this->resolution / 2.0));

    double lineDotNormal = 0.0;
    double d = 0.0;
    double outD = std::numeric_limits<double>::max();
    octomap::point3d intersect;
    bool found = false;

    // Find the intersection (if any) with each place
    // Line dot normal will be zero if they are parallel, in which case no intersection can be the entry one
    // if there is an intersection does it occur in the bounded plane of the voxel
    // if yes keep only the closest (smallest distance to sensor origin).
    if((lineDotNormal = normalX.dot(direction))){
      d = (pointXNeg - origin).dot(normalX) / lineDotNormal;
      intersect = direction * float(d) + origin;
      if(!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6) ||
         intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
        outD = std::min(outD, d);
        found = true;
      }

      d = (pointXPos - origin).dot(normalX) / lineDotNormal;
      intersect = direction * float(d) + origin;
      if(!(intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6) ||
         intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
        outD = std::min(outD, d);
        found = true;
      }
    }

    if((lineDotNormal = normalY.dot(direction))){
      d = (pointYNeg - origin).dot(normalY) / lineDotNormal;
      intersect = direction * float(d) + origin;
      if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
         intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
        outD = std::min(outD, d);
        found = true;
      }

      d = (pointYPos - origin).dot(normalY) / lineDotNormal;
      intersect = direction * float(d) + origin;
      if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
         intersect(2) < (pointZNeg(2) - 1e-6) || intersect(2) > (pointZPos(2) + 1e-6))){
        outD = std::min(outD, d);
        found = true;
      }
    }

    if((lineDotNormal = normalZ.dot(direction))){
      d = (pointZNeg - origin).dot(normalZ) / lineDotNormal;
      intersect = direction * float(d) + origin;
      if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
         intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
        outD = std::min(outD, d);
        found = true;
      }

      d = (pointZPos - origin).dot(normalZ) / lineDotNormal;
      intersect = direction * float(d) + origin;
      if(!(intersect(0) < (pointXNeg(0) - 1e-6) || intersect(0) > (pointXPos(0) + 1e-6) ||
         intersect(1) < (pointYNeg(1) - 1e-6) || intersect(1) > (pointYPos(1) + 1e-6))){
        outD = std::min(outD, d);
        found = true;
      }
    }

    // Substract (add) a fraction to ensure no ambiguity on the starting voxel
    // Don't start on a bondary.
    if(found)
      intersection = direction * float(outD + delta) + origin;
    
    return found;
  }


  template <class NODE> inline bool 
  OccupancyOcTreeBase<NODE>::integrateMissOnRay(const point3d& origin, const point3d& end, bool lazy_eval) {

    if (!this->computeRayKeys(origin, end, this->keyrays.at(0))) {
      return false;
    }
    
    for(KeyRay::iterator it=this->keyrays[0].begin(); it != this->keyrays[0].end(); it++) {
      updateNode(*it, false, lazy_eval); // insert freespace measurement
    }
  
    return true;
  }

  template <class NODE> bool 
  OccupancyOcTreeBase<NODE>::insertRay(const point3d& origin, const point3d& end, double maxrange, bool lazy_eval)
  {
    // cut ray at maxrange
    if ((maxrange > 0) && ((end - origin).norm () > maxrange)) 
      {
        point3d direction = (end - origin).normalized ();
        point3d new_end = origin + direction * (float) maxrange;
        return integrateMissOnRay(origin, new_end,lazy_eval);
      }
    // insert complete ray
    else 
      {
        if (!integrateMissOnRay(origin, end,lazy_eval))
          return false;
        updateNode(end, true, lazy_eval); // insert hit cell
        return true;
      }
  }
  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::setBBXMin (point3d& min) { 
    bbx_min = min; 
    if (!this->coordToKeyChecked(bbx_min, bbx_min_key)) {
      OCTOMAP_ERROR("ERROR while generating bbx min key.\n");
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::setBBXMax (point3d& max) {
    bbx_max = max; 
    if (!this->coordToKeyChecked(bbx_max, bbx_max_key)) {
      OCTOMAP_ERROR("ERROR while generating bbx max key.\n");
    }
  }


  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::inBBX(const point3d& p) const {
    return ((p.x() >= bbx_min.x()) && (p.y() >= bbx_min.y()) && (p.z() >= bbx_min.z()) &&
            (p.x() <= bbx_max.x()) && (p.y() <= bbx_max.y()) && (p.z() <= bbx_max.z()) );
  }


  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::inBBX(const OcTreeKey& key) const {
    return ((key[0] >= bbx_min_key[0]) && (key[1] >= bbx_min_key[1]) && (key[2] >= bbx_min_key[2]) &&
            (key[0] <= bbx_max_key[0]) && (key[1] <= bbx_max_key[1]) && (key[2] <= bbx_max_key[2]) );
  }

  template <class NODE>
  point3d OccupancyOcTreeBase<NODE>::getBBXBounds () const {
    octomap::point3d obj_bounds = (bbx_max - bbx_min);
    obj_bounds /= 2.;
    return obj_bounds;
  }

  template <class NODE>
  point3d OccupancyOcTreeBase<NODE>::getBBXCenter () const {
    octomap::point3d obj_bounds = (bbx_max - bbx_min);
    obj_bounds /= 2.;
    return bbx_min + obj_bounds;
  }

  // -- I/O  -----------------------------------------

  template <class NODE>
  std::istream& OccupancyOcTreeBase<NODE>::readBinaryData(std::istream &s){
    // tree needs to be newly created or cleared externally
    if (this->root) {
      OCTOMAP_ERROR_STR("Trying to read into an existing tree.");
      return s;
    }

    this->root = new NODE();
    this->readBinaryNode(s, this->root);
    this->size_changed = true;
    this->tree_size = OcTreeBaseImpl<NODE,AbstractOccupancyOcTree>::calcNumNodes();  // compute number of nodes    
    return s;
  }

  template <class NODE>
  std::ostream& OccupancyOcTreeBase<NODE>::writeBinaryData(std::ostream &s) const{
    OCTOMAP_DEBUG("Writing %zu nodes to output stream...", this->size());
    if (this->root)
      this->writeBinaryNode(s, this->root);
    return s;
  }

  template <class NODE>
  std::istream& OccupancyOcTreeBase<NODE>::readBinaryNode(std::istream &s, NODE* node) const {

    assert(node);

    char child1to4_char;
    char child5to8_char;
    s.read((char*)&child1to4_char, sizeof(char));
    s.read((char*)&child5to8_char, sizeof(char));

    std::bitset<8> child1to4 ((unsigned long long) child1to4_char);
    std::bitset<8> child5to8 ((unsigned long long) child5to8_char);

    //     std::cout << "read:  "
    //        << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
    //        << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;


    // inner nodes default to occupied
    node->setLogOdds(this->clamping_thres_max);

    for (unsigned int i=0; i<4; i++) {
      if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 0)) {
        // child is free leaf
        node->createChild(i);
        node->getChild(i)->setLogOdds(this->clamping_thres_min);
      }
      else if ((child1to4[i*2] == 0) && (child1to4[i*2+1] == 1)) {
        // child is occupied leaf
        node->createChild(i);
        node->getChild(i)->setLogOdds(this->clamping_thres_max);
      }
      else if ((child1to4[i*2] == 1) && (child1to4[i*2+1] == 1)) {
        // child has children
        node->createChild(i);
        node->getChild(i)->setLogOdds(-200.); // child is unkown, we leave it uninitialized
      }
    }
    for (unsigned int i=0; i<4; i++) {
      if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 0)) {
        // child is free leaf
        node->createChild(i+4);
        node->getChild(i+4)->setLogOdds(this->clamping_thres_min);
      }
      else if ((child5to8[i*2] == 0) && (child5to8[i*2+1] == 1)) {
        // child is occupied leaf
        node->createChild(i+4);
        node->getChild(i+4)->setLogOdds(this->clamping_thres_max);
      }
      else if ((child5to8[i*2] == 1) && (child5to8[i*2+1] == 1)) {
        // child has children
        node->createChild(i+4);
        node->getChild(i+4)->setLogOdds(-200.); // set occupancy when all children have been read
      }
      // child is unkown, we leave it uninitialized
    }

    // read children's children and set the label
    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        NODE* child = node->getChild(i);
        if (fabs(child->getLogOdds() + 200.)<1e-3) {
          readBinaryNode(s, child);
          child->setLogOdds(child->getMaxChildLogOdds());
        }
      } // end if child exists
    } // end for children

    return s;
  }

  template <class NODE>
  std::ostream& OccupancyOcTreeBase<NODE>::writeBinaryNode(std::ostream &s, const NODE* node) const{

    assert(node);

    // 2 bits for each children, 8 children per node -> 16 bits
    std::bitset<8> child1to4;
    std::bitset<8> child5to8;

    // 10 : child is free node
    // 01 : child is occupied node
    // 00 : child is unkown node
    // 11 : child has children


    // speedup: only set bits to 1, rest is init with 0 anyway,
    //          can be one logic expression per bit

    for (unsigned int i=0; i<4; i++) {
      if (node->childExists(i)) {
        const NODE* child = node->getChild(i);
        if      (child->hasChildren())  { child1to4[i*2] = 1; child1to4[i*2+1] = 1; }
        else if (this->isNodeOccupied(child)) { child1to4[i*2] = 0; child1to4[i*2+1] = 1; }
        else                            { child1to4[i*2] = 1; child1to4[i*2+1] = 0; }
      }
      else {
        child1to4[i*2] = 0; child1to4[i*2+1] = 0;
      }
    }

    for (unsigned int i=0; i<4; i++) {
      if (node->childExists(i+4)) {
        const NODE* child = node->getChild(i+4);
        if      (child->hasChildren())  { child5to8[i*2] = 1; child5to8[i*2+1] = 1; }
        else if (this->isNodeOccupied(child)) { child5to8[i*2] = 0; child5to8[i*2+1] = 1; }
        else                            { child5to8[i*2] = 1; child5to8[i*2+1] = 0; }
      }
      else {
        child5to8[i*2] = 0; child5to8[i*2+1] = 0;
      }
    }
    //     std::cout << "wrote: "
    //        << child1to4.to_string<char,std::char_traits<char>,std::allocator<char> >() << " "
    //        << child5to8.to_string<char,std::char_traits<char>,std::allocator<char> >() << std::endl;

    char child1to4_char = (char) child1to4.to_ulong();
    char child5to8_char = (char) child5to8.to_ulong();

    s.write((char*)&child1to4_char, sizeof(char));
    s.write((char*)&child5to8_char, sizeof(char));

    // write children's children
    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        const NODE* child = node->getChild(i);
        if (child->hasChildren()) {
          writeBinaryNode(s, child);
        }
      }
    }

    return s;
  }

  //-- Occupancy queries on nodes:

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::updateNodeLogOdds(NODE* occupancyNode, const float& update) const {
    occupancyNode->addValue(update);
    if (occupancyNode->getLogOdds() < this->clamping_thres_min) {
      occupancyNode->setLogOdds(this->clamping_thres_min);
      return;
    }
    if (occupancyNode->getLogOdds() > this->clamping_thres_max) {
      occupancyNode->setLogOdds(this->clamping_thres_max);
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::integrateHit(NODE* occupancyNode) const {
    updateNodeLogOdds(occupancyNode, this->prob_hit_log);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::integrateMiss(NODE* occupancyNode) const {
    updateNodeLogOdds(occupancyNode, this->prob_miss_log);
  }
  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::nodeToMaxLikelihood(NODE* occupancyNode) const{
    if (this->isNodeOccupied(occupancyNode))
      occupancyNode->setLogOdds(this->clamping_thres_max);
    else
      occupancyNode->setLogOdds(this->clamping_thres_min);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::nodeToMaxLikelihood(NODE& occupancyNode) const{
    if (this->isNodeOccupied(occupancyNode))
      occupancyNode.setLogOdds(this->clamping_thres_max);
    else
      occupancyNode.setLogOdds(this->clamping_thres_min);
  }

} // namespace
