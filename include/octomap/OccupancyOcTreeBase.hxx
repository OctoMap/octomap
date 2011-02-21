// $Id$

/**
 * OctoMap:
 * A probabilistic, flexible, and compact 3D mapping library for robotic systems.
 * @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
 * @see http://octomap.sourceforge.net/
 * License: New BSD License
 */

/*
 * Copyright (c) 2009-2011, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
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

#include <octomap/CountingOcTree.h>


namespace octomap {


  template <class NODE>
  OccupancyOcTreeBase<NODE>::OccupancyOcTreeBase(double _resolution)
    : OcTreeBase<NODE>(_resolution), use_bbx_limit(false)
  {

  }


  template <class NODE>
  OccupancyOcTreeBase<NODE>::~OccupancyOcTreeBase(){
  }



  // performs transformation to data and sensor origin first
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScan(const ScanNode& scan, double maxrange, bool pruning) {
    Pointcloud& cloud = *(scan.scan);
    pose6d frame_origin = scan.pose;
    point3d sensor_origin = frame_origin.inv().transform(scan.pose.trans());
    insertScan(cloud, sensor_origin, frame_origin, maxrange, pruning);
  }


  // deprecated: use above method instead, or the new interface below
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScan(const Pointcloud& pc, const pose6d& originPose,
                          double maxrange, bool pruning) {
    point3d sensor_origin = originPose.trans();
    pose6d frame_origin = originPose;
    insertScan(pc, sensor_origin, frame_origin, maxrange, pruning);
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScan(const Pointcloud& scan, const octomap::point3d& sensor_origin, 
                                             double maxrange, bool pruning) {

    point3d_list free_cells;
    point3d_list occupied_cells;
    computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);    

    // insert data into tree  -----------------------
    for (point3d_list::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
      updateNode(*it, false);
    }
    for (point3d_list::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
      updateNode(*it, true);
    }

    if (pruning) this->prune();
  } 


  // performs transformation to data and sensor origin first
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScan(const Pointcloud& pc, const point3d& sensor_origin, const pose6d& frame_origin, 
                                             double maxrange, bool pruning) {
    Pointcloud transformed_scan (pc);
    transformed_scan.transform(frame_origin);
    point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
    insertScan(transformed_scan, transformed_sensor_origin, maxrange, pruning); 
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScanNaive(const Pointcloud& pc, const point3d& origin, double maxrange, bool pruning) {
    if (pc.size() < 1)
      return;

    // integrate each single beam
    octomap::point3d p;
    for (octomap::Pointcloud::const_iterator point_it = pc.begin();
         point_it != pc.end(); point_it++) {
      this->insertRay(origin, *point_it, maxrange);
    }

    if (pruning)
      this->prune();
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::computeUpdate(const Pointcloud& scan, const octomap::point3d& origin, 
                                                point3d_list& free_cells, 
                                                point3d_list& occupied_cells,
                                                double maxrange) {

    CountingOcTree free_tree     (this->getResolution());
    CountingOcTree occupied_tree (this->getResolution());

    for (Pointcloud::const_iterator point_it = scan.begin(); point_it != scan.end(); point_it++) {
      const point3d& p = *point_it;


      if (!use_bbx_limit) {

        // -------------- no BBX specified ---------------

        if ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) { // is not maxrange meas.
          // free cells
          if (this->computeRayKeys(origin, p, this->keyray)){
            for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
              free_tree.updateNode(*it);
            }
          }
          // occupied cells
          occupied_tree.updateNode(p);
        } // end if NOT maxrange

        else { // user set a maxrange and this is reached
          point3d direction = (p - origin).normalized ();
          point3d new_end = origin + direction * maxrange;
          if (this->computeRayKeys(origin, new_end, this->keyray)){
            for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
              
              free_tree.updateNode(*it);
            }
          }
        } // end if maxrange      
      }

      else {

        // --- update limited by user specified BBX  -----

        // endpoint in bbx and not maxrange?
        if ( inBBX(p) && ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) )  {

          // update endpoint
          occupied_tree.updateNode(p);
         
          // update freespace, break as soon as bbx limit is reached
          if (this->computeRayKeys(origin, p, this->keyray)){
            for(KeyRay::reverse_iterator rit=this->keyray.rbegin(); rit != this->keyray.rend(); rit++) {
              if (inBBX(*rit)) {
                free_tree.updateNode(*rit);
              }
              else break;
            }
          }

        } } // end bbx case
          
    } // end for all points
    
    
    free_cells.clear();
    free_tree.getLeafNodes(free_cells);

    occupied_cells.clear();
    occupied_tree.getLeafNodes(occupied_cells);

    if(occupied_tree.size() > 1)  // Occupied tree has more than just root node (ie not empty)
      {
        // delete free cells if cell is also measured occupied
        for (point3d_list::iterator cellit = free_cells.begin(); cellit != free_cells.end();){
          if ( occupied_tree.search(*cellit) ) {
            cellit = free_cells.erase(cellit);
          }
          else {
            cellit++;
          }
        } // end for
      }
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const point3d& value, bool occupied) {

    OcTreeKey key;
    if (!this->genKey(value, key)) return NULL;
    return updateNode(key, occupied);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const point3d& value, float log_odds_update) {

    OcTreeKey key;
    if (!this->genKey(value, key)) return NULL;
    return updateNode(key, log_odds_update);    
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const OcTreeKey& key, bool occupied) {

    NODE* leaf = this->search(key);
    if (leaf) {
      if ((leaf->atThreshold()) && (leaf->isOccupied() == occupied)) {
        return leaf;
      }
    }
    return updateNodeRecurs(this->itsRoot, false, key, 0, occupied);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const OcTreeKey& key, float log_odds_update) {
    return updateNodeRecurs(this->itsRoot, false, key, 0, log_odds_update);    
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNodeRecurs(NODE* node, bool node_just_created,
                                                    const OcTreeKey& key, unsigned int depth,
                                                    bool occupied) {

    unsigned int pos (0);
    this->genPos(key, this->tree_depth-1-depth, pos);
    bool created_node = false;

    // follow down to last level
    if (depth < this->tree_depth) {
      if (!node->childExists(pos)) {
        // child does not exist, but maybe it's a pruned node?
        if ((!node->hasChildren()) && !node_just_created && (node != this->itsRoot)) {
          // current node does not have children AND it is not a new node 
	  // AND its not the root node
          // -> expand pruned node
          node->expandNode();
          this->tree_size+=8;
          this->sizeChanged = true;

        }
        else {
          // not a pruned node, create requested child
          node->createChild(pos);
          this->tree_size++;
          this->sizeChanged = true;
        }
        created_node = true;
      }
      NODE* retval = updateNodeRecurs(node->getChild(pos), created_node, 
                                      key, depth+1, occupied);

      // set own probability according to prob of children
      node->updateOccupancyChildren(); 

      return retval;
    }

    // at last level, update node, end of recursion
    else {
      if (occupied) node->integrateHit();
      else          node->integrateMiss();
      return node;
    }
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                                                    unsigned int depth, const float& log_odds_update) {
    unsigned int pos (0);
    this->genPos(key, this->tree_depth-1-depth, pos);
    bool created_node = false;

    // follow down to last level
    if (depth < this->tree_depth) {
      if (!node->childExists(pos)) {
        // child does not exist, but maybe it's a pruned node?
        if ((!node->hasChildren()) && !node_just_created && (node != this->itsRoot)) {
          // current node does not have children AND it is not a new node 
	  // AND its not the root node
          // -> expand pruned node
          node->expandNode();
          this->tree_size+=8;
          this->sizeChanged = true;

        }
        else {
          // not a pruned node, create requested child
          node->createChild(pos);
          this->tree_size++;
          this->sizeChanged = true;
        }
        created_node = true;
      }
      NODE* retval = updateNodeRecurs(node->getChild(pos), created_node, 
                                      key, depth+1, log_odds_update);

      // set own probability according to prob of children
      node->updateOccupancyChildren(); 
      return retval;
    }

    // at last level, update node, end of recursion
    else {
      node->setLogOdds(node->getLogOdds() + log_odds_update);
      return node;
    }
  }

  
  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::castRay(const point3d& origin, const point3d& directionP, point3d& end, 
                                          bool ignoreUnknown, double maxRange) const {

    /// ----------  see OcTreeBase::computeRayKeys  -----------

    // Initialization phase -------------------------------------------------------
    OcTreeKey current_key;
    if ( !OcTreeBase<NODE>::genKey(origin, current_key) ) {
      std::cerr << "WARNING: coordinates out of bounds during ray casting" << std::endl;
      return false;
    }

    NODE* startingNode = this->search(current_key);
    if (startingNode){
      if (startingNode->isOccupied()){
        // Occupied node found at origin 
        end = origin;
        return true;
      }
    } else if(!ignoreUnknown){
      std::cerr << "ERROR: Origin node at " << origin << " for raycasting not found, does the node exist?\n";
      return false;
    }

    point3d direction = directionP.normalized ();
    bool max_range_set = (maxRange > 0.);

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
        float voxelBorder(0);
        this->genCoordFromKey(current_key[i], voxelBorder); // negative corner point of voxel
        if (step[i] > 0) voxelBorder += this->resolution;   // positive corner point of voxel

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    // for speedup:
    point3d origin_scaled = origin;  
    origin_scaled /= this->resolution;  
    double maxrange_2 = maxRange / this->resolution;  // scale
    maxrange_2 = maxrange_2*maxrange_2; // squared dist
    double res_2 = this->resolution/2.;
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

      assert (current_key[dim] < 2*this->tree_max_val);

      // generate world coords from key
      point3d current_endpoint;
      double dist_from_origin(0);
      for (unsigned int j = 0; j < 3; j++) {
        double coord = (double) current_key[j] - (double) this->tree_max_val + res_2; // center of voxel
        dist_from_origin += (coord - origin_scaled(j)) * (coord - origin_scaled(j));
        current_endpoint(j) = coord * this->resolution;
      }

      if (max_range_set && (dist_from_origin > maxrange_2) ) { // reached user specified maxrange
        //         end = current_endpoint;
        done = true;
        return false;
      }

      NODE* currentNode = this->search(current_key);
      if ( currentNode){
        if (currentNode->isOccupied()) {
          end = current_endpoint;
          done = true;
          break;
        }
        // otherwise: node is free and valid, raycasting continues
      } 
      
      else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
        std::cerr << "Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end << std::endl;
        return false;
      }
    } // end while

    return true;
  }


  template <class NODE> inline bool 
  OccupancyOcTreeBase<NODE>::integrateMissOnRay(const point3d& origin, const point3d& end) {

    if (!this->computeRayKeys(origin, end, this->keyray)) {
      return false;
    }
    
    for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
      updateNode(*it, false); // insert freespace measurement
    }
  
    return true;
  }



  template <class NODE> bool 
  OccupancyOcTreeBase<NODE>::insertRay(const point3d& origin, const point3d& end, double maxrange)
  {
    // cut ray at maxrange
    if ((maxrange > 0) && ((end - origin).norm () > maxrange)) 
      {
        point3d direction = (end - origin).normalized ();
        point3d new_end = origin + direction * maxrange;
        return integrateMissOnRay(origin, new_end);
      }
    // insert complete ray
    else 
      {
        if (!integrateMissOnRay(origin, end)) 
          return false;
        updateNode(end, true); // insert hit cell
        return true;
      }
  }
  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupied(point3d_list& node_centers, unsigned int max_depth) const {

    if (max_depth == 0)  max_depth = this->tree_depth;

    OcTreeKey root_key;
    root_key[0] = root_key[1] = root_key[2] = this->tree_max_val;
    getOccupiedRecurs(node_centers, max_depth, this->itsRoot, 0, root_key);    
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupied(std::list<OcTreeVolume>& occupied_nodes, unsigned int max_depth) const{

    if (max_depth == 0)  max_depth = this->tree_depth;

    std::list<OcTreeVolume> delta_nodes;
    getOccupied(occupied_nodes, delta_nodes, max_depth);
    occupied_nodes.insert(occupied_nodes.end(), delta_nodes.begin(), delta_nodes.end());
  }

  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupied(std::list<OcTreeVolume>& binary_nodes,
                                              std::list<OcTreeVolume>& delta_nodes,
                                              unsigned int max_depth) const{
    
    if (max_depth == 0)
      max_depth = this->tree_depth;
   
    OcTreeKey root_key;
    root_key[0] = root_key[1] = root_key[2] = this->tree_max_val;
    
    getOccupiedRecurs(binary_nodes, delta_nodes, max_depth, this->itsRoot, 0, root_key);
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupiedRecurs( point3d_list& node_centers,
                                                     unsigned int max_depth,
                                                     NODE* node, unsigned int depth,
                                                     const OcTreeKey& parent_key) const {

    if (depth < max_depth && node->hasChildren()) {

      unsigned short int center_offset_key = this->tree_max_val >> (depth + 1);
      OcTreeKey search_key;

      for (unsigned int i=0; i<8; ++i) {
        if (node->childExists(i)) {
          OcTreeBase<NODE>::computeChildKey(i, center_offset_key, parent_key, search_key);
          getOccupiedRecurs(node_centers, max_depth, node->getChild(i), depth+1, search_key);
        }
      }
    }

    else { // max level reached

      if (node->isOccupied()) {
        point3d p;
	this->genCoords(parent_key, depth, p);
        node_centers.push_back(p);        
      }
    }
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupiedRecurs( std::list<OcTreeVolume>& binary_nodes,
                                                     std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth,
                                                     NODE* node, unsigned int depth,
                                                     const OcTreeKey& parent_key) const {

    if (depth < max_depth && node->hasChildren()) {

      unsigned short int center_offset_key = this->tree_max_val >> (depth + 1);
      OcTreeKey search_key;

      for (unsigned int i=0; i<8; ++i) {
        if (node->childExists(i)) {
          OcTreeBase<NODE>::computeChildKey(i, center_offset_key, parent_key, search_key);
          getOccupiedRecurs(binary_nodes, delta_nodes, max_depth, node->getChild(i), depth+1, search_key);
        }
      }
    }

    else { // max level reached

      if (node->isOccupied()) {

        OcTreeVolume cell;

        cell.second = this->resolution * double(1 << (this->tree_depth - depth));
	this->genCoords(parent_key, depth, cell.first);

        if (!node->atThreshold()) {
          delta_nodes.push_back(cell);
        }
        else {
          binary_nodes.push_back(cell);
        }
      }
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getFreespace(std::list<OcTreeVolume>& free_nodes, unsigned int max_depth) const{
    std::list<OcTreeVolume> delta_nodes;

    getFreespace(free_nodes, delta_nodes, max_depth);
    free_nodes.insert(free_nodes.end(), delta_nodes.begin(), delta_nodes.end());
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getFreespace(std::list<OcTreeVolume>& binary_nodes,
                                               std::list<OcTreeVolume>& delta_nodes,
                                               unsigned int max_depth) const{

    if (max_depth == 0)
      max_depth = this->tree_depth;

    this->getFreespaceRecurs(binary_nodes, delta_nodes, max_depth,  this->itsRoot, 0, this->tree_center);
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getFreespaceRecurs(std::list<OcTreeVolume>& binary_nodes,
                                                     std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth,
                                                     NODE* node, unsigned int depth, const point3d& parent_center) const{

    if (depth < max_depth && node->hasChildren()) {

      double center_offset = this->tree_center(0) / pow( 2., (double) depth+1);
      point3d search_center;

      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {

          OcTreeBase<NODE>::computeChildCenter(i, center_offset, parent_center, search_center);
          getFreespaceRecurs(binary_nodes, delta_nodes, max_depth, node->getChild(i), depth+1, search_center);

        } // GetChild
      } // depth
    }
    else {    // max level reached

      if (!node->isOccupied()) {
        double voxelSize = this->resolution * pow(2., double(this->tree_depth - depth));
        if (!node->atThreshold()) {
          delta_nodes.push_back(std::make_pair<point3d, double>(parent_center - this->tree_center, voxelSize));
        }
        else {
          binary_nodes.push_back(std::make_pair<point3d, double>(parent_center - this->tree_center, voxelSize));
        }
      }
      
    }
  }
  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::setBBXMin (point3d& min) { 
    bbx_min = min; 
    if (!this->genKey(bbx_min, bbx_min_key)) {
      fprintf(stderr, "ERROR while generating bbx min key.\n");
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::setBBXMax (point3d& max) {
    bbx_max = max; 
    if (!this->genKey(bbx_max, bbx_max_key)) {
      fprintf(stderr, "ERROR while generating bbx max key.\n");
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



  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupiedLeafsBBX(point3d_list& node_centers, point3d min, point3d max) const {

    OcTreeKey root_key, min_key, max_key;
    root_key[0] = root_key[1] = root_key[2] = this->tree_max_val; 
    if (!this->genKey(min, min_key)) return;
    if (!this->genKey(max, max_key)) return;
    getOccupiedLeafsBBXRecurs(node_centers, this->tree_depth, this->itsRoot, 0, root_key, min_key, max_key);
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupiedLeafsBBXRecurs( point3d_list& node_centers, unsigned int max_depth, 
                                                             NODE* node, unsigned int depth, const OcTreeKey& parent_key, 
                                                             const OcTreeKey& min, const OcTreeKey& max) const {

    if (depth == max_depth) { // max level reached
      if (node->isOccupied()) {
        point3d p;
        this->genCoords(parent_key, depth, p);
        node_centers.push_back(p);        
      }
    }

    if (!node->hasChildren()) return;

    unsigned short int center_offset_key = this->tree_max_val >> (depth + 1);

    OcTreeKey child_key;

    for (unsigned int i=0; i<8; ++i) {
      if (node->childExists(i)) {

        OcTreeBase<NODE>::computeChildKey(i, center_offset_key, parent_key, child_key);

        // overlap of query bbx and child bbx?
        if (!( 
              ( min[0] > (child_key[0] + center_offset_key)) ||
              ( max[0] < (child_key[0] - center_offset_key)) ||
              ( min[1] > (child_key[1] + center_offset_key)) ||
              ( max[1] < (child_key[1] - center_offset_key)) ||
              ( min[2] > (child_key[2] + center_offset_key)) ||
              ( max[2] < (child_key[2] - center_offset_key))
               )) {
          getOccupiedLeafsBBXRecurs(node_centers, max_depth, node->getChild(i), depth+1, child_key, min, max);
        }
      }
    }
  }



} // namespace



