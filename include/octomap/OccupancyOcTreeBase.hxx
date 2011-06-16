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
    // some sane default values:
    setOccupancyThres(0.5);   // = 0.0 in logodds
    setProbHit(0.7);          // = 0.85 in logodds
    setProbMiss(0.4);         // = -0.4 in logodds

    setClampingThresMin(0.1192); // = -2 in log odds
    setClampingThresMax(0.971); // = 3.5 in log odds

  }


  template <class NODE>
  OccupancyOcTreeBase<NODE>::~OccupancyOcTreeBase(){
  }



  // performs transformation to data and sensor origin first
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScan(const ScanNode& scan, double maxrange, bool pruning, bool dirty) {
    Pointcloud& cloud = *(scan.scan);
    pose6d frame_origin = scan.pose;
    point3d sensor_origin = frame_origin.inv().transform(scan.pose.trans());
    insertScan(cloud, sensor_origin, frame_origin, maxrange, pruning, dirty);
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
                                             double maxrange, bool pruning, bool dirty) {

    UpdateList free_cells, occupied_cells;
    computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);    

    // insert data into tree  -----------------------
    for (UpdateList::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
      updateNode(*it, false, dirty);
    }
    for (UpdateList::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
      updateNode(*it, true, dirty);
    }

    // TODO: does pruning make sense if tree is left "dirty"?
    if (pruning) this->prune();
  } 


  // performs transformation to data and sensor origin first
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::insertScan(const Pointcloud& pc, const point3d& sensor_origin, const pose6d& frame_origin, 
                                             double maxrange, bool pruning, bool dirty) {
    Pointcloud transformed_scan (pc);
    transformed_scan.transform(frame_origin);
    point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
    insertScan(transformed_scan, transformed_sensor_origin, maxrange, pruning, dirty);
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
                                                UpdateList& free_cells,
                                                UpdateList& occupied_cells,
                                                double maxrange) {

    for (Pointcloud::const_iterator point_it = scan.begin(); point_it != scan.end(); point_it++) {
      const point3d& p = *point_it;


      if (!use_bbx_limit) {

        // -------------- no BBX specified ---------------

        if ((maxrange < 0.0) || ((p - origin).norm() <= maxrange) ) { // is not maxrange meas.
          // free cells
          if (this->computeRayKeys(origin, p, this->keyray)){
            free_cells.insert(this->keyray.begin(), this->keyray.end());
          }
          // occupied endpoint
          OcTreeKey key;
          if (this->genKey(p, key))
            occupied_cells.insert(key);
        } // end if NOT maxrange

        else { // user set a maxrange and this is reached
          point3d direction = (p - origin).normalized ();
          point3d new_end = origin + direction * maxrange;
          if (this->computeRayKeys(origin, new_end, this->keyray)){
            free_cells.insert(this->keyray.begin(), this->keyray.end());
          }
        } // end if maxrange
      }

      else {
        // --- update limited by user specified BBX  -----

        // endpoint in bbx and not maxrange?
        if ( inBBX(p) && ((maxrange < 0.0) || ((p - origin).norm () <= maxrange) ) )  {

          // occupied endpoint
          OcTreeKey key;
          if (this->genKey(p, key))
            occupied_cells.insert(key);

          // update freespace, break as soon as bbx limit is reached
          if (this->computeRayKeys(origin, p, this->keyray)){
            for(KeyRay::reverse_iterator rit=this->keyray.rbegin(); rit != this->keyray.rend(); rit++) {
              if (inBBX(*rit)) {
                free_cells.insert(*rit);
              }
              else break;
            }
          }

        }
        } // end bbx case

    } // end for all points

    for(UpdateList::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ){
      if (occupied_cells.find(*it) != occupied_cells.end()){
        it = free_cells.erase(it);
      } else{
        ++it;
      }
    }
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const point3d& value, bool occupied, bool dirty) {

    OcTreeKey key;
    if (!this->genKey(value, key)) return NULL;
    return updateNode(key, occupied, dirty);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const point3d& value, float log_odds_update, bool dirty) {

    OcTreeKey key;
    if (!this->genKey(value, key)) return NULL;
    return updateNode(key, log_odds_update, dirty);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const OcTreeKey& key, bool occupied, bool dirty) {

    NODE* leaf = this->search(key);
    if (leaf) {
      if ((isNodeAtThreshold(leaf)) && (isNodeOccupied(leaf) == occupied)) {
        return leaf;
      }
    }
    return updateNodeRecurs(this->itsRoot, false, key, 0, occupied, dirty);
  }

  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNode(const OcTreeKey& key, float log_odds_update, bool dirty) {
    return updateNodeRecurs(this->itsRoot, false, key, 0, log_odds_update, dirty);
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNodeRecurs(NODE* node, bool node_just_created,
                                                    const OcTreeKey& key, unsigned int depth,
                                                    bool occupied, bool dirty) {

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
      if (dirty)
        return updateNodeRecurs(node->getChild(pos), created_node, key, depth+1, occupied, dirty);
      else{
        NODE* retval = updateNodeRecurs(node->getChild(pos), created_node, key, depth+1, occupied, dirty);
        // set own probability according to prob of children
        node->updateOccupancyChildren();
        return retval;
      }

    }

    // at last level, update node, end of recursion
    else {
      if (occupied) integrateHit(node);
      else          integrateMiss(node);
      return node;
    }
  }


  template <class NODE>
  NODE* OccupancyOcTreeBase<NODE>::updateNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                                                    unsigned int depth, const float& log_odds_update, bool dirty) {
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

      if (dirty)
        return updateNodeRecurs(node->getChild(pos), created_node, key, depth+1, log_odds_update, dirty);
      else{
        NODE* retval = updateNodeRecurs(node->getChild(pos), created_node, key, depth+1, log_odds_update, dirty);
        // set own probability according to prob of children
        node->updateOccupancyChildren();
        return retval;
      }
    }

    // at last level, update node, end of recursion
    else {
      node->setLogOdds(node->getLogOdds() + log_odds_update);
      return node;
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::updateInnerOccupancy(){
    this->updateInnerOccupancyRecurs(this->itsRoot, 0);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::updateInnerOccupancyRecurs(NODE* node, unsigned int depth){
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

    // convert bottom up
    for (unsigned int depth=this->tree_depth; depth>0; depth--) {
      toMaxLikelihoodRecurs(this->itsRoot, 0, depth);
    }

    // convert root
    nodeToMaxLikelihood(this->itsRoot);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::toMaxLikelihoodRecurs(NODE* node, unsigned int depth,
      unsigned int max_depth) {

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
  bool OccupancyOcTreeBase<NODE>::castRay(const point3d& origin, const point3d& directionP, point3d& end, 
                                          bool ignoreUnknown, double maxRange) const {

    /// ----------  see OcTreeBase::computeRayKeys  -----------

    // Initialization phase -------------------------------------------------------
    OcTreeKey current_key;
    if ( !OcTreeBase<NODE>::genKey(origin, current_key) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }

    NODE* startingNode = this->search(current_key);
    if (startingNode){
      if (isNodeOccupied(startingNode)){
        // Occupied node found at origin 
        end = origin;
        return true;
      }
    } else if(!ignoreUnknown){
      OCTOMAP_ERROR_STR("Origin node at " << origin << " for raycasting not found, does the node exist?");
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
        if (isNodeOccupied(currentNode)) {
          end = current_endpoint;
          done = true;
          break;
        }
        // otherwise: node is free and valid, raycasting continues
      } 
      
      else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
        OCTOMAP_WARNING_STR("Search failed in OcTree::castRay() => an unknown area was hit in the map: " << end);
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
    for(typename OccupancyOcTreeBase<NODE>::leaf_iterator it = this->begin(max_depth),
        end=this->end(); it!= end; ++it)
    {
      if(this->isNodeOccupied(*it))
        node_centers.push_back(it.getCoordinate());
    }
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupied(std::list<OcTreeVolume>& occupied_nodes, unsigned int max_depth) const{

    if (max_depth == 0)  max_depth = this->tree_depth;

    for(typename OccupancyOcTreeBase<NODE>::leaf_iterator it = this->begin(max_depth),
            end=this->end(); it!= end; ++it)
    {
      if(this->isNodeOccupied(*it))
        occupied_nodes.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
    }

  }

  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getOccupied(std::list<OcTreeVolume>& binary_nodes,
                                              std::list<OcTreeVolume>& delta_nodes,
                                              unsigned int max_depth) const{
    
    if (max_depth == 0)  max_depth = this->tree_depth;

    for(typename OccupancyOcTreeBase<NODE>::leaf_iterator it = this->begin(max_depth),
            end=this->end(); it!= end; ++it)
    {
      if(this->isNodeOccupied(*it)){
        if (it->getLogOdds() >= clampingThresMax)
          binary_nodes.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
        else
          delta_nodes.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
      }
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getFreespace(std::list<OcTreeVolume>& free_nodes, unsigned int max_depth) const{

    if (max_depth == 0)  max_depth = this->tree_depth;

    for(typename OccupancyOcTreeBase<NODE>::leaf_iterator it = this->begin(max_depth),
            end=this->end(); it!= end; ++it)
    {
      if(!this->isNodeOccupied(*it))
        free_nodes.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
    }
  }


  template <class NODE>
  void OccupancyOcTreeBase<NODE>::getFreespace(std::list<OcTreeVolume>& binary_nodes,
                                               std::list<OcTreeVolume>& delta_nodes,
                                               unsigned int max_depth) const{

    if (max_depth == 0)  max_depth = this->tree_depth;

    for(typename OccupancyOcTreeBase<NODE>::leaf_iterator it = this->begin(max_depth),
            end=this->end(); it!= end; ++it)
    {
      if(!this->isNodeOccupied(*it)){
        if (it->getLogOdds() <= clampingThresMin)
          binary_nodes.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
        else
          delta_nodes.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
      }
    }
  }

  
  template <class NODE>
  void OccupancyOcTreeBase<NODE>::setBBXMin (point3d& min) { 
    bbx_min = min; 
    if (!this->genKey(bbx_min, bbx_min_key)) {
      OCTOMAP_ERROR("ERROR while generating bbx min key.\n");
    }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::setBBXMax (point3d& max) {
    bbx_max = max; 
    if (!this->genKey(bbx_max, bbx_max_key)) {
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
      if (isNodeOccupied(node)) {
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

        computeChildKey(i, center_offset_key, parent_key, child_key);

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

  // -- I/O  -----------------------------------------

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::readBinary(const std::string& filename){
    std::ifstream binary_infile( filename.c_str(), std::ios_base::binary);
    if (!binary_infile.is_open()){
      OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing read.");
      return;
    } else {
      readBinary(binary_infile);
      binary_infile.close();
    }
  }

  template <class NODE>
  std::istream& OccupancyOcTreeBase<NODE>::readBinary(std::istream &s) {

	  if (!s.good()){
	    OCTOMAP_WARNING_STR("Input filestream not \"good\" in OcTree::readBinary");
	  }

	  int tree_type = -1;
	  s.read((char*)&tree_type, sizeof(tree_type));
	  // TODO Treetype checks disabled, do they make any sense here?
	  if (tree_type == 3){

		  this->tree_size = 0;
		  this->sizeChanged = true;

		  // clear tree if there are nodes
		  if (this->itsRoot->hasChildren()) {
			  delete this->itsRoot;
			  this->itsRoot = new NODE();
		  }

		  double tree_resolution;
		  s.read((char*)&tree_resolution, sizeof(tree_resolution));

		  this->setResolution(tree_resolution);

		  unsigned int tree_read_size = 0;
		  s.read((char*)&tree_read_size, sizeof(tree_read_size));
		  OCTOMAP_DEBUG_STR("Reading " << tree_read_size << " nodes from bonsai tree file...");

		  this->itsRoot->readBinary(s);
		  // TODO workaround: do this to ensure all nodes have the tree's occupancy thres
		  // instead of their fixed ones
		  toMaxLikelihood();
	    this->sizeChanged = true;
		  this->tree_size = OcTreeBase<NODE>::calcNumNodes();  // compute number of nodes

		  OCTOMAP_DEBUG_STR("done.");
	  } else{
	    OCTOMAP_ERROR_STR("Binary file does not contain an OcTree!");
	  }


	  return s;
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::writeBinary(const std::string& filename){
	  std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

	  if (!binary_outfile.is_open()){
	    OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
		  return;
	  } else {
		  writeBinary(binary_outfile);
		  binary_outfile.close();
	  }
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::writeBinaryConst(const std::string& filename) const{
	  std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

	  if (!binary_outfile.is_open()){
	    OCTOMAP_ERROR_STR("Filestream to "<< filename << " not open, nothing written.");
		  return;
	  }
	  else {
		  writeBinaryConst(binary_outfile);
		  binary_outfile.close();
	  }
  }

  template <class NODE>
  std::ostream& OccupancyOcTreeBase<NODE>::writeBinary(std::ostream &s){

	  // format:    treetype | resolution | num nodes | [binary nodes]

	  //    this->toMaxLikelihood();  (disabled, not necessary, KMW)
	  this->prune();

	  return writeBinaryConst(s);
  }

  template <class NODE>
  std::ostream& OccupancyOcTreeBase<NODE>::writeBinaryConst(std::ostream &s) const{

	  // format:    treetype | resolution | num nodes | [binary nodes]

    // TODO: Fix constant use => one for all occupancy trees?
	  unsigned int tree_type = 3;
	  s.write((char*)&tree_type, sizeof(tree_type));

	  double tree_resolution = this->resolution;
	  s.write((char*)&tree_resolution, sizeof(tree_resolution));

	  unsigned int tree_write_size = this->size();
	  OCTOMAP_DEBUG_STR("Writing " << tree_write_size << " nodes to output stream...");
	  s.write((char*)&tree_write_size, sizeof(tree_write_size));

	  this->itsRoot->writeBinary(s);
	  OCTOMAP_DEBUG_STR(" done.");
	  return s;
  }

  /*
   * Experimental stuff:
   */
  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::isNodeOccupied(NODE* occupancyNode) const{
    return (occupancyNode->getLogOdds() >= occProbThresLog);
  }
  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::isNodeOccupied(const NODE& occupancyNode) const{
    return (occupancyNode.getLogOdds() >= occProbThresLog);
  }

  template <class NODE>
  bool OccupancyOcTreeBase<NODE>::isNodeAtThreshold(NODE* occupancyNode) const{
	  return (occupancyNode->getLogOdds() >= clampingThresMax
	      || occupancyNode->getLogOdds() <= clampingThresMin);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::integrateHit(NODE* occupancyNode) const{
    occupancyNode->addValue(probHitLog);

    // assuming a hit can only increase a node's value:
    if (occupancyNode->getLogOdds() > clampingThresMax)
      occupancyNode->setLogOdds(clampingThresMax);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::integrateMiss(NODE* occupancyNode) const{
    occupancyNode->addValue(probMissLog);

    // assuming a miss can only decrease a node's value:
    if (occupancyNode->getLogOdds() < clampingThresMin)
      occupancyNode->setLogOdds(clampingThresMin);
  }

  template <class NODE>
  void OccupancyOcTreeBase<NODE>::nodeToMaxLikelihood(NODE* occupancyNode) const{
    if (isNodeOccupied(occupancyNode))
      occupancyNode->setLogOdds(clampingThresMax);
    else
      occupancyNode->setLogOdds(clampingThresMin);

  }

} // namespace



