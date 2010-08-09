// $Id: OccupancyOcTreeBase.hxx 124 2010-08-05 13:00:15Z kai_wurm $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
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


namespace octomap {


  template <class NODE>
  OccupancyOcTreeBaseSE<NODE>::OccupancyOcTreeBaseSE(double _resolution)
    : OcTreeBaseSE<NODE>(_resolution)
  {

  }


  template <class NODE>
  OccupancyOcTreeBaseSE<NODE>::~OccupancyOcTreeBaseSE(){
  }



  template <class NODE>
  NODE* OccupancyOcTreeBaseSE<NODE>::updateNode(const point3d& value, bool occupied) {

    OcTreeKey key;
    if (!this->genKey(value, key)) return NULL;

    return updateNode(key, occupied);
  }


  template <class NODE>
  NODE* OccupancyOcTreeBaseSE<NODE>::updateNode(const OcTreeKey& key, bool occupied) {

    NODE* leaf = this->searchKey(key);
    if (leaf) {
      if ((leaf->atThreshold()) && (leaf->isOccupied() == occupied)) {
        return leaf;
      }
    }

    return updateNodeRecurs(this->itsRoot, false, key, 0, occupied);
  }

  

  template <class NODE>
  NODE* OccupancyOcTreeBaseSE<NODE>::updateNodeRecurs(NODE* node, bool node_just_created,
                                           const OcTreeKey& key, unsigned int depth,
                                           bool occupied) {


    unsigned int pos = this->genPos(key, this->tree_depth-1-depth);
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
  bool OccupancyOcTreeBaseSE<NODE>::castRay(const point3d& origin, const point3d& directionP, 
                                          point3d& end, bool ignoreUnknown, double maxRange) const {
    
    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    // Initialization phase -------------------------------------------------------
    end = point3d(0.0, 0.0, 0.0);

    point3d direction = directionP.unit();

    NODE* startingNode = this->search(origin);
    if (startingNode){
      if (startingNode->isOccupied()){
        // Occupied node => ray hits starting node (at d=0)
        end = origin;
        return true;
      }
    } else if(!ignoreUnknown){
      std::cerr << "ERROR: Origin node at " << origin << " for raycasting not found, does the node exist?\n";
      return false;
    }


    // Voxel integer coordinates are the indices of the OcTree cells
    // at the lowest level (they may exist or not).

    unsigned short int voxelIdx[3];  // voxel integer coords
    int step[3];                     // step direction

    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      if (!this->genKeyValue(origin(i), voxelIdx[i])) {
        std::cerr << "Error in OcTree::computeRay(): Coordinate "<<i<<" of origin out of OcTree bounds: "<< origin(i)<<"\n";
        return false;
      }

      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      double voxelBorder = (double) ( (int) voxelIdx[i] - (int) this->tree_max_val ) * this->resolution;
      if (step[i] > 0) voxelBorder += this->resolution;

      if (direction(i) != 0.0) {
        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] = 1e6;
        tDelta[i] = 1e6;
      }
    }

    // Incremental phase  ---------------------------------------------------------

    bool done = false;
    while (!done) {
      unsigned int i;

      // find minimum tMax:
      if (tMax[0] < tMax[1]){
        if (tMax[0] < tMax[2]) i = 0;
        else                   i = 2;
      }
      else {
        if (tMax[1] < tMax[2]) i = 1;
        else                   i = 2;
      }

      // advance in direction "i":
      voxelIdx[i] += step[i];

      if ((voxelIdx[i] >= 0) && (voxelIdx[i] < 2*this->tree_max_val)){
        tMax[i] += tDelta[i];
      }
      else {
        std::cerr << "WARNING: Ray casting in OcTree::castRay() hit the boundary in dim. "<< i << std::endl;
        return false;
      }

      // generate world coords from tree indices
      double val[3];
      for (unsigned int j = 0; j < 3; j++) {
        if(!this->genCoordFromKey( voxelIdx[j], val[j] )){
          std::cerr << "Error in OcTree::castRay(): genVal failed!\n";
          return false;
        }
      }
      end = point3d(val[0], val[1], val[2]);
      // reached maxRange?
      if (maxRange > 0 && (end - origin).norm2() > maxRange) {
        return false;
      }

      NODE* currentNode = this->search(end);
      if ( currentNode){
        if (currentNode->isOccupied())
          done = true;

        // otherwise: node is free and valid, raycast continues
      } else if (!ignoreUnknown){ // no node found, this usually means we are in "unknown" areas
        std::cerr << "Search failed in OcTree::castRay() => an unknown area was hit in the map: "
                  << end << std::endl;
        return false;
      }


    } // end while

    return true;
  }


//   template <class NODE>
//   void OccupancyOcTreeBaseSE<NODE>::integrateMissOnRay(const point3d& origin, const point3d& end) {

//     std::vector<point3d> ray;
//     if (this->computeRay(origin, end, ray)){

//       for(std::vector<point3d>::iterator it=ray.begin(); it != ray.end(); it++) {
//         //      std::cout << "miss cell " << *it << std::endl;
//         updateNode(*it, false); // insert miss cell
//       }
//     }

//   }


  template <class NODE>
  bool OccupancyOcTreeBaseSE<NODE>::integrateMissOnRay(const point3d& origin, const point3d& end) {

    if (!this->computeRayKeys(origin, end, this->keyray)) {
      return false;
    }
    
    for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
      updateNode(*it, false); // insert freespace measurement
    }
  
    return true;
  }


  template <class NODE>
  bool OccupancyOcTreeBaseSE<NODE>::insertRay(const point3d& origin, const point3d& end, double maxrange){

    // cut ray at maxrange
    if ((maxrange > 0) 
        && ((end - origin).norm2() > maxrange)) {

      point3d direction = (end - origin).unit();
      point3d new_end = origin + direction * maxrange;
      return integrateMissOnRay(origin, new_end);
    }
    // insert complete ray
    else {
      if (!integrateMissOnRay(origin, end)) return false;
      updateNode(end, true); // insert hit cell
      return true;
    }
  }


  template <class NODE>
  void OccupancyOcTreeBaseSE<NODE>::getOccupied(std::list<OcTreeVolume>& occupied_nodes, unsigned int max_depth) const{
    std::list<OcTreeVolume> delta_nodes;

    getOccupied(occupied_nodes, delta_nodes, max_depth);
    occupied_nodes.insert(occupied_nodes.end(), delta_nodes.begin(), delta_nodes.end());
  }

  
  template <class NODE>
  void OccupancyOcTreeBaseSE<NODE>::getOccupied(std::list<OcTreeVolume>& binary_nodes,
                                     std::list<OcTreeVolume>& delta_nodes,
                                     unsigned int max_depth) const{

    if (max_depth == 0)
      max_depth = this->tree_depth;

    this->getOccupiedRecurs(binary_nodes, delta_nodes, max_depth, this->itsRoot, 0, this->tree_center);
  }


  template <class NODE>
  void OccupancyOcTreeBaseSE<NODE>::getOccupiedRecurs( std::list<OcTreeVolume>& binary_nodes,
                                            std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth,
                                            NODE* node, unsigned int depth,
                                            const point3d& parent_center) const {

    if (depth < max_depth && node->hasChildren()) {

      double center_offset = this->tree_center(0) / pow( 2., (double) depth+1);
      point3d search_center;

      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {

          // x-axis
          if (i & 1)  search_center(0) = parent_center(0) + center_offset;
          else        search_center(0) = parent_center(0) - center_offset;

          // y-axis
          if (i & 2)  search_center(1) = parent_center(1) + center_offset;
          else        search_center(1) = parent_center(1) - center_offset;
          // z-axis
          if (i & 4)  search_center(2) = parent_center(2) + center_offset;
          else        search_center(2) = parent_center(2) - center_offset;

          getOccupiedRecurs(binary_nodes, delta_nodes, max_depth,  node->getChild(i), depth+1, search_center);
        }
      }
    }

    else { // max level reached

      if (node->isOccupied()) {
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
  void OccupancyOcTreeBaseSE<NODE>::getFreespace(std::list<OcTreeVolume>& free_nodes, unsigned int max_depth) const{
    std::list<OcTreeVolume> delta_nodes;

    getFreespace(free_nodes, delta_nodes, max_depth);
    free_nodes.insert(free_nodes.end(), delta_nodes.begin(), delta_nodes.end());
  }

  template <class NODE>
  void OccupancyOcTreeBaseSE<NODE>::getFreespace(std::list<OcTreeVolume>& binary_nodes,
                                      std::list<OcTreeVolume>& delta_nodes,
                                      unsigned int max_depth) const{

    if (max_depth == 0)
      max_depth = this->tree_depth;

    this->getFreespaceRecurs(binary_nodes, delta_nodes, max_depth,  this->itsRoot, 0, this->tree_center);
  }

  template <class NODE>
  void OccupancyOcTreeBaseSE<NODE>::getFreespaceRecurs(std::list<OcTreeVolume>& binary_nodes,
                                            std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth,
                                            NODE* node, unsigned int depth, const point3d& parent_center) const{

    if (depth < max_depth && node->hasChildren()) {

      double center_offset = this->tree_center(0) / pow( 2., (double) depth+1);
      point3d search_center;

      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {

          // x-axis
          if (i & 1)  search_center(0) = parent_center(0) + center_offset;
          else        search_center(0) = parent_center(0) - center_offset;

          // y-axis
          if (i & 2)  search_center(1) = parent_center(1) + center_offset;
          else        search_center(1) = parent_center(1) - center_offset;
          // z-axis
          if (i & 4)  search_center(2) = parent_center(2) + center_offset;
          else        search_center(2) = parent_center(2) - center_offset;

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


}
