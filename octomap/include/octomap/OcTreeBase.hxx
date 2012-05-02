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

#undef max
#undef min
#include <limits>

namespace octomap {


  /*template <class NODE>
  const OcTreeBase<NODE>::leaf_iterator OcTreeBase<NODE>::leaf_iterator_end = OcTreeBase<NODE>::leaf_iterator();*/

  template <class NODE>
  OcTreeBase<NODE>::OcTreeBase(double _resolution) :
    itsRoot(NULL), tree_depth(16), tree_max_val(32768), 
    resolution(_resolution), tree_size(0) {
    
    this->setResolution(_resolution);
    for (unsigned i = 0; i< 3; i++){
      maxValue[i] = -(std::numeric_limits<double>::max( ));
      minValue[i] = std::numeric_limits<double>::max( );
    }
    sizeChanged = true;
  }


  template <class NODE>
  OcTreeBase<NODE>::~OcTreeBase(){
    delete itsRoot;
  }

  template <class NODE>
  void OcTreeBase<NODE>::setResolution(double r) {
    resolution = r;
    resolution_factor = 1. / resolution;

    tree_center(0) = tree_center(1) = tree_center(2) 
      = (float) (((double) tree_max_val) / resolution_factor);

    // init node size lookup table:
    sizeLookupTable.resize(tree_depth+1);
    for(unsigned i = 0; i <= tree_depth; ++i){
      sizeLookupTable[i] = resolution * double(1 << (tree_depth - i));
    }
  }


  template <class NODE>
  bool OcTreeBase<NODE>::genKeyValue(double coordinate, unsigned short int& keyval) const {

    // scale to resolution and shift center for tree_max_val
    int scaled_coord =  ((int) floor(resolution_factor * coordinate)) + tree_max_val;

    // keyval within range of tree?
    if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2*tree_max_val))) {
      keyval = scaled_coord;
      return true;
    }
    return false;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genKey(const point3d& point, OcTreeKey& key) const{

    for (unsigned int i=0;i<3;i++) {
      if (!genKeyValue( point(i), key[i])) return false;
    }
    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genKeyValueAtDepth(const unsigned short int keyval, unsigned int depth, unsigned short int &out_keyval) const {

    if (keyval >= 2*tree_max_val)
      return false;
    
    unsigned int diff = tree_depth - depth;
    if(!diff) {
      out_keyval = keyval;
    }
    else {
      out_keyval = (((keyval-tree_max_val) >> diff) << diff) + (1 << (diff-1)) + tree_max_val;
    }
    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genKeyAtDepth(const OcTreeKey& key, unsigned int depth, OcTreeKey& out_key) const {
    for (unsigned int i=0;i<3;i++) {
      if (!genKeyValueAtDepth( key[i], depth, out_key[i])) return false;
    }
    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genCoordFromKey(const unsigned short int& key, float& coord, unsigned depth) const {
    if (key >= 2*tree_max_val)
      return false;
    coord = float(genCoordFromKey(key, depth));
    return true;
  }

  template <class NODE>
  double OcTreeBase<NODE>::genCoordFromKey(const unsigned short int& key, unsigned depth) const {
    assert(depth <= tree_depth);
   
    // root is centered on 0 = 0.0
    if (depth == 0) {
      return 0.0; 
    }
    else if (depth == tree_depth) {
      return (double( (int) key - (int) this->tree_max_val ) +0.5) * this->resolution; 
    }
    else {
      return (floor( (double(key)-double(this->tree_max_val)) /double(1 << (tree_depth - depth)) )  + 0.5 ) * this->getNodeSize(depth);
    }
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genCoords(const OcTreeKey& key, unsigned int depth, point3d& point) const {
    assert (depth <= tree_depth);

    for (unsigned int i=0; i<3; ++i) {
      if ( !genCoordFromKey(key[i], point(i), depth) ) {  
        // TODO someday: move content of call here
        return false;
      }
    }
    return true;
  }

  template <class NODE> void  // do not inline
  OcTreeBase<NODE>::genPos(const OcTreeKey& key, int depth, unsigned int& pos) const {
    pos = 0;
    if (key.k[0] & (1 << depth)) pos += 1;
    if (key.k[1] & (1 << depth)) pos += 2;
    if (key.k[2] & (1 << depth)) pos += 4;
  }


  template <class NODE> 
  void OcTreeBase<NODE>::computeChildCenter (const unsigned int& pos, 
                                             const float& center_offset, 
                                             const point3d& parent_center, 
                                             point3d& child_center) const {
    // x-axis
    if (pos & 1) child_center(0) = parent_center(0) + center_offset;
    else  	 child_center(0) = parent_center(0) - center_offset;

    // y-axis
    if (pos & 2) child_center(1) = parent_center(1) + center_offset;
    else	 child_center(1) = parent_center(1) - center_offset;
    // z-axis
    if (pos & 4) child_center(2) = parent_center(2) + center_offset;
    else	 child_center(2) = parent_center(2) - center_offset;
  }


  template <class NODE>
  NODE* OcTreeBase<NODE>::search(const point3d& value) const {

    // Search is a variant of insert which aborts if
    // it had to insert nodes

    OcTreeKey key;
    if (!genKey(value, key)){
      OCTOMAP_ERROR_STR("Error in search: ["<< value <<"] is out of OcTree bounds!");
      return NULL;
    }
    else {
      return this->search(key);
    }

  }

  template <class NODE>
  NODE* OcTreeBase<NODE>::search(float x, float y, float z) const {
    return this->search(point3d(x,y,z));
  }


  template <class NODE>
  NODE* OcTreeBase<NODE>::search (const OcTreeKey& key) const {

    NODE* curNode (itsRoot);
    unsigned int pos (0);

    // follow nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      genPos(key, i, pos);

      if (curNode->childExists(pos)) {
        // cast needed: (nodes need to ensure it's the right pointer)
        curNode = static_cast<NODE*>( curNode->getChild(pos) );
      }
      else {
        // we expected a child but did not get it
        // is the current node a leaf already?
        if (!curNode->hasChildren()) {
          return curNode;
        }
        else {
          // it is not, search failed
          return NULL;
        }
      }
    } // end for
    return curNode;
  }


  template <class NODE>
  bool OcTreeBase<NODE>::deleteNode(const point3d& value, unsigned int depth) {
    OcTreeKey key;
    if (!genKey(value, key)){
      OCTOMAP_ERROR_STR("Error in deleteNode: ["<< value <<"] is out of OcTree bounds!");
      return false;
    }
    else {
      return this->deleteNode(key, depth);
    }

  }

  template <class NODE>
  bool OcTreeBase<NODE>::deleteNode(float x, float y, float z, unsigned int depth) {
    return this->deleteNode(point3d(x,y,z), depth);
  }


  template <class NODE>
  bool OcTreeBase<NODE>::deleteNode(const OcTreeKey& key, unsigned int depth) {
    if (depth == 0)
      depth = tree_depth;

    return deleteNodeRecurs(itsRoot, 0, depth, key);
  }

  template <class NODE>
  void OcTreeBase<NODE>::clear() {
    // don't clear if the tree is empty:
    if (this->itsRoot->hasChildren()) {
      delete this->itsRoot;
      this->itsRoot = new NODE();
    }
    this->tree_size = 1;
    // max extent of tree changed:
    this->sizeChanged = true;
  }


  template <class NODE>
  void OcTreeBase<NODE>::prune() {
    for (unsigned int depth=tree_depth-1; depth>0; depth--) {
      unsigned int num_pruned = 0;
      pruneRecurs(this->itsRoot, 0, depth, num_pruned);
      if (num_pruned == 0) break;
    }
  }

  template <class NODE>
  void OcTreeBase<NODE>::expand() {
    expandRecurs(itsRoot,0, tree_depth);
  }

  template <class NODE>
  bool OcTreeBase<NODE>::computeRayKeys(const point3d& origin, 
                                          const point3d& end, 
                                          KeyRay& ray) const {

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    ray.reset();

    OcTreeKey key_origin, key_end;
    if ( !OcTreeBase<NODE>::genKey(origin, key_origin) || 
         !OcTreeBase<NODE>::genKey(end, key_end) ) {
      OCTOMAP_WARNING_STR("coordinates ( "
                << origin << " -> " << end << ") out of bounds in computeRayKeys");
      return false;
    }

    
    if (key_origin == key_end) return true; // same tree cell, we're done.

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
        float voxelBorder(0);
        this->genCoordFromKey(current_key[i], voxelBorder); 
        voxelBorder += (float) (step[i] * this->resolution * 0.5);

        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max( );
        tDelta[i] = std::numeric_limits<double>::max( );
      }
    }

    // for speedup:
    point3d origin_scaled = origin;  
    origin_scaled /= (float) this->resolution;  
    double length_scaled = length - this->resolution/4.; // safety margin
    length_scaled /= this->resolution;  // scale 
    length_scaled = length_scaled*length_scaled;  // avoid sqrt in dist comp.

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

      // reached endpoint, key equv?
      if (current_key == key_end) {
        done = true;
        break;
      }
      else {

        // reached endpoint world coords?
        double dist_from_origin = 0;
        for (unsigned int j = 0; j < 3; j++) {
          double coord = (double) current_key[j] - (double) this->tree_max_val;
          dist_from_origin += (coord - origin_scaled(j)) * (coord - origin_scaled(j));
        }
        if (dist_from_origin > length_scaled) {
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
  bool OcTreeBase<NODE>::computeRay(const point3d& origin, const point3d& end, 
                                    std::vector<point3d>& _ray) {
    _ray.clear();
    if (!computeRayKeys(origin, end, keyray)) return false;
    for (KeyRay::const_iterator it = keyray.begin(); it != keyray.end(); ++it) {
      point3d p;
      if (!genCoords(*it, tree_depth, p)) return false;
      _ray.push_back(p);
    }
    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::deleteNodeRecurs(NODE* node, unsigned int depth, unsigned int max_depth, const OcTreeKey& key){
    if (depth >= max_depth) // on last level: delete child when going up
      return true;

    unsigned int pos (0);
    this->genPos(key, this->tree_depth-1-depth, pos);

    if (!node->childExists(pos)) {
      // child does not exist, but maybe it's a pruned node?
      if ((!node->hasChildren()) && (node != this->itsRoot)) {
        // current node does not have children AND it's not the root node
        // -> expand pruned node
        node->expandNode();
        this->tree_size+=8;
        this->sizeChanged = true;
      } else { // no branch here, node does not exist
        return false;
      }
    }

    // follow down further, fix inner nodes on way back up
    bool deleteChild = deleteNodeRecurs(node->getChild(pos), depth+1, max_depth, key);
    if (deleteChild){
      // TODO: lazy eval?
      node->deleteChild(pos);
      this->tree_size-=1;
      this->sizeChanged = true;
      if (!node->hasChildren())
        return true;
      else{
        node->updateOccupancyChildren();
      }
    }
    // node did not lose a child, or still has other children
    return false;

  }

  template <class NODE>
  void OcTreeBase<NODE>::pruneRecurs(NODE* node, unsigned int depth,
         unsigned int max_depth, unsigned int& num_pruned) {

    if (depth < max_depth) {
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          pruneRecurs(node->getChild(i), depth+1, max_depth, num_pruned);
        }
      }
    } // end if depth

    else {
      // max level reached
      if (node->pruneNode()) {
        num_pruned++;
        tree_size -= 8;
        sizeChanged = true;
      }
    }
  }


  template <class NODE>
  void OcTreeBase<NODE>::expandRecurs(NODE* node, unsigned int depth,
                                      unsigned int max_depth) {
    if (depth >= max_depth) return;

    // current node has no children => can be expanded
    if (!node->hasChildren()){
      node->expandNode();
      tree_size +=8;
      sizeChanged = true;
    }
    // recursively expand children
    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        expandRecurs(node->getChild(i), depth+1, max_depth);
      }
    }
  }


  template <class NODE>
  std::ostream& OcTreeBase<NODE>::writeData(std::ostream &s){
    this->prune();
    return this->writeDataConst(s);
  }

  template <class NODE>
  std::ostream& OcTreeBase<NODE>::writeDataConst(std::ostream &s) const{
    itsRoot->writeValue(s);
    return s;
  }

  template <class NODE>
  std::istream& OcTreeBase<NODE>::readData(std::istream &s) {

    if (!s.good()){
      OCTOMAP_WARNING_STR(__FILE__ << ":" << __LINE__ << "Warning: Input filestream not \"good\"");
    }

    this->tree_size = 0;
    sizeChanged = true;

    // tree needs to be newly created or cleared externally!
    if (itsRoot->hasChildren()) {
      OCTOMAP_ERROR_STR("Trying to read into an existing tree.");
      return s;
    }

    itsRoot->readValue(s);
    tree_size = calcNumNodes();  // compute number of nodes
    return s;
  }




  template <class NODE>
  size_t OcTreeBase<NODE>::memoryFullGrid() {
    double size_x, size_y, size_z;
    getMetricSize(size_x, size_y,size_z);
    
    // assuming best case (one big array and efficient addressing)
    return (size_t) (ceil(resolution_factor * (double) size_x) * //sizeof (unsigned int*) *
                           ceil(resolution_factor * (double) size_y) * //sizeof (unsigned int*) *
                           ceil(resolution_factor * (double) size_z)) *
                           sizeof(itsRoot->getValue());

  }


  // non-const versions, 
  // change min/max/sizeChanged members

  template <class NODE>
  void OcTreeBase<NODE>::getMetricSize(double& x, double& y, double& z){

    double minX, minY, minZ;
    double maxX, maxY, maxZ;

    getMetricMax(maxX, maxY, maxZ);
    getMetricMin(minX, minY, minZ);

    x = maxX - minX;
    y = maxY - minY;
    z = maxZ - minZ;
  }

  template <class NODE>
  void OcTreeBase<NODE>::calcMinMax() {
    if (!sizeChanged)
      return;

    for (unsigned i = 0; i< 3; i++){
      maxValue[i] = -std::numeric_limits<double>::max();
      minValue[i] = std::numeric_limits<double>::max();
    }

    for(typename OcTreeBase<NODE>::leaf_iterator it = this->begin(),
        end=this->end(); it!= end; ++it)
    {
      double size = it.getSize();
      double halfSize = size/2.0;
      double x = it.getX() - halfSize;
      double y = it.getY() - halfSize;
      double z = it.getZ() - halfSize;
      if (x < minValue[0]) minValue[0] = x;
      if (y < minValue[1]) minValue[1] = y;
      if (z < minValue[2]) minValue[2] = z;

      x += size;
      y += size;
      z += size;
      if (x > maxValue[0]) maxValue[0] = x;
      if (y > maxValue[1]) maxValue[1] = y;
      if (z > maxValue[2]) maxValue[2] = z;

    }

    sizeChanged = false;
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMin(double& x, double& y, double& z){
    calcMinMax();
    x = minValue[0];
    y = minValue[1];
    z = minValue[2];
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMax(double& x, double& y, double& z){

    calcMinMax();
    
    x = maxValue[0];
    y = maxValue[1];
    z = maxValue[2];
  }

  // const versions

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMin(double& mx, double& my, double& mz) const {
    mx = my = mz = std::numeric_limits<double>::max( );
    if (sizeChanged) {
      for(typename OcTreeBase<NODE>::leaf_iterator it = this->begin(),
              end=this->end(); it!= end; ++it) {
        double halfSize = it.getSize()/2.0;
        double x = it.getX() - halfSize;
        double y = it.getY() - halfSize;
        double z = it.getZ() - halfSize;
        if (x < mx) mx = x;
        if (y < my) my = y;
        if (z < mz) mz = z;
      }
    } // end if size changed 
    else {
      mx = minValue[0];
      my = minValue[1];
      mz = minValue[2];
    }
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMax(double& mx, double& my, double& mz) const {
    mx = my = mz = -std::numeric_limits<double>::max( );
    if (sizeChanged) {
      for(typename OcTreeBase<NODE>::leaf_iterator it = this->begin(),
            end=this->end(); it!= end; ++it) {
        double halfSize = it.getSize()/2.0;
        double x = it.getX() + halfSize;
        double y = it.getY() + halfSize;
        double z = it.getZ() + halfSize;
        if (x > mx) mx = x;
        if (y > my) my = y;
        if (z > mz) mz = z;
      }
    } 
    else {
      mx = maxValue[0];
      my = maxValue[1];
      mz = maxValue[2];
    }
  }

  template <class NODE>
  size_t OcTreeBase<NODE>::calcNumNodes() const {
    size_t retval = 1; // root node
    calcNumNodesRecurs(itsRoot, retval);
    return retval;
  }

  template <class NODE>
  void OcTreeBase<NODE>::calcNumNodesRecurs(NODE* node, size_t& num_nodes) const {
    assert (node != NULL);
    if (node->hasChildren()) {
      for (unsigned int i=0; i<8; ++i) {
        if (node->childExists(i)) {
          num_nodes++;
          calcNumNodesRecurs(node->getChild(i), num_nodes);
        }
      }
    }
  }

  template <class NODE>
  size_t OcTreeBase<NODE>::memoryUsage() const{
    size_t num_leaf_nodes = this->getNumLeafNodes();
    size_t num_inner_nodes = tree_size - num_leaf_nodes;
    return (sizeof(OcTreeBase<NODE>) + memoryUsageNode() * tree_size + num_inner_nodes * sizeof(NODE*[8]));
  }

  template <class NODE>
  void OcTreeBase<NODE>::getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax) const {

    float diff[3];
    unsigned int steps[3];
    for (int i=0;i<3;++i) {
      diff[i] = pmax(i) - pmin(i);
      steps[i] = floor(diff[i] / this->resolution);
      //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
    }
    
    point3d p = pmin;
    NODE* res;
    for (unsigned int x=0; x<steps[0]; ++x) {
      p.x() += this->resolution;
      for (unsigned int y=0; y<steps[1]; ++y) {
        p.y() += this->resolution;
        for (unsigned int z=0; z<steps[2]; ++z) {
          //          std::cout << "querying p=" << p << std::endl;
          p.z() += this->resolution;
          res = this->search(p);
          if (res == NULL) {
            node_centers.push_back(p);
          }
        }
        p.z() = pmin.z();
      }
      p.y() = pmin.y();
    }
  }


  template <class NODE>
  size_t OcTreeBase<NODE>::getNumLeafNodes() const {
    return getNumLeafNodesRecurs(itsRoot);
  }


  template <class NODE>
  size_t OcTreeBase<NODE>::getNumLeafNodesRecurs(const NODE* parent) const {

    if (!parent->hasChildren()) return 1;  // this is a leaf -> terminate
    
    size_t sum_leafs_children = 0;
    for (unsigned int i=0; i<8; ++i) {
      if (parent->childExists(i)) {
        sum_leafs_children += getNumLeafNodesRecurs(parent->getChild(i));
      }
    }
    return sum_leafs_children;
  }


  template <class NODE>
  double OcTreeBase<NODE>::volume() {
    double x,  y,  z;
    getMetricSize(x, y, z);
    return x*y*z;
  }


}
