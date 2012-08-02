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


  template <class NODE>
  OcTreeBase<NODE>::OcTreeBase(double resolution) :
    root(NULL), tree_depth(16), tree_max_val(32768), 
    resolution(resolution), tree_size(0)
  {
    
    this->setResolution(resolution);
    for (unsigned i = 0; i< 3; i++){
      max_value[i] = -(std::numeric_limits<double>::max( ));
      min_value[i] = std::numeric_limits<double>::max( );
    }
    size_changed = true;

    // init root node:
    root = new NODE();
    tree_size++;
  }

  template <class NODE>
  OcTreeBase<NODE>::OcTreeBase(double resolution, unsigned int tree_depth, unsigned int tree_max_val) :
    root(NULL), tree_depth(tree_depth), tree_max_val(tree_max_val),
    resolution(resolution), tree_size(0)
  {
    this->setResolution(resolution);
    for (unsigned i = 0; i< 3; i++){
      max_value[i] = -(std::numeric_limits<double>::max( ));
      min_value[i] = std::numeric_limits<double>::max( );
    }
    size_changed = true;

    // init root node:
    root = new NODE();
    tree_size++;
  }


  template <class NODE>
  OcTreeBase<NODE>::~OcTreeBase(){
    delete root;
  }


  template <class NODE>
  OcTreeBase<NODE>::OcTreeBase(const OcTreeBase<NODE>& rhs) :
    root(NULL), tree_depth(rhs.tree_depth), tree_max_val(rhs.tree_max_val),
    resolution(rhs.resolution), tree_size(rhs.tree_size)
  {
    this->setResolution(resolution);
    for (unsigned i = 0; i< 3; i++){
      max_value[i] = rhs.max_value[i];
      min_value[i] = rhs.min_value[i];
    }

    // copy nodes recursively:
    root = new NODE(*(rhs.root));


  }

  template <class NODE>
  bool OcTreeBase<NODE>::operator== (const OcTreeBase<NODE>& other) const{
    if (tree_depth != other.tree_depth || tree_max_val != other.tree_max_val
        || resolution != other.resolution || tree_size != other.tree_size){
      return false;
    }

    // traverse all nodes, check if structure the same
    OcTreeBase<NODE>::tree_iterator it = this->begin_tree();
    OcTreeBase<NODE>::tree_iterator end = this->end_tree();
    OcTreeBase<NODE>::tree_iterator other_it = other.begin_tree();
    OcTreeBase<NODE>::tree_iterator other_end = other.end_tree();

    for (; it != end; ++it, ++other_it){
      if (other_it == other_end)
        return false;

      if (it.getDepth() != other_it.getDepth()
          || it.getKey() != other_it.getKey()
          || !(*it == *other_it))
      {
        return false;
      }
    }

    if (other_it != other_end)
      return false;

    return true;
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
  inline unsigned short int OcTreeBase<NODE>::coordToKey(double coordinate, unsigned depth) const{
    assert (depth <= tree_depth);
    int keyval = ((int) floor(resolution_factor * coordinate));

    unsigned int diff = tree_depth - depth;
    if(!diff) // same as coordToKey without depth
      return keyval + tree_max_val;
    else // shift right and left => erase last bits. Then add offset.
      return ((keyval >> diff) << diff) + (1 << (diff-1)) + tree_max_val;
  }


  template <class NODE>
  bool OcTreeBase<NODE>::coordToKeyChecked(double coordinate, unsigned short int& keyval) const {

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
  bool OcTreeBase<NODE>::coordToKeyChecked(double coordinate, unsigned depth, unsigned short int& keyval) const {

    // scale to resolution and shift center for tree_max_val
    int scaled_coord =  ((int) floor(resolution_factor * coordinate)) + tree_max_val;

    // keyval within range of tree?
    if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2*tree_max_val))) {
      keyval = scaled_coord;
      keyval = adjustKeyAtDepth(keyval, depth);
      return true;
    }
    return false;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::coordToKeyChecked(const point3d& point, OcTreeKey& key) const{

    for (unsigned int i=0;i<3;i++) {
      if (!coordToKeyChecked( point(i), key[i])) return false;
    }
    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::coordToKeyChecked(const point3d& point, unsigned depth, OcTreeKey& key) const{

    for (unsigned int i=0;i<3;i++) {
      if (!coordToKeyChecked( point(i), depth, key[i])) return false;
    }
    return true;
  }

  template <class NODE>
  unsigned short int OcTreeBase<NODE>::adjustKeyAtDepth(unsigned short int key, unsigned int depth) const{
    unsigned int diff = tree_depth - depth;

    if(diff == 0)
      return key;
    else
      return (((key-tree_max_val) >> diff) << diff) + (1 << (diff-1)) + tree_max_val;
  }

  template <class NODE>
  double OcTreeBase<NODE>::keyToCoord(unsigned short int key, unsigned depth) const{
    assert(depth <= tree_depth);

    // root is centered on 0 = 0.0
    if (depth == 0) {
      return 0.0;
    } else if (depth == tree_depth) {
      return keyToCoord(key);
    } else {
      return (floor( (double(key)-double(this->tree_max_val)) /double(1 << (tree_depth - depth)) )  + 0.5 ) * this->getNodeSize(depth);
    }
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
  NODE* OcTreeBase<NODE>::search(const point3d& value, unsigned int depth) const {

    // Search is a variant of insert which aborts if
    // it had to insert nodes

    OcTreeKey key;
    if (!coordToKeyChecked(value, key)){
      OCTOMAP_ERROR_STR("Error in search: ["<< value <<"] is out of OcTree bounds!");
      return NULL;
    }
    else {
      return this->search(key, depth);
    }

  }

  template <class NODE>
  NODE* OcTreeBase<NODE>::search(float x, float y, float z, unsigned int depth) const {
    return this->search(point3d(x,y,z), depth);
  }


  template <class NODE>
  NODE* OcTreeBase<NODE>::search (const OcTreeKey& key, unsigned int depth) const {
    assert(depth <= tree_depth);

    if (depth == 0)
      depth = tree_depth;



    // generate appropriate key_at_depth for queried depth
    OcTreeKey key_at_depth = key;
    if (depth != tree_depth)
      key_at_depth = adjustKeyAtDepth(key, depth);

    NODE* curNode (root);

    unsigned int diff = tree_depth - depth;

    // follow nodes down to requested level (for diff = 0 it's the last level)
    for (unsigned i=(tree_depth-1); i>=diff; --i) {
      unsigned int pos = computeChildIdx(key_at_depth, i);
      if (curNode->childExists(pos)) {
        // cast needed: (nodes need to ensure it's the right pointer)
        curNode = static_cast<NODE*>( curNode->getChild(pos) );
      } else {
        // we expected a child but did not get it
        // is the current node a leaf already?
        if (!curNode->hasChildren()) {
          return curNode;
        } else {
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
    if (!coordToKeyChecked(value, key)){
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

    return deleteNodeRecurs(root, 0, depth, key);
  }

  template <class NODE>
  void OcTreeBase<NODE>::clear() {
    // don't clear if the tree is empty:
    if (this->root->hasChildren()) {
      delete this->root;
      this->root = new NODE();
    }
    this->tree_size = 1;
    // max extent of tree changed:
    this->size_changed = true;
  }


  template <class NODE>
  void OcTreeBase<NODE>::prune() {
    for (unsigned int depth=tree_depth-1; depth>0; depth--) {
      unsigned int num_pruned = 0;
      pruneRecurs(this->root, 0, depth, num_pruned);
      if (num_pruned == 0) break;
    }
  }

  template <class NODE>
  void OcTreeBase<NODE>::expand() {
    expandRecurs(root,0, tree_depth);
  }

  template <class NODE>
  bool OcTreeBase<NODE>::computeRayKeys(const point3d& origin, 
                                          const point3d& end, 
                                          KeyRay& ray) const {

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    ray.reset();

    OcTreeKey key_origin, key_end;
    if ( !OcTreeBase<NODE>::coordToKeyChecked(origin, key_origin) ||
         !OcTreeBase<NODE>::coordToKeyChecked(end, key_end) ) {
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
        double voxelBorder = this->keyToCoord(current_key[i]);
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
      _ray.push_back(keyToCoord(*it));
    }
    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::deleteNodeRecurs(NODE* node, unsigned int depth, unsigned int max_depth, const OcTreeKey& key){
    if (depth >= max_depth) // on last level: delete child when going up
      return true;

    unsigned int pos = computeChildIdx(key, this->tree_depth-1-depth);

    if (!node->childExists(pos)) {
      // child does not exist, but maybe it's a pruned node?
      if ((!node->hasChildren()) && (node != this->root)) {
        // current node does not have children AND it's not the root node
        // -> expand pruned node
        node->expandNode();
        this->tree_size+=8;
        this->size_changed = true;
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
      this->size_changed = true;
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
        size_changed = true;
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
      size_changed = true;
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
    root->writeValue(s);
    return s;
  }

  template <class NODE>
  std::istream& OcTreeBase<NODE>::readData(std::istream &s) {

    if (!s.good()){
      OCTOMAP_WARNING_STR(__FILE__ << ":" << __LINE__ << "Warning: Input filestream not \"good\"");
    }

    this->tree_size = 0;
    size_changed = true;

    // tree needs to be newly created or cleared externally!
    if (root->hasChildren()) {
      OCTOMAP_ERROR_STR("Trying to read into an existing tree.");
      return s;
    }

    root->readValue(s);
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
                           sizeof(root->getValue());

  }


  // non-const versions, 
  // change min/max/size_changed members

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
    if (!size_changed)
      return;

    for (unsigned i = 0; i< 3; i++){
      max_value[i] = -std::numeric_limits<double>::max();
      min_value[i] = std::numeric_limits<double>::max();
    }

    for(typename OcTreeBase<NODE>::leaf_iterator it = this->begin(),
        end=this->end(); it!= end; ++it)
    {
      double size = it.getSize();
      double halfSize = size/2.0;
      double x = it.getX() - halfSize;
      double y = it.getY() - halfSize;
      double z = it.getZ() - halfSize;
      if (x < min_value[0]) min_value[0] = x;
      if (y < min_value[1]) min_value[1] = y;
      if (z < min_value[2]) min_value[2] = z;

      x += size;
      y += size;
      z += size;
      if (x > max_value[0]) max_value[0] = x;
      if (y > max_value[1]) max_value[1] = y;
      if (z > max_value[2]) max_value[2] = z;

    }

    size_changed = false;
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMin(double& x, double& y, double& z){
    calcMinMax();
    x = min_value[0];
    y = min_value[1];
    z = min_value[2];
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMax(double& x, double& y, double& z){

    calcMinMax();
    
    x = max_value[0];
    y = max_value[1];
    z = max_value[2];
  }

  // const versions

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMin(double& mx, double& my, double& mz) const {
    mx = my = mz = std::numeric_limits<double>::max( );
    if (size_changed) {
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
      mx = min_value[0];
      my = min_value[1];
      mz = min_value[2];
    }
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMax(double& mx, double& my, double& mz) const {
    mx = my = mz = -std::numeric_limits<double>::max( );
    if (size_changed) {
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
      mx = max_value[0];
      my = max_value[1];
      mz = max_value[2];
    }
  }

  template <class NODE>
  size_t OcTreeBase<NODE>::calcNumNodes() const {
    size_t retval = 1; // root node
    calcNumNodesRecurs(root, retval);
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
    return getNumLeafNodesRecurs(root);
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
