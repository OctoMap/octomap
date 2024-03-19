/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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

#undef max
#undef min
#include <limits>

#ifdef _OPENMP
  #include <omp.h>
#endif

namespace octomap {


  template <class NODE,class I>
  OcTreeBaseImpl<NODE,I>::OcTreeBaseImpl(double in_resolution) :
    I(), root(NULL), tree_depth(16), tree_max_val(32768),
    resolution(in_resolution), tree_size(0)
  {

    init();

    // no longer create an empty root node - only on demand
  }

  template <class NODE,class I>
  OcTreeBaseImpl<NODE,I>::OcTreeBaseImpl(double in_resolution, unsigned int in_tree_depth, unsigned int in_tree_max_val) :
    I(), root(NULL), tree_depth(in_tree_depth), tree_max_val(in_tree_max_val),
    resolution(in_resolution), tree_size(0)
  {
    init();

    // no longer create an empty root node - only on demand
  }


  template <class NODE,class I>
  OcTreeBaseImpl<NODE,I>::~OcTreeBaseImpl(){
    clear();
  }


  template <class NODE,class I>
  OcTreeBaseImpl<NODE,I>::OcTreeBaseImpl(const OcTreeBaseImpl<NODE,I>& rhs) :
    root(NULL), tree_depth(rhs.tree_depth), tree_max_val(rhs.tree_max_val),
    resolution(rhs.resolution), tree_size(rhs.tree_size)
  {
    init();

    // copy nodes recursively:
    if (rhs.root)
      root = new NODE(*(rhs.root));

  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::init(){

    this->setResolution(this->resolution);
    for (unsigned i = 0; i< 3; i++){
      max_value[i] = -(std::numeric_limits<double>::max( ));
      min_value[i] = std::numeric_limits<double>::max( );
    }
    size_changed = true;

    // create as many KeyRays as there are OMP_THREADS defined,
    // one buffer for each thread
#ifdef _OPENMP
    #pragma omp parallel
    #pragma omp critical
    {
      if (omp_get_thread_num() == 0){
        this->keyrays.resize(omp_get_num_threads());
      }

    }
#else
    this->keyrays.resize(1);
#endif

  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::swapContent(OcTreeBaseImpl<NODE,I>& other){
    NODE* this_root = root;
    root = other.root;
    other.root = this_root;

    size_t this_size = this->tree_size;
    this->tree_size = other.tree_size;
    other.tree_size = this_size;
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::operator== (const OcTreeBaseImpl<NODE,I>& other) const{
    if (tree_depth != other.tree_depth || tree_max_val != other.tree_max_val
        || resolution != other.resolution || tree_size != other.tree_size){
      return false;
    }

    // traverse all nodes, check if structure the same
    typename OcTreeBaseImpl<NODE,I>::tree_iterator it = this->begin_tree();
    typename OcTreeBaseImpl<NODE,I>::tree_iterator end = this->end_tree();
    typename OcTreeBaseImpl<NODE,I>::tree_iterator other_it = other.begin_tree();
    typename OcTreeBaseImpl<NODE,I>::tree_iterator other_end = other.end_tree();

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

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::setResolution(double r) {
    resolution = r;
    resolution_factor = 1. / resolution;

    tree_center(0) = tree_center(1) = tree_center(2)
      = (float) (((double) tree_max_val) / resolution_factor);

    // init node size lookup table:
    sizeLookupTable.resize(tree_depth+1);
    for(unsigned i = 0; i <= tree_depth; ++i){
      sizeLookupTable[i] = resolution * double(1 << (tree_depth - i));
    }

    size_changed = true;
  }

  template <class NODE,class I>
  NODE* OcTreeBaseImpl<NODE,I>::createNodeChild(NODE* node, unsigned int childIdx){
    assert(childIdx < 8);
    if (node->children == NULL) {
      allocNodeChildren(node);
    }
    assert (node->children[childIdx] == NULL);
    NODE* newNode = new NODE();
    node->children[childIdx] = static_cast<AbstractOcTreeNode*>(newNode);

    tree_size++;
    size_changed = true;

    return newNode;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::deleteNodeChild(NODE* node, unsigned int childIdx){
    assert((childIdx < 8) && (node->children != NULL));
    assert(node->children[childIdx] != NULL);
    delete static_cast<NODE*>(node->children[childIdx]); // TODO delete check if empty
    node->children[childIdx] = NULL;

    tree_size--;
    size_changed = true;
  }

  template <class NODE,class I>
  NODE* OcTreeBaseImpl<NODE,I>::getNodeChild(NODE* node, unsigned int childIdx) const{
    assert((childIdx < 8) && (node->children != NULL));
    assert(node->children[childIdx] != NULL);
    return static_cast<NODE*>(node->children[childIdx]);
  }

  template <class NODE,class I>
  const NODE* OcTreeBaseImpl<NODE,I>::getNodeChild(const NODE* node, unsigned int childIdx) const{
    assert((childIdx < 8) && (node->children != NULL));
    assert(node->children[childIdx] != NULL);
    return static_cast<const NODE*>(node->children[childIdx]);
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::isNodeCollapsible(const NODE* node) const{
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
    if (!nodeChildExists(node, 0))
      return false;

    const NODE* firstChild = getNodeChild(node, 0);
    if (nodeHasChildren(firstChild))
      return false;

    for (unsigned int i = 1; i<8; i++) {
      // comparison via getChild so that casts of derived classes ensure
      // that the right == operator gets called
      if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(*(getNodeChild(node, i)) == *(firstChild)))
        return false;
    }

    return true;
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::nodeChildExists(const NODE* node, unsigned int childIdx) const{
    assert(childIdx < 8);
    if ((node->children != NULL) && (node->children[childIdx] != NULL))
      return true;
    else
      return false;
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::nodeHasChildren(const NODE* node) const {
    if (node->children == NULL)
      return false;

    for (unsigned int i = 0; i<8; i++){
      if (node->children[i] != NULL)
        return true;
    }
    return false;
  }


  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::expandNode(NODE* node){
    assert(!nodeHasChildren(node));

    for (unsigned int k=0; k<8; k++) {
      NODE* newNode = createNodeChild(node, k);
      newNode->copyData(*node);
    }
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::pruneNode(NODE* node){

    if (!isNodeCollapsible(node))
      return false;

    // set value to children's values (all assumed equal)
    node->copyData(*(getNodeChild(node, 0)));

    // delete children (known to be leafs at this point!)
    for (unsigned int i=0;i<8;i++) {
      deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;

    return true;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::allocNodeChildren(NODE* node){
    // TODO NODE*
    node->children = new AbstractOcTreeNode*[8];
    for (unsigned int i=0; i<8; i++) {
      node->children[i] = NULL;
    }
  }



  template <class NODE,class I>
  inline key_type OcTreeBaseImpl<NODE,I>::coordToKey(double coordinate, unsigned depth) const{
    assert (depth <= tree_depth);
    int keyval = ((int) floor(resolution_factor * coordinate));

    unsigned int diff = tree_depth - depth;
    if(!diff) // same as coordToKey without depth
      return keyval + tree_max_val;
    else // shift right and left => erase last bits. Then add offset.
      return ((keyval >> diff) << diff) + (1 << (diff-1)) + tree_max_val;
  }


  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::coordToKeyChecked(double coordinate, key_type& keyval) const {

    // scale to resolution and shift center for tree_max_val
    int scaled_coord =  ((int) floor(resolution_factor * coordinate)) + tree_max_val;

    // keyval within range of tree?
    if (( scaled_coord >= 0) && (((unsigned int) scaled_coord) < (2*tree_max_val))) {
      keyval = scaled_coord;
      return true;
    }
    return false;
  }


  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::coordToKeyChecked(double coordinate, unsigned depth, key_type& keyval) const {

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

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::coordToKeyChecked(const point3d& point, OcTreeKey& key) const{

    for (unsigned int i=0;i<3;i++) {
      if (!coordToKeyChecked( point(i), key[i])) return false;
    }
    return true;
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::coordToKeyChecked(const point3d& point, unsigned depth, OcTreeKey& key) const{

    for (unsigned int i=0;i<3;i++) {
      if (!coordToKeyChecked( point(i), depth, key[i])) return false;
    }
    return true;
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::coordToKeyChecked(double x, double y, double z, OcTreeKey& key) const{

    if (!(coordToKeyChecked(x, key[0])
       && coordToKeyChecked(y, key[1])
       && coordToKeyChecked(z, key[2])))
    {
      return false;
    } else {
      return true;
    }
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::coordToKeyChecked(double x, double y, double z, unsigned depth, OcTreeKey& key) const{

    if (!(coordToKeyChecked(x, depth, key[0])
       && coordToKeyChecked(y, depth, key[1])
       && coordToKeyChecked(z, depth, key[2])))
    {
      return false;
    } else {
      return true;
    }
  }

  template <class NODE,class I>
  key_type OcTreeBaseImpl<NODE,I>::adjustKeyAtDepth(key_type key, unsigned int depth) const{
    unsigned int diff = tree_depth - depth;

    if(diff == 0)
      return key;
    else
      return (((key-tree_max_val) >> diff) << diff) + (1 << (diff-1)) + tree_max_val;
  }

  template <class NODE,class I>
  double OcTreeBaseImpl<NODE,I>::keyToCoord(key_type key, unsigned depth) const{
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

  template <class NODE,class I>
  NODE* OcTreeBaseImpl<NODE,I>::search(const point3d& value, unsigned int depth) const {
    OcTreeKey key;
    if (!coordToKeyChecked(value, key)){
      OCTOMAP_ERROR_STR("Error in search: ["<< value <<"] is out of OcTree bounds!");
      return NULL;
    }
    else {
      return this->search(key, depth);
    }

  }

  template <class NODE,class I>
  NODE* OcTreeBaseImpl<NODE,I>::search(double x, double y, double z, unsigned int depth) const {
    OcTreeKey key;
    if (!coordToKeyChecked(x, y, z, key)){
      OCTOMAP_ERROR_STR("Error in search: ["<< x <<" "<< y << " " << z << "] is out of OcTree bounds!");
      return NULL;
    }
    else {
      return this->search(key, depth);
    }
  }


  template <class NODE,class I>
  NODE* OcTreeBaseImpl<NODE,I>::search (const OcTreeKey& key, unsigned int depth) const {
    assert(depth <= tree_depth);
    if (root == NULL)
      return NULL;

    if (depth == 0)
      depth = tree_depth;



    // generate appropriate key_at_depth for queried depth
    OcTreeKey key_at_depth = key;
    if (depth != tree_depth)
      key_at_depth = adjustKeyAtDepth(key, depth);

    NODE* curNode (root);

    int diff = tree_depth - depth;

    // follow nodes down to requested level (for diff = 0 it's the last level)
    for (int i=(tree_depth-1); i>=diff; --i) {
      unsigned int pos = computeChildIdx(key_at_depth, i);
      if (nodeChildExists(curNode, pos)) {
        // cast needed: (nodes need to ensure it's the right pointer)
        curNode = getNodeChild(curNode, pos);
      } else {
        // we expected a child but did not get it
        // is the current node a leaf already?
        if (!nodeHasChildren(curNode)) { // TODO similar check to nodeChildExists?
          return curNode;
        } else {
          // it is not, search failed
          return NULL;
        }
      }
    } // end for
    return curNode;
  }


  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::deleteNode(const point3d& value, unsigned int depth) {
    OcTreeKey key;
    if (!coordToKeyChecked(value, key)){
      OCTOMAP_ERROR_STR("Error in deleteNode: ["<< value <<"] is out of OcTree bounds!");
      return false;
    }
    else {
      return this->deleteNode(key, depth);
    }

  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::deleteNode(double x, double y, double z, unsigned int depth) {
    OcTreeKey key;
    if (!coordToKeyChecked(x, y, z, key)){
      OCTOMAP_ERROR_STR("Error in deleteNode: ["<< x <<" "<< y << " " << z << "] is out of OcTree bounds!");
      return false;
    }
    else {
      return this->deleteNode(key, depth);
    }
  }


  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::deleteNode(const OcTreeKey& key, unsigned int depth) {
    if (root == NULL)
      return true;

    if (depth == 0)
      depth = tree_depth;

    return deleteNodeRecurs(root, 0, depth, key);
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::clear() {
    if (this->root){
      deleteNodeRecurs(root);
      this->tree_size = 0;
      this->root = NULL;
      // max extent of tree changed:
      this->size_changed = true;
    }
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::prune() {
    if (root == NULL)
      return;

    for (unsigned int depth=tree_depth-1; depth > 0; --depth) {
      unsigned int num_pruned = 0;
      pruneRecurs(this->root, 0, depth, num_pruned);
      if (num_pruned == 0)
        break;
    }
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::expand() {
    if (root)
      expandRecurs(root,0, tree_depth);
  }

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::computeRayKeys(const point3d& origin,
                                          const point3d& end,
                                          KeyRay& ray) const {

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    ray.reset();

    OcTreeKey key_origin, key_end;
    if ( !OcTreeBaseImpl<NODE,I>::coordToKeyChecked(origin, key_origin) ||
         !OcTreeBaseImpl<NODE,I>::coordToKeyChecked(end, key_end) ) {
      OCTOMAP_WARNING_STR("coordinates ( "
                << origin << " -> " << end << ") out of bounds in computeRayKeys");
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
        // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
        double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
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

  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::computeRay(const point3d& origin, const point3d& end,
                                    std::vector<point3d>& _ray) {
    _ray.clear();
    if (!computeRayKeys(origin, end, keyrays.at(0))) return false;
    for (KeyRay::const_iterator it = keyrays[0].begin(); it != keyrays[0].end(); ++it) {
      _ray.push_back(keyToCoord(*it));
    }
    return true;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::deleteNodeRecurs(NODE* node){
    assert(node);
    // TODO: maintain tree size?

    if (node->children != NULL) {
      for (unsigned int i=0; i<8; i++) {
        if (node->children[i] != NULL){
          this->deleteNodeRecurs(static_cast<NODE*>(node->children[i]));
        }
      }
      delete[] node->children;
      node->children = NULL;
    } // else: node has no children

    delete node;
  }


  template <class NODE,class I>
  bool OcTreeBaseImpl<NODE,I>::deleteNodeRecurs(NODE* node, unsigned int depth, unsigned int max_depth, const OcTreeKey& key){
    if (depth >= max_depth) // on last level: delete child when going up
      return true;

    assert(node);

    unsigned int pos = computeChildIdx(key, this->tree_depth-1-depth);

    if (!nodeChildExists(node, pos)) {
      // child does not exist, but maybe it's a pruned node?
      if ((!nodeHasChildren(node)) && (node != this->root)) { // TODO double check for exists / root exception?
        // current node does not have children AND it's not the root node
        // -> expand pruned node
        expandNode(node);
        // tree_size and size_changed adjusted in createNodeChild(...)
      } else { // no branch here, node does not exist
        return false;
      }
    }

    // follow down further, fix inner nodes on way back up
    bool deleteChild = deleteNodeRecurs(getNodeChild(node, pos), depth+1, max_depth, key);
    if (deleteChild){
      // TODO: lazy eval?
      // TODO delete check depth, what happens to inner nodes with children?
      this->deleteNodeChild(node, pos);

      if (!nodeHasChildren(node))
        return true;
      else{
        node->updateOccupancyChildren(); // TODO: occupancy?
      }
    }
    // node did not lose a child, or still has other children
    return false;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::pruneRecurs(NODE* node, unsigned int depth,
         unsigned int max_depth, unsigned int& num_pruned) {

    assert(node);

    if (depth < max_depth) {
      for (unsigned int i=0; i<8; i++) {
        if (nodeChildExists(node, i)) {
          pruneRecurs(getNodeChild(node, i), depth+1, max_depth, num_pruned);
        }
      }
    } // end if depth

    else {
      // max level reached
      if (pruneNode(node)) {
        num_pruned++;
      }
    }
  }


  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::expandRecurs(NODE* node, unsigned int depth,
                                      unsigned int max_depth) {
    if (depth >= max_depth)
      return;

    assert(node);

    // current node has no children => can be expanded
    if (!nodeHasChildren(node)){
      expandNode(node);
    }
    // recursively expand children
    for (unsigned int i=0; i<8; i++) {
      if (nodeChildExists(node, i)) { // TODO double check (node != NULL)
        expandRecurs(getNodeChild(node, i), depth+1, max_depth);
      }
    }
  }


  template <class NODE,class I>
  std::ostream& OcTreeBaseImpl<NODE,I>::writeData(std::ostream &s) const{
    if (root)
      writeNodesRecurs(root, s);

    return s;
  }

  template <class NODE,class I>
  std::ostream& OcTreeBaseImpl<NODE,I>::writeNodesRecurs(const NODE* node, std::ostream &s) const{
    node->writeData(s);

    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i=0; i<8; i++) {
      if (nodeChildExists(node, i))
        children[i] = 1;
      else
        children[i] = 0;
    }

    char children_char = (char) children.to_ulong();
    s.write((char*)&children_char, sizeof(char));

//     std::cout << "wrote: " << value << " "
//               << children.to_string<char,std::char_traits<char>,std::allocator<char> >()
//               << std::endl;

    // recursively write children
    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1) {
        this->writeNodesRecurs(getNodeChild(node, i), s);
      }
    }

    return s;
  }

  template <class NODE,class I>
  std::istream& OcTreeBaseImpl<NODE,I>::readData(std::istream &s) {

    if (!s.good()){
      OCTOMAP_WARNING_STR(__FILE__ << ":" << __LINE__ << "Warning: Input filestream not \"good\"");
    }

    this->tree_size = 0;
    size_changed = true;

    // tree needs to be newly created or cleared externally
    if (root) {
      OCTOMAP_ERROR_STR("Trying to read into an existing tree.");
      return s;
    }

    root = new NODE();
    readNodesRecurs(root, s);

    tree_size = calcNumNodes();  // compute number of nodes
    return s;
  }

  template <class NODE,class I>
  std::istream& OcTreeBaseImpl<NODE,I>::readNodesRecurs(NODE* node, std::istream &s) {

    node->readData(s);

    char children_char;
    s.read((char*)&children_char, sizeof(char));
    std::bitset<8> children ((unsigned long long) children_char);

    //std::cout << "read: " << node->getValue() << " "
    //            << children.to_string<char,std::char_traits<char>,std::allocator<char> >()
    //            << std::endl;

    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        NODE* newNode = createNodeChild(node, i);
        readNodesRecurs(newNode, s);
      }
    }

    return s;
  }




  template <class NODE,class I>
  unsigned long long OcTreeBaseImpl<NODE,I>::memoryFullGrid() const{
    if (root == NULL)
      return 0;

    double size_x, size_y, size_z;
    this->getMetricSize(size_x, size_y,size_z);

    // assuming best case (one big array and efficient addressing)
    // we can avoid "ceil" since size already accounts for voxels

    // Note: this can be larger than the adressable memory
    //   - size_t may not be enough to hold it!
    return (unsigned long long)((size_x/resolution) * (size_y/resolution) * (size_z/resolution)
        * sizeof(root->getValue()));

  }


  // non-const versions,
  // change min/max/size_changed members

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getMetricSize(double& x, double& y, double& z){

    double minX, minY, minZ;
    double maxX, maxY, maxZ;

    getMetricMax(maxX, maxY, maxZ);
    getMetricMin(minX, minY, minZ);

    x = maxX - minX;
    y = maxY - minY;
    z = maxZ - minZ;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getMetricSize(double& x, double& y, double& z) const{

    double minX, minY, minZ;
    double maxX, maxY, maxZ;

    getMetricMax(maxX, maxY, maxZ);
    getMetricMin(minX, minY, minZ);

    x = maxX - minX;
    y = maxY - minY;
    z = maxZ - minZ;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::calcMinMax() {
    if (!size_changed)
      return;

    // empty tree
    if (root == NULL){
      min_value[0] = min_value[1] = min_value[2] = 0.0;
      max_value[0] = max_value[1] = max_value[2] = 0.0;
      size_changed = false;
      return;
    }

    for (unsigned i = 0; i< 3; i++){
      max_value[i] = -std::numeric_limits<double>::max();
      min_value[i] = std::numeric_limits<double>::max();
    }

    for(typename OcTreeBaseImpl<NODE,I>::leaf_iterator it = this->begin(),
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

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getMetricMin(double& x, double& y, double& z){
    calcMinMax();
    x = min_value[0];
    y = min_value[1];
    z = min_value[2];
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getMetricMax(double& x, double& y, double& z){
    calcMinMax();
    x = max_value[0];
    y = max_value[1];
    z = max_value[2];
  }

  // const versions

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getMetricMin(double& mx, double& my, double& mz) const {
    mx = my = mz = std::numeric_limits<double>::max( );
    if (size_changed) {
      // empty tree
      if (root == NULL){
        mx = my = mz = 0.0;
        return;
      }

      for(typename OcTreeBaseImpl<NODE,I>::leaf_iterator it = this->begin(),
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

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getMetricMax(double& mx, double& my, double& mz) const {
    mx = my = mz = -std::numeric_limits<double>::max( );
    if (size_changed) {
      // empty tree
      if (root == NULL){
        mx = my = mz = 0.0;
        return;
      }

      for(typename OcTreeBaseImpl<NODE,I>::leaf_iterator it = this->begin(),
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

  template <class NODE,class I>
  size_t OcTreeBaseImpl<NODE,I>::calcNumNodes() const {
    size_t retval = 0; // root node
    if (root){
      retval++;
      calcNumNodesRecurs(root, retval);
    }
    return retval;
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::calcNumNodesRecurs(NODE* node, size_t& num_nodes) const {
    assert (node);
    if (nodeHasChildren(node)) {
      for (unsigned int i=0; i<8; ++i) {
        if (nodeChildExists(node, i)) {
          num_nodes++;
          calcNumNodesRecurs(getNodeChild(node, i), num_nodes);
        }
      }
    }
  }

  template <class NODE,class I>
  size_t OcTreeBaseImpl<NODE,I>::memoryUsage() const{
    size_t num_leaf_nodes = this->getNumLeafNodes();
    size_t num_inner_nodes = tree_size - num_leaf_nodes;
    return (sizeof(OcTreeBaseImpl<NODE,I>) + memoryUsageNode() * tree_size + num_inner_nodes * sizeof(NODE*[8]));
  }

  template <class NODE,class I>
  void OcTreeBaseImpl<NODE,I>::getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax, unsigned int depth) const {

    assert(depth <= tree_depth);
    if (depth == 0)
      depth = tree_depth;

    point3d pmin_clamped = this->keyToCoord(this->coordToKey(pmin, depth), depth);
    point3d pmax_clamped = this->keyToCoord(this->coordToKey(pmax, depth), depth);

    float diff[3];
    unsigned int steps[3];
    float step_size = this->resolution * pow(2, tree_depth-depth);
    for (int i=0;i<3;++i) {
      diff[i] = pmax_clamped(i) - pmin_clamped(i);
      steps[i] = static_cast<unsigned int>(floor(diff[i] / step_size));
      //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
    }

    point3d p = pmin_clamped;
    NODE* res;
    for (unsigned int x=0; x<steps[0]; ++x) {
      p.x() += step_size;
      for (unsigned int y=0; y<steps[1]; ++y) {
        p.y() += step_size;
        for (unsigned int z=0; z<steps[2]; ++z) {
          //          std::cout << "querying p=" << p << std::endl;
          p.z() += step_size;
          res = this->search(p,depth);
          if (res == NULL) {
            node_centers.push_back(p);
          }
        }
        p.z() = pmin_clamped.z();
      }
      p.y() = pmin_clamped.y();
    }
  }


  template <class NODE,class I>
  size_t OcTreeBaseImpl<NODE,I>::getNumLeafNodes() const {
    if (root == NULL)
      return 0;

    return getNumLeafNodesRecurs(root);
  }


  template <class NODE,class I>
  size_t OcTreeBaseImpl<NODE,I>::getNumLeafNodesRecurs(const NODE* parent) const {
    assert(parent);

    if (!nodeHasChildren(parent)) // this is a leaf -> terminate
      return 1;

    size_t sum_leafs_children = 0;
    for (unsigned int i=0; i<8; ++i) {
      if (nodeChildExists(parent, i)) {
        sum_leafs_children += getNumLeafNodesRecurs(getNodeChild(parent, i));
      }
    }
    return sum_leafs_children;
  }


  template <class NODE,class I>
  double OcTreeBaseImpl<NODE,I>::volume() {
    double x,  y,  z;
    getMetricSize(x, y, z);
    return x*y*z;
  }


}
