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


namespace octomap {


  /*template <class NODE>
  const OcTreeBase<NODE>::leaf_iterator OcTreeBase<NODE>::leaf_iterator_end = OcTreeBase<NODE>::leaf_iterator();*/

  template <class NODE>
  OcTreeBase<NODE>::OcTreeBase(double _resolution) :
    itsRoot(NULL), tree_depth(16), tree_max_val(32768), 
    resolution(_resolution), tree_size(0) {
    
    this->setResolution(_resolution);
    for (unsigned i = 0; i< 3; i++){
      maxValue[i] = -std::numeric_limits<double>::max();
      minValue[i] = std::numeric_limits<double>::max();
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

    tree_center(0) = tree_center(1) = tree_center(2) = ((double) tree_max_val) / resolution_factor;
  }


  template <class NODE>
  bool OcTreeBase<NODE>::genKeyValue(double coordinate, unsigned short int& keyval) const {

    // scale to resolution and shift center for tree_max_val
    int scaled_coord =  ((int) floor(resolution_factor * coordinate)) + tree_max_val;

    // keyval within range of tree?
    if (( scaled_coord > 0) && (((unsigned int) scaled_coord) < (2*tree_max_val))) {
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

    if(!diff)
    {
      out_keyval = keyval;
    }
    else
    {
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
  bool OcTreeBase<NODE>::genCoordFromKey(const unsigned short int& key, float& coord) const {

    if (key >= 2*tree_max_val)
      return false;

    coord = float(genCoordFromKey(key));

    return true;
  }

  template <class NODE>
  double OcTreeBase<NODE>::genCoordFromKey(const unsigned short int& key) const {

    return (double( (int) key - (int) this->tree_max_val ) +0.5) * this->resolution;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genLastCoordFromKey(const unsigned short int& key, float& coord) const {

    if (key >= 2*tree_max_val)
      return false;

    coord = ((double) ( (int) key - (int) this->tree_max_val ) + 0.5) * this->resolution;

    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genCoords(const OcTreeKey& key, unsigned int depth, point3d& point) const {

    if(depth < this->tree_depth)
    {
      for (unsigned int i=0; i<3; ++i) {
        if ( !genCoordFromKey(key[i], point(i)) ) {
          return false;
        }
      }
    }
    else
    {
      for (unsigned int i=0; i<3; ++i) {
        if ( !genLastCoordFromKey(key[i], point(i)) ) {
          return false;
        }
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
  void OcTreeBase<NODE>::computeChildCenter (const unsigned int& pos, const double& center_offset, 
                                             const point3d& parent_center, point3d& child_center) const {
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
  void OcTreeBase<NODE>::prune() {
  
    for (unsigned int depth=tree_depth-1; depth>0; depth--) {
      unsigned int num_pruned = 0;
      pruneRecurs(this->itsRoot, 0, depth, num_pruned);
      if (num_pruned == 0) break;
    }
   
  }

  template <class NODE>
  void OcTreeBase<NODE>::expand() {
    unsigned int num_expanded = 0;
    expandRecurs(itsRoot,0, tree_depth, num_expanded);
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
                << origin << " -> " << end << ") out of bounds during ray casting");
      return false;
    }

    ray.addKey(key_origin);
    
    if (key_origin == key_end) return true; // same tree cell, we're done.


    // Initialization phase -------------------------------------------------------

    point3d direction = (end - origin);
    double length = direction.norm();
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
                                    std::vector<point3d>& _ray) const {

    _ray.clear();

    // Initialization phase -------------------------------------------------------

    point3d direction = (end - origin).normalized ();
    double maxLength = (end - origin).norm ();

    // Voxel integer coordinates are the indices of the OcTree cells
    // at the lowest level (they may exist or not).

    unsigned short int voxelIdx[3];  // voxel integer coords
    unsigned short int endIdx[3];    // end voxel integer coords
    int step[3];                     // step direction

    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      if (!genKeyValue(origin(i), voxelIdx[i])) {
        OCTOMAP_ERROR_STR("Error in OcTree::computeRay(): Coordinate "<<i<<" of origin out of OcTree bounds: "<< origin(i));
        return false;
      }
      if (!genKeyValue(end(i), endIdx[i])) {
        OCTOMAP_ERROR_STR("Error in OcTree::computeRay(): Coordinate "<<i<<" of endpoint out of OcTree bounds"<< end(i));
        return false;
      }


      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      double voxelBorder = (double) ( (int) voxelIdx[i] - (int) tree_max_val ) * resolution;
      if (step[i] > 0) voxelBorder += resolution;

      if (direction(i) != 0.0) {
        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = resolution / fabs( direction(i) );
      }
      else {
        tMax[i] = 1e6;
        tDelta[i] = 1e6;
      }
    }

    // origin and end in same cell: done (ray empty)
    if (voxelIdx[0] == endIdx[0] && voxelIdx[1] == endIdx[1] && voxelIdx[2] == endIdx[2]){
      return true;
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

      if ((voxelIdx[i] >= 0) && (voxelIdx[i] < 2*tree_max_val)){
        tMax[i] += tDelta[i];
      }
      else {
        OCTOMAP_WARNING_STR("Ray casting in OcTreeBaseNODE>::getCellsOnRay hit the boundary in dim. "<< i);
        return false;
      }

      // generate world coords from tree indices
      double val[3];
      for (unsigned int j = 0; j < 3; j++) {
        if(!genCoordFromKey( voxelIdx[j], val[j] )){
          OCTOMAP_ERROR_STR("Error in OcTree::computeRay(): genCoordFromKey failed!");
          return false;
          val[j] += this->resolution * 0.5;  // center of voxel          
        }
      }
      point3d value(val[0], val[1], val[2]);

      // reached endpoint?
      if ((value - origin).norm () > maxLength) {
        done = true;
        break;
      }
      else {
        _ray.push_back(value);
      }
    } // end while

    return true;
  }


  template <class NODE>
  void OcTreeBase<NODE>::getLeafNodes(point3d_list& node_centers) const{
    assert(itsRoot);
    if (tree_size <= 1) return; // A tree with only the root node is an empty tree (by definition)

    getLeafNodesRecurs(node_centers, tree_depth, itsRoot, 0, tree_center);
  }


  template <class NODE>
  void OcTreeBase<NODE>::getLeafNodesRecurs(point3d_list& node_centers,
                                            unsigned int max_depth,
                                            NODE* node, unsigned int depth,
                                            const point3d& parent_center) const {

    if ((depth <= max_depth) && (node != NULL) ) {

      if (node->hasChildren() && (depth != max_depth)) {

        double center_offset = tree_center(0) / pow( 2., (double) depth+1);
        point3d search_center;

        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {

            computeChildCenter(i, center_offset, parent_center, search_center);
            // cast needed: (nodes need to ensure it's the right pointer)
            NODE* childNode = static_cast<NODE*>(node->getChild(i));
            getLeafNodesRecurs(node_centers, max_depth, childNode, depth+1, search_center);

          } // GetChild
        }
      }
      else {    // node is a leaf node or max depth reached
//         double voxelSize = resolution * pow(2., double(tree_depth - depth));
        node_centers.push_back(parent_center - tree_center);
      }
    }

  }


  // deprecated!
  template <class NODE>
  void OcTreeBase<NODE>::getLeafNodes(std::list<OcTreeVolume>& nodes, unsigned int max_depth) const {
    point3d_list node_centers;
    this->getLeafNodes(node_centers);
    for (point3d_list::iterator it = node_centers.begin(); it != node_centers.end(); it++) {
      OcTreeVolume v = std::make_pair(*it, this->resolution);
      nodes.push_back(v);
    }
  }


  template <class NODE>
  void OcTreeBase<NODE>::getVoxels(std::list<OcTreeVolume>& voxels, unsigned int max_depth) const{
    assert(itsRoot);

    if (max_depth == 0)
      max_depth = tree_depth;

    getVoxelsRecurs(voxels,max_depth,itsRoot, 0, tree_center);
  }



  template <class NODE>
  void OcTreeBase<NODE>::getVoxelsRecurs(std::list<OcTreeVolume>& voxels,
                                         unsigned int max_depth,
                                         NODE* node, unsigned int depth,
                                         const point3d& parent_center) const{

    if ((depth <= max_depth) && (node != NULL) ) {
      if (node->hasChildren() && (depth != max_depth)) {

        double center_offset = tree_center(0) / pow(2., (double) depth + 1);
        point3d search_center;

        for (unsigned int i = 0; i < 8; i++) {
          if (node->childExists(i)) {

            computeChildCenter(i, center_offset, parent_center, search_center);
            getVoxelsRecurs(voxels, max_depth, node->getChild(i), depth + 1, search_center);

          }
        } // depth
      }
      double voxelSize = resolution * pow(2., double(tree_depth - depth));
      voxels.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
    }
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
                            unsigned int max_depth, unsigned int& num_expanded) {

    if (depth < max_depth) {
      // current node has no children => can be expanded
      if (!node->hasChildren()){
        node->expandNode();
        num_expanded +=8;
        tree_size +=8;
        sizeChanged = true;
      }

      // recursively expand children:
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          expandRecurs(node->getChild(i), depth+1, max_depth, num_expanded);
        }
      }
    } // end if depth
  }


  template <class NODE>
  std::ostream& OcTreeBase<NODE>::write(std::ostream &s){
    this->prune();

    return this->writeConst(s);
  }

  template <class NODE>
  std::ostream& OcTreeBase<NODE>::writeConst(std::ostream &s) const{
    itsRoot->writeValue(s);

    return s;
  }

  template <class NODE>
  std::istream& OcTreeBase<NODE>::read(std::istream &s) {

    if (!s.good()){
      OCTOMAP_WARNING_STR(__PRETTY_FUNCTION__ << "Warning: Input filestream not \"good\"");
    }

    this->tree_size = 0;
    sizeChanged = true;

    // tree needs to be newly created or cleared externally!
    if (itsRoot->hasChildren()) {
      OCTOMAP_ERROR_STR("Trying to read into a tree that is already constructed!");
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

    calcMinMaxRecurs(this->itsRoot,  0, tree_center);

    sizeChanged = false;
  }

  template <class NODE>
  void OcTreeBase<NODE>::calcMinMaxRecurs(NODE* node, unsigned int depth,
                                          const point3d& parent_center) {

    // terminate recursion
    if (!node->hasChildren()) { // node is a leaf ...
      float voxel_size = resolution;  // ... at lowest level
      if (depth < tree_depth) // ... and a pruned node
        voxel_size = resolution * pow(2., double(tree_depth - depth));

      point3d node_center = parent_center - tree_center;
      // update min/max
      float& x = node_center.x();
      float& y = node_center.y();
      float& z = node_center.z();
      float half_size = voxel_size/2.;

      if (x-half_size < minValue[0]) minValue[0] = x-half_size;
      if (y-half_size < minValue[1]) minValue[1] = y-half_size;
      if (z-half_size < minValue[2]) minValue[2] = z-half_size;

      if (x+half_size > maxValue[0]) maxValue[0] = x+half_size;
      if (y+half_size > maxValue[1]) maxValue[1] = y+half_size;
      if (z+half_size > maxValue[2]) maxValue[2] = z+half_size;

      return;
    }

    // recursive call
    double center_offset = tree_center(0) / pow( 2., (double) depth+1);
    point3d search_center;
    
    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        computeChildCenter(i, center_offset, parent_center, search_center);
        NODE* childNode = static_cast<NODE*>(node->getChild(i));
        calcMinMaxRecurs(childNode, depth+1, search_center);
      }
    }

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
    mx = my = mz = std::numeric_limits<double>::max();
    if (sizeChanged) {
      point3d_list leafs;
      this->getLeafNodes(leafs);
      for (point3d_list::const_iterator it = leafs.begin(); it != leafs.end(); ++it){
        double x = it->x();
        double y = it->y();
        double z = it->z();
        double halfSize = this->resolution/2.0;
        if (x-halfSize < mx) mx = x-halfSize;
        if (y-halfSize < my) my = y-halfSize;
        if (z-halfSize < mz) mz = z-halfSize;
      }
    } else {
      mx = minValue[0];
      my = minValue[1];
      mz = minValue[2];
    }
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMax(double& mx, double& my, double& mz) const {
    mx = my = mz = -std::numeric_limits<double>::max();
    if (sizeChanged) {
      point3d_list leafs;
      this->getLeafNodes(leafs);
      for (point3d_list::const_iterator it = leafs.begin(); it != leafs.end(); ++it){
        double x = it->x();
        double y = it->y();
        double z = it->z();
        double halfSize = this->resolution/2.0;
        if (x+halfSize > mx) mx = x+halfSize;
        if (y+halfSize > my) my = y+halfSize;
        if (z+halfSize > mz) mz = z+halfSize;
      }
    } else {
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

    size_t node_size = sizeof(NODE);
    size_t num_leaf_nodes = this->getNumLeafNodes();
    size_t num_inner_nodes = tree_size - num_leaf_nodes;

    return (sizeof(OcTreeBase<NODE>) + node_size * tree_size + num_inner_nodes * sizeof(NODE*[8]));
  }



  template <class NODE>
  void OcTreeBase<NODE>::getUnknownLeafCenters(point3d_list& node_centers, point3d min, point3d max) const {

    float diff[3];
    unsigned int steps[3];
    for (int i=0;i<3;++i) {
      diff[i] = max(i) - min(i);
      steps[i] = floor(diff[i] / this->resolution);
      //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
    }
    
    point3d p = min;
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
        p.z() = min.z();
      }
      p.y() = min.y();
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
