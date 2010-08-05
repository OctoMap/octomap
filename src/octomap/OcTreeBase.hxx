// $Id$

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

    if (
        ( !genKeyValue( point(0), key.k0 ) ) || 
        ( !genKeyValue( point(1), key.k1 ) ) ||
        ( !genKeyValue( point(2), key.k2 ) )
        ) return false;

    return true;
  }

  template <class NODE>
  bool OcTreeBase<NODE>::genCoordFromKey(unsigned short int& key, double& coord) const {

    if (key >= 2*tree_max_val)
      return false;

    coord = ((double(key) - tree_max_val) + 0.5) * resolution;

    return true;
  }


  template <class NODE>
  unsigned int OcTreeBase<NODE>::genPos(OcTreeKey& key, int i) const {

    unsigned int retval = 0;
    if (key.k0 & (1 << i)) retval += 1;
    if (key.k1 & (1 << i)) retval += 2;
    if (key.k2 & (1 << i)) retval += 4;
    return retval;
  }

  template <class NODE>
  NODE* OcTreeBase<NODE>::search(const point3d& value) const {

    // Search is a variant of insert which aborts if
    // it had to insert nodes

    OcTreeKey key;
    if (!genKey(value, key)){
      std::cerr << "Error in search: ["<< value <<"] is out of OcTree bounds!\n";
      return NULL;
    }
    else {
      return this->searchKey(key);
    }

  }

  template <class NODE>
  NODE* OcTreeBase<NODE>::search(double x, double y, double z) const {

    point3d p (x,y,z);
    return this->search(p);
  }


  template <class NODE>
  NODE* OcTreeBase<NODE>::searchKey (OcTreeKey& key) const {

    NODE* curNode = itsRoot;

    // follow nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      unsigned int pos = genPos(key, i);

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
  bool OcTreeBase<NODE>::computeRay(const point3d& origin, const point3d& end, 
                                    std::vector<point3d>& _ray) const {

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    _ray.clear();

    // Initialization phase -------------------------------------------------------

    point3d direction = (end - origin).unit();
    double maxLength = (end - origin).norm2();

    // Voxel integer coordinates are the indices of the OcTree cells
    // at the lowest level (they may exist or not).

    unsigned short int voxelIdx[3];  // voxel integer coords
    unsigned short int endIdx[3];    // end voxel integer coords
    int step[3];                     // step direction

    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      if (!genKeyValue(origin(i), voxelIdx[i])) {
        std::cerr << "Error in OcTree::computeRay(): Coordinate "<<i<<" of origin out of OcTree bounds: "<< origin(i)<<"\n";
        return false;
      }
      if (!genKeyValue(end(i), endIdx[i])) {
        std::cerr << "Error in OcTree::computeRay(): Coordinate "<<i<<" of endpoint out of OcTree bounds"<< end(i)<<"\n";
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
        std::cerr << "WARNING: Ray casting in OcTreeBaseNODE>::getCellsOnRay hit the boundary in dim. "<< i << std::endl;
        return false;
      }

      // generate world coords from tree indices
      double val[3];
      for (unsigned int j = 0; j < 3; j++) {
        if(!genCoordFromKey( voxelIdx[j], val[j] )){
          std::cerr << "Error in OcTree::computeRay(): genCoordFromKey failed!\n";
          return false;
        }
      }
      point3d value(val[0], val[1], val[2]);

      // reached endpoint?
      if ((value - origin).norm2() > maxLength) {
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
  void OcTreeBase<NODE>::getLeafNodes(std::list<OcTreeVolume>& nodes, unsigned int max_depth) const{
    assert(itsRoot);
    if (tree_size <= 1) return; // A tree with only the root is an empty tree (by definition)

    if (max_depth == 0)
      max_depth = tree_depth;

    getLeafNodesRecurs(nodes, max_depth, itsRoot, 0, tree_center);
  }


  template <class NODE>
  void OcTreeBase<NODE>::getLeafNodesRecurs(std::list<OcTreeVolume>& nodes,
                                            unsigned int max_depth,
                                            NODE* node, unsigned int depth,
                                            const point3d& parent_center) const{

    if ((depth <= max_depth) && (node != NULL) ) {

      if (node->hasChildren() && (depth != max_depth)) {

        double center_offset = tree_center(0) / pow( 2., (double) depth+1);
        point3d search_center;

        for (unsigned int i=0; i<8; i++) {
          if (node->childExists(i)) {

            // x-axis
            if (i & 1)	search_center(0) = parent_center(0) + center_offset;
            else  	search_center(0) = parent_center(0) - center_offset;

            // y-axis
            if (i & 2)	search_center(1) = parent_center(1) + center_offset;
            else	search_center(1) = parent_center(1) - center_offset;
            // z-axis
            if (i & 4)	search_center(2) = parent_center(2) + center_offset;
            else	search_center(2) = parent_center(2) - center_offset;

            // cast needed: (nodes need to ensure it's the right pointer)
            NODE* childNode = static_cast<NODE*>(node->getChild(i));
            getLeafNodesRecurs(nodes,max_depth,childNode, depth+1, search_center);

          } // GetChild
        }
      }
      else {    // node is a leaf node or max depth reached
        double voxelSize = resolution * pow(2., double(tree_depth - depth));
        nodes.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
      }
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

            // x-axis
            if (i & 1)
              search_center(0) = parent_center(0) + center_offset;
            else
              search_center(0) = parent_center(0) - center_offset;

            // y-axis
            if (i & 2)
              search_center(1) = parent_center(1) + center_offset;
            else
              search_center(1) = parent_center(1) - center_offset;
            // z-axis
            if (i & 4)
              search_center(2) = parent_center(2) + center_offset;
            else
              search_center(2) = parent_center(2) - center_offset;

            getVoxelsRecurs(voxels, max_depth, node->getChild(i), depth + 1, search_center);

          } else{ // GetChild
            double voxelSize = resolution * pow(2., double(tree_depth - depth));
            voxels.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
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
      std::cerr << __PRETTY_FUNCTION__ << "Warning: Input filestream not \"good\"\n";
    }

    this->tree_size = 0;
    sizeChanged = true;

    // tree needs to be newly created or cleared externally!
    if (itsRoot->hasChildren()) {
      std::cerr << "Error: Trying to read into a tree that is already constructed!\n";
      return s;
    }

    itsRoot->readValue(s);

    tree_size = calcNumNodes();  // compute number of nodes

    return s;
  }


  template <class NODE>
  void OcTreeBase<NODE>::calcMinMax() {
    if (!sizeChanged)
      return;

    //    std::cout << "Recomputing min and max values of OcTree... "<<std::flush;

    for (unsigned i = 0; i< 3; i++){
      maxValue[i] = -std::numeric_limits<double>::max();
      minValue[i] = std::numeric_limits<double>::max();
    }

    std::list<OcTreeVolume> leafs;
    this->getLeafNodes(leafs);

    for (std::list<OcTreeVolume>::const_iterator it = leafs.begin(); it != leafs.end(); ++it){
      double x = it->first(0);
      double y = it->first(1);
      double z = it->first(2);
      double halfSize = it->second/2.0;

      if (x-halfSize < minValue[0]) minValue[0] = x-halfSize;
      if (y-halfSize < minValue[1]) minValue[1] = y-halfSize;
      if (z-halfSize < minValue[2]) minValue[2] = z-halfSize;

      if (x+halfSize > maxValue[0]) maxValue[0] = x+halfSize;
      if (y+halfSize > maxValue[1]) maxValue[1] = y+halfSize;
      if (z+halfSize > maxValue[2]) maxValue[2] = z+halfSize;
    }
    //    std::cout<< "done.\n";
    sizeChanged = false;
  }


  template <class NODE>
  unsigned int OcTreeBase<NODE>::memoryFullGrid() {
    double size_x, size_y, size_z;
    getMetricSize(size_x, size_y,size_z);
    
    // assuming best case (one big array and efficient addressing)
    return (unsigned int) (ceil(1./resolution * (double) size_x) * //sizeof (unsigned int*) *
                           ceil(1./resolution * (double) size_y) * //sizeof (unsigned int*) *
                           ceil(1./resolution * (double) size_z)) *
      sizeof(itsRoot->getValue());

  }


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
  void OcTreeBase<NODE>::getMetricMin(double& x, double& y, double& z){
    calcMinMax();
    
    x = minValue[0];
    y = minValue[1];
    z = minValue[2];
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMin(double& mx, double& my, double& mz) const {
    mx = my = mz = std::numeric_limits<double>::max();
    if (sizeChanged) {
      std::list<OcTreeVolume> leafs;
      this->getLeafNodes(leafs);
      for (std::list<OcTreeVolume>::const_iterator it = leafs.begin(); it != leafs.end(); ++it){
        double x = it->first(0);
        double y = it->first(1);
        double z = it->first(2);
        double halfSize = it->second/2.0;
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
  void OcTreeBase<NODE>::getMetricMax(double& x, double& y, double& z){
    calcMinMax();
    
    x = maxValue[0];
    y = maxValue[1];
    z = maxValue[2];
  }

  template <class NODE>
  void OcTreeBase<NODE>::getMetricMax(double& mx, double& my, double& mz) const {
    mx = my = mz = -std::numeric_limits<double>::max();
    if (sizeChanged) {
      std::list<OcTreeVolume> leafs;
      this->getLeafNodes(leafs);
      for (std::list<OcTreeVolume>::const_iterator it = leafs.begin(); it != leafs.end(); ++it){
        double x = it->first(0);
        double y = it->first(1);
        double z = it->first(2);
        double halfSize = it->second/2.0;
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
  unsigned int OcTreeBase<NODE>::calcNumNodes() const{
    unsigned int retval = 1; // root node
    calcNumNodesRecurs(itsRoot, retval);
    return retval;
  }

  template <class NODE>
  void OcTreeBase<NODE>::calcNumNodesRecurs(NODE* node, unsigned int& num_nodes) const {

    assert (node != NULL);

    if (node->hasChildren()) {
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          num_nodes++;
          calcNumNodesRecurs(node->getChild(i), num_nodes);
        }
      }
    }
  }


}
