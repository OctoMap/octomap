// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <bitset>
#include <cassert>
#include <fstream>
#include <stdlib.h>

#include "OcTree.h"
#include "CountingOcTree.h"

// switch to true to disable uniform sampling of scans (will "eat" the floor)
#define NO_UNIFORM_SAMPLING false

namespace octomap {

  OcTree::OcTree(double _resolution)
    : AbstractOcTree<OcTreeNode> (_resolution)  {
    itsRoot = new OcTreeNode();
    tree_size++;
  }

  OcTree::OcTree(std::string _filename)
    : AbstractOcTree<OcTreeNode> (0.1)  {
    itsRoot = new OcTreeNode();
    tree_size++;

    // TODO: creates a warning because the resolution is going to change anyways
    readBinary(_filename);
  }


  OcTree::~OcTree() {
    delete itsRoot;
  }


  OcTreeNode* OcTree::updateNode(const point3d& value, bool occupied) {

    // if (leaf exists)
    //    AND (it is binary) AND (the new information does not contradict the prior):
    //       return leaf
    OcTreeNode* leaf;
    if (this->search(value, leaf)) { // TODO: Possible speedup: avoid search in every insert?
      if ((!leaf->isDelta()) && (leaf->isOccupied() == occupied)) {
        return leaf;
      }
    }

    // generate key for addressing in tree
    unsigned short int key[3];
    for (unsigned int i=0; i<3; i++) {
      if ( !genKey( value(i), key[i]) )
        return NULL;
    }

    return updateNodeRecurs(itsRoot, false, key, 0, occupied);
  }
  

  OcTreeNode* OcTree::updateNodeRecurs(OcTreeNode* node, bool node_just_created,
				       unsigned short int key[3], unsigned int depth,
				       bool occupied) {

    unsigned int pos = genPos(key, tree_depth-1-depth);
    bool created_node = false;

    // follow down to last level
    if (depth < tree_depth) {
      if (!node->childExists(pos)) {
        // child does not exist, but maybe it's a pruned node?
        if ((!node->hasChildren()) && !node_just_created && (node != itsRoot)) {
          // current node does not have children AND it is not a new node 
	  // AND its not the root node
          // -> expand pruned node
          for (unsigned int k=0; k<8; k++) {
            node->createChild(k);
            tree_size++;
            sizeChanged = true;
            //node->getChild(k)->setLabel(node->getLabel());
          }
        }
        else {
          // not a pruned node, create requested child
          node->createChild(pos);
          tree_size++;
          sizeChanged = true;
        }
        created_node = true;
      }
      OcTreeNode* retval = updateNodeRecurs(node->getChild(pos), created_node, 
					    key, depth+1, occupied);

      // set own probability to mean prob of children
      //      node->setLogOdds(node->getMeanChildLogOdds());
      node->setLogOdds(node->getMaxChildLogOdds());  // conservative

      //       std::cout << "depth: " << depth << " node prob: " << node->getLogOdds()
      // 		<< " label: " << (int) node->getLabel() << " isDelta: "
      // 		<< node->isDelta() << " isValid: "  << node->valid() << std::endl;
      return retval;
    }

    // at last level, update node, end of recursion
    else {
      if (occupied) node->integrateHit();
      else          node->integrateMiss();
      return node;
    }

  }


  void OcTree::getOccupied(unsigned int max_depth, double occ_thres,
                           std::list<OcTreeVolume>& binary_nodes, 
			   std::list<OcTreeVolume>& delta_nodes) const{
    getOccupiedRecurs(itsRoot, 0, max_depth, occ_thres, tree_center, binary_nodes, delta_nodes);
  }


  void OcTree::getOccupiedRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth,
				 double occ_thres, const point3d& parent_center,
				 std::list<OcTreeVolume>& binary_nodes,
				 std::list<OcTreeVolume>& delta_nodes) const{

    if (depth < max_depth && node->hasChildren()) {

      double center_offset = tree_center(0) / pow( 2., (double) depth+1);
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

          getOccupiedRecurs(node->getChild(i), depth+1, max_depth, 
			    occ_thres, search_center, binary_nodes, delta_nodes);
        }
      }
    }

    else { // max level reached

      if (node->isOccupied()) {
	double voxelSize = resolution * pow(2., double(tree_depth - depth));
	if (node->isDelta()) {
	  delta_nodes.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
	}
	else {
	  binary_nodes.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
	}
      }
    }
  }


  void OcTree::getFreespace(unsigned int max_depth, double occ_thres,
			    std::list<OcTreeVolume>& binary_nodes, 
			    std::list<OcTreeVolume>& delta_nodes) const{
    getFreespaceRecurs(itsRoot, 0, max_depth, occ_thres, tree_center, binary_nodes, delta_nodes);
  }


  void OcTree::getFreespaceRecurs(OcTreeNode* node, unsigned int depth, 
				  unsigned int max_depth, double occ_thres,
				  const point3d& parent_center, 
				  std::list<OcTreeVolume>& binary_nodes,
				  std::list<OcTreeVolume>& delta_nodes) const{

    if (depth < max_depth && node->hasChildren()) {

      double center_offset = tree_center(0) / pow( 2., (double) depth+1);
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

          getFreespaceRecurs(node->getChild(i), depth+1, max_depth, occ_thres, search_center, binary_nodes, delta_nodes);

        } // GetChild
      } // depth
    }
    else {    // max level reached

      if (!node->isOccupied()) {
        double voxelSize = resolution * pow(2., double(tree_depth - depth));
        if (node->isDelta()) {
          delta_nodes.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
        }
        else {
          binary_nodes.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
        }
      }
      
    }
  }


  void OcTree::prune() {
    for (unsigned int depth=tree_depth-1; depth>0; depth--) {
      unsigned int num_pruned = 0;
      pruneRecurs(this->itsRoot, 0, depth, num_pruned);
      if (num_pruned == 0) break;
    }
  }

  void OcTree::pruneRecurs(OcTreeNode* node, unsigned int depth,
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


  // -----------------------------------------------

  void OcTree::deltaToBinary() {

    // convert bottom up
    for (unsigned int depth=tree_depth; depth>0; depth--) {
      deltaToBinaryRecurs(this->itsRoot, 0, depth);
    }

    // convert root
    if (itsRoot->isDelta()) itsRoot->convertToBinary();
  }

  void OcTree::deltaToBinaryRecurs(OcTreeNode* node, unsigned int depth, 
				   unsigned int max_depth) {

    if (depth < max_depth) {
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          deltaToBinaryRecurs(node->getChild(i), depth+1, max_depth);
        }
      }
    }

    else { // max level reached
      //      printf("level %d\n ", depth);
      if (node->isDelta()) node->convertToBinary();
    }
  }


  void OcTree::insertScan(const ScanNode& scan) {
    if (scan.scan->size()< 1)
      return;

    if (NO_UNIFORM_SAMPLING){
      std::cerr << "Warning: Uniform sampling of scan is disabled!\n";

      octomath::Pose6D scan_pose (scan.pose);

      // integrate beams
      octomap::point3d origin (scan_pose.x(), scan_pose.y(), scan_pose.z());
      octomap::point3d p;

      for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); 
	   point_it != scan.scan->end(); point_it++) {
        p = scan_pose.transform(**point_it);
        this->insertRay(origin, p);
      } // end for all points
    } else{
      this->insertScanUniform(scan);
    }
  }


  bool OcTree::computeRay(const point3d& origin, const point3d& end, 
			  std::vector<point3d>& _ray) const {

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    _ray.clear();

    // Initialization phase -------------------------------------------------------

    octomath::Vector3 direction = (end - origin).unit();
    double maxLength = (end - origin).norm2();

    // Voxel integer coordinates are the indices of the OcTree cells
    // at the lowest level (they may exist or not).

    unsigned short int voxelIdx[3];  // voxel integer coords
    unsigned short int endIdx[3];    // end voxel integer coords
    int step[3];                     // step direction

    double tMax[3];
    double tDelta[3];

    for(unsigned int i=0; i < 3; ++i) {
      if (!genKey(origin(i), voxelIdx[i])) {
        std::cerr << "Error in OcTree::computeRay(): Coordinate "<<i<<" of origin out of OcTree bounds: "<< origin(i)<<"\n";
        return false;
      }
      if (!genKey(end(i), endIdx[i])) {
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
        std::cerr << "WARNING: Ray casting in AbstractOcTreeNODE>::getCellsOnRay hit the boundary in dim. "<< i << std::endl;
        return false;
      }

      // generate world coords from tree indices
      double val[3];
      for (unsigned int j = 0; j < 3; j++) {
        if(!genVal( voxelIdx[j], val[j] )){
          std::cerr << "Error in OcTree::computeRay(): genVal failed!\n";
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


  void OcTree::integrateMissOnRay(const point3d& origin, const point3d& end) {

    std::vector<point3d> ray;
    if (this->computeRay(origin, end, ray)){

      for(std::vector<point3d>::iterator it=ray.begin(); it != ray.end(); it++) {
        //      std::cout << "miss cell " << *it << std::endl;
        updateNode(*it, false); // insert miss cell
      }
    }

  }


  bool OcTree::insertRay(const point3d& origin, const point3d& end){

    integrateMissOnRay(origin, end);
    updateNode(end, true); // insert hit cell

    return true;
  }


  void OcTree::insertScanFreeOrOccupied(const ScanNode& scan, bool freespace) {
    octomath::Pose6D scan_pose (scan.pose);
    octomap::point3d origin (scan_pose.x(), scan_pose.y(), scan_pose.z());
    octomap::point3d p;
    for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); point_it != scan.scan->end(); point_it++) {
      p = scan_pose.transform(**point_it);
      if (freespace) {
        this->integrateMissOnRay(origin, p);
      }
      else {
        updateNode(p, true);
      }
    }
  }

  void OcTree::insertScanUniform(const ScanNode& scan) {
    octomath::Pose6D scan_pose (scan.pose);
    octomap::point3d origin (scan_pose.x(), scan_pose.y(), scan_pose.z());
    octomap::point3d p;

    // preprocess data  --------------------------

    // free cells
    CountingOcTree free_tree(this->getResolution());
    std::vector<point3d> ray;
    for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); point_it != scan.scan->end(); point_it++) {
      p = scan_pose.transform(**point_it);

      if (this->computeRay(origin, p, ray)){
        for(std::vector<point3d>::iterator it=ray.begin(); it != ray.end(); it++) {
          free_tree.updateNode(*it);
        }
      }
    }
    std::list<OcTreeVolume> free_cells;
    free_tree.getLeafNodes(16, free_cells);

    // occupied cells
    CountingOcTree occupied_tree(this->getResolution());
    for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); point_it != scan.scan->end(); point_it++) {
      p = scan_pose.transform(**point_it);
      occupied_tree.updateNode(p);
    }
    std::list<OcTreeVolume> occupied_cells;
    occupied_tree.getLeafNodes(16, occupied_cells);


    // delete free cells if cell is also measured occupied
    for (std::list<OcTreeVolume>::iterator cellit = free_cells.begin(); cellit != free_cells.end();){
      CountingOcTreeNode* hitNode = NULL;
      if ( occupied_tree.search(cellit->first, hitNode) ) {
        cellit = free_cells.erase(cellit);
      }
      else {
        cellit++;
      }
    } // end for


    // insert into tree -----------------------
    for (std::list<OcTreeVolume>::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
      updateNode(it->first, false);
    }
    for (std::list<OcTreeVolume>::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
      updateNode(it->first, true);
    }

    unsigned int num_delta = 0;
    unsigned int num_binary = 0;
    calcNumberOfNodesPerType(num_binary, num_delta);
    std::cout << "Inserted scan, num delta: "<< num_delta << ", num binary: "<< num_binary<< std::endl;

  }


  unsigned int OcTree::memoryUsage() const{

    unsigned int node_size = sizeof(OcTreeNode);
    std::list<OcTreeVolume> leafs;
    this->getLeafNodes(16, leafs);
    unsigned int inner_nodes = tree_size - leafs.size();
    return node_size * tree_size + inner_nodes * sizeof(OcTreeNode*[8]);
  }

  unsigned int OcTree::memoryUsageEightPointers() {

    unsigned int node_size = sizeof (OcTreeNodeEightPointers);

    // number of initialized nodes
    unsigned int num_binary = 0; unsigned int num_delta = 0;
    calcNumberOfNodesPerType(num_binary, num_delta);
    unsigned int total_nodes = num_binary + num_delta;

    return node_size * total_nodes;
  }

  unsigned int OcTree::memoryFullGrid(){
    double size_x, size_y, size_z;
    getMetricSize(size_x, size_y,size_z);


    // assuming best case (one big array and intelligent addressing)
    return (unsigned int) (ceil(1./resolution * (double) size_x) * //sizeof (unsigned int*) *
        ceil(1./resolution * (double) size_y) * //sizeof (unsigned int*) *
        ceil(1./resolution * (double) size_z)) *
        sizeof(GridData);
  }

  void OcTree::getMetricSize(double& x, double& y, double& z){

    double minX, minY, minZ;
    double maxX, maxY, maxZ;

    getMetricMax(maxX, maxY, maxZ);
    getMetricMin(minX, minY, minZ);

    x = maxX - minX;
    y = maxY - minY;
    z = maxZ - minZ;
  }

  void OcTree::getMetricMin(double& x, double& y, double& z){
    if (sizeChanged)
      calcMinMax();

    x = minValue[0];
    y = minValue[1];
    z = minValue[2];
  }

  void OcTree::getMetricMax(double& x, double& y, double& z){
    if (sizeChanged)
          calcMinMax();

    x = maxValue[0];
    y = maxValue[1];
    z = maxValue[2];
  }

  void OcTree::calcMinMax(){
    std::cout << "Recomputing min and max values of OcTree... "<<std::flush;

    std::list<OcTreeVolume> leafs;
    this->getLeafNodes(16, leafs);

    for (std::list<OcTreeVolume>::iterator it = leafs.begin(); it != leafs.end(); ++it){
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
  std::cout<< "done.\n";
  sizeChanged = false;

  }


  void OcTree::calcNumberOfNodesPerType(unsigned int& num_binary,
					unsigned int& num_delta) const{

    num_binary = 0;
    num_delta = 0;
    calcNumberOfNodesPerTypeRecurs(itsRoot, num_binary, num_delta);
  }


  void OcTree::calcNumberOfNodesPerTypeRecurs(OcTreeNode* node,
					      unsigned int& num_binary,
					      unsigned int& num_delta) const{

    assert(node != NULL);

    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        OcTreeNode* child_node = node->getChild(i);
        if (child_node->isDelta()) num_delta++;
        else num_binary++;
        calcNumberOfNodesPerTypeRecurs(child_node, num_binary, num_delta);
      } // end if child
    } // end for children
  }


  unsigned int OcTree::calcNumNodes() const{
    unsigned int retval = 1; // root node
    calcNumNodesRecurs(itsRoot, retval);
    return retval;
  }


  void OcTree::calcNumNodesRecurs(OcTreeNode* node, unsigned int& num_nodes) const{

    assert (node != NULL);

    if (node->hasChildren()) {
      //      num_nodes+=8;
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          num_nodes++;
          calcNumNodesRecurs(node->getChild(i), num_nodes);
        }
      }
    }
  }

  void OcTree::readBinary(std::string filename){
    std::ifstream binary_infile( filename.c_str(), std::ios_base::binary);
    readBinary(binary_infile);
    binary_infile.close();
  }


  std::istream& OcTree::readBinary(std::ifstream &s) {

    if (!s.is_open()){
      std::cerr << "ERROR: Could not read from input filestream in OcTree::readBinary, exiting!\n";
      exit(0);
    } else if (!s.good()){
      std::cerr << "Warning: Input filestream not \"good\" in OcTree::readBinary\n";
    }

    this->tree_size = 0;
    sizeChanged = true;

    int tree_type = -1;
    s.read((char*)&tree_type, sizeof(tree_type));
    if (tree_type != OcTree::TREETYPE){
      std::cerr << "Binary file does not contain an OcTree!\n";
      return s;
    }

    double tree_resolution;
    s.read((char*)&tree_resolution, sizeof(tree_resolution));

    if (fabs(this->resolution - tree_resolution) > 1e-3) {
      std::cerr << "WARNING: resolution of tree (" 
		<< this->resolution << ") and file ("
		<< tree_resolution 
		<<") dont match. Changing tree res.\n";
      this->setResolution(tree_resolution);
    }

    unsigned int tree_read_size = 0;
    s.read((char*)&tree_read_size, sizeof(tree_read_size));
    std::cout << "Reading "
	      << tree_read_size 
	      << " nodes from bonsai tree file..." <<std::flush;

    itsRoot->readBinary(s);

    tree_size = calcNumNodes();  // compute number of nodes including invalid nodes

    std::cout << " done.\n";

    return s;
  }


  void OcTree::writeBinary(std::string filename){
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);
    writeBinary(binary_outfile);
    binary_outfile.close();
  }

  std::ostream& OcTree::writeBinary(std::ostream &s){

    // format:    treetype | resolution | num nodes | [binary nodes]

    this->deltaToBinary();
    this->prune();

    unsigned int tree_type = OcTree::TREETYPE;
    s.write((char*)&tree_type, sizeof(tree_type));

    double tree_resolution = resolution;
    s.write((char*)&tree_resolution, sizeof(tree_resolution));

    unsigned int tree_write_size = this->size(); // size includes invalid nodes
    fprintf(stderr, "writing %d nodes to bonsai tree file...", tree_write_size); fflush(stderr);
    s.write((char*)&tree_write_size, sizeof(tree_write_size));

    itsRoot->writeBinary(s);

    fprintf(stderr, " done.\n");

    return s;
  }

}
