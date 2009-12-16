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
    : OcTreeBase<OcTreeNode> (_resolution)  {
    itsRoot = new OcTreeNode();
    tree_size++;
  }

  OcTree::OcTree(std::string _filename)
    : OcTreeBase<OcTreeNode> (0.1)  { // resolution will be set according to tree file
    itsRoot = new OcTreeNode();
    tree_size++;

    readBinary(_filename);
  }


  OcTree::~OcTree() {
    delete itsRoot;
  }




  void OcTree::prune() {
    for (unsigned int depth=tree_depth-1; depth>0; depth--) {
      unsigned int num_pruned = 0;
      pruneRecurs(this->itsRoot, 0, depth, num_pruned);
      if (num_pruned == 0) break;
    }
  }

  void OcTree::expand() {
    unsigned int num_expanded = 0;
    expandRecurs(itsRoot,0, tree_depth, num_expanded);
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

  void OcTree::expandRecurs(OcTreeNode* node, unsigned int depth,
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


  // -----------------------------------------------

  void OcTree::toMaxLikelihood() {

    // convert bottom up
    for (unsigned int depth=tree_depth; depth>0; depth--) {
      toMaxLikelihoodRecurs(this->itsRoot, 0, depth);
    }

    // convert root
    itsRoot->toMaxLikelihood();
  }

  void OcTree::toMaxLikelihoodRecurs(OcTreeNode* node, unsigned int depth,
				   unsigned int max_depth) {

    if (depth < max_depth) {
      for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
          toMaxLikelihoodRecurs(node->getChild(i), depth+1, max_depth);
        }
      }
    }

    else { // max level reached
      //      printf("level %d\n ", depth);
      node->toMaxLikelihood();
    }
  }


  void OcTree::insertScanFreeOrOccupied(const ScanNode& scan, bool freespace) {
    pose6d scan_pose (scan.pose);
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


  void OcTree::insertScan(const ScanNode& scan) {
    if (scan.scan->size()< 1)
      return;

    if (NO_UNIFORM_SAMPLING){
      std::cerr << "Warning: Uniform sampling of scan is disabled!\n";

      pose6d scan_pose (scan.pose);

      // integrate beams
      octomap::point3d origin (scan_pose.x(), scan_pose.y(), scan_pose.z());
      octomap::point3d p;

      for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); 
	   point_it != scan.scan->end(); point_it++) {
        p = scan_pose.transform(**point_it);
        this->insertRay(origin, p);
      } // end for all points
    } 
    else {
      this->insertScanUniform(scan);
    }
  }


  void OcTree::insertScanUniform(const ScanNode& scan) {
    octomap::pose6d scan_pose (scan.pose);
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
    free_tree.getLeafNodes(free_cells);

    // occupied cells
    CountingOcTree occupied_tree(this->getResolution());
    for (octomap::Pointcloud::iterator point_it = scan.scan->begin(); point_it != scan.scan->end(); point_it++) {
      p = scan_pose.transform(**point_it);
      occupied_tree.updateNode(p);
    }
    std::list<OcTreeVolume> occupied_cells;
    occupied_tree.getLeafNodes(occupied_cells);


    // delete free cells if cell is also measured occupied
    for (std::list<OcTreeVolume>::iterator cellit = free_cells.begin(); cellit != free_cells.end();){
      if ( occupied_tree.search(cellit->first) ) {
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
    this->getLeafNodes(leafs);
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
    this->getLeafNodes(leafs);

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
        if (!child_node->atThreshold()) num_delta++;
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
    if (!binary_infile.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing read.\n";
      return;
    } else {
      readBinary(binary_infile);
      binary_infile.close();
    }
  }


  std::istream& OcTree::readBinary(std::istream &s) {
    if (!s.good()){
      std::cerr << "Warning: Input filestream not \"good\" in OcTree::readBinary\n";
    }


    int tree_type = -1;
    s.read((char*)&tree_type, sizeof(tree_type));
    if (tree_type != OcTree::TREETYPE){
      std::cerr << "Binary file does not contain an OcTree!\n";
      return s;
    }

    this->tree_size = 0;
    sizeChanged = true;

    // clear tree if there are nodes:
    if (itsRoot->hasChildren()){
      delete itsRoot;
      itsRoot = new OcTreeNode();
    }

    double tree_resolution;
    s.read((char*)&tree_resolution, sizeof(tree_resolution));

    this->setResolution(tree_resolution);

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

    if (!binary_outfile.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return;
    } else {
      writeBinary(binary_outfile);
      binary_outfile.close();
    }
  }

  std::ostream& OcTree::writeBinary(std::ostream &s){

    // format:    treetype | resolution | num nodes | [binary nodes]

    this->toMaxLikelihood();
    this->prune();

    return writeBinaryConst(s);
  }

  std::ostream& OcTree::writeBinaryConst(std::ostream &s) const{

    // TODO: this is no longer an indicator of a purely ML tree... =>fix?
    if (!itsRoot->atThreshold()){
      std::cerr << "Error: trying to write a tree with delta nodes to binary!\n";
      return s;
    }

    // format:    treetype | resolution | num nodes | [binary nodes]

    unsigned int tree_type = OcTree::TREETYPE;
    s.write((char*)&tree_type, sizeof(tree_type));

    double tree_resolution = resolution;
    s.write((char*)&tree_resolution, sizeof(tree_resolution));

    unsigned int tree_write_size = this->size(); // size includes invalid nodes
    fprintf(stderr, "writing %d nodes to output stream...", tree_write_size); fflush(stderr);
    s.write((char*)&tree_write_size, sizeof(tree_write_size));

    itsRoot->writeBinary(s);

    fprintf(stderr, " done.\n");

    return s;
  }

}
