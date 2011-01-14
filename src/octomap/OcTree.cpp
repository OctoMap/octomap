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

#include <cassert>
#include <fstream>
#include <stdlib.h>

#include "OcTree.h"
#include "CountingOcTree.h"


// switch to true to disable uniform sampling of scans (will "eat" the floor)
#define NO_UNIFORM_SAMPLING false


namespace octomap {

  OcTree::OcTree(double _resolution)
    : OccupancyOcTreeBase<OcTreeNode> (_resolution)  {
    itsRoot = new OcTreeNode();
    tree_size++;
  }

  OcTree::OcTree(std::string _filename)
    : OccupancyOcTreeBase<OcTreeNode> (0.1)  { // resolution will be set according to tree file
    itsRoot = new OcTreeNode();
    tree_size++;

    readBinary(_filename);
  }

  void OcTree::insertScan(const Pointcloud& pc, const pose6d& originPose, double maxrange, bool pruning){
    if (pc.size() < 1)
      return;

    if (NO_UNIFORM_SAMPLING){
      std::cerr << "Warning: Uniform sampling of scan is disabled!\n";

      // integrate beams
      octomap::point3d p;
      for (octomap::Pointcloud::const_iterator point_it = pc.begin();
          point_it != pc.end(); point_it++)
      {
        p = originPose.transform(**point_it);
        this->insertRay(originPose.trans(), p, maxrange);
      } // end for all points
    }
    else {
      this->insertScanUniform(pc, originPose, maxrange);
    }

    if (pruning)
      this->prune();


  }


  void OcTree::insertScan(const ScanNode& scan, double maxrange, bool pruning) {
    insertScan(*(scan.scan), scan.pose, maxrange, pruning);

  }



  void OcTree::toMaxLikelihood() {

    // convert bottom up
    for (unsigned int depth=tree_depth; depth>0; depth--) {
      toMaxLikelihoodRecurs(this->itsRoot, 0, depth);
    }

    // convert root
    itsRoot->toMaxLikelihood();
  }

  
  // -- Information  ---------------------------------  

  unsigned int OcTree::memoryUsage() const{
    unsigned int node_size = sizeof(OcTreeNode);
    std::list<OcTreeVolume> leafs;
    this->getLeafNodes(leafs);
    unsigned int inner_nodes = tree_size - leafs.size();
    return (sizeof(OcTree) + node_size * tree_size + inner_nodes * sizeof(OcTreeNode*[8]));
  }

  
  void OcTree::calcNumThresholdedNodes(unsigned int& num_thresholded, 
                                       unsigned int& num_other) const {
    num_thresholded = 0;
    num_other = 0;
    calcNumThresholdedNodesRecurs(itsRoot, num_thresholded, num_other);
  }


  void OcTree::readBinary(const std::string& filename){
    std::ifstream binary_infile( filename.c_str(), std::ios_base::binary);
    if (!binary_infile.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing read.\n";
      return;
    } else {
      readBinary(binary_infile);
      binary_infile.close();
    }
  }


  // -- I/O  -----------------------------------------

  std::istream& OcTree::readBinary(std::istream &s) {
    
    if (!s.good()){
      std::cerr << "Warning: Input filestream not \"good\" in OcTree::readBinary\n";
    }

    int tree_type = -1;
    s.read((char*)&tree_type, sizeof(tree_type));
    if (tree_type == OcTree::TREETYPE){

      this->tree_size = 0;
      sizeChanged = true;

      // clear tree if there are nodes
      if (itsRoot->hasChildren()) {
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

      tree_size = calcNumNodes();  // compute number of nodes

      std::cout << " done.\n";
    } else if (tree_type == OcTree::TREETYPE+1){
      this->read(s);
    } else{
      std::cerr << "Binary file does not contain an OcTree!\n";
    }

    return s;
  }


  void OcTree::writeBinary(const std::string& filename){
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

    if (!binary_outfile.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return;
    } else {
      writeBinary(binary_outfile);
      binary_outfile.close();
    }
  }

  void OcTree::writeBinaryConst(const std::string& filename) const{
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

    if (!binary_outfile.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return;
    } 
    else {
      writeBinaryConst(binary_outfile);
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

    // format:    treetype | resolution | num nodes | [binary nodes]

    unsigned int tree_type = OcTree::TREETYPE;
    s.write((char*)&tree_type, sizeof(tree_type));

    double tree_resolution = resolution;
    s.write((char*)&tree_resolution, sizeof(tree_resolution));

    unsigned int tree_write_size = this->size(); 
    std::cout << "Writing "<< tree_write_size<<" nodes to output stream..." << std::flush;
    s.write((char*)&tree_write_size, sizeof(tree_write_size));

    itsRoot->writeBinary(s);

    std::cout << " done.\n";

    return s;
  }



  // --  protected  --------------------------------------------

  void OcTree::insertScanUniform(const Pointcloud& pc, const pose6d& scan_pose, double maxrange) {
    
    octomap::point3d origin (scan_pose.trans());


    // preprocess data  --------------------------

    octomap::point3d p;

    CountingOcTree free_tree    (this->getResolution());
    CountingOcTree occupied_tree(this->getResolution());

    for (octomap::Pointcloud::const_iterator point_it = pc.begin(); point_it != pc.end(); point_it++) {

      p = scan_pose.transform(**point_it);

      bool is_maxrange = false;
      if ( (maxrange > 0.0) && ((p - origin).norm() > maxrange) ) is_maxrange = true;

      if (!is_maxrange) {
        // free cells
        if (this->computeRayKeys(origin, p, this->keyray)){
          for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
            free_tree.updateNode(*it);
          }
        }
        // occupied cells
        occupied_tree.updateNode(p);
      } // end if NOT maxrange

      else { // used set a maxrange and this is reached
        point3d direction = (p - origin).normalized();
        point3d new_end = origin + direction * maxrange;
        if (this->computeRayKeys(origin, new_end, this->keyray)){
          for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
            free_tree.updateNode(*it);
          }
        }
      } // end if maxrange


    } // end for all points

    std::list<OcTreeVolume> free_cells;
    free_tree.getLeafNodes(free_cells);

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


    // insert data into tree  -----------------------
    for (std::list<OcTreeVolume>::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
      updateNode(it->first, false);
    }
    for (std::list<OcTreeVolume>::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
      updateNode(it->first, true);
    }

//    unsigned int num_thres = 0;
//    unsigned int num_other = 0;
//    calcNumThresholdedNodes(num_thres, num_other);
//    std::cout << "Inserted scan, total num of thresholded nodes: "<< num_thres << ", num of other nodes: "<< num_other << std::endl;

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
      node->toMaxLikelihood();
    }
  }


  void OcTree::calcNumThresholdedNodesRecurs (OcTreeNode* node,
                                              unsigned int& num_thresholded, 
                                              unsigned int& num_other) const { 
    assert(node != NULL);

    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        OcTreeNode* child_node = node->getChild(i);
        if (child_node->atThreshold()) num_thresholded++;
        else num_other++;
        calcNumThresholdedNodesRecurs(child_node, num_thresholded, num_other);
      } // end if child
    } // end for children
  }

} // namespace
