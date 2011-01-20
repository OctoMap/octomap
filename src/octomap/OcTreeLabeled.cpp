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

#include <fstream>

#include <octomap/OcTree.h>
#include <octomap/OcTreeLabeled.h>


namespace octomap {

  OcTreeLabeled::OcTreeLabeled(double _resolution) 
    : OccupancyOcTreeBase<OcTreeNodeLabeled> (_resolution)  {
    itsRoot = new OcTreeNodeLabeled();
    tree_size++;
  }


  OcTreeLabeled::~OcTreeLabeled() {
    delete itsRoot;
  }


  void OcTreeLabeled::toMaxLikelihood() {

    // convert bottom up
    for (unsigned int depth=tree_depth; depth>0; depth--) {
      toMaxLikelihoodRecurs(this->itsRoot, 0, depth);
    }

    // convert root
    itsRoot->toMaxLikelihood();
  }

  void OcTreeLabeled::toMaxLikelihoodRecurs(OcTreeNodeLabeled* node, unsigned int depth,
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


  void OcTreeLabeled::getChangedFreespace(std::list<OcTreeVolume>& nodes) const{
    getChangedFreespaceRecurs(itsRoot, 0, 16, tree_center, nodes);
  }

  void OcTreeLabeled::getChangedFreespaceRecurs(OcTreeNodeLabeled* node, unsigned int depth, unsigned int max_depth, 
                                                const point3d& parent_center, std::list<OcTreeVolume>& nodes) const {

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

          getChangedFreespaceRecurs(node->getChild(i), depth+1, max_depth, search_center, nodes);

        } // GetChild
      } // depth
    }

   // max level reached
    else {       
      if ( (node->getLabel() == OcTreeNodeLabeled::FREE) && (!node->atThreshold()) ) {

        // node is labeled as FREE (i.e. occupancy prob. has previously reach FREE threshold) 
        // but the occupancy probability is not at threshold any more
        // -> freespace changed 

        double voxelSize = resolution * pow(2., double(tree_depth - depth));
        nodes.push_back(std::make_pair<point3d, double>(parent_center - tree_center, voxelSize));
      }
    }
  }


  void OcTreeLabeled::writeBinary(std::string filename){
    std::ofstream binary_outfile( filename.c_str(), std::ios_base::binary);

    if (!binary_outfile.is_open()){
      std::cerr << "ERROR: Filestream to "<< filename << " not open, nothing written.\n";
      return;
    } 
    else {
      unsigned int tree_type = OcTree::TREETYPE;
      binary_outfile.write((char*)&tree_type, sizeof(tree_type));

      double tree_resolution = resolution;
      binary_outfile.write((char*)&tree_resolution, sizeof(tree_resolution));

      unsigned int tree_write_size = this->size();
      binary_outfile.write((char*)&tree_write_size, sizeof(tree_write_size));
      itsRoot->writeBinary(binary_outfile);
      binary_outfile.close();
    }
  }


} // end namespace

