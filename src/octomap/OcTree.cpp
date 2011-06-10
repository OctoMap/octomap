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

#include <octomap/OcTree.h>
#include <octomap/CountingOcTree.h>


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

  
  
  // -- Information  ---------------------------------  

  
  void OcTree::calcNumThresholdedNodes(unsigned int& num_thresholded, 
                                       unsigned int& num_other) const {
    num_thresholded = 0;
    num_other = 0;
    calcNumThresholdedNodesRecurs(itsRoot, num_thresholded, num_other);
  }



//   // --  protected  --------------------------------------------

//   void OcTree::insertScanUniform(const Pointcloud& pc, const pose6d& scan_pose, double maxrange) {
    
//     octomap::point3d origin (scan_pose.trans());


//     // preprocess data  --------------------------

//     octomap::point3d p;

//     CountingOcTree free_tree    (this->getResolution());
//     CountingOcTree occupied_tree(this->getResolution());

//     for (octomap::Pointcloud::const_iterator point_it = pc.begin(); point_it != pc.end(); point_it++) {

//       p = scan_pose.transform(*point_it);

//       bool is_maxrange = false;
//       if ( (maxrange > 0.0) && ((p - origin).norm() > maxrange) ) is_maxrange = true;

//       if (!is_maxrange) {
//         // free cells
//         if (this->computeRayKeys(origin, p, this->keyray)){
//           for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
//             free_tree.updateNode(*it);
//           }
//         }
//         // occupied cells
//         occupied_tree.updateNode(p);
//       } // end if NOT maxrange

//       else { // used set a maxrange and this is reached
//         point3d direction = (p - origin).normalized();
//         point3d new_end = origin + direction * maxrange;
//         if (this->computeRayKeys(origin, new_end, this->keyray)){
//           for(KeyRay::iterator it=this->keyray.begin(); it != this->keyray.end(); it++) {
//             free_tree.updateNode(*it);
//           }
//         }
//       } // end if maxrange


//     } // end for all points

//     point3d_list free_cells;
//     free_tree.getLeafNodes(free_cells);

//     point3d_list occupied_cells;
//     occupied_tree.getLeafNodes(occupied_cells);


//     // delete free cells if cell is also measured occupied
//     for (point3d_list::iterator cellit = free_cells.begin(); cellit != free_cells.end();){
//       if ( occupied_tree.search(*cellit) ) {
//         cellit = free_cells.erase(cellit);
//       }
//       else {
//         cellit++;
//       }
//     } // end for


//     // insert data into tree  -----------------------
//     for (point3d_list::iterator it = free_cells.begin(); it != free_cells.end(); it++) {
//       updateNode(*it, false);
//     }
//     for (point3d_list::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++) {
//       updateNode(*it, true);
//     }

// //    unsigned int num_thres = 0;
// //    unsigned int num_other = 0;
// //    calcNumThresholdedNodes(num_thres, num_other);
// //    std::cout << "Inserted scan, total num of thresholded nodes: "<< num_thres << ", num of other nodes: "<< num_other << std::endl;

//   }



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
