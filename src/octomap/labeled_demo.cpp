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

#include <octomap/octomap.h>
#include <octomap/OcTreeLabeled.h>

using namespace std;
using namespace octomap;


string echo_node_label(OcTreeNodeLabeled::Label l) {
  if (l == OcTreeNodeLabeled::FREE) return "FREE";
  if (l == OcTreeNodeLabeled::OCCUPIED) return "OCCUPIED";
  if (l == OcTreeNodeLabeled::MIXED) return "MIXED";
  else return "UNKNOWN";
}

void print_node_info(OcTreeNodeLabeled* node) {
 if (node != NULL) {
    cout << "occupancy probability : " << node->getOccupancy() << endl;
    cout << "label                 : " << echo_node_label(node->getLabel()) << endl;
  }
  else 
    cout << "occupancy probability : is unknown" << endl;    
}

int main(int argc, char** argv) {

  cout << endl;
  cout << "generating example map..." << endl;
  
  OcTreeLabeled tree(0.1);

  for (int x=-10; x<10; x++) {
    for (int y=-10; y<10; y++) {
      for (int z=-10; z<10; z++) {
        point3d endpoint ((double) x*0.09, (double) y*0.09, (double) z*0.09);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  // add more measurements in one corner

  for (int x=-10; x<10; x++) {
    for (int y=-10; y<10; y++) {
      for (int z=-10; z<10; z++) {
        point3d endpoint ((double) x*0.04 + 0.8, (double) y*0.04 + 0.8, (double) z*0.04 + 0.8);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  cout << endl;

  cout << "our environment now consists of two intersecting cubes of freespace." << endl;
  double minx, miny,minz;
  double maxx, maxy,maxz;
  tree.getMetricMin(minx, miny, minz);
  tree.getMetricMax(maxx, maxy, maxz);

  cout << "its bounding box is: (" << minx << "," << miny  << "," << minz  
       << ")  x  (" << maxx  << "," << maxy  << "," << maxz << ")" << endl;

  cout << endl;

  point3d query (0.9, 0.9, 0.9);
  cout << "let's query a point in the intersection: " << query << endl;

  OcTreeNodeLabeled* result = tree.search (query);
  print_node_info(result);

  cout << endl;

  query = point3d(0.1,0.1,0.1);
  cout << "point " << query << " has not been measured that often"  << endl;
  result = tree.search (query);
  print_node_info(result);

  cout << endl;

  query = point3d(0.9,0.9,0.9);
  cout << "after intergrating an 'occupied' measurement at point " << query << endl;

  tree.updateNode(query, true);  // integrate 'occupied' measurement

  result = tree.search (query);
  print_node_info(result);



  cout << endl;
  cout << "generate a list of nodes with a FREE label but occupancy" << endl;
  cout << "probabilities which are not at the threshold. These are" << endl;
  cout << "their center points:" << endl;
  cout << endl;

  std::list<OcTreeVolume> changed_nodes;
  tree.getChangedFreespace(changed_nodes);
  if (changed_nodes.size()) {
    for (std::list<OcTreeVolume>::iterator it = changed_nodes.begin(); it != changed_nodes.end(); it++) {
      cout << it->first << endl;
    }
  }
  else {
      cout << "no freespace nodes have changed" << endl;
  }

  cout << endl;
  tree.writeBinary("label_test.bt");
  cout << "wrote example file label_test.bt" << endl;
  cout << "you can use octovis to visualize the data: octovis label_tree.bt" << endl;
  cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;
 
}

