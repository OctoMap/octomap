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

#include "octomap.h"
#include "OcTreeLabeled.h"

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

