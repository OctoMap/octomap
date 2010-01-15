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

using namespace std;
using namespace octomap;


int main(int argc, char** argv) {


  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree (0.1);  // create empty tree with resolution 0.1


  // insert some measurements of occupied cells

  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((double) x*0.05, (double) y*0.05, (double) z*0.05);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  // insert some measurements of free cells

  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((double) x*0.02-1., (double) y*0.02-1., (double) z*0.02-1.);
        tree.updateNode(endpoint, false);  // integrate 'free' measurement
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;

  point3d query (0., 0., 0.);
  OcTreeNode* result = tree.search (query);
  if (result != NULL) 
    cout << "occupancy probability at " << query << ":    " << result->getOccupancy() << endl;
  else 
    cout << "occupancy probability at " << query << ":    is unknown" << endl;    

  query = point3d(-1.,-1.,-1.);
  result = tree.search (query);
  if (result != NULL) 
    cout << "occupancy probability at " << query << ": " << result->getOccupancy() << endl;
  else 
    cout << "occupancy probability at " << query << ":    is unknown" << endl;    

  query = point3d(1.,1.,1.);
  result = tree.search (query);
  if (result != NULL) 
    cout << "occupancy probability at " << query << ":    " << result->getOccupancy() << endl;
  else 
    cout << "occupancy probability at " << query << ":    is unknown" << endl;    




  cout << endl;
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
  cout << "now you can use octovis to visualize: octovis simple_tree.bt" << endl << endl;
  
}

