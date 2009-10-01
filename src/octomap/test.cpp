// $Id: graph2tree.cpp 22 2009-09-28 09:59:31Z ahornung $

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
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace octomap;

int main(int argc, char** argv) {

  OcTree t(0.1);

  for (int i=-100; i<100; i++) {
    for (int j=-100; j<100; j++) {
      for (int k=-100; k<100; k++) {
	point3d p(((double) i)*0.01, ((double) j)*0.01, ((double) k)*0.01);
	t.updateNode(p, true);
      }
    }
  }

  for (int i=-50; i<50; i++) {
    for (int j=-50; j<50; j++) {
      for (int k=-50; k<50; k++) {
	point3d p(((double) i)*0.01+1., ((double) j)*0.01+1., ((double) k)*0.01+1.);
	t.updateNode(p, false);
      }
    }
  }


  unsigned int numBinary, numDelta;
  t.calcNumberOfNodesPerType(numBinary, numDelta);
  cout << "Tree size: " << t.size() 
       <<" ( " <<numBinary<<" binary, "<< numDelta << " delta)\n";
  
  cout << "size should be 10460\n\n";

  t.prune();

  t.calcNumberOfNodesPerType(numBinary, numDelta);
  cout << "Tree size after pruning: " << t.size() 
       <<" ( " <<numBinary<<" binary, "<< numDelta << " delta)\n";

  cout << "size should be 1596\n\n";

  t.writeBinary("testing.bt");

}
