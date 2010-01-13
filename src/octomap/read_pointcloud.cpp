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
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using namespace octomap;


int main(int argc, char** argv) {

  if (argc != 4) {

    printf("\nusage: pointcloud.dat laser_height outfile.graph\n\n");

    printf("This tool is used to convert ASCII datasets\n");
    printf("to binary scan graphs. These can be read by\n");
    printf("the viewer (octovis) and the graph2tree tool.\n\n");

    printf("ASCII data is given in the following format:\n");
    printf("x1 y2 z1\n");
    printf("x2 y2 z2\n");
    printf("x3 y3 z3\n");
    printf("...\n\n");

    printf("The laser height above zero is given as a float.\n\n");


    exit(0);
  }


  // read pointcloud from file
  std::ifstream s(argv[1]);
  Pointcloud pc;

  if (!s) {
    std::cout <<"ERROR: could not read " << argv[1] << endl;
    return -1;
  }

  std::string tmp;
  point3d p;
  while (!s.eof()) {

    for (unsigned int i=0; i<3; i++){
      s >> p(i);   
    }
    pc.push_back(p);
  }    

  double laser_offset = atof(argv[2]);
  pose6d offset_trans(0,0,-laser_offset,0,0,0);
  pose6d laser_pose(0,0,laser_offset,0,0,0);
  pc.transform(offset_trans);
  
  ScanGraph graph;
  graph.addNode(&pc, laser_pose);
  graph.writeBinary(argv[3]);
  

}
