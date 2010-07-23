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

    printf("\n");
    printf("usage: pointcloud.dat laser_height outfile.graph\n\n");
    
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

