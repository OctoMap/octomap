/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
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

#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>

using namespace std;
using namespace octomap;

void printUsage(char* self){
  cerr << "USAGE: " << self << " <InputFile.bt> <OutputFile.pcd>\n";
  cerr << "This tool creates a point cloud of the occupied cells\n";
  exit(0);
}


int main(int argc, char** argv) {
  if (argc != 3)
    printUsage(argv[0]);

  string inputFilename = argv[1];
  string outputFilename = argv[2];

  OcTree* tree = new OcTree(0.1);
  if (!tree->readBinary(inputFilename)){
    OCTOMAP_ERROR("Could not open file, exiting.\n");
    exit(1);
  }

  unsigned int maxDepth = tree->getTreeDepth();
  cout << "tree depth is " << maxDepth << endl;
  
  // expand collapsed occupied nodes until all occupied leaves are at maximum depth
  vector<OcTreeNode*> collapsed_occ_nodes;
  do {
    collapsed_occ_nodes.clear();
    for (OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
    {
      if(tree->isNodeOccupied(*it) && it.getDepth() < maxDepth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (vector<OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
    {
      tree->expandNode(*it);
    }
    cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
  } while(collapsed_occ_nodes.size() > 0);

  vector<point3d> pcl;
  for (OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
  {
    if(tree->isNodeOccupied(*it))
    {
      pcl.push_back(it.getCoordinate());
    }
  }

  delete tree;

  ofstream f(outputFilename.c_str(), ofstream::out);
  f << "# .PCD v0.7" << endl
    << "VERSION 0.7" << endl
    << "FIELDS x y z" << endl
    << "SIZE 4 4 4" << endl
    << "TYPE F F F" << endl
    << "COUNT 1 1 1" << endl
    << "WIDTH " << pcl.size() << endl
    << "HEIGHT 1" << endl
    << "VIEWPOINT 0 0 0 0 0 0 1" << endl
    << "POINTS " << pcl.size() << endl
    << "DATA ascii" << endl;
  for (size_t i = 0; i < pcl.size(); i++)
      f << pcl[i].x() << " " << pcl[i].y() << " " << pcl[i].z() << endl;
  f.close();
  
  return 0;
}
