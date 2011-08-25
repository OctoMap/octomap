// $Id: simple.cpp 271 2011-08-19 10:02:26Z kai_wurm $

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
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace octomap;


void print_query_info(point3d query, ColorOcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    cout << "color of node is: [" << (unsigned int) node->getColor().r << " , "
         << (unsigned int) node->getColor().g << " , " 
         << (unsigned int) node->getColor().b << "]" 
         << endl;    
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;  
}


int main(int argc, char** argv) {

  ColorOcTree tree (0.1);  // create empty tree with resolution 0.1
  // insert some measurements of occupied cells
  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20; z<20; z++) {
        point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
        n->setColor(255,0,0); // set color to red
      }
    }
  }

  // insert some measurements of free cells
  for (int x=-30; x<30; x++) {
    for (int y=-30; y<30; y++) {
      for (int z=-30; z<30; z++) {
        point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
        ColorOcTreeNode* n = tree.updateNode(endpoint, false); 
        n->setColor(0,0,255); // set color to blue
      }
    }
  }

  cout << endl;
  cout << "performing some queries:" << endl;
  
  {
    point3d query (0., 0., 0.);
    ColorOcTreeNode* result = tree.search (query);
    print_query_info(query, result);
    
    query = point3d(-1.,-1.,-1.);
    result = tree.search (query);
    print_query_info(query, result);
    
    query = point3d(1.,1.,1.);
    result = tree.search (query);
    print_query_info(query, result);
  }
 
  std::string filename ("simple_tree.ct");
  // write color tree
  std::ofstream outfile(filename.c_str(), std::ios_base::out | std::ios_base::binary);
  if (outfile.is_open()){
    tree.writeConst(outfile); 
    outfile.close();
    cout << "color tree written "<< filename <<"\n";
  }
  else {
    cout << "could not open file "<< filename << " for writing\n";
    return -1;
  }

  // read tree file
    std::ifstream infile(filename.c_str(), std::ios_base::in |std::ios_base::binary);
  if (!infile.is_open()) {
    cout << "file "<< filename << " could not be opened for writing.\n";
    return -1;
  }

  ColorOcTree read_tree (0.1);
  read_tree.read(infile);
  infile.close();
  cout << "color tree read from "<< filename <<"\n"; 

  // perform queries again
  {
    point3d query (0., 0., 0.);
    ColorOcTreeNode* result = read_tree.search (query);
    print_query_info(query, result);

    query = point3d(-1.,-1.,-1.);
    result = read_tree.search (query);
    print_query_info(query, result);

    query = point3d(1.,1.,1.);
    result = read_tree.search (query);
    print_query_info(query, result);
  }

  return 0;
}
