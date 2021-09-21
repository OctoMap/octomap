/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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

#include <octomap/octomap.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>

#ifdef _MSC_VER // fix missing isnan for VC++
#define isnan(x) _isnan(x)  
#endif

// on MacOS, isnan is in std (also C++11)
using namespace std;

using namespace octomap;

void printUsage(char* self){
  std::cerr << "\nUSAGE: " << self << " tree1.ot tree2.ot\n\n";

  std::cerr << "Compare two octrees for accuracy / compression.\n\n";

  exit(0);
}

int main(int argc, char** argv) {

  if (argc != 3 || (argc > 1 && strcmp(argv[1], "-h") == 0)){
    printUsage(argv[0]);
  }

  std::string filename1 = std::string(argv[1]);
  std::string filename2 = std::string(argv[2]);

  cout << "\nReading octree files...\n";

  OcTree* tree1 = dynamic_cast<OcTree*>(OcTree::read(filename1));
  OcTree* tree2 = dynamic_cast<OcTree*>(OcTree::read(filename2));

  if (fabs(tree1->getResolution()-tree2->getResolution()) > 1e-6){
    OCTOMAP_ERROR("Error: Tree resolutions don't match!");
    exit(-1);
  }


  cout << "Expanding octrees... \n";
  // expand both to full resolution:
  tree1->expand();
  tree2->expand();

  if (tree1->getNumLeafNodes() != tree2->getNumLeafNodes()){
      OCTOMAP_ERROR_STR("Octrees have different size: " << tree1->getNumLeafNodes() << "!=" <<tree2->getNumLeafNodes() << endl);
      exit(-1);
  }

  cout << "Expanded num. leafs: " << tree1->getNumLeafNodes() << endl;

  // check bbx:
  double x1, x2, y1, y2, z1, z2;
  tree1->getMetricSize(x1, y1, z1);
  tree2->getMetricSize(x2, y2, z2);

  if ((fabs(x1-x2) > 1e-6)
      || (fabs(y1-y2) > 1e-6)
      || (fabs(z1-z2) > 1e-6))
  {
    OCTOMAP_WARNING("Trees span over different volumes, results may be wrong\n");
    exit(1);
  }

  double kld_sum = 0.0;
  cout << "Comparing trees... \n";
  for (OcTree::leaf_iterator it = tree1->begin_leafs(),
      end = tree1->end_leafs();  it != end; ++it)
  {
    OcTreeNode* n = tree2->search(it.getKey());
    if (!n){
      OCTOMAP_ERROR("Could not find coordinate of 1st octree in 2nd octree\n");
    } else{
      // check occupancy prob:
      double p1 = it->getOccupancy();
      double p2 = n->getOccupancy();
      if (p1 < 0.0 || p1 > 1.0)
        OCTOMAP_ERROR("p1 wrong: %f", p1);
      if (p2 < 0.0 || p2 > 1.0)
        OCTOMAP_ERROR("p2 wrong: %f", p2);

//      if (p1 > 0.1 || p2 > 0.1)
      if (p1 > 0.001 && p2 < 0.001)
        OCTOMAP_WARNING("p2 near 0, p1 > 0 => inf?");
      if (p1 < 0.999 && p2 > 0.999)
         OCTOMAP_WARNING("p2 near 1, p1 < 1 => inf?");

      double kld = 0;
      if (p1 < 0.0001)
        kld =log((1-p1)/(1-p2))*(1-p1);
      else if (p1 > 0.9999)
        kld =log(p1/p2)*p1;
      else
        kld +=log(p1/p2)*p1 + log((1-p1)/(1-p2))*(1-p1);

#if __cplusplus >= 201103L
      if (std::isnan(kld)){
#else
      if (isnan(kld)){
#endif
        OCTOMAP_ERROR("KLD is nan! KLD(%f,%f)=%f; sum = %f", p1, p2, kld, kld_sum);
        exit(-1);
      }

      kld_sum+=kld;

      //if (p1 <)
//      if (fabs(p1-p2) > 1e-6)
//        cout << "diff: " << p1-p2 << endl;
    }


  }

  cout << "KLD: " << kld_sum << endl;



  delete tree1;
  delete tree2;
  
  return 0;
}
