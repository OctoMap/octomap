/**
* dynamicEDT3D:
* A library for incrementally updatable Euclidean distance transforms in 3D.
* @author C. Sprunk, B. Lau, W. Burgard, University of Freiburg, Copyright (C) 2011.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2011-2012, C. Sprunk, B. Lau, W. Burgard, University of Freiburg
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

#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <iostream>



int main( int argc, char *argv[] ) {
  if(argc<=1){
    std::cout<<"usage: "<<argv[0]<<" <octoMap.bt>"<<std::endl;
    exit(0);
  }

  octomap::OcTree *tree = NULL;
  tree = new octomap::OcTree(0.05);

  //read in octotree
  tree->readBinary(argv[1]);

  std::cout<<"read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;

  double x,y,z;
  tree->getMetricMin(x,y,z);
  octomap::point3d min(x,y,z);
  //std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
  tree->getMetricMax(x,y,z);
  octomap::point3d max(x,y,z);
  //std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

  bool unknownAsOccupied = true;
  unknownAsOccupied = false;
  float maxDist = 1.0;
  //- the first argument ist the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  DynamicEDTOctomap distmap(maxDist, tree, min, max, unknownAsOccupied);

  //This computes the distance map
  distmap.update(); 

  //This is how you can query the map
  octomap::point3d p(5.0,5.0,0.6);
  //As we don't know what the dimension of the loaded map are, we modify this point
  p.x() = min.x() + 0.3 * (max.x() - min.x());
  p.y() = min.y() + 0.6 * (max.y() - min.y());
  p.z() = min.z() + 0.5 * (max.z() - min.z());

  octomap::point3d closestObst;
  float distance;

  distmap.getDistanceAndClosestObstacle(p, distance, closestObst);

  std::cout<<"\n\ndistance at point "<<p.x()<<","<<p.y()<<","<<p.z()<<" is "<<distance<<std::endl;
  if(distance < distmap.getMaxDist())
    std::cout<<"closest obstacle to "<<p.x()<<","<<p.y()<<","<<p.z()<<" is at "<<closestObst.x()<<","<<closestObst.y()<<","<<closestObst.z()<<std::endl;

  //if you modify the octree via tree->insertScan() or tree->updateNode()
  //just call distmap.update() again to adapt the distance map to the changes made

  delete tree;

  return 0;
}
