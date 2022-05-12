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

#include <vector>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

int main(int /*argc*/, char** /*argv*/) {

  cout << "generating example map" << endl;

  OcTree tree (0.1);  // create empty tree with resolution 0.1

  // insert some measurements of free cells

  for (float x = -2; x <= 0; x += 0.02f) {
    for (float y = -2; y <= 0; y += 0.02f) {
      for (float z = -2; z <= 0; z += 0.02f) {
        point3d endpoint(x, y, z);
        tree.updateNode(endpoint, false); // integrate 'free' measurement
      }
    }
  }

  // insert some measurements of occupied cells (twice as much)
  for (float x = -1; x <= 0; x += 0.01f) {
    for (float y = -1; y <= 0; y += 0.01f) {
      for (float z = -1; z <= 0; z += 0.01f) {
        point3d endpoint(x, y, z);
        tree.updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }

  point3d origin(-1.5, -1.5, -0.5);
  point3d direction;
  point3d ray_end;

  
  for(float z = 0; z <= 0.25; z += 0.125){
    direction = point3d(1, 1, z);
    cout << endl;
    cout << "casting ray from " << origin  << " in the " << direction << " direction"<< endl;
    bool success = tree.castRay(origin, direction, ray_end);

    if(success){
      cout << "ray hit cell with center " << ray_end << endl;
      
      point3d intersection;
      success = tree.getRayIntersection(origin, direction, ray_end, intersection);
      if(success)
        cout << "entrance point is " << intersection << endl;
    }
  }
	
  return 0;
}
