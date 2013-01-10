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


#include <limits>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

namespace octomap {


  template <class NODE>
  OcTreeBaseSE<NODE>::OcTreeBaseSE (double _resolution) :
    OcTreeBase<NODE>(_resolution) {
    
    lut = new OcTreeLUT (this->tree_depth);
  }

  template <class NODE>
  OcTreeBaseSE<NODE>::~OcTreeBaseSE () {
    delete lut;
  }


  template <class NODE>
  bool OcTreeBaseSE<NODE>::computeRayKeys(const point3d& origin, 
                                          const point3d& end, 
                                          KeyRay& ray) const {

    //    std::cout << "using key ray method\n";

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    ray.reset();

    OcTreeKey key_origin, key_end;
    if ( !OcTreeBase<NODE>::coordToKeyChecked(origin, key_origin) || 
         !OcTreeBase<NODE>::coordToKeyChecked(end, key_end) ) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }

    ray.addKey(key_origin);
    
    if (key_origin == key_end) return true; // same tree cell, we're done.


    // Initialization phase -------------------------------------------------------

    point3d direction = (end - origin);
    double length = direction.norm2();
    direction /= length; // normalize vector

    int    step[3];
    double tMax[3];
    double tDelta[3];

    OcTreeKey current_key = key_origin; 

    for(unsigned int i=0; i < 3; ++i) {

      // compute step direction
      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      // compute tMax, tDelta
      double voxelBorder = this->keyToCoord(current_key[i]); // negative corner point of voxel
      voxelBorder += double(step[i] * this->resolution * 0.5);

      if (step[i] != 0) {
        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] =  std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    // for speedup:
    point3d origin_scaled = origin;  
    origin_scaled /= this->resolution;  
    double length_scaled = length - this->resolution/2.; // safety margin
    length_scaled /= this->resolution;  // scale 
    length_scaled = length_scaled*length_scaled;  // avoid sqrt in dist comp.

    // Incremental phase  ---------------------------------------------------------

    bool done = false;
    while (!done) {

      unsigned int dim;

      // find minimum tMax:
      if (tMax[0] < tMax[1]){
        if (tMax[0] < tMax[2]) dim = 0;
        else                   dim = 2;
      }
      else {
        if (tMax[1] < tMax[2]) dim = 1;
        else                   dim = 2;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];

      assert ((current_key[dim] >= 0) && (current_key[dim] < 2*this->tree_max_val));

      // reached endpoint, key equv?
      if (current_key == key_end) {
        done = true;
        break;
      }
      else {

        // reached endpoint world coords?
        double dist_from_endpoint = 0;
        for (unsigned int j = 0; j < 3; j++) {
          double coord = (double) current_key[j] - (double) this->tree_max_val;
          dist_from_endpoint += (coord - origin_scaled(j)) * (coord - origin_scaled(j));
        }
        if (dist_from_endpoint > length_scaled) {
          done = true;
          break;
        }
        
        else {  // continue to add freespace cells
          ray.addKey(current_key);
        }
      }

      assert ( ray.size() < ray.sizeMax() - 1);
      
    } // end while

    return true;
  }



  template <class NODE>
  NODE* OcTreeBaseSE<NODE>::getLUTNeighbor (const point3d& node_coord, OcTreeLUT::NeighborDirection dir) const {

    OcTreeKey start_key;

    if (! OcTreeBase<NODE>::coordToKeyChecked(node_coord, start_key)) {
      OCTOMAP_ERROR_STR("Error in search: ["<< node_coord <<"] is out of OcTree bounds!");
      return NULL;
    }

    OcTreeKey neighbor_key;
    lut->genNeighborKey(start_key, (signed char&) dir, neighbor_key);
    return this->search(neighbor_key);
  }



}
