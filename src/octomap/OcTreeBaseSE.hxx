// $Id:  $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2010.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2010, K. M. Wurm, A. Hornung, University of Freiburg
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

//     ray.clear();
    ray.reset();

    OcTreeKey key_origin, key_end;
    if ( !OcTreeBase<NODE>::genKey(origin, key_origin) || 
         !OcTreeBase<NODE>::genKey(end, key_end) ) {
      std::cerr << "WARNING: coordinates out of bounds during ray casting" << std::endl;
      return false;
    }

    //    ray.push_back(key_origin);
    ray.addKey(key_origin);
    
    if (key_origin == key_end) return true; // same tree cell, we're done.


    // Initialization phase -------------------------------------------------------

    point3d direction = (end - origin).unit();

    int    step[3];
    double tMax[3];
    double tDelta[3];

    OcTreeKey current_key = key_origin; 

    for(unsigned int i=0; i < 3; ++i) {
      if (direction(i) > 0.0) step[i] =  1;
      else if (direction(i) < 0.0)   step[i] = -1;
      else step[i] = 0;

      double voxelBorder = (double) ( (int) current_key[i] - (int) this->tree_max_val ) * this->resolution;
      if (step[i] > 0) voxelBorder += this->resolution;

      if (direction(i) != 0.0) {
        tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
        tDelta[i] = this->resolution / fabs( direction(i) );
      }
      else {
        tMax[i] = 1e6;
        tDelta[i] = 1e6;
      }
    }

    // Incremental phase  ---------------------------------------------------------

    bool done = false;
    while (!done) {
      unsigned int i;

      // find minimum tMax:
      if (tMax[0] < tMax[1]){
        if (tMax[0] < tMax[2]) i = 0;
        else                   i = 2;
      }
      else {
        if (tMax[1] < tMax[2]) i = 1;
        else                   i = 2;
      }

      // advance in direction "i"
      current_key[i] += step[i];
      tMax[i] += tDelta[i];

      assert ((current_key[i] >= 0) && (current_key[i] < 2*this->tree_max_val));
      
      // reached endpoint?
      if (current_key == key_end){
        done = true;
        break;
      }
      else {
        //        ray.push_back(current_key);
        ray.addKey(current_key);
      }

    } // end while

    return true;
  }



  template <class NODE>
  NODE* OcTreeBaseSE<NODE>::getLUTNeighbor (const point3d& node_coord, OcTreeLUT::NeighborDirection dir) const {

    OcTreeKey start_key;

    if (! OcTreeBase<NODE>::genKey(node_coord, start_key)) {
      std::cerr << "Error in search: ["<< node_coord <<"] is out of OcTree bounds!\n";
      return NULL;
    }

    OcTreeKey neighbor_key;
    lut->genNeighborKey(start_key, (signed char&) dir, neighbor_key);
    return this->searchKey(neighbor_key);
  }



}
