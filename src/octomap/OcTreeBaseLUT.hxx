// $Id:  $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
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
  OcTreeBaseLUT<NODE>::OcTreeBaseLUT (double _resolution) :
    OcTreeBase<NODE>(_resolution) {
    
    lut = new OcTreeLUT (this->tree_depth);

    ancestry = new std::vector<NODE*> (this->tree_depth);
    for (unsigned int i=0; i<this->tree_depth; i++) ancestry->at(i) = NULL;
  }

  template <class NODE>
  OcTreeBaseLUT<NODE>::~OcTreeBaseLUT () {
    delete lut;
    delete ancestry;
  }


  template <class NODE>
  void OcTreeBaseLUT<NODE>::discardAncestry () {
    ancestry->at(0) = NULL;
  }


  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::getLUTNeighbor (const point3d& node_coord, OcTreeLUT::NeighborDirection dir) const {

    OcTreeKey start_key;

    if (! OcTreeBase<NODE>::genKey(node_coord, start_key)) {
      std::cerr << "Error in search: ["<< node_coord <<"] is out of OcTree bounds!\n";
      return NULL;
    }

    OcTreeKey neighbor_key;
    lut->genNeighborKey(start_key, (signed char&) dir, neighbor_key);
    return this->searchKey(neighbor_key);
  }


  template <class NODE>
  unsigned int  OcTreeBaseLUT<NODE>::compareKeys (OcTreeKey& key1, OcTreeKey& key2) const {
    
    unsigned short int _xor[3];

    for (unsigned int i=0;i<3;i++) {
      _xor[i] = key1[i]^key2[i];
    }

    unsigned int i(0);

    while (!( _xor[0] & (32768 >> i) ) 
           && !( _xor[1] & (32768 >> i) )
           && !( _xor[2] & (32768 >> i) )
           &&  (i < this->tree_depth)
           ) {
      i++;
    }
    // 15 >= i >= 0
    return (this->tree_depth-1) - i;
  };



  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::jump (const point3d& point) {

    OcTreeKey key;

    if ( !OcTreeBase<NODE>::genKey(point, key)) {
      std::cerr << "Error in search: ["<< point <<"] is out of OcTree bounds!\n";
      return NULL;
    }
    return this->jump(key);      
  }

  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::jump (double& x, double& y, double& z) {
    point3d p(x,y,z);
    return this->jump(p);
  }


  
  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::jump (OcTreeKey& key) {
    
    // first jump?
    if (ancestry->at(0) == NULL) {
      
      std::cout << "first jump, creating ancestry. " << std::endl;
      lastkey = key;
      
      // just like OcTreeBase::searchKey -----------------
      NODE* curNode = this->itsRoot;

      // follow nodes down to last level...
      for (int i=(this->tree_depth-1); i>=0; i--) {
        
        ancestry->at(i) = curNode;
        //        std::cout << i << " > " << ancestry->at(i) << std::endl;
        

        unsigned int pos = OcTreeBase<NODE>::genPos(key, i);
        if (curNode->childExists(pos)) {
          curNode = static_cast<NODE*>( curNode->getChild(pos) );
        }
        else {
          // we expected a child but did not get it
          // is the current node a leaf already?
          if (!curNode->hasChildren()) {
            return curNode;
          }
          else {
            // it is not, search failed
//             std::cout << "jump failed (search failed), discarding ancestry"<< std::endl;
//             this->discardAncestry();  // do we need to discard here?
            return NULL;
          }
        }
      } // end for
      
      return curNode;
      // -------------------------------------------------
    }
    
    // use ancestry of last jump
    else {
      unsigned short int branching_point = compareKeys(key, lastkey);
      std::cout << "branching point: " << branching_point << std::endl;

      NODE* curNode = ancestry->at(branching_point);

      if (!curNode) {
        std::cout << "error while jumping, branching node does not exist." << std::endl;
        this->discardAncestry();
        return jump(key);
      }
      
      // follow nodes down to last level...
      for (int i=branching_point; i>=0; i--) {
        
        ancestry->at(i) = curNode;
        //        std::cout << i << " > " << ancestry->at(i) << std::endl;
        
        unsigned int pos = OcTreeBase<NODE>::genPos(key, i);
        if (curNode->childExists(pos)) {
          curNode = static_cast<NODE*>( curNode->getChild(pos) );
        }
        else {
          // we expected a child but did not get it
          // is the current node a leaf already?
          if (!curNode->hasChildren()) {
            lastkey = key;
            return curNode;
          }
          else {
            // it is not, search failed
//             std::cout << "jump failed (not found), discarding ancestry"<< std::endl;
//             this->discardAncestry();
            return NULL;
          }
        }
      } // end for
      
      lastkey = key;
      return curNode;
           
    } // end ancestry jump
  }


  template <class NODE>
  void OcTreeBaseLUT<NODE>::prune() {
    OcTreeBase<NODE>::prune();
    this->discardAncestry();
  }
  

  template <class NODE>
  bool OcTreeBaseLUT<NODE>::computeRayKeys(const point3d& origin, const point3d& end, 
                                          std::vector<OcTreeKey>& ray) const {

    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // basically: DDA in 3D

    ray.clear();

    OcTreeKey key_origin, key_end;
    if ( !OcTreeBase<NODE>::genKey(origin, key_origin) || 
         !OcTreeBase<NODE>::genKey(end, key_end) ) {
      std::cerr << "WARNING: endpoint out of bounds during ray casting" << std::endl;
      return false;
    }

    ray.push_back(key_origin);
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
        ray.push_back(current_key);
      }

    } // end while

    return true;
  }




}
