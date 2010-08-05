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
    lastkey[0] = lastkey[1] = lastkey[2] = 0;
  }

  template <class NODE>
  OcTreeBaseLUT<NODE>::~OcTreeBaseLUT () {
    delete lut;
    delete ancestry;
  }


  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::getLUTNeighbor (const point3d& node_coord, OcTreeLUT::NeighborDirection dir) const {

    unsigned short int start_key[3];

    if (   !OcTreeBase<NODE>::genKey(node_coord.x(), start_key[0]) 
        || !OcTreeBase<NODE>::genKey(node_coord.y(), start_key[1]) 
        || !OcTreeBase<NODE>::genKey(node_coord.z(), start_key[2])) {
      std::cerr << "Error in search: ["<< node_coord <<"] is out of OcTree bounds!\n";
      return NULL;
    }

    unsigned short int neighbor_key[3];
    lut->genNeighborKey(start_key, (signed char) dir, neighbor_key);
    return this->searchKey(neighbor_key);
  }


  template <class NODE>
  unsigned int  OcTreeBaseLUT<NODE>::compareKeys (unsigned short int key1[], unsigned short int key2[]) const {
    
    unsigned short int _xor[3];

    for(int k=0; k<3; k++){
      _xor[k] = key1[k]^key2[k];
    }

    unsigned int i(0);

    while (    !( _xor[0] & (32768 >> i) ) 
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
  NODE* OcTreeBaseLUT<NODE>::jump (point3d& coordinate) {
    return this->jump(coordinate(0), coordinate(1), coordinate(2));
  }

  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::jump (double& x, double& y, double& z) {

    unsigned short int key[3];

    if (   !OcTreeBase<NODE>::genKey(x, key[0]) 
        || !OcTreeBase<NODE>::genKey(y, key[1]) 
        || !OcTreeBase<NODE>::genKey(z, key[2])) {
      std::cerr << "Error in search: ["<< x << "," << y << "," << z <<"] is out of OcTree bounds!\n";
      return NULL;
    }
    
    return this->jump(key);
      
  }
  
  template <class NODE>
  NODE* OcTreeBaseLUT<NODE>::jump(unsigned short int (&key)[3]) {
    
    // first jump?
    if (ancestry->at(0) == NULL) {
      
      std::cout << "first jump, creating ancestry. " << std::endl;
      lastkey[0] = key[0];
      lastkey[1] = key[1];
      lastkey[2] = key[2];
      
      // just like OcTreeBase::searchKey -----------------
      NODE* curNode = this->itsRoot;

      // follow nodes down to last level...
      for (int i=(this->tree_depth-1); i>=0; i--) {
        
        ancestry->at(i) = curNode;
        std::cout << i << " > " << ancestry->at(i) << std::endl;
        

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
            ancestry->at(0) = NULL; // reset ancestry
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
        ancestry->at(0) = NULL;
        return jump(key);
      }
      
      // follow nodes down to last level...
      for (int i=branching_point; i>=0; i--) {
        
        ancestry->at(i) = curNode;
        std::cout << i << " > " << ancestry->at(i) << std::endl;
        
        unsigned int pos = OcTreeBase<NODE>::genPos(key, i);
        if (curNode->childExists(pos)) {
          curNode = static_cast<NODE*>( curNode->getChild(pos) );
        }
        else {
          // we expected a child but did not get it
          // is the current node a leaf already?
          if (!curNode->hasChildren()) {
            lastkey[0] = key[0];
            lastkey[1] = key[1];
            lastkey[2] = key[2];
            return curNode;
          }
          else {
            // it is not, search failed
            ancestry->at(0) = NULL; // reset ancestry
            return NULL;
          }
        }
      } // end for
      
      lastkey[0] = key[0];
      lastkey[1] = key[1];
      lastkey[2] = key[2];
      return curNode;
           
    } // end ancestry jump
  }


}
