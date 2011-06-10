#ifndef OCTOMAP_OCTREE_KEY_H
#define OCTOMAP_OCTREE_KEY_H

// $Id$

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

#include <assert.h>

namespace octomap {


  /**
   * OcTreeKey is a container class for internal key addressing
   */
  class OcTreeKey {
    
  public:  

    OcTreeKey () {}
    OcTreeKey (unsigned short int a, unsigned short int b, unsigned short int c)
      { k[0] = a; k[1] = b; k[2] = c; }
    OcTreeKey(const OcTreeKey& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2];
    }
    bool operator== (const OcTreeKey &other) const { 
      return ((k[0] == other[0]) && (k[1] == other[1]) && (k[2] == other[2]));
    }

    bool operator!= (const OcTreeKey &other) const {
      return( (k[0] != other[0]) || (k[1] != other[1]) || (k[2] != other[2]) );
    }

    OcTreeKey& operator=(const OcTreeKey& other){
      k[0] = other.k[0]; k[1] = other.k[1]; k[2] = other.k[2];
      return *this;
    }

    const unsigned short int& operator[] (unsigned int i) const { 
      return k[i];
    }

    unsigned short int& operator[] (unsigned int i) { 
      return k[i];
    }

    unsigned short int k[3];

    /// Provides a hash function on Keys
    struct KeyHash{
      size_t operator()(const OcTreeKey& key) const{
        // a very simple hashing function for now:
        return key.k[0] + 1337*key.k[1] + 345637*key.k[2];
      }

    };
  };

  
  class KeyRay {

  public:
    
    KeyRay () {
      ray.resize(100000);
      reset();
    }

    void reset() {
      end_of_ray = begin();
    }

    void addKey(OcTreeKey& k) {
      assert(end_of_ray != ray.end());
      *end_of_ray = k;
      end_of_ray++;
    }

    unsigned int size() const { return end_of_ray - ray.begin(); }
    unsigned int sizeMax() const { return 100000; }

    typedef std::vector<OcTreeKey>::iterator iterator;
    typedef std::vector<OcTreeKey>::const_iterator const_iterator;
    typedef std::vector<OcTreeKey>::reverse_iterator reverse_iterator;
    
    iterator begin() { return ray.begin(); }
    iterator end() { return end_of_ray; }
    const_iterator begin() const { return ray.begin(); }
    const_iterator end() const   { return end_of_ray; }

    reverse_iterator rbegin() { return (reverse_iterator) end_of_ray; }
    reverse_iterator rend() { return ray.rend(); }
   

  public:

    std::vector<OcTreeKey> ray;
    std::vector<OcTreeKey>::iterator end_of_ray;

  };



  inline void computeChildKey (const unsigned int& pos, const unsigned short int& center_offset_key,
                                          const OcTreeKey& parent_key, OcTreeKey& child_key){

    if (pos & 1) child_key[0] = parent_key[0] + center_offset_key;
    else         child_key[0] = parent_key[0] - center_offset_key - (center_offset_key ? 0 : 1);
    // y-axis
    if (pos & 2) child_key[1] = parent_key[1] + center_offset_key;
    else         child_key[1] = parent_key[1] - center_offset_key - (center_offset_key ? 0 : 1);
    // z-axis
    if (pos & 4) child_key[2] = parent_key[2] + center_offset_key;
    else         child_key[2] = parent_key[2] - center_offset_key - (center_offset_key ? 0 : 1);
  }

}

#endif
