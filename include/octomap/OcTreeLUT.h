#ifndef OCTOMAP_OCTREE_LUT_H
#define OCTOMAP_OCTREE_LUT_H

// $Id$

/**
 * OctoMap:
 * A probabilistic, flexible, and compact 3D mapping library for robotic systems.
 * @author Raphael Schmitt, K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2010.
 * @see http://octomap.sourceforge.net/
 * License: New BSD License
 */

/*
 * Copyright (c) 2010, Raphael Schmitt, K. M. Wurm, A. Hornung, University of Freiburg
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


#include "OcTreeLUTdefs.h"
#include "octomap_types.h"
#include "OcTreeKey.h"

namespace octomap {


  //! comparator for keys
  struct equal_keys {
    bool operator() (const unsigned short int* key1, const unsigned short int* key2) const {
      return ((key1[0]==key2[0]) && (key1[1] == key2[1]) && (key1[2] == key2[2]));
    }
  };

  struct hash_key {
    unsigned short int operator()(const unsigned short int* key) const {
      return (((31 + key[0]) * 31 + key[1]) * 31 + key[2]);
    }
  };


  
  /**
   *   Implements a lookup table that allows to computer keys of neighbor cells directly, 
   *   see: Samet 1989, "Implementing ray tracing with octrees and neighbor finding"
   */
  class OcTreeLUT {

  public:

    /**
     *  (N)orth: positive X   (S)outh:  negative X
     *  (W)est : positive Y   (E)ast:   negative Y
     *  (T)op  : positive Z   (B)ottom: negative Z
     */

    typedef enum {
      W = 0, E, N, S , T , B,                         // face neighbors
      SW, NW, SE, NE, TW, BW, TE, BE, TN, TS, BN, BS, // edge neighbors
      TNW, TSW, TNE, TSE, BNW, BSW, BNE, BSE          // vertex neighbors
    } NeighborDirection;


  public:

    OcTreeLUT(unsigned int _max_depth);
    ~OcTreeLUT();
    
    bool genNeighborKey(OcTreeKey node_key, signed char& dir, OcTreeKey neighbor_key) const;

  protected:

    void initLUT();

    unsigned int genPos(OcTreeKey key, int i) const;
    void changeKey(int val, OcTreeKey key, unsigned short int i) const;

  protected:

    unsigned int max_depth;

    signed char nf_values[8][26];
    signed char nf_rec_values[8][26];
    signed char nf_multiple_values[26][4];
  }; 

} // namespace

#endif
