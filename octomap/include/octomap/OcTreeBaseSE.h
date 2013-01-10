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

#ifndef OCTOMAP_OCTREE_BASE_SE_H
#define OCTOMAP_OCTREE_BASE_SE_H


#include "OcTreeBase.h"
#include "OcTreeLUT.h"


namespace octomap {


  template <class NODE>
    class OcTreeBaseSE: public OcTreeBase<NODE> {

  public:

    OcTreeBaseSE(double _resolution);
    virtual ~OcTreeBaseSE();
    
   /**
    * Traces a ray from origin to end (excluding), returning the
    * coordinates of all nodes traversed by the beam.
    * (Essentially using the DDA algorithm in 3D).
    *
    * @param origin start coordinate of ray
    * @param end end coordinate of ray
    * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
    * @return Success of operation. Returning false usually means that one of the coordinates is out of the OcTree's range
    */
    bool computeRayKeys(const point3d& origin, const point3d& end, KeyRay& ray) const;

    NODE* getLUTNeighbor(const point3d& value, OcTreeLUT::NeighborDirection dir) const;


  protected:

    KeyRay keyray;
    OcTreeLUT* lut;

  };


}

#include "OcTreeBaseSE.hxx"

#endif
