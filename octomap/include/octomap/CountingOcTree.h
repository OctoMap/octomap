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

#ifndef OCTOMAP_COUNTING_OCTREE_HH
#define OCTOMAP_COUNTING_OCTREE_HH


#include <stdio.h>
#include "OcTreeBase.h"
#include "OcTreeDataNode.h"

namespace octomap {

  /**
   * An Octree-node which stores an internal counter per node / volume.
   *
   * Count is recursive, parent nodes have the summed count of their
   * children.
   *
   * \note In our mapping system this data structure is used in
   *       CountingOcTree in the sensor model only
   */
  class CountingOcTreeNode : public OcTreeDataNode<unsigned int> {

  public:

    CountingOcTreeNode();
    ~CountingOcTreeNode();
    
    inline unsigned int getCount() const { return getValue(); }
    inline void increaseCount() { value++; }
    inline void setCount(unsigned c) {this->setValue(c); }

  };



  /**
   * An AbstractOcTree which stores an internal counter per node / volume.
   *
   * Count is recursive, parent nodes have the summed count of their
   * children.
   *
   * \note Was only used internally, not used anymore
   */
  class CountingOcTree : public OcTreeBase <CountingOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    CountingOcTree(double resolution);
    virtual CountingOcTreeNode* updateNode(const point3d& value);
    CountingOcTreeNode* updateNode(const OcTreeKey& k);
    void getCentersMinHits(point3d_list& node_centers, unsigned int min_hits) const;

  protected:

    void getCentersMinHitsRecurs( point3d_list& node_centers,
                                  unsigned int& min_hits,
                                  unsigned int max_depth,
                                  CountingOcTreeNode* node, unsigned int depth,
                                  const OcTreeKey& parent_key) const;

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a 
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer() {
           CountingOcTree* tree = new CountingOcTree(0.1);
           tree->clearKeyRays();
           AbstractOcTree::registerTreeType(tree);
         }

         /**
         * Dummy function to ensure that MSVC does not drop the
         * StaticMemberInitializer, causing this tree failing to register.
         * Needs to be called from the constructor of this octree.
         */
         void ensureLinking() {};
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer countingOcTreeMemberInit;
  };


}


#endif
