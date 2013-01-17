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

#ifndef OCTOMAP_OCTREE_DATA_NODE_H
#define OCTOMAP_OCTREE_DATA_NODE_H


#include "octomap_types.h"
#include "assert.h"
#include <bitset>

namespace octomap {

  class AbstractOcTreeNode {


  };

  /**
   * Basic node in the OcTree that can hold arbitrary data of type T in value.
   * This is the base class for nodes used in an OcTree. The used implementation
   * for occupancy mapping is in OcTreeNode.#
   * \tparam T data to be stored in the node (e.g. a float for probabilities)
   *
   */
  template<typename T> class OcTreeDataNode: public AbstractOcTreeNode {

  public:

    OcTreeDataNode();
    OcTreeDataNode(T initVal);
    /// Copy constructor, performs a recursive deep-copy of all children
    OcTreeDataNode(const OcTreeDataNode& rhs);

    ~OcTreeDataNode();

    /// Equals operator, compares if the stored value is identical
    bool operator==(const OcTreeDataNode& rhs) const;


    // -- children  ----------------------------------


    /// initialize i-th child, allocate children array if needed
    bool createChild(unsigned int i);

    /// Safe test to check of the i-th child exists,
    /// first tests if there are any children.
    /// \return true if the i-th child exists
    bool childExists(unsigned int i) const;

    /// \return a pointer to the i-th child of the node. The child needs to exist.
    OcTreeDataNode<T>* getChild(unsigned int i);

    /// \return a const pointer to the i-th child of the node. The child needs to exist.
    const OcTreeDataNode<T>* getChild(unsigned int i) const;

    /// \return true if the node has at least one child
    bool hasChildren() const;

    /// A node is collapsible if all children exist, don't have children of their own
    /// and have the same occupancy value
    bool collapsible() const;

    /// Deletes the i-th child of the node
    void deleteChild(unsigned int i);

    // -- pruning of children  -----------------------


    /**
     * Prunes a node when it is collapsible
     * @return true if pruning was successful
     */
    bool pruneNode();

    /**
     * Expands a node (reverse of pruning): All children are created and
     * their occupancy probability is set to the node's value.
     *
     * You need to verify that this is indeed a pruned node (i.e. not a
     * leaf at the lowest level)
     *
     */
    void expandNode();

    /// @return value stored in the node
    T getValue() const{return value;};
    /// sets value to be stored in the node
    void setValue(T v) {value = v;};

    // file IO:

    /**
     * Read node from binary stream (incl. float value),
     * recursively continue with all children.
     *
     * @param s
     * @return
     */
    std::istream& readValue(std::istream &s);

    /**
     * Write node to binary stream (incl float value),
     * recursively continue with all children.
     * This preserves the complete state of the node.
     *
     * @param s
     * @return
     */
    std::ostream& writeValue(std::ostream &s) const;


    /// Make the templated data type available from the outside
    typedef T DataType;


  protected:
    void allocChildren();

    /// pointer to array of children, may be NULL
    OcTreeDataNode<T>** children;
    /// stored data (payload)
    T value;

  };


} // end namespace

#include "octomap/OcTreeDataNode.hxx"

#endif
