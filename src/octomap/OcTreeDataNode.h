#ifndef OCTOMAP_OCTREE_DATA_NODE_H
#define OCTOMAP_OCTREE_DATA_NODE_H

// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "octomap_types.h"
#include <bitset>

namespace octomap {

  /**
   * Basic node in the OcTree that can hold arbitrary data of type T in value.
   * This is the base class for nodes used in an OcTree. The used implementation
   * for occupancy mapping is in OcTreeNode.
   *
   */
  template<typename T> class OcTreeDataNode {

  public:

    OcTreeDataNode();
    OcTreeDataNode(T initVal);
    virtual ~OcTreeDataNode();

    /**
     * Creates a new node of the same type and returns a pointer to it.
     *
     * If you derive from this class, you need to overload this function
     * and create a node of your overloaded type instead
     */
    virtual OcTreeDataNode<T>* newNode() const;

    // -- children  ----------------------------------


    /// initialize i-th child, allocate children array if needed
    virtual bool createChild(unsigned int i);

    /// \return true if the i-th child exists
    virtual bool childExists(unsigned int i) const;

    /// \return a pointer to the i-th child of the node. The child needs to exist.
    virtual OcTreeDataNode<T>* getChild(unsigned int i);

    /// \return a const pointer to the i-th child of the node. The child needs to exist.
    virtual const OcTreeDataNode<T>* getChild(unsigned int i) const;

    /// \return true if the node has at least one child
    virtual bool hasChildren() const;

    /// A node is collapsible if all children exist, don't have children of their own
    /// and have the same occupancy value
    virtual bool collapsible() const;

    // -- pruning of children  -----------------------


    /**
     * Prunes a node when it is collapsible
     * @return true if pruning was successful
     */
    virtual bool pruneNode();

    /**
     * Expands a node (reverse of pruning): All children are created and
     * their occupancy probability is set to the node's value.
     *
     * You need to verify that this is indeed a pruned node (i.e. not a
     * leaf at the lowest level)
     *
     */
    virtual void expandNode();

    /// @return value stored in the node
    virtual T getValue() const{return value;};
    /// sets value to be stored in the node
    virtual void setValue(T v) {value = v;};

    // file IO:

    /**
     * Read node from binary stream (incl. float value),
     * recursively continue with all children.
     *
     * @note This is an experimental feature!
     *
     * @param s
     * @return
     */
    virtual std::istream& readValue(std::istream &s);

    /**
     * Write node to binary stream (incl float value),
     * recursively continue with all children.
     * This preserves the complete state of the node.
     *
     * @note This is an experimental feature!
     *
     * @param s
     * @return
     */
    virtual std::ostream& writeValue(std::ostream &s) const;


  protected:
    void allocChildren();

    /// pointer to array of children, may be NULL
    OcTreeDataNode<T>** itsChildren;
    /// stored data
    T value;

  };


} // end namespace

#include "OcTreeDataNode.hxx"

#endif
