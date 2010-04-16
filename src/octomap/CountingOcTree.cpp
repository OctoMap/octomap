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

#include <cassert>
#include "CountingOcTree.h"

namespace octomap {


  /// implementation of CountingOcTreeNode  ----------------------------------

  CountingOcTreeNode::CountingOcTreeNode()
    : OcTreeDataNode<unsigned int>(0)
  {
  }

  CountingOcTreeNode::~CountingOcTreeNode() {

  }

  void CountingOcTreeNode::expandNode(){
    assert(!hasChildren());

    // divide "counts" evenly to children
    unsigned int childCount = (unsigned int)(value/ 8.0 +0.5);
    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      itsChildren[k]->setValue(childCount);
    }
  }

  bool CountingOcTreeNode::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    assert (itsChildren[i] == NULL);
    itsChildren[i] = new CountingOcTreeNode();
    return true;
  }


  /// implementation of CountingOcTree  --------------------------------------

  CountingOcTree::CountingOcTree(double _resolution)
    : OcTreeBase<CountingOcTreeNode>(_resolution)   {

    itsRoot = new CountingOcTreeNode();
    tree_size++;
  }


  CountingOcTreeNode* CountingOcTree::updateNode(const point3d& value) {

    unsigned short int key[3];

    if (!genKeys(value, key))
        return NULL;

    CountingOcTreeNode* curNode = this->getRoot();
    curNode->increaseCount();

    // follow or construct nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      unsigned int pos = genPos(key, i);

      // requested node does not exist
      if (!curNode->childExists(pos)) {
        curNode->createChild(pos);
        tree_size++;
      }

      // descent tree
      // cast needed: (node needs to ensure it's the right pointer)
      curNode = static_cast<CountingOcTreeNode*> (curNode->getChild(pos));
      // modify traversed nodes
      curNode->increaseCount();
    }

    return curNode;
  }



} // namespace
