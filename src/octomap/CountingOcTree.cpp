// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
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

    OcTreeKey key;
    if (!genKey(value, key)) return NULL;
    return updateNode(key);
  }

    
  CountingOcTreeNode* CountingOcTree::updateNode(const OcTreeKey& k) {

    CountingOcTreeNode* curNode = this->getRoot();
    curNode->increaseCount();

    // follow or construct nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      unsigned int pos = genPos(k, i);

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
