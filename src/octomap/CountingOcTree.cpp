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


  CountingOcTreeNode::CountingOcTreeNode() : count(0) {
    for (unsigned int i = 0; i<8; i++)
      itsChildren[i] = NULL;
  }

  CountingOcTreeNode::~CountingOcTreeNode() {
    for (unsigned int i = 0; i<8; i++) {
      if (itsChildren[i]){
	delete itsChildren[i];
      }
    }
  }

  bool CountingOcTreeNode::childExists(unsigned int i) const {
    assert(i < 8);
    if (itsChildren[i] != NULL) return true;
    else return false;
  }


  //! \return i-th child of node, NULL if there is none
  const CountingOcTreeNode* CountingOcTreeNode::getChild(unsigned int i) const {
    assert(i < 8);
    return itsChildren[i];
  }

  CountingOcTreeNode* CountingOcTreeNode::getChild(unsigned int i) {
    assert(i < 8);
    return itsChildren[i];
  }


    //! set i-th child
  void CountingOcTreeNode::setChild(unsigned int i, CountingOcTreeNode* child) {
    assert(i < 8);
    itsChildren[i] = child;
  }

  bool CountingOcTreeNode::hasChildren() const {
      for (unsigned int i = 0; i<8; i++)
	if (childExists(i)) return true;
      return false;
  }




  CountingOcTree::CountingOcTree(double _resolution)
    : OcTreeBase<CountingOcTreeNode>(_resolution)   {

    itsRoot = new CountingOcTreeNode();
    tree_size++;
  }

  CountingOcTree::~CountingOcTree(){
    delete itsRoot;
  }

  CountingOcTreeNode* CountingOcTree::updateNode(const point3d& value) {

    unsigned short int key[3];

    if (!genKeys(value, key))
        return NULL;

    CountingOcTreeNode* curNode = this->getRoot();
    this->traverseNode(curNode);

    // follow or construct nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      unsigned int pos = genPos(key, i);

      // requested node does not exist
      if (!curNode->childExists(pos)) {
        curNode->setChild(pos, new CountingOcTreeNode());
        tree_size++;
      }

      // descent tree
      curNode = curNode->getChild(pos);
      // modify traversed nodes
      this->traverseNode(curNode);
    }

    return curNode;
  }

  void CountingOcTree::traverseNode(CountingOcTreeNode* traversedNode){
    traversedNode->increaseCount();
  }


} // namespace
