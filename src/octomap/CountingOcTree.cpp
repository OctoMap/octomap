
// =====================================================
// octomap
// -----------------------------------------------------
// Kai M. Wurm <wurm@uni-freiburg.de>
// Armin Hornung <hornunga@informatik.uni-freiburg.de>
// =====================================================

#include "CountingOcTree.h"

namespace octomap {

  /***   CountingOcTreeNode  **********************************/

  CountingOcTreeNode::CountingOcTreeNode() : count(0) {
    for (uint i = 0; i<8; i++) 
      itsChildren[i] = NULL;
  }
  
  CountingOcTreeNode::~CountingOcTreeNode() {
    for (uint i = 0; i<8; i++) {
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
      for (uint i = 0; i<8; i++) 
	if (childExists(i)) return true;
      return false;
  }
    

  // tree   ################################################


  CountingOcTree::CountingOcTree(double _resolution)
    : AbstractOcTree<CountingOcTreeNode>(_resolution)   {

    itsRoot = new CountingOcTreeNode();
    tree_size++;
  }

  CountingOcTree::~CountingOcTree(){
    delete itsRoot;
  }

  CountingOcTreeNode* CountingOcTree::updateNode(const point3d& value) {

    unsigned short int key[3];

    for (uint i=0; i<3; i++) {
      if ( !genKey( value(i), key[i]) ) return NULL;
    }

    CountingOcTreeNode* curNode = this->getRoot();
    this->traverseNode(curNode);

    // follow or construct nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      uint pos = genPos(key, i);

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
