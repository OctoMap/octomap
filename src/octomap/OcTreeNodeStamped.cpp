// $Id$

#include "octomap/OcTreeNodeStamped.h"

namespace octomap {


  OcTreeNodeStamped::OcTreeNodeStamped()
    : OcTreeNode(), timestamp(0) {
  }


  bool OcTreeNodeStamped::createChild(unsigned int i) {
    if (itsChildren == NULL) {
      allocChildren();
    }
    itsChildren[i] = new OcTreeNodeStamped();
    return true;
  }


} // end namespace


