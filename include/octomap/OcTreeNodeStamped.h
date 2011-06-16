#ifndef OCTOMAP_OCTREE_NODE_STAMPED_H
#define OCTOMAP_OCTREE_NODE_STAMPED_H

// $Id$


#include "octomap/OcTreeNode.h"

namespace octomap {


  class OcTreeNodeStamped : public OcTreeNode {

  public:

    OcTreeNodeStamped();


    // -- children  ----------------------------------

    inline OcTreeNodeStamped* getChild(unsigned int i) {
      return static_cast<OcTreeNodeStamped*> (OcTreeNode::getChild(i));
    }
    inline const OcTreeNodeStamped* getChild(unsigned int i) const {
      return static_cast<const OcTreeNodeStamped*> (OcTreeNode::getChild(i));
    }
    bool createChild(unsigned int i);

    inline unsigned int getTimestamp() const { return timestamp; }
    inline void updateTimestamp() { timestamp = time(NULL);}
    inline void setTimestamp(unsigned int timestamp) {this->timestamp = timestamp; }

    // update timesteps of inner nodes 
    inline void updateOccupancyChildren() {      
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
      updateTimestamp();
    }


  protected:

    unsigned int timestamp; 

  };



} // end namespace



#endif
