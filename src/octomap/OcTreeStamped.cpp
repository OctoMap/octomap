// $Id$

#include "octomap/OcTreeStamped.h"

namespace octomap {

  OcTreeStamped::OcTreeStamped(double _resolution)
    : OccupancyOcTreeBase<OcTreeNodeStamped> (_resolution)  {

    itsRoot = new OcTreeNodeStamped();
    tree_size++;
  }


  unsigned int OcTreeStamped::getLastUpdateTime() {
    return itsRoot->getTimestamp();
  }


  void OcTreeStamped::degradeOutdatedNodes(unsigned int time_thres) {
    unsigned int query_time = time(NULL); 
    degradeOutdatedNodesRecurs(itsRoot, time_thres, query_time);
  }
  

  void OcTreeStamped::degradeOutdatedNodesRecurs(OcTreeNodeStamped* node, unsigned int& time_thres,
                                            unsigned int& query_time) {
    

    if (node == NULL) return;

    if (isNodeOccupied(node) &&
        ((query_time - node->getTimestamp()) > time_thres))
    {
      integrateMissNoTime(node);
    }

    for (unsigned int i=0; i<8; i++) {
      if (node->childExists(i)) {
        degradeOutdatedNodesRecurs(node->getChild(i), time_thres, query_time);
      }
    }
  }


  void OcTreeStamped::integrateHit(OcTreeNodeStamped* occupancyNode) const{
    OccupancyOcTreeBase<OcTreeNodeStamped>::integrateHit(occupancyNode);
    occupancyNode->updateTimestamp();
  }

  void OcTreeStamped::integrateMiss(OcTreeNodeStamped* occupancyNode) const{
    OccupancyOcTreeBase<OcTreeNodeStamped>::integrateMiss(occupancyNode);
    occupancyNode->updateTimestamp();
  }

  void OcTreeStamped::integrateMissNoTime(OcTreeNodeStamped* occupancyNode) const{
    OccupancyOcTreeBase<OcTreeNodeStamped>::integrateMiss(occupancyNode);
  }

  

} // end namespace

