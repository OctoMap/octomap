#ifndef OCTOMAP_OCTREE_STAMPED_H
#define OCTOMAP_OCTREE_STAMPED_H

// $Id$

#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/OcTreeNodeStamped.h>

namespace octomap {

  class OcTreeStamped : public OccupancyOcTreeBase <OcTreeNodeStamped> {

  public:
    static const int TREETYPE=5;

  public:

    OcTreeStamped(double _resolution);

    //! \return timestamp of last update
    unsigned int  getLastUpdateTime();

    void degradeOutdatedNodes(unsigned int time_thres);
    
    virtual void integrateHit(OcTreeNodeStamped* occupancyNode) const;
    virtual void integrateMiss(OcTreeNodeStamped* occupancyNode) const;
    virtual void integrateMissNoTime(OcTreeNodeStamped* occupancyNode) const;

  protected:

    void degradeOutdatedNodesRecurs(OcTreeNodeStamped* node, unsigned int& time_thres,
                                    unsigned int& query_time);

    
  };


} // end namespace

#endif
