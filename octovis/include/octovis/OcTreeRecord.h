#ifndef OCTOVIS_OC_TREE_RECORD
#define OCTOVIS_OC_TREE_RECORD

#include <octovis/OcTreeDrawer.h>

namespace octomap {

  class OcTreeRecord {
  public:
    AbstractOcTree*  octree;
    OcTreeDrawer*    octree_drawer;
    unsigned int     id;
    pose6d           origin;
  };

} // namespace

#endif
