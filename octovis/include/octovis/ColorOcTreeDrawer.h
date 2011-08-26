#ifndef OCTOVIS_COLOR_OCTREEDRAWER_H_
#define OCTOVIS_COLOR_OCTREEDRAWER_H_

#include <octovis/OcTreeDrawer.h>
#include <octomap/ColorOcTree.h>

namespace octomap {

  class ColorOcTreeDrawer : public OcTreeDrawer {
  public:
    ColorOcTreeDrawer();
    virtual ~ColorOcTreeDrawer();

    virtual void setOcTree(const AbstractOcTree* tree_pnt, octomap::pose6d origin, int map_id_);

  protected:
    
    
  };


} // end namespace

#endif
