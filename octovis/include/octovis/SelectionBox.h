#ifndef SELECTIONBOX_H_
#define SELECTIONBOX_H_

#include <qglviewer.h>

namespace octomap {
  class SelectionBox{

  public:
    SelectionBox();
    virtual ~SelectionBox();
    void draw() const;

  protected:
    bool m_visible;
    qglviewer::Vec minPt;
    qglviewer::Vec maxPt;

  };



}




#endif
