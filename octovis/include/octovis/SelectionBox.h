#ifndef SELECTIONBOX_H_
#define SELECTIONBOX_H_

#include <qglviewer.h>

namespace octomap {
  class SelectionBox{

  public:
    SelectionBox();
    virtual ~SelectionBox();
    void draw(bool withNames = false);
    const qglviewer::ManipulatedFrame* frame (unsigned short i) const { return m_frames.at(i); }
    qglviewer::ManipulatedFrame* frame (unsigned short i) { return m_frames.at(i); }
    void getBBXMin(float& x, float& y, float& z) const;
    void getBBXMax(float& x, float& y, float& z) const;
    int getGrabbedFrame() const;

  protected:
    void drawAxis(float length = 0.2f) const;

    bool m_visible;
    std::vector<qglviewer::ManipulatedFrame*> m_frames;
    unsigned short m_selectedFrame;
    qglviewer::Vec m_minPt;
    qglviewer::Vec m_maxPt;
    float m_arrowLength;

  };



}




#endif
