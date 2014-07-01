/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
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
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

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
