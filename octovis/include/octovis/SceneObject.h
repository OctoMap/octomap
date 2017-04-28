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

#ifndef SCENEOBJECT_H_
#define SCENEOBJECT_H_

// fix Windows includes
#include <qglobal.h>
#if defined(Q_WS_WIN) || defined(Q_OS_WIN)
  #include <QtCore/qt_windows.h>
#endif

#if defined(Q_WS_MAC) || defined(Q_OS_MAC)
  #include <OpenGL/glu.h>
#else
  #include <GL/glu.h>
#endif

#include <octomap/octomap.h>

namespace octomap {

  /**
  * Abstract base class for objects to be drawn in the ViewerWidget.
  *
  */
  class SceneObject {
  public:
    enum ColorMode {
      CM_FLAT,
      CM_PRINTOUT,
      CM_COLOR_HEIGHT,
      CM_GRAY_HEIGHT,
      CM_SEMANTIC
    };

  public:
    SceneObject();
    virtual ~SceneObject(){};

    /**
    * Actual draw function which will be called to visualize the object
    */
    virtual void draw() const = 0;

    /**
    * Clears the object's representation (will be called when it gets invalid)
    */
    virtual void clear(){};

  public:
    //! the color mode has to be set before calling OcTreDrawer::setMap()
    //! because the cubes are generated in OcTreDrawer::setMap() using the color information
    inline void setColorMode(ColorMode mode) { m_colorMode = mode; }
    inline void enablePrintoutMode(bool enabled = true) { if (enabled) m_colorMode = CM_PRINTOUT; else m_colorMode = CM_FLAT; }
    inline void enableHeightColorMode(bool enabled = true) { if (enabled) m_colorMode = CM_COLOR_HEIGHT; else m_colorMode = CM_FLAT; }
    inline void enableSemanticColoring(bool enabled = true) { if (enabled) m_colorMode = CM_SEMANTIC; else m_colorMode = CM_FLAT; }

  protected:
    /// writes rgb values which correspond to a rel. height in the map.
    /// (glArrayPos needs to have at least size 3!)
    void heightMapColor(double h, GLfloat* glArrayPos) const;
    void heightMapGray(double h, GLfloat* glArrayPos) const;
    double m_zMin;
    double m_zMax;
    ColorMode m_colorMode;
  };




  /**
  * Abstract base class for all objects visualizing ScanGraphs.
  */
  class ScanGraphDrawer : public SceneObject {
  public:
    ScanGraphDrawer(): SceneObject(){};
    virtual ~ScanGraphDrawer(){};

    /**
    * Notifies drawer of a new or changed ScanGraph, so that the internal
    * representation can be rebuilt. Needs to be overloaded by each specific
    * drawer.
    *
    * @param graph ScanGraph to be visualized
    */
    virtual void setScanGraph(const octomap::ScanGraph& graph) = 0;
  };

}

#endif /* SCENEOBJECT_H_ */
