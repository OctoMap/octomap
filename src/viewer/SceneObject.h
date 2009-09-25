/*
 * SceneObject.h
 *
 *  Created on: Sep 21, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#ifndef SCENEOBJECT_H_
#define SCENEOBJECT_H_

#include <QGLViewer/qglviewer.h>
#include <octomap.h>

namespace octomap {

  /**
  * Abstract base class for objects to be drawn in the ViewerWidget.
  *
  */
  class SceneObject {
  public:
    SceneObject(){};
    virtual ~SceneObject(){};

    /**
    * Actual draw function which will be called to visualize the object
    */
    virtual void draw() const = 0;

    /**
    * Clears the object's representation (will be called when it gets invalid)
    */
    virtual void clear(){};
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
