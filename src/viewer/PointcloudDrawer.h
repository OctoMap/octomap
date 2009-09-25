/*
 * PointcloudDrawer.h
 *
 *  Created on: Sep 24, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#ifndef POINTCLOUDDRAWER_H_
#define POINTCLOUDDRAWER_H_

#include "SceneObject.h"

namespace octomap {

  class PointcloudDrawer: public ScanGraphDrawer {
  public:
    PointcloudDrawer();
    PointcloudDrawer(const ScanGraph& graph);
    virtual ~PointcloudDrawer();

    virtual void draw() const;
    virtual void clear();
    virtual void setScanGraph(const ScanGraph& graph);

  protected:
    GLfloat* m_pointsArray;
    unsigned m_numberPoints;

  };

}

#endif /* POINTCLOUDDRAWER_H_ */
