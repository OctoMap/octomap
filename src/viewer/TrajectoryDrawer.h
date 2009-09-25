/*
 * TrajectoryDrawer.h
 *
 *  Created on: Sep 21, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#ifndef TRAJECTORYDRAWER_H_
#define TRAJECTORYDRAWER_H_

#include "SceneObject.h"
#include <vector>

namespace octomap {

  class TrajectoryDrawer : public ScanGraphDrawer{
  public:
    TrajectoryDrawer();
    TrajectoryDrawer(const ScanGraph& graph);
    virtual ~TrajectoryDrawer();
    virtual void draw() const;
    virtual void clear();
    virtual void setScanGraph(const octomap::ScanGraph& graph);

  protected:
    GLfloat* m_trajectoryVertexArray;
    GLfloat* m_trajectoryColorArray;
    unsigned int m_trajectorySize; //!< number of nodes in the ScanGraph
  };

}

#endif /* TRAJECTORYDRAWER_H_ */
