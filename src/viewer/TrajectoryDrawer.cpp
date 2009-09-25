/*
 * TrajectoryDrawer.cpp
 *
 *  Created on: Sep 21, 2009
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#include "TrajectoryDrawer.h"

namespace octomap {

  TrajectoryDrawer::TrajectoryDrawer()
  : ScanGraphDrawer(), m_trajectoryVertexArray(NULL), m_trajectoryColorArray(NULL), m_trajectorySize(0)
  {

  }

  TrajectoryDrawer::TrajectoryDrawer(const octomap::ScanGraph& graph)
  : ScanGraphDrawer(), m_trajectoryVertexArray(NULL), m_trajectoryColorArray(NULL), m_trajectorySize(0)
  {
    this->setScanGraph(graph);
  }

  TrajectoryDrawer::~TrajectoryDrawer() {
    clear();

  }

  void TrajectoryDrawer::draw() const{
    if (m_trajectorySize == 0)
      return;

    // draw lines:
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glLineWidth(3.0f);
    glVertexPointer(3, GL_FLOAT, 0, m_trajectoryVertexArray);
    glColorPointer(4, GL_FLOAT, 0, m_trajectoryColorArray);
    glDrawArrays(GL_LINE_STRIP, 0, m_trajectorySize);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    // draw nodes:
    GLUquadricObj* quadric=gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    for (uint i = 0; i < m_trajectorySize; ++i){
      glPushMatrix();
      glTranslatef(m_trajectoryVertexArray[3*i], m_trajectoryVertexArray[3*i +1], m_trajectoryVertexArray[3*i +2]);
      glColor4f(m_trajectoryColorArray[4*i],m_trajectoryColorArray[4*i+1],m_trajectoryColorArray[4*i+2],m_trajectoryColorArray[4*i+3]);
      gluSphere(quadric, 0.05, 32, 32);
      glPopMatrix();
    }

    gluDeleteQuadric(quadric);
  }

  void TrajectoryDrawer::clear(){

    if (m_trajectorySize != 0) {
      delete[] m_trajectoryVertexArray;
      delete[] m_trajectoryColorArray;
      m_trajectorySize = 0;
    }
  }

  void TrajectoryDrawer::setScanGraph(const octomap::ScanGraph& graph){

    clear();

    m_trajectorySize = graph.size();
    m_trajectoryVertexArray = new GLfloat[m_trajectorySize * 3];
    m_trajectoryColorArray = new GLfloat[m_trajectorySize * 4];

    uint i = 0;
    for (octomap::ScanGraph::const_iterator it = graph.begin(); it != graph.end(); it++) {
      m_trajectoryVertexArray[i] = (*it)->pose.trans().x();
      m_trajectoryVertexArray[i+1] = (*it)->pose.trans().y();
      m_trajectoryVertexArray[i+2] = (*it)->pose.trans().z();
      i+=3;
    }

    for (unsigned int j=0; j < m_trajectorySize*4; j+=4) {
      m_trajectoryColorArray[j]   = 0.; // r
      m_trajectoryColorArray[j+1] = 0.; // g
      m_trajectoryColorArray[j+2] = 1.; // b
      m_trajectoryColorArray[j+3] = 1.; // alpha
    }
  }

}

