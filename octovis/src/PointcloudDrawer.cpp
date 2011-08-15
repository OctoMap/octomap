// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2011.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
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
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <octovis/PointcloudDrawer.h>

namespace octomap {

  PointcloudDrawer::PointcloudDrawer()
    : ScanGraphDrawer(), m_pointsArray(NULL), m_numberPoints(0)
  {
  }

  PointcloudDrawer::PointcloudDrawer(const ScanGraph& graph)
    : ScanGraphDrawer(), m_pointsArray(NULL), m_numberPoints(0)
  {
    this->setScanGraph(graph);

  }

  PointcloudDrawer::~PointcloudDrawer() {
    clear();

  }

  void PointcloudDrawer::setScanGraph(const ScanGraph& graph){
    clear();

    // count points first:
    for (octomap::ScanGraph::const_iterator it = graph.begin(); it != graph.end(); it++) {
      m_numberPoints += (*it)->scan->size();
    }

    m_pointsArray = new GLfloat[3*m_numberPoints];

    unsigned i = 0;
    for (octomap::ScanGraph::const_iterator graph_it = graph.begin(); graph_it != graph.end(); graph_it++) {
      octomap::Pointcloud* scan = new Pointcloud((*graph_it)->scan);
      scan->transformAbsolute((*graph_it)->pose);

      for (Pointcloud::iterator pc_it = scan->begin(); pc_it != scan->end(); ++pc_it){
        m_pointsArray[3*i] = pc_it->x();
        m_pointsArray[3*i +1] = pc_it->y();
        m_pointsArray[3*i +2] = pc_it->z();

        i++;
      }
      delete scan;
    }
  }

  void PointcloudDrawer::clear(){

    if (m_numberPoints != 0) {
      delete[] m_pointsArray;
      m_numberPoints = 0;
    }
  }

  void PointcloudDrawer::draw() const{
    if (m_numberPoints == 0)
      return;

    glEnable(GL_POINT_SMOOTH);
    glEnableClientState(GL_VERTEX_ARRAY);

   // TODO: find a solution for printout-mode (=> separate drawer?)
//    if (m_printoutMode){
//      if (!m_drawFree) {
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//      }
//      glColor4f(0.0, 0.0, 0.0, 1.);
//    } else{
//      glColor4f(1.0, 0.0, 0.0, 1.);
//    }

    glPointSize(1.0);
    glColor4f(1.0, 0.0, 0.0, 1.);

    glVertexPointer(3, GL_FLOAT, 0, m_pointsArray);
    glDrawArrays(GL_POINTS, 0, m_numberPoints);
    glDisableClientState(GL_VERTEX_ARRAY);
  }

}
