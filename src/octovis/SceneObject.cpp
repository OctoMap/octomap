// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-20111.
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

#include "SceneObject.h"

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

namespace octomap {

  SceneObject::SceneObject() :
    m_zMin(0.0), m_zMax(1.0), m_printoutMode(false), m_heightColorMode(false), m_display_axes(false) {

  }

  void SceneObject::heightMapColor(double h, GLfloat* glArrayPos) const {
    if (m_zMin >= m_zMax)
      h = 0.5;
    else{
      h = (1.0 - std::min(std::max((h-m_zMin)/ (m_zMax - m_zMin), 0.0), 1.0)) *0.8;
    }

    // blend over HSV-values (more colors)
    double r, g, b;
    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
      f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:
      r = v; g = n; b = m;
      break;
    case 1:
      r = n; g = v; b = m;
      break;
    case 2:
      r = m; g = v; b = n;
      break;
    case 3:
      r = m; g = n; b = v;
      break;
    case 4:
      r = n; g = m; b = v;
      break;
    case 5:
      r = v; g = m; b = n;
      break;
    default:
      r = 1; g = 0.5; b = 0.5;
      break;
    }

    glArrayPos[0] = r;
    glArrayPos[1] = g;
    glArrayPos[2] = b;
  }


  
  void SceneObject::drawAxes() const {

    glPushMatrix();

    float length = .15; 

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_COLOR_MATERIAL);

    glTranslatef(origin.trans().x(), origin.trans().y(), origin.trans().z());

    double angle= 2 * acos(origin.rot().u());
    double scale = sqrt (origin.rot().x()*origin.rot().x() + origin.rot().y()*origin.rot().y() + origin.rot().z()*origin.rot().z());
    double ax= origin.rot().x() / scale;
    double ay= origin.rot().y() / scale;
    double az= origin.rot().z() / scale;

    if (angle > 0) glRotatef(RAD2DEG(angle), ax, ay, az);

    float color[4];
    color[0] = 0.7f;  color[1] = 0.7f;  color[2] = 1.0f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    QGLViewer::drawArrow(length, 0.01*length);

    color[0] = 1.0f;  color[1] = 0.7f;  color[2] = 0.7f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0, 0.0, 1.0, 0.0);
    QGLViewer::drawArrow(length, 0.01*length);
    glPopMatrix();

    color[0] = 0.7f;  color[1] = 1.0f;  color[2] = 0.7f;  color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    QGLViewer::drawArrow(length, 0.01*length);
    glPopMatrix();

    if (colorMaterial)
      glEnable(GL_COLOR_MATERIAL);
    if (!lighting)
      glDisable(GL_LIGHTING);

    glPopMatrix();

  }

}
