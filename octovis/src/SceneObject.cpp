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

#define NOMINMAX
#include <octovis/SceneObject.h>

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.29577951308232087721)
#endif

namespace octomap {

  SceneObject::SceneObject() :
    m_zMin(0.0), m_zMax(1.0), m_colorMode(CM_FLAT) {
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

  void SceneObject::heightMapGray(double h, GLfloat* glArrayPos) const {
    if (m_zMin >= m_zMax)
      h = 0.5;
    else{
      h = std::min(std::max((h-m_zMin)/ (m_zMax - m_zMin), 0.0), 1.0) * 0.4 + 0.3; // h \in [0.3, 0.7]
    }

    glArrayPos[0] = h;
    glArrayPos[1] = h;
    glArrayPos[2] = h;
  }
  

}
