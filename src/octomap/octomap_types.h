// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
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

#ifndef OCTOMAP_TYPES_H
#define OCTOMAP_TYPES_H

#include <vector>

#include <octomath/Vector3.h>
#include <octomath/Pose6D.h>

namespace octomap {

  typedef octomath::Vector3               point3d;
  typedef std::vector<octomath::Vector3*> point3d_collection;
  /// A voxel defined by its center point3d and its side length
  typedef std::pair<point3d, double> OcTreeVolume;

  #define m3d_max(a,b) (a>b)?a:b
  #define M3D_MAX(a,b) (a>b)?a:b
  #define m3d_min(a,b) (a<b)?a:b
  #define M3D_MIN(a,b) (a<b)?a:b

  #define M3D_RAD2DEG(x) ((x)/M_PI*180.)
  #define M3D_DEG2RAD(x) ((x)/180.*M_PI)

  #define M3D_M_PI_2 1.57079633

  #define COLOR_RED       "\033[31m"
  #define COLOR_GREEN     "\033[32m"
  #define COLOR_YELLOW    "\033[33m"
  #define COLOR_MAGENTA   "\033[35m"
  #define COLOR_BLUE      "\033[36m"
  #define COLOR_NORMAL    "\033[0m"

}

#endif
