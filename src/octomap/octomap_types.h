#ifndef MAPPING3D_TYPES_H
#define MAPPING3D_TYPES_H

// ==================================================
// mapping3d
// Kai M. Wurm <wurm@uni-freiburg.de>
// ==================================================

#include <Vector3.h>
#include <Pose6D.h>

namespace octomap {

  typedef octomath::Vector3               point3d;
  typedef std::vector<octomath::Vector3*> point3d_collection;
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
