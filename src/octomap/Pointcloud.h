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

#ifndef OCTOMAP_POINTCLOUD_H
#define OCTOMAP_POINTCLOUD_H

#include <vector>
#include <list>
#include <octomap/octomap_types.h>

namespace octomap {

  /**
   * A collection of 3D coordinates (point3d), which are regarded as endpoints of a
   * 3D laser scan.
   */
  class Pointcloud {

  public:

    Pointcloud();
    ~Pointcloud();

    Pointcloud(const Pointcloud& other);
    Pointcloud(Pointcloud* other);

    unsigned int size() const {  return points.size(); }
    void clear();

    void push_back(double x, double y, double z); ///< adds the endpoint of a beam to the Pointcloud
    void push_back(point3d& p); ///< adds the endpoint of a beam to the Pointcloud
    void push_back(point3d* p); ///< adds the endpoint of a beam to the Pointcloud

    /// Add points from other Pointcloud
    void push_back(const Pointcloud& other);

    /// Export the Pointcloud to a VRML file
    void writeVrml(std::string filename);

    /// Apply transform to each point
    void transform(pose6d transform);

    /// Rotate each point in pointcloud
    void rotate(double roll, double pitch, double yaw);

    /// Apply transform to each point, undo previous transforms
    void transformAbsolute(pose6d transform);

    /// Calculate bounding box of Pointcloud
    void calcBBX(point3d& lowerBound, point3d& upperBound) const;
    /// Crop Pointcloud to given bounding box
    void crop(point3d lowerBound, point3d upperBound);

    // removes any points closer than [thres] to (0,0,0)
    void minDist(double thres);

    void subSampleRandom(unsigned int num_samples, Pointcloud& sample_cloud);

    // iterators ------------------

    typedef point3d_collection::iterator iterator;
    typedef point3d_collection::const_iterator const_iterator;
    iterator begin() { return points.begin(); }
    iterator end()   { return points.end(); }
    const_iterator begin() const { return points.begin(); }
    const_iterator end() const  { return points.end(); }
    point3d* back()  { return points.back(); }
    point3d* getPoint(unsigned int i);   // may return NULL

    // I/O methods

    std::istream& readBinary(std::istream &s);
    std::istream& read(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;

  protected:
    point3d_collection   points;
    pose6d     current_inv_transform;
  };

}


#endif
