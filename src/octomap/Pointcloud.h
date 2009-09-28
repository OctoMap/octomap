#ifndef OCTOMAP_POINTCLOUD_H
#define OCTOMAP_POINTCLOUD_H

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

#include<vector>
#include<list>
#include <octomap_types.h>

namespace octomap {

  class Pointcloud {

  public:

    Pointcloud();
    ~Pointcloud();

    Pointcloud(const Pointcloud& other);
    Pointcloud(Pointcloud* other);

    unsigned int size() const {  return points.size(); }
    void clear();

    // add beam endpoint
    void push_back(double x, double y, double z);
    void push_back(point3d& p);
    void push_back(point3d* p);

    // add points from other Pointcloud
    void push_back(const Pointcloud& other);

    void writeVrml(std::string filename);

    // apply transform to each point
    void transform(octomath::Pose6D transform);

    // rotate each point in pointcloud
    void rotate(double roll, double pitch, double yaw);

    // apply transform to each point, undo prev transform
    void transformAbsolute(octomath::Pose6D transform);

    void calcBBX(point3d& lowerBound, point3d& upperBound) const;
    void crop(point3d lowerBound, point3d upperBound);

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
    std::ostream& writeBinary(std::ostream &s) const;
    std::ostream& appendBinary(std::ostream& s) const;

  protected:
    point3d_collection   points;
    octomath::Pose6D     current_inv_transform;
  };

}


#endif
