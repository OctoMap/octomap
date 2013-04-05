/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

    size_t size() const {  return points.size(); }
    void clear();
    inline void reserve(size_t size) {points.reserve(size); }

    inline void push_back(float x, float y, float z) {
      points.push_back(point3d(x,y,z));
    }
    inline void push_back(const point3d& p) {
      points.push_back(p);
    }
    inline void push_back(point3d* p) {
       points.push_back(*p);
    }

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
    point3d back()  { return points.back(); }
    /// Returns a copy of the ith point in point cloud.
    /// Use operator[] for direct access to point reference.
    point3d getPoint(unsigned int i) const;   // may return NULL

    inline const point3d& operator[] (size_t i) const { return points[i]; }
    inline point3d& operator[] (size_t i) { return points[i]; }

    // I/O methods

    std::istream& readBinary(std::istream &s);
    std::istream& read(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;

  protected:
    pose6d               current_inv_transform;
    point3d_collection   points;
  };

}


#endif
