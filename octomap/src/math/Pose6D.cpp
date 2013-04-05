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

#include <octomap/math/Pose6D.h>
#include <math.h>

namespace octomath {

  Pose6D::Pose6D() {
  }


  Pose6D::Pose6D(const Vector3 &trans, const Quaternion &rot) :
    translation(trans),
    rotation(rot)
  { }



  Pose6D::Pose6D(float x, float y, float z, double roll, double pitch, double yaw) :
    translation(x, y, z),
    rotation(roll, pitch, yaw)
  { }

  Pose6D::~Pose6D() {
  }

  Pose6D& Pose6D::operator= (const Pose6D& other) {
    translation = other.trans();
    rotation = other.rot();
    return *this;
  }


  Pose6D Pose6D::inv() const {
    Pose6D result(*this);
    result.rot() = rot().inv().normalized();
    result.trans() = result.rot().rotate(-trans());
    return result;
  }


  Pose6D& Pose6D::inv_IP() {
    rot() = rot().inv().normalized();
    trans() = rot().rotate(-trans());
    return *this;
  }


  Vector3 Pose6D::transform (const Vector3 &v) const {
    Vector3 res = this->rot().rotate(v);
    res = res + this->trans();
    return res;
  }

  Pose6D Pose6D::operator* (const Pose6D& other) const {
    Quaternion rot_new   = rotation * other.rot();
    Vector3    trans_new = rotation.rotate(other.trans()) + trans();
    return Pose6D(trans_new, rot_new.normalized());
  }

  const Pose6D& Pose6D::operator*= (const Pose6D& other) {
    trans() += rotation.rotate(other.trans());
    rot() = rot() * other.rot();
    return *this;
  }

  double Pose6D::distance (const Pose6D &other) const {
    double dist_x = x() - other.x();
    double dist_y = y() - other.y();
    double dist_z = z() - other.z();
    return sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
  }

  double Pose6D::transLength() const {
    return sqrt(x()*x() + y()*y() + z()*z());
  }


  bool Pose6D::operator==(const Pose6D& other) const {
    return translation == other.translation
      && rotation == other.rotation;
  }

  bool Pose6D::operator!=(const Pose6D& other) const {
    return !(*this == other);
  }

  std::istream& Pose6D::read(std::istream &s) {
    translation.read(s);
    rotation.read(s);
    return s;
  }


  std::ostream& Pose6D::write(std::ostream &s) const {
    translation.write(s);
    s << " ";
    rotation.write(s);
    return s;
  }


  std::istream& Pose6D::readBinary(std::istream &s) {
    translation.readBinary(s);
    rotation.readBinary(s);
    return s;
  }


  std::ostream& Pose6D::writeBinary(std::ostream &s) const {
    translation.writeBinary(s);
    rotation.writeBinary(s);
    return s;
  }

  std::ostream& operator<<(std::ostream& s, const Pose6D& p) {
    s << "(" << p.x() << " " << p.y() << " " << p.z()
      << ", " << p.rot().u() << " " << p.rot().x() << " " << p.rot().y() << " " << p.rot().z() << ")";
    return s;
  }

}
