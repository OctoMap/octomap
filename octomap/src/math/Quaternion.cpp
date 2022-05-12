/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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

#include <octomap/math/Vector3.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Utils.h>

#include <cassert>
#include <math.h>
#include <algorithm>


// used from Vector: norm2, unit, *

namespace octomath {


  Quaternion::Quaternion(const Quaternion &other) {
    data[0] = other(0);
    data[1] = other(1);
    data[2] = other(2);
    data[3] = other(3);
  }

  Quaternion::Quaternion(float uu, float xx, float yy, float zz) {
    u() = uu;
    x() = xx;
    y() = yy;
    z() = zz;
  }

  Quaternion::Quaternion(const Vector3 &other) {
    operator=(Quaternion(other.roll(), other.pitch(), other.yaw()));
  }

  Quaternion::Quaternion(double roll, double pitch, double yaw) {
    double sroll   = sin(roll);
    double spitch = sin(pitch);
    double syaw   = sin(yaw);
    double croll   = cos(roll);
    double cpitch = cos(pitch);
    double cyaw   = cos(yaw);

    double m[3][3] = { //create rotational Matrix
      {cyaw*cpitch, cyaw*spitch*sroll - syaw*croll, cyaw*spitch*croll + syaw*sroll},
      {syaw*cpitch, syaw*spitch*sroll + cyaw*croll, syaw*spitch*croll - cyaw*sroll},
      {    -spitch,                  cpitch*sroll,                  cpitch*croll}
    };

    float _u = (float) (sqrt(std::max(0., 1 + m[0][0] + m[1][1] + m[2][2]))/2.0);
    float _x = (float) (sqrt(std::max(0., 1 + m[0][0] - m[1][1] - m[2][2]))/2.0);
    float _y = (float) (sqrt(std::max(0., 1 - m[0][0] + m[1][1] - m[2][2]))/2.0);
    float _z = (float) (sqrt(std::max(0., 1 - m[0][0] - m[1][1] + m[2][2]))/2.0);
    u() = _u;
    x() = (m[2][1] - m[1][2])>=0?fabs(_x):-fabs(_x);
    y() = (m[0][2] - m[2][0])>=0?fabs(_y):-fabs(_y);
    z() = (m[1][0] - m[0][1])>=0?fabs(_z):-fabs(_z);
  }

  Quaternion::Quaternion(const Vector3& axis, double angle) {
    double sa = sin(angle/2);
    double ca = cos(angle/2);
    x() = (float) (axis.x()*sa);
    y() = (float) (axis.y()*sa);
    z() = (float) (axis.z()*sa);
    u() = (float) ca;
  }

  float Quaternion::norm () const {
    double n = 0;
    for (unsigned int i=0; i<4; i++) {
      n += operator()(i) * operator()(i);
    }
    return (float) sqrt(n);
  }

  void Quaternion::operator/= (float x) 
  { 
    for (unsigned int i=0; i<4; ++i)
      operator()(i) /= x;
  }

  bool Quaternion::operator== (const Quaternion& other) const 
  {
    for (unsigned int i=0; i<4; i++) 
    {
      if (operator()(i) != other(i)) 
        return false;
    }
    return true;
  }


  Vector3 Quaternion::toEuler() const {
    // create rotational matrix
    double n = norm ();
    double s = n > 0?2./(n*n):0.;

    double xs = x()*s;
    double ys = y()*s;
    double zs = z()*s;

    double ux = u()*xs;
    double uy = u()*ys;
    double uz = u()*zs;

    double xx = x()*xs;
    double xy = x()*ys;
    double xz = x()*zs;

    double yy = y()*ys;
    double yz = y()*zs;
    double zz = z()*zs;

    double m[3][3];

    m[0][0] = 1.0 - (yy + zz);
    m[1][1] = 1.0 - (xx + zz);
    m[2][2] = 1.0 - (xx + yy);

    m[1][0] = xy + uz;
    m[0][1] = xy - uz;

    m[2][0] = xz - uy;
    m[0][2] = xz + uy;
    m[2][1] = yz + ux;
    m[1][2] = yz - ux;

    float roll  = (float) atan2(m[2][1], m[2][2]);
    float pitch = (float) atan2(-m[2][0], sqrt(m[2][1]*m[2][1] + m[2][2]*m[2][2]));
    float yaw   = (float) atan2(m[1][0], m[0][0]);

    return Vector3(roll, pitch, yaw);
  }


  void Quaternion::toRotMatrix(std::vector <double>& rot_matrix_3_3) const {

    // create rotational matrix
    double n = norm ();
    double s = n > 0?2./(n*n):0.;

    double xs = x()*s;
    double ys = y()*s;
    double zs = z()*s;

    double ux = u()*xs;
    double uy = u()*ys;
    double uz = u()*zs;

    double xx = x()*xs;
    double xy = x()*ys;
    double xz = x()*zs;

    double yy = y()*ys;
    double yz = y()*zs;
    double zz = z()*zs;

    double m[3][3];
    m[0][0] = 1.0 - (yy + zz);
    m[1][1] = 1.0 - (xx + zz);
    m[2][2] = 1.0 - (xx + yy);

    m[1][0] = xy + uz;
    m[0][1] = xy - uz;

    m[2][0] = xz - uy;
    m[0][2] = xz + uy;
    m[2][1] = yz + ux;
    m[1][2] = yz - ux;

    rot_matrix_3_3.clear();
    rot_matrix_3_3.resize(9,0.);
    for (unsigned int i=0; i<3; i++) {
      rot_matrix_3_3[i*3] = m[i][0];
      rot_matrix_3_3[i*3+1] = m[i][1];
      rot_matrix_3_3[i*3+2] = m[i][2];
    }
  }

  Quaternion& Quaternion::operator= (const Quaternion& other) {
    u() = other.u();
    x() = other.x();
    y() = other.y();
    z() = other.z();
    return *this;
  }

  Quaternion Quaternion::operator* (const Quaternion& other) const {
    return Quaternion(u()*other.u() - x()*other.x() - y()*other.y() - z()*other.z(),
		      y()*other.z() - other.y()*z() + u()*other.x() + other.u()*x(),
		      z()*other.x() - other.z()*x() + u()*other.y() + other.u()*y(),
		      x()*other.y() - other.x()*y() + u()*other.z() + other.u()*z());
  }

  Quaternion Quaternion::operator* (const Vector3& v) const {
    return *this * Quaternion(0, v(0), v(1), v(2));
  }

  Quaternion operator* (const Vector3& v, const Quaternion& q) {
    return Quaternion(0, v(0), v(1), v(2)) * q;
  }

  Quaternion& Quaternion::normalize (){
    double len = norm ();
    if (len > 0)
      *this /= (float) len;
    return *this;
  }

  Quaternion Quaternion::normalized () const {
    Quaternion result(*this);
    result.normalize ();
    return result;
  }


  Quaternion& Quaternion::inv_IP() {
    x() = -x();
    y() = -y();
    z() = -z();
    return *this;
  }

  Vector3 Quaternion::rotate(const Vector3& v) const {
    Quaternion q = *this * v * this->inv();
    return Vector3(q.x(), q.y(), q.z());
  }


  std::istream& Quaternion::read(std::istream &s) {
    int temp;
    s >> temp; // should be 4
    for (unsigned int i=0; i<4; i++)
      s >> operator()(i);
    return s;
  }


  std::ostream& Quaternion::write(std::ostream &s) const {
    s << 4;
    for (unsigned int i=0; i<4; i++)
      s << " " << operator()(i);
    return s;
  }



  std::istream& Quaternion::readBinary(std::istream &s) {
    int temp;
    s.read((char*)&temp, sizeof(temp));
    double val = 0;
    for (unsigned int i=0; i<4; i++) {
      s.read((char*)&val, sizeof(val));
      operator()(i) = (float) val;
    }
    return s;
  }


  std::ostream& Quaternion::writeBinary(std::ostream &s) const {
    int temp = 4;
    s.write((char*)&temp, sizeof(temp));
    double val = 0;
    for (unsigned int i=0; i<4; i++) {
      val = operator()(i);
      s.write((char*)&val, sizeof(val));
    }
    return s;
  }



  std::ostream& operator<<(std::ostream& s, const Quaternion& q) {
    s << "(" << q.u() << " " << q.x() << " " << q.y() << " " << q.z() << ")";
    return s;
  }

}
