// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
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

#include "Quaternion.h"
#include "Vector3.h"

#include <cassert>
#include <math.h>
#include "Utils.h"


// used from Vector: norm2, unit, *

namespace octomath {

  Quaternion::Quaternion(){
    u() = 1;
    x() = 0;
    y() = 0;
    z() = 0;
  }


  Quaternion::Quaternion(const Quaternion &other) {
    for (unsigned int i=0; i<4; i++) {
      operator()(i) = other(i);
    }
  }

  Quaternion::Quaternion(double uu, double xx, double yy, double zz) {
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

    double _u = sqrt(std::max(0., 1 + m[0][0] + m[1][1] + m[2][2]))/2.0;
    double _x = sqrt(std::max(0., 1 + m[0][0] - m[1][1] - m[2][2]))/2.0;
    double _y = sqrt(std::max(0., 1 - m[0][0] + m[1][1] - m[2][2]))/2.0;
    double _z = sqrt(std::max(0., 1 - m[0][0] - m[1][1] + m[2][2]))/2.0;
    u() = _u;
    x() = (m[2][1] - m[1][2])>=0?fabs(_x):-fabs(_x);
    y() = (m[0][2] - m[2][0])>=0?fabs(_y):-fabs(_y);
    z() = (m[1][0] - m[0][1])>=0?fabs(_z):-fabs(_z);
  }


  double Quaternion::norm2() const {
    double n = 0;
    for (unsigned int i=0; i<4; i++) {
      n += operator()(i) * operator()(i);
    }
    return sqrt(n);
  }

  void Quaternion::operator/= (double x) {
    for (unsigned int i=0; i<4; i++) {
      operator()(i) /= x;
    }
  }

  bool Quaternion::operator== (const Quaternion& other) const {
    for (unsigned int i=0; i<4; i++) {
      if (operator()(i) != other(i)) return false;
    }
    return true;
  }


  Vector3 Quaternion::toEuler() const {
    // create rotational matrix
    double n = norm2();
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

    double roll  = atan2(m[2][1], m[2][2]);
    double pitch = atan2(-m[2][0], sqrt(m[2][1]*m[2][1] + m[2][2]*m[2][2]));
    double yaw   = atan2(m[1][0], m[0][0]);

    return Vector3(roll, pitch, yaw);
  }


  Quaternion& Quaternion::operator= (const Quaternion& other) {
    u() = other.u();
    x() = other.x();
    y() = other.y();
    z() = other.z();
    return *this;
  }


  Quaternion& Quaternion::unit_IP (){
    double len = norm2();
    if (len > 0)
      *this /= len;
    return *this;
  }

  Quaternion Quaternion::unit () const {
    Quaternion result(*this);
    result.unit_IP();
    return result;
  }

  Quaternion Quaternion::normalized() const {
    return unit();
  }

  Quaternion& Quaternion::normalize() {
    unit_IP();
    return *this;
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
      operator()(i) = val;
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
