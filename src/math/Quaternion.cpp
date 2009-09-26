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

  const double& Quaternion::operator() (unsigned int i) const{ 
    return data[i];
  }
  double& Quaternion::operator() (unsigned int i){ 
    return data[i];
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

    double _u = sqrt(OUR_MAX(0., 1 + m[0][0] + m[1][1] + m[2][2]))/2.0;
    double _x = sqrt(OUR_MAX(0., 1 + m[0][0] - m[1][1] - m[2][2]))/2.0;
    double _y = sqrt(OUR_MAX(0., 1 - m[0][0] + m[1][1] - m[2][2]))/2.0;
    double _z = sqrt(OUR_MAX(0., 1 - m[0][0] - m[1][1] + m[2][2]))/2.0;
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

  Quaternion Quaternion::inv() const {
    return Quaternion(u(), -x(), -y(), -z());
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

  double& Quaternion::u() {
    return operator()(0);
  }

  double& Quaternion::x() {
    return operator()(1);
  }

  double& Quaternion::y() {
    return operator()(2);
  }

  double& Quaternion::z() {
    return operator()(3);
  }

  const double& Quaternion::u() const {
    return operator()(0);
  }

  const double& Quaternion::x() const {
    return operator()(1);
  }

  const double& Quaternion::y() const {
    return operator()(2);
  }

  const double& Quaternion::z() const {
    return operator()(3);
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
