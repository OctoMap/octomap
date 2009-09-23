#include "Quaternion.h"
#include "Vector3.h"

#include <cassert>
#include <math.h>
#include "Utils.h"


// used from Vector: norm2, unit, *

namespace octomath {

  Quaternion::Quaternion() : Vector(4){
    u() = 1;
  }

  Quaternion::Quaternion(const Vector &other) : Vector(other) {
    assert(other.size() == 4);
  }

  Quaternion::Quaternion(OUR_REAL uu, OUR_REAL xx, OUR_REAL yy, OUR_REAL zz) : Vector(4, Vector::UINIT) {
    u() = uu;
    x() = xx;
    y() = yy;
    z() = zz;
  }

  Quaternion::Quaternion(const Vector3 &other) : Vector(4) {
    operator=(Quaternion(other.roll(), other.pitch(), other.yaw()));
  }

  Quaternion::Quaternion(OUR_REAL roll, OUR_REAL pitch, OUR_REAL yaw) : Vector(4, Vector::UINIT) {
    OUR_REAL sroll   = sin(roll);
    OUR_REAL spitch = sin(pitch);
    OUR_REAL syaw   = sin(yaw);
    OUR_REAL croll   = cos(roll);
    OUR_REAL cpitch = cos(pitch);
    OUR_REAL cyaw   = cos(yaw);

    OUR_REAL m[3][3] = { //create rotational Matrix
      {cyaw*cpitch, cyaw*spitch*sroll - syaw*croll, cyaw*spitch*croll + syaw*sroll},
      {syaw*cpitch, syaw*spitch*sroll + cyaw*croll, syaw*spitch*croll - cyaw*sroll},
      {    -spitch,                  cpitch*sroll,                  cpitch*croll}
    };

    OUR_REAL _u = sqrt(OUR_MAX(0., 1 + m[0][0] + m[1][1] + m[2][2]))/2.0;
    OUR_REAL _x = sqrt(OUR_MAX(0., 1 + m[0][0] - m[1][1] - m[2][2]))/2.0;
    OUR_REAL _y = sqrt(OUR_MAX(0., 1 - m[0][0] + m[1][1] - m[2][2]))/2.0;
    OUR_REAL _z = sqrt(OUR_MAX(0., 1 - m[0][0] - m[1][1] + m[2][2]))/2.0;
    u() = _u;
    x() = (m[2][1] - m[1][2])>=0?fabs(_x):-fabs(_x);
    y() = (m[0][2] - m[2][0])>=0?fabs(_y):-fabs(_y);
    z() = (m[1][0] - m[0][1])>=0?fabs(_z):-fabs(_z);
  }

  
  Vector3 Quaternion::toEuler() const {
    // create rotational matrix
    OUR_REAL n = norm2(); 
    OUR_REAL s = n > 0?2./(n*n):0.;

    OUR_REAL xs = x()*s;
    OUR_REAL ys = y()*s;
    OUR_REAL zs = z()*s;

    OUR_REAL ux = u()*xs;
    OUR_REAL uy = u()*ys;
    OUR_REAL uz = u()*zs;

    OUR_REAL xx = x()*xs;
    OUR_REAL xy = x()*ys;
    OUR_REAL xz = x()*zs;

    OUR_REAL yy = y()*ys;
    OUR_REAL yz = y()*zs;
    OUR_REAL zz = z()*zs;

    OUR_REAL m[3][3];

    m[0][0] = 1.0 - (yy + zz);
    m[1][1] = 1.0 - (xx + zz);
    m[2][2] = 1.0 - (xx + yy);

    m[1][0] = xy + uz;
    m[0][1] = xy - uz;

    m[2][0] = xz - uy;
    m[0][2] = xz + uy;
    m[2][1] = yz + ux;
    m[1][2] = yz - ux;

    OUR_REAL roll  = atan2(m[2][1], m[2][2]);
    OUR_REAL pitch = atan2(-m[2][0], sqrt(m[2][1]*m[2][1] + m[2][2]*m[2][2]));
    OUR_REAL yaw   = atan2(m[1][0], m[0][0]);

    return Vector3(roll, pitch, yaw);
  }

  
  Quaternion& Quaternion::operator= (const Quaternion& other) {
    u() = other.u();
    x() = other.x();
    y() = other.y();
    z() = other.z();
    return *this;
  }

  Quaternion& Quaternion::operator= (const Vector& other) {
    assert(other.size() == 4);
    *this = Quaternion(other);
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

  OUR_REAL& Quaternion::u() {
    return operator()(0);
  }

  OUR_REAL& Quaternion::x() {
    return operator()(1);
  }

  OUR_REAL& Quaternion::y() {
    return operator()(2);
  }

  OUR_REAL& Quaternion::z() {
    return operator()(3);
  }

  const OUR_REAL& Quaternion::u() const {
    return operator()(0);
  }

  const OUR_REAL& Quaternion::x() const {
    return operator()(1);
  }

  const OUR_REAL& Quaternion::y() const {
    return operator()(2);
  }

  const OUR_REAL& Quaternion::z() const {
    return operator()(3);
  }

  std::ostream& operator<<(std::ostream& s, const Quaternion& q) {
    s << "(" << q.u() << " " << q.x() << " " << q.y() << " " << q.z() << ")";
    return s;
  }

}
