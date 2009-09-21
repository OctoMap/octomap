#include "Vector3.h"
#include <cassert>
#include <math.h>

namespace octomath {

  Vector3::Vector3() : Vector(3) {
  }

  Vector3::Vector3(const Vector& other) : Vector(other) {
    assert(other.size() == 3);
  }

  Vector3::Vector3(double x, double y, double z) : Vector(3, Vector::UINIT) {
    operator()(0) = x;
    operator()(1) = y;
    operator()(2) = z;
  }

  Vector3 & Vector3::operator = (const Vector& other) {
    assert(other.size() == 3);
    *this = Vector3(other);
    return *this;
  }

  void Vector3::operator-= (const Vector3& other){ 
    this->x() -= other.x();
    this->y() -= other.y();
    this->z() -= other.z();
  }


  Vector3 Vector3::crossProduct(const Vector3& other) const {
    return Vector3(y()*other.z() - z()*other.y(),
		   z()*other.x() - x()*other.z(),
		   x()*other.y() - y()*other.x());
  }

  double Vector3::dotProduct(const Vector3& other) const {
    return x()*other.x() + y()*other.y() + z()*other.z();
  }


  double Vector3::angleTo(const Vector3& other) const {
    
    double dot_prod = this->dotProduct(other);
    double len1 = this->norm2();
    double len2 = other.norm2();

    return acos(dot_prod / (len1*len2));
  }

  Vector3& Vector3::rotate_IP (double roll, double pitch, double yaw) {

    double x, y, z;

    // pitch (around y)
    x = (*this)(0); z = (*this)(2);
    (*this)(0) = z * sin(pitch) + x * cos(pitch);
    (*this)(2) = z * cos(pitch) - x * sin(pitch);


    // yaw (around z)
    x = (*this)(0); y = (*this)(1);
    (*this)(0) = x * cos(yaw) - y * sin(yaw);
    (*this)(1) = x * sin(yaw) + y * cos(yaw);

    // roll (around x)
    y = (*this)(1); z = (*this)(2);
    (*this)(1) = y * cos(roll) - z * sin(roll);
    (*this)(2) = y * sin(roll) + z * cos(roll);

    return *this;
  }

  double Vector3::sqrDist(const Vector3& other) const {
    double diff = (*this)(0) - other(0);
    double ret = diff*diff;
    diff = (*this)(1) - other(1);
    ret += diff*diff;
    diff = (*this)(2) - other(2);
    ret += diff*diff;
    return ret;
  }

  double Vector3::dist(const Vector3& other) const {
    return sqrt(this->sqrDist(other));
  }

  
  double& Vector3::x() {
    return (*this)(0);
  }

  double& Vector3::y() {
    return operator()(1);
  }

  double& Vector3::z() {
    return operator()(2);
  }

  const double& Vector3::x() const {
    return operator()(0);
  }

  const double& Vector3::y() const {
    return operator()(1);
  }

  const double& Vector3::z() const {
    return operator()(2);
  }

  double& Vector3::roll() {
    return operator()(0);
  }

  double& Vector3::pitch() {
    return operator()(1);
  }

  double& Vector3::yaw() {
    return operator()(2);
  }

  const double& Vector3::roll() const {
    return operator()(0);
  }

  const double& Vector3::pitch() const {
    return operator()(1);
  }

  const double& Vector3::yaw() const {
    return operator()(2);
  }


  std::ostream& operator<<(std::ostream& out, octomath::Vector3 const& v) {
    return out << '(' << v.x() << ' ' << v.y() << ' ' << v.z() << ')';
  }

}

