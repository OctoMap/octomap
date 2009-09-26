#include "Vector3.h"
#include <cassert>
#include <math.h>
#include <string.h>

namespace octomath {

  Vector3::Vector3() {
  }

  Vector3::Vector3(const Vector3& other) {
    for (unsigned int i=0; i<3; i++) {
      operator()(i) = other(i);
    }
  }

  Vector3::Vector3(double x, double y, double z) {
    operator()(0) = x;
    operator()(1) = y;
    operator()(2) = z;
  }

  Vector3 & Vector3::operator= (const Vector3& other) {
    for (unsigned int i=0; i<3; i++) {
      operator()(i) = other(i);
    }
    return *this;
  }

  void Vector3::operator-= (const Vector3& other){ 
    for (unsigned int i=0; i<3; i++) {
      operator()(i) -= other(i);
    }
  }


  Vector3 Vector3::crossProduct(const Vector3& other) const {
    return Vector3(y()*other.z() - z()*other.y(),
		   z()*other.x() - x()*other.z(),
		   x()*other.y() - y()*other.x());
  }

  double Vector3::dotProduct(const Vector3& other) const {
    return x()*other.x() + y()*other.y() + z()*other.z();
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


  const double& Vector3::operator() (unsigned int i) const{ 
    return data[i];
  }
  double& Vector3::operator() (unsigned int i){ 
    return data[i];
  }
  
  double& Vector3::x() {
    return operator()(0);
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

  Vector3 Vector3::operator- () const {
    Vector3 result(*this);
    for (unsigned int i=0; i<3; ++i) {
      result(i) = -result(i);
    }
    return result;
  }

  Vector3 Vector3::operator+ (const Vector3 &other) const {
    Vector3 result(*this);
    for (unsigned int i=0; i<3; ++i) {
      result(i) += other(i);
    }
    return result;
  }

  Vector3 Vector3::operator- (const Vector3 &other) const {
    Vector3 result(*this);
    for (unsigned int i=0; i<3; ++i) {
      result(i) -= other(i);
    }
    return result;
  }
  
  void Vector3::operator+= (const Vector3 &other){
    for (unsigned int i=0; i<3; i++) {
      operator()(i) += other(i);
    }
  }

  bool Vector3::operator== (const Vector3 &other) const { 
    for (unsigned int i=0; i<3; i++) {
      if (operator()(i) != other(i)) return false;
    }
    return true;
  }

  void Vector3::operator/= (double x) {
    for (unsigned int i=0; i<3; i++) {
      operator()(i) /= x;
    }
  }

  
  double Vector3::norm2() const {
    double n = 0;
    for (unsigned int i=0; i<3; i++) {
      n += operator()(i) * operator()(i);
    }
    return sqrt(n);
  }


  Vector3& Vector3::unit_IP (){ 
    double len = norm2();
    if (len > 0)
      *this /= len;
    return *this;
  }
  
  Vector3 Vector3::unit () const {
    Vector3 result(*this);
    result.unit_IP();
    return result;
  }



//   void Vector3::read(unsigned char * src, unsigned int size){
//     memcpy(&data[0],src, sizeof(double));    
//     memcpy(&data[1],src, sizeof(double));    
//     memcpy(&data[2],src, sizeof(double));    
//   }


  std::istream& Vector3::read(std::istream &s) {
    int temp;
    s >> temp; // should be 3
    for (unsigned int i=0; i<3; i++)
      s >> operator()(i);
    return s;
  }


  std::ostream& Vector3::write(std::ostream &s) const {
    s << 3;
    for (unsigned int i=0; i<3; i++)
      s << " " << operator()(i);
    return s;
  }



  std::istream& Vector3::readBinary(std::istream &s) {
    int temp;
    s.read((char*)&temp, sizeof(temp));
    double val = 0;
    for (unsigned int i=0; i<3; i++) {
      s.read((char*)&val, sizeof(val));
      operator()(i) = val;
    }
    return s;
  }


  std::ostream& Vector3::writeBinary(std::ostream &s) const {
    int temp = 3;
    s.write((char*)&temp, sizeof(temp));
    double val = 0;
    for (unsigned int i=0; i<3; i++) {
      val = operator()(i);
      s.write((char*)&val, sizeof(val));
    }
    return s;
  }


  std::ostream& operator<<(std::ostream& out, octomath::Vector3 const& v) {
    return out << '(' << v.x() << ' ' << v.y() << ' ' << v.z() << ')';
  }

}

