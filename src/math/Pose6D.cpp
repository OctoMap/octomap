#include "Pose6D.h"
#include <math.h>

namespace octomath {

  Pose6D::Pose6D() {
  }


  Pose6D::Pose6D(const Vector3 &trans, const Quaternion &rot) :
    translation(trans),
    rotation(rot)
  { }



  Pose6D::Pose6D(double x, double y, double z, double roll, double pitch, double yaw) :
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


  Vector3& Pose6D::trans() {
    return translation;
  }


  const Vector3& Pose6D::trans() const {
    return translation;
  }


  Quaternion& Pose6D::rot() {
    return rotation;
  }

  
  const Quaternion& Pose6D::rot() const {
    return rotation;
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


  double Pose6D::TransLength() const {
    return transLength();
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
