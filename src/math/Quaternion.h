#ifndef FERN_QUATERNION_H
#define FERN_QUATERNION_H

#include "Vector3.h"

#include <iostream>


namespace octomath {

  /*!
   * \brief This class represents a Quaternion.
   *
   * The Unit Quaternion is one possible representation of the
   * attitude of an object in tree-dimensional space.
   *
   * This Quaternion class is implemented according to Diebel,
   * James. Representing Attitude: Euler Angle, Unit Quaternions, and
   * Rotation Vectors. Stanford University. 2006. - Technical Report.
   */

  class Quaternion {

  public:

    /*!
     * \brief Default constructor
     *
     * Constructs the (1,0,0,0) Unit Quaternion
     * representing the identity rotation.
     */
    Quaternion();

    /*!
     * \brief Copy constructor
     */
    Quaternion(const Quaternion& other);

    /*!
     * \brief Constructor
     *
     * Constructs a Quaternion from four single
     * values
     */
    Quaternion(double u, double x, double y, double z);

    /*!
     * \brief Constructor
     *
     * @param other a vector containing euler angles
     */
    Quaternion(const Vector3& other);

    /*!
     * \brief Constructor from Euler angles
     *
     * Constructs a Unit Quaternion from Euler angles / Tait Bryan
     * angles (in radians) according to the 1-2-3 convention.
     * @param roll phi/roll angle (rotation about x-axis)
     * @param pitch theta/pitch angle (rotation about y-axis)
     * @param yaw psi/yaw angle (rotation about z-axis)
     */
    Quaternion(double roll, double pitch, double yaw);


    /*!
     * \brief Conversion to Euler angles
     *
     * Converts the attitude represented by <this> to
     * Euler angles (roll, pitch, yaw).
     */
    Vector3 toEuler() const;


    const double& operator() (unsigned int i) const;
    double& operator() (unsigned int i);

    double norm2() const;
    Quaternion  unit () const;
    Quaternion& unit_IP ();

    
    void operator/= (double x);
    Quaternion& operator= (const Quaternion& other);
    bool operator== (const Quaternion&other) const; 

    /*!
     * \brief Quaternion multiplication
     *
     * Standard Quaternion multiplication which is not
     * commutative.
     * @return <this> * other
     */
    Quaternion operator* (const Quaternion& other) const;

    /*!
     * \brief Quaternion multiplication with extended vector
     *
     * @return q * (0, v)
     */
    Quaternion operator* (const Vector3 &v) const;

    /*!
     * \brief Quaternion multiplication with extended vector
     *
     * @return (0, v) * q
     */
    friend Quaternion operator* (const Vector3 &v, const Quaternion &q);

    /*!
     * \brief Normalization
     *
     * @return A copy of <this> Quaternion as Unit Quaternion
     */
    Quaternion normalized() const;

    /*!
     * \brief Normalization
     *
     * Normalizes <this> Quaternion.
     * @return a reference to <this> Quaternion
     */
    Quaternion& normalize();

    /*!
     * \brief Inversion
     *
     * @return A copy of <this> Quaterion inverted
     */
    Quaternion inv() const;

    /*!
     * \brief Inversion
     *
     * Inverts <this> Quaternion
     * @return a reference to <this> Quaternion
     */
    Quaternion& inv_IP();

    /*!
     * \brief Rotate a vector
     *
     * Rotates a vector to the body fixed coordinate
     * system according to the attitude represented by
     * <this> Quaternion.
     * @param v a vector represented in world coordinates
     * @return v represented in body-fixed coordinates
     */
    Vector3 rotate(const Vector3 &v) const;

    double& u();
    double& x();
    double& y();
    double& z();
    const double& u() const;
    const double& x() const;
    const double& y() const;
    const double& z() const;

    std::istream& read(std::istream &s);
    std::ostream& write(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;

  protected:
    
    double data[4];

  };

  //! user friendly output in format (u x y z)
  std::ostream& operator<<(std::ostream& s, const Quaternion& q);

}

#endif
