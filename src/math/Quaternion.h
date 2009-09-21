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

  class Quaternion : public Vector {
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
     *
     * @param other a vector of dimension 4
     */
    Quaternion(const Vector& other);

    /*!
     * \brief Constructor
     *
     * Constructs a Quaternion from four single
     * values
     */
    Quaternion(OUR_REAL u, OUR_REAL x, OUR_REAL y, OUR_REAL z);

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
    Quaternion(OUR_REAL roll, OUR_REAL pitch, OUR_REAL yaw);


    /*!
     * \brief Conversion to Euler angles
     *
     * Converts the attitude represented by <this> to
     * Euler angles (roll, pitch, yaw).
     */
    Vector3 toEuler() const;
    
    /*!
     * \brief Assignment operator
     */
    Quaternion& operator= (const Quaternion& other);

    /*!
     * \brief Assignment operator
     *
     * @param other a vector of dimension 4
     */
    Quaternion& operator= (const Vector& other);

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
    Quaternion inverse() const __attribute__ ((deprecated)) { return inv(); } // for inversion with copy use inv()

    /*!
     * \brief Inversion
     *
     * Inverts <this> Quaternion
     * @return a reference to <this> Quaternion
     */
    Quaternion& inv_IP();
    Quaternion& invert() __attribute__ ((deprecated)) { return inv_IP(); } // for inversion in place use inv_IP()

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

    OUR_REAL& u();
    OUR_REAL& x();
    OUR_REAL& y();
    OUR_REAL& z();
    const OUR_REAL& u() const;
    const OUR_REAL& x() const;
    const OUR_REAL& y() const;
    const OUR_REAL& z() const;
  };

  //! user friendly output in format (u x y z)
  std::ostream& operator<<(std::ostream& s, const Quaternion& q);

}

#endif
