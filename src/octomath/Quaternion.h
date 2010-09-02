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

#ifndef OCTOMATH_QUATERNION_H
#define OCTOMATH_QUATERNION_H

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
     * Converts the attitude represented by this to
     * Euler angles (roll, pitch, yaw).
     */
    Vector3 toEuler() const;


    inline const double& operator() (unsigned int i) const {
      return data[i];
    }

    inline double& operator() (unsigned int i) {
      return data[i];
    }

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
     * @return this * other
     */
    Quaternion operator* (const Quaternion& other) const {
      return Quaternion(u()*other.u() - x()*other.x() - y()*other.y() - z()*other.z(),
                        y()*other.z() - other.y()*z() + u()*other.x() + other.u()*x(),
                        z()*other.x() - other.z()*x() + u()*other.y() + other.u()*y(),
                        x()*other.y() - other.x()*y() + u()*other.z() + other.u()*z());
    }

    /*!
     * \brief Quaternion multiplication with extended vector
     *
     * @return q * (0, v)
     */
    inline Quaternion operator* (const Vector3 &v) const {
      return *this * Quaternion(0, v(0), v(1), v(2));
    }


    /*!
     * \brief Quaternion multiplication with extended vector
     *
     * @return (0, v) * q
     */
    inline friend Quaternion operator* (const Vector3 &v, const Quaternion &q) {
      return Quaternion(0, v(0), v(1), v(2)) * q;
    }

    /*!
     * \brief Normalization
     *
     * @return A copy of this Quaternion as Unit Quaternion
     */
    Quaternion normalized() const;

    /*!
     * \brief Normalization
     *
     * Normalizes this Quaternion.
     * @return a reference to this Quaternion
     */
    Quaternion& normalize();

    /*!
     * \brief Inversion
     *
     * @return A copy of this Quaterion inverted
     */
    inline Quaternion inv() const {
      return Quaternion(u(), -x(), -y(), -z());
    }

    /*!
     * \brief Inversion
     *
     * Inverts this Quaternion
     * @return a reference to this Quaternion
     */
    Quaternion& inv_IP();

    /*!
     * \brief Rotate a vector
     *
     * Rotates a vector to the body fixed coordinate
     * system according to the attitude represented by
     * this Quaternion.
     * @param v a vector represented in world coordinates
     * @return v represented in body-fixed coordinates
     */
    Vector3 rotate(const Vector3 &v) const;

    inline double& u(){ return operator()(0); }
    inline double& x(){ return operator()(1); } 
    inline double& y(){ return operator()(2); }
    inline double& z(){ return operator()(3); }

    inline const double& u() const { return operator()(0); }
    inline const double& x() const { return operator()(1); }
    inline const double& y() const { return operator()(2); }
    inline const double& z() const { return operator()(3); }

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
