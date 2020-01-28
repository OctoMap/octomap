/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
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

#ifndef OCTOMATH_QUATERNION_H
#define OCTOMATH_QUATERNION_H

#include "Vector3.h"

#include <iostream>
#include <vector>

#ifdef __CUDACC__
#ifndef CUDA_CALLABLE
#define CUDA_CALLABLE __host__ __device__
#endif
#else
#ifndef CUDA_CALLABLE
#define CUDA_CALLABLE
#endif
#endif

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
    CUDA_CALLABLE inline Quaternion() { u() = 1;  x() = 0; y() = 0; z() = 0;  }

    /*!
     * \brief Copy constructor
     */
    CUDA_CALLABLE Quaternion(const Quaternion& other);

    /*!
     * \brief Constructor
     *
     * Constructs a Quaternion from four single
     * values
     */
    CUDA_CALLABLE Quaternion(float u, float x, float y, float z);

    /*!
     * \brief Constructor
     *
     * @param other a vector containing euler angles
     */
    CUDA_CALLABLE Quaternion(const Vector3& other);

    /*!
     * \brief Constructor from Euler angles
     *
     * Constructs a Unit Quaternion from Euler angles / Tait Bryan
     * angles (in radians) according to the 1-2-3 convention.
     * @param roll phi/roll angle (rotation about x-axis)
     * @param pitch theta/pitch angle (rotation about y-axis)
     * @param yaw psi/yaw angle (rotation about z-axis)
     */
    CUDA_CALLABLE Quaternion(double roll, double pitch, double yaw);


     
    //! Constructs a Unit Quaternion from a rotation angle and axis.  
    CUDA_CALLABLE Quaternion(const Vector3& axis, double angle);


    /*!
     * \brief Conversion to Euler angles
     *
     * Converts the attitude represented by this to
     * Euler angles (roll, pitch, yaw).
     */
    CUDA_CALLABLE Vector3 toEuler() const;

    CUDA_CALLABLE void toRotMatrix(std::vector <double>& rot_matrix_3_3) const;


    CUDA_CALLABLE inline const float& operator() (unsigned int i) const { return data[i]; }
    CUDA_CALLABLE inline float& operator() (unsigned int i) { return data[i]; }

    CUDA_CALLABLE float norm () const;
    CUDA_CALLABLE Quaternion  normalized () const;
    CUDA_CALLABLE Quaternion& normalize ();


    CUDA_CALLABLE void operator/= (float x);
    CUDA_CALLABLE Quaternion& operator= (const Quaternion& other);
    CUDA_CALLABLE bool operator== (const Quaternion& other) const;

    /*!
     * \brief Quaternion multiplication
     *
     * Standard Quaternion multiplication which is not
     * commutative.
     * @return this * other
     */
    CUDA_CALLABLE Quaternion operator* (const Quaternion& other) const;

    /*!
     * \brief Quaternion multiplication with extended vector
     *
     * @return q * (0, v)
     */
    CUDA_CALLABLE Quaternion operator* (const Vector3 &v) const;

    /*!
     * \brief Quaternion multiplication with extended vector
     *
     * @return (0, v) * q
     */
    CUDA_CALLABLE friend Quaternion operator* (const Vector3 &v, const Quaternion &q);

    /*!
     * \brief Inversion
     *
     * @return A copy of this Quaterion inverted
     */
    CUDA_CALLABLE inline Quaternion inv() const {  return Quaternion(u(), -x(), -y(), -z()); }


    /*!
     * \brief Inversion
     *
     * Inverts this Quaternion
     * @return a reference to this Quaternion
     */
    CUDA_CALLABLE Quaternion& inv_IP();

    /*!
     * \brief Rotate a vector
     *
     * Rotates a vector to the body fixed coordinate
     * system according to the attitude represented by
     * this Quaternion.
     * @param v a vector represented in world coordinates
     * @return v represented in body-fixed coordinates
     */
    CUDA_CALLABLE Vector3 rotate(const Vector3 &v) const;

    CUDA_CALLABLE inline float& u() { return data[0]; }
    CUDA_CALLABLE inline float& x() { return data[1]; }
    CUDA_CALLABLE inline float& y() { return data[2]; }
    CUDA_CALLABLE inline float& z() { return data[3]; }

    CUDA_CALLABLE inline const float& u() const { return data[0]; }
    CUDA_CALLABLE inline const float& x() const { return data[1]; }
    CUDA_CALLABLE inline const float& y() const { return data[2]; }
    CUDA_CALLABLE inline const float& z() const { return data[3]; }

    std::istream& read(std::istream &s);
    std::ostream& write(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;

  protected:
    float data[4];

  };

  //! user friendly output in format (u x y z)
  std::ostream& operator<<(std::ostream& s, const Quaternion& q);

}

#endif
