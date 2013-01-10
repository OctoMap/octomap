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

#ifndef OCTOMATH_POSE6D_H
#define OCTOMATH_POSE6D_H

#include "Vector3.h"
#include "Quaternion.h"

namespace octomath {

  /*!
   * \brief This class represents a tree-dimensional pose of an object
   *
   * The tree-dimensional pose is represented by a three-dimensional
   * translation vector representing the position of the object and
   * a Quaternion representing the attitude of the object
   */
  class Pose6D {
  public:

    Pose6D();
    ~Pose6D();

    /*!
     * \brief Constructor
     *
     * Constructs a pose from given translation and rotation.
     */
    Pose6D(const Vector3& trans, const Quaternion& rot);

    /*!
     * \brief Constructor
     *
     * Constructs a pose from a translation represented by
     * its x, y, z-values and a rotation represented by its
     * Tait-Bryan angles roll, pitch, and yaw
     */
    Pose6D(float x, float y, float z, double roll, double pitch, double yaw);

    Pose6D& operator= (const Pose6D& other);
    bool operator==(const Pose6D& other) const;
    bool operator!=(const Pose6D& other) const;

    /*!
     * \brief Translational component
     *
     * @return the translational component of this pose
     */
    inline Vector3& trans() { return translation; }
    /*!
     * \brief Rotational component
     *
     * @return the rotational component of this pose
     */
    inline Quaternion& rot() { return rotation; }
    /*!
     * \brief Translational component
     *
     * @return the translational component of this pose
     */
    const Vector3& trans() const { return translation; }
    /*!
     * \brief Rotational component
     * @return the rotational component of this pose
     */
    const Quaternion& rot() const { return rotation; }


    inline float& x() { return translation(0); }
    inline float& y() { return translation(1); }
    inline float& z() { return translation(2); }
    inline const float& x() const { return translation(0); }
    inline const float& y() const { return translation(1); }
    inline const float& z() const { return translation(2); }

    inline double roll()  const {return (rotation.toEuler())(0); }
    inline double pitch() const {return (rotation.toEuler())(1); }
    inline double yaw()   const {return (rotation.toEuler())(2); }

    /*!
     * \brief Transformation of a vector
     *
     * Transforms the vector v by the transformation which is
     * specified by this.
     * @return the vector which is translated by the translation of
     * this and afterwards rotated by the rotation of this.
     */
    Vector3 transform (const Vector3 &v) const;

    /*!
     * \brief Inversion
     *
     * Inverts the coordinate transformation represented by this pose
     * @return a copy of this pose inverted
     */
    Pose6D inv() const;

    /*!
     * \brief Inversion
     *
     * Inverts the coordinate transformation represented by this pose
     * @return a reference to this pose
     */
    Pose6D& inv_IP();

    /*!
     * \brief Concatenation
     *
     * Concatenates the coordinate transformations represented
     * by this and p.
     * @return this * p (applies first this, then p)
     */
    Pose6D operator* (const Pose6D &p) const;
    /*!
     * \brief In place concatenation
     *
     * Concatenates p to this Pose6D.
     * @return this which results from first moving by this and
     * afterwards by p
     */
    const Pose6D& operator*= (const Pose6D &p);

    /*!
     * \brief Translational distance
     *
     * @return the translational (euclidian) distance to p
     */
    double distance(const Pose6D &other) const;

    /*!
     * \brief Translational length
     *
     * @return the translational (euclidian) length of the translation
     * vector of this Pose6D
     */
    double transLength() const;

    /*!
     * \brief Output operator
     *
     * Output to stream in a format which can be parsed using read().
     */
    std::ostream& write(std::ostream &s) const;
    /*!
     * \brief Input operator
     *
     * Parsing from stream which was written by write().
     */
    std::istream& read(std::istream &s);
    /*!
     * \brief Binary output operator
     *
     * Output to stream in a binary format which can be parsed using readBinary().
     */
    std::ostream& writeBinary(std::ostream &s) const;
    /*!
     * \brief Binary input operator
     *
     * Parsing from binary stream which was written by writeBinary().
     */
    std::istream& readBinary(std::istream &s);

  protected:
    Vector3 translation;
    Quaternion rotation;
  };

  //! user friendly output in format (x y z, u x y z) which is (translation, rotation)
  std::ostream& operator<<(std::ostream& s, const Pose6D& p);

}

#endif
