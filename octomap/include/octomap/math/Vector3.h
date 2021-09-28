/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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

#ifndef OCTOMATH_VECTOR3_H
#define OCTOMATH_VECTOR3_H

#include <iostream>
#include <math.h>


namespace octomath {

  /*!
   * \brief This class represents a three-dimensional vector
   *
   * The three-dimensional vector can be used to represent a
   * translation in three-dimensional space or to represent the
   * attitude of an object using Euler angle.
   */
  class Vector3 {
  public:

    /*!
     * \brief Default constructor
     */
    Vector3 () { data[0] = data[1] = data[2] = 0.0; }

    /*!
     * \brief Copy constructor
     *
     * @param other a vector of dimension 3
     */
    Vector3 (const Vector3& other) {
      data[0] = other(0);
      data[1] = other(1);
      data[2] = other(2);
    }

    /*!
     * \brief Constructor
     *
     * Constructs a three-dimensional vector from
     * three single values x, y, z or roll, pitch, yaw
     */
    Vector3 (float x, float y, float z) {
      data[0] = x;
      data[1] = y;
      data[2] = z;
    }


    /* inline Eigen3::Vector3f getVector3f() const { return Eigen3::Vector3f(data[0], data[1], data[2]) ; } */
    /* inline Eigen3::Vector4f& getVector4f() { return data; } */
    /* inline Eigen3::Vector4f getVector4f() const { return data; } */

    /*!
     * \brief Assignment operator
     *
     * @param other a vector of dimension 3
     */
    inline Vector3& operator= (const Vector3& other)  {
      data[0] = other(0);
      data[1] = other(1);
      data[2] = other(2);      
      return *this;
    }


    /*!
     * \brief Three-dimensional vector (cross) product
     *
     * Calculates the tree-dimensional cross product, which
     * represents the vector orthogonal to the plane defined
     * by this and other.
     * @return this x other
     */
    inline Vector3 cross (const Vector3& other) const 
    {
      //return (data.start<3> ().cross (other.data.start<3> ()));
      // \note should this be renamed?
      return Vector3(y()*other.z() - z()*other.y(),
                     z()*other.x() - x()*other.z(),
                     x()*other.y() - y()*other.x());
    }

    /// dot product
    inline double dot (const Vector3& other) const 
    {
      return x()*other.x() + y()*other.y() + z()*other.z();
    }

    inline const float& operator() (unsigned int i) const
    {
      return data[i];
    }
    inline float& operator() (unsigned int i)
    {
      return data[i];
    }

    inline float& x() 
    {
      return operator()(0);
    }

    inline float& y() 
    {
      return operator()(1);
    }

    inline float& z() 
    {
      return operator()(2);
    }

    inline const float& x() const 
    {
      return operator()(0);
    }

    inline const float& y() const 
    {
      return operator()(1);
    }

    inline const float& z() const 
    {
      return operator()(2);
    }

    inline float& roll() 
    {
      return operator()(0);
    }

    inline float& pitch() 
    {
      return operator()(1);
    }

    inline float& yaw() 
    {
      return operator()(2);
    }

    inline const float& roll() const 
    {
      return operator()(0);
    }

    inline const float& pitch() const 
    {
      return operator()(1);
    }

    inline const float& yaw() const 
    {
      return operator()(2);
    }

    inline Vector3 operator- () const 
    {
      Vector3 result;
      result(0) = -data[0];
      result(1) = -data[1];
      result(2) = -data[2];
      return result;
    }

    inline Vector3 operator+ (const Vector3 &other) const 
    {
      Vector3 result(*this);
      result(0) += other(0);
      result(1) += other(1);
      result(2) += other(2);
      return result;
    }

    inline Vector3 operator*  (float x) const {
      Vector3 result(*this);
      result(0) *= x;
      result(1) *= x;
      result(2) *= x;
      return result;
    }

    inline Vector3 operator- (const Vector3 &other) const 
    {
      Vector3 result(*this);
      result(0) -= other(0);
      result(1) -= other(1);
      result(2) -= other(2);
      return result;
    }

    inline void operator+= (const Vector3 &other)
    {
      data[0] += other(0);
      data[1] += other(1);
      data[2] += other(2);      
    }

    inline void operator-= (const Vector3& other) {
      data[0] -= other(0);
      data[1] -= other(1);
      data[2] -= other(2);      
    }

    inline void operator/= (float x) {
      data[0] /= x;
      data[1] /= x;
      data[2] /= x;      
    }

    inline void operator*= (float x) {    
      data[0] *= x;
      data[1] *= x;
      data[2] *= x;      
    }

    inline bool operator== (const Vector3 &other) const {
      for (unsigned int i=0; i<3; i++) {
        if (operator()(i) != other(i)) 
          return false;
      }
      return true;
    }
        
    /// @return length of the vector ("L2 norm")
    inline double norm () const {
      return sqrt(norm_sq());
    }

    /// @return squared length ("L2 norm") of the vector
    inline double norm_sq() const {
      return (x()*x() + y()*y() + z()*z());
    }

    /// normalizes this vector, so that it has norm=1.0
    inline Vector3& normalize () {
      double len = norm();
      if (len > 0)
        *this /= (float) len;
      return *this;
    }

    /// @return normalized vector, this one remains unchanged
    inline Vector3 normalized () const {
      Vector3 result(*this);
      result.normalize ();
      return result;
    }

    inline double angleTo(const Vector3& other) const { 
      double dot_prod = this->dot(other);
      double len1 = this->norm();
      double len2 = other.norm();
      return acos(dot_prod / (len1*len2));
    }


    inline double distance (const Vector3& other) const {
      double dist_x = x() - other.x();
      double dist_y = y() - other.y();
      double dist_z = z() - other.z();
      return sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);
    }

    inline double distanceXY (const Vector3& other) const {
      double dist_x = x() - other.x();
      double dist_y = y() - other.y();
      return sqrt(dist_x*dist_x + dist_y*dist_y);
    }

    Vector3& rotate_IP (double roll, double pitch, double yaw);

    //    void read (unsigned char * src, unsigned int size);
    std::istream& read(std::istream &s);
    std::ostream& write(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;


  protected:
    float data[3];

  };


  //! user friendly output in format (x y z)
  std::ostream& operator<<(std::ostream& out, octomath::Vector3 const& v);

}


#endif
