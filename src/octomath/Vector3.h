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

#ifndef OCTOMATH_VECTOR3_H
#define OCTOMATH_VECTOR3_H

#include <iostream>


namespace octomath {

  /*!
   * \brief This class represents a tree-dimensional vector
   *
   * The tree-dimensional vector can be used to represent a
   * translation in tree-dimensional space or to represent the
   * attitude of an object using Euler angle.
   */
  class Vector3 {
  public:

    /*!
     * \brief Default constructor
     */
    Vector3();

    /*!
     * \brief Copy constructor
     *
     * @param other a vector of dimension 3
     */
    Vector3(const Vector3& other);

    /*!
     * \brief Constructor
     *
     * Constructs a tree-dimensional vector from
     * tree single values x, y, z or roll, pitch, yaw
     */
    Vector3(double x, double y, double z);

    /*!
     * \brief Assignment operator
     *
     * @param other a vector of dimension 3
     */
    Vector3& operator = (const Vector3& other);

    void operator-= (const Vector3& other);

    /*!
     * \brief Three-dimensional vector (cross) product
     *
     * Calculates the tree-dimensional cross product, which
     * represents the vector orthogonal to the plane defined
     * by this and other.
     * @return this x other
     */
    Vector3 crossProduct(const Vector3& other) const;

    double dotProduct(const Vector3& other) const;
    double innerProd(const Vector3& other) const { return dotProduct(other); }

    Vector3& rotate_IP (double roll, double pitch, double yaw);

    double sqrDist(const Vector3& other) const;
    double dist(const Vector3& other) const;

    const double& operator() (unsigned int i) const;
    double& operator() (unsigned int i);

    double& x();
    double& y();
    double& z();
    const double& x() const;
    const double& y() const;
    const double& z() const;

    double& roll();
    double& pitch();
    double& yaw();
    const double& roll() const;
    const double& pitch() const;
    const double& yaw() const;

    Vector3 operator- () const;
    Vector3 operator- (const Vector3 &other) const;
    Vector3 operator+ (const Vector3 &other) const;
    Vector3 operator*  (double x) const;
    void    operator+= (const Vector3 &other);
    bool    operator== (const Vector3 &other) const;
    void operator/= (double x);


    double norm2() const;
    Vector3  unit () const;
    Vector3& unit_IP ();


    //    void read (unsigned char * src, unsigned int size);
    std::istream& read(std::istream &s);
    std::ostream& write(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;


  protected:
    double data[3];
  };


  //! user friendly output in format (x y z)
  std::ostream& operator<<(std::ostream& out, octomath::Vector3 const& v);

}


#endif
