// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
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

 //   ~Vector3();


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
     * by <this> and other.
     * @return <this> x other
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
