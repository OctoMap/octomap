#ifndef FERN_VECTOR3_H
#define FERN_VECTOR3_H

#include "Vector.h"

namespace octomath {

  /*!
   * \brief This class represents a tree-dimensional vector
   *
   * The tree-dimensional vector can be used to represent a
   * translation in tree-dimensional space or to represent the
   * attitude of an object using Euler angle.
   */
  class Vector3 : public Vector {
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
    Vector3(const Vector& other);

    /*!
     * \brief Constructor
     *
     * Constructs a tree-dimensional vector from
     * tree single values x, y, z or roll, pitch, yaw
     */
    Vector3(OUR_REAL x, OUR_REAL y, OUR_REAL z);

    /*!
     * \brief Assignment operator
     *
     * @param other a vector of dimension 3
     */
    Vector3 & operator = (const Vector& other);

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

    OUR_REAL dotProduct(const Vector3& other) const;
    OUR_REAL innerProd(const Vector3& other) const { return dotProduct(other); }
    
    OUR_REAL angleTo(const Vector3& other) const;

    Vector3& rotate_IP (double roll, double pitch, double yaw);

    double sqrDist(const Vector3& other) const;
    double dist(const Vector3& other) const;

    OUR_REAL& x();
    OUR_REAL& y();
    OUR_REAL& z();
    const OUR_REAL& x() const;
    const OUR_REAL& y() const;
    const OUR_REAL& z() const;

    OUR_REAL& roll();
    OUR_REAL& pitch();
    OUR_REAL& yaw();
    const OUR_REAL& roll() const;
    const OUR_REAL& pitch() const;
    const OUR_REAL& yaw() const;
  };

  //! user friendly output in format (x y z)
  std::ostream& operator<<(std::ostream& out, octomath::Vector3 const& v);

}


#endif
