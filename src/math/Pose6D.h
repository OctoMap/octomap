#ifndef FERN_POSE6D_H
#define FERN_POSE6D_H

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
    Pose6D(double x, double y, double z, double roll, double pitch, double yaw);

    Pose6D& operator= (const Pose6D& other);
    bool operator==(const Pose6D& other) const;
    bool operator!=(const Pose6D& other) const;

    /*!
     * \brief Translational component
     *
     * @return the translational component of <this> pose
     */
    Vector3& trans();
    /*!
     * \brief Rotational component
     *
     * @return the rotational component of <this> pose
     */
    Quaternion& rot();
    /*!
     * \brief Translational component
     *
     * @return the translational component of <this> pose
     */
    const Vector3& trans() const;
    /*!
     * \brief Rotational component
     * @return the rotational component of <this> pose
     */
    const Quaternion& rot() const;

    double& x() { return translation(0); }
    double& y() { return translation(1); }
    double& z() { return translation(2); }
    const double& x() const { return translation(0); }
    const double& y() const { return translation(1); }
    const double& z() const { return translation(2); }

    double roll() const {return (rotation.toEuler())(0); }
    double pitch() const {return (rotation.toEuler())(1); }
    double yaw() const {return (rotation.toEuler())(2); }

    /*!
     * \brief Transformation a vector
     *
     * Transforms the vector <v> by the transformation which is
     * specified by <this>.
     * @return the vector which is translated by the translation of
     * <this> and afterwards rotated by the rotation of <this>.
     */
    Vector3 transform (const Vector3 &v) const;

    /*!
     * \brief Inversion
     *
     * Inverts the coordinate transformation represented by <this> pose
     * @return a copy of <this> pose inverted
     */
    Pose6D inv() const;
    Pose6D invert() const __attribute__ ((deprecated)) { return inv(); } // for inversion with copy use inv()

    /*!
     * \brief Inversion
     *
     * Inverts the coordinate transformation represented by <this> pose
     * @return a reference to <this> pose
     */
    Pose6D& inv_IP();
    Pose6D& invert_IP() __attribute__ ((deprecated)) { return inv_IP(); } // for inversion in place use inv_IP()

    /*!
     * \brief Concatenation
     *
     * Concatenates the coordinate transformations represented
     * by <this> and p.
     * @return <this> * p (applies first <this>, then p)
     */
    Pose6D operator* (const Pose6D &p) const;
    /*!
     * \brief In place concatenation
     *
     * Concatenates p to <this> Pose6D.
     * @return <this> which results from first moving by <this> and
     * afterwards by <p>
     */
    const Pose6D& operator*= (const Pose6D &p);

    /*!
     * \brief Translational distance
     *
     * @return the translational (euclidian) distance to <p>
     */
    double distance(const Pose6D &other) const;

    /*!
     * \brief Translational length
     *
     * @return the translational (euclidian) length of the translation
     * vector of <this> Pose6D
     */
    double transLength() const;
    double TransLength() const __attribute__ ((deprecated)); // use 'transLength' instead

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
