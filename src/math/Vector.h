#ifndef FERN_VECTOR_H
#define FERN_VECTOR_H

#include <vector>
#include <iostream>

#include <gsl/gsl_vector.h>

#include "OurTypes.h"

namespace octomath
{
  
 
  class Matrix;
  class VectorView;
  
  /**
   * \brief This class represents a vector.
   */
  class Vector{
  public:

    enum initMode {UINIT, ZERO};
    
    Vector();
    Vector(unsigned int size, initMode mode = ZERO);
    Vector(const std::vector<double> &stdVec);
    Vector(unsigned int size, const double *data);
    Vector(const Vector &other);
    ~Vector();

    bool isInit() const { return m_vec != NULL; };
    
    virtual Vector& operator= (double x);
    virtual Vector& operator= (const Vector &other);
    
    const double& operator() (unsigned int i) const;
    double& operator() (unsigned int i);
    
    Vector operator- () const;

    Vector operator+ (double x) const;
    Vector operator- (double x) const;
    Vector operator* (double x) const;
    Vector operator/ (double x) const;
    
    void operator+= (double x);
    void operator-= (double x);
    void operator*= (double x);
    void operator/= (double x);

    Vector operator+ (const Vector &other) const;
    Vector operator- (const Vector &other) const;
    double operator* (const Vector &other) const;

    void operator+= (const Vector &other);
    void operator-= (const Vector &other);

    bool operator== (const Vector &other) const;
    bool operator!= (const Vector &other) const;

    unsigned int size() const { return m_vec->size; };

    Vector& resize_IP(unsigned int newSize);
    Vector resize(unsigned int newSize) const;

    //! elementwise multiplication in place
    Vector& ewMul_IP (const Vector &other);
    //! elementwise multiplication
    Vector ewMul (const Vector &other) const;
    //! elementwise division in place
    Vector& ewDiv_IP (const Vector &other);
    //! elementwise division
    Vector ewDiv (const Vector &other) const;

    double sum() const;
    double norm1() const;
    double norm2() const;
    
    Vector  unit () const;
    Vector& unit_IP ();

    double angleTo(const Vector& other) const;

    double min() const;
    double max() const;
    VectorView subvector(uint b, uint e) const;

    Vector abs() const;

    void write(unsigned char * dest);
    void read(unsigned char * src, uint size);

    //! dot product, scalar product
    double innerProd(const Vector &other) const;
    //! dot product, scalar product with itself
    double innerProd() const;
    //! outer product
    Matrix outerProd(const Vector &other) const;
    //! outer product with itself
    Matrix outerProd() const;

    std::vector<double> toStdVector() const;
    
    Matrix toMatrixV();
    Matrix toMatrixH();
    Vector sort();
    Vector sort(Vector &index);

    friend class Matrix;
    friend class VectorView;

    std::istream& read(std::istream &s);
    std::ostream& write(std::ostream &s) const;
    std::istream& readBinary(std::istream &s);
    std::ostream& writeBinary(std::ostream &s) const;
    
  protected:
    OUR_VECTOR *m_vec;
  };
  
  class VectorView:public Vector{
  public:
    VectorView(const Vector& other);
    VectorView(const OUR_VECTOR_FUN(view)& gvv);
    //~VectorView();
    friend class Matrix;
  };

  //! user friendly output to stream. This output is NOT for parsing!
  std::ostream& operator<< (std::ostream &s, const Vector &v);
  std::istream& operator>> (std::istream &s, Vector &v) __attribute__ ((deprecated)); // for parsing use read() or readBinary() - they can read streams written by write() and writeBinary()

  Vector operator+ (double x, const Vector &vec);
  Vector operator- (double x, const Vector &vec);
  Vector operator* (double x, const Vector &vec);

}

#endif
