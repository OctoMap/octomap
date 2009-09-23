#include "Vector.h"

#include <cstdlib>
#include <cstring>
#include <math.h>
#include <cassert>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_sort_vector.h>
#include <gsl/gsl_permute_vector.h>

namespace octomath {


  Vector::Vector() 
    : m_vec(NULL) {
  }

  Vector::Vector(unsigned int size, initMode mode){
    if (mode == ZERO)
      m_vec = OUR_VECTOR_FUN(calloc)(size);
    else
      m_vec = OUR_VECTOR_FUN(alloc)(size);
  }


  Vector::Vector(const Vector& other){
    if(other.m_vec!=NULL){
      m_vec = OUR_VECTOR_FUN(alloc)(other.m_vec->size);
      OUR_VECTOR_FUN(memcpy)(m_vec, other.m_vec);
    }
    else
      m_vec = NULL;
  }
  
  
  Vector::Vector(const std::vector<OUR_REAL> &stdVec) {
    m_vec = OUR_VECTOR_FUN(alloc)(stdVec.size());
    for (unsigned int i = 0; i < stdVec.size(); ++i)
      OUR_VECTOR_FUN(set)(m_vec, i, stdVec[i]);
  }


  Vector::Vector(unsigned int size, const OUR_REAL *data) {
    m_vec = OUR_VECTOR_FUN(alloc)(size);
    for (unsigned int i = 0; i < size; ++i)
      OUR_VECTOR_FUN(set)(m_vec, i, data[i]);
  }

  
  Vector::~Vector(){
    if(m_vec!=NULL)
      OUR_VECTOR_FUN(free)(m_vec);
  }


  const OUR_REAL& Vector::operator() (unsigned int i) const{ 
    return *OUR_VECTOR_FUN(ptr)(m_vec, i);
  }


  OUR_REAL& Vector::operator() (unsigned int i){ 
    return *OUR_VECTOR_FUN(ptr)(m_vec, i);
  }


  Vector& Vector::operator= (OUR_REAL x){ 
    assert(m_vec != NULL);
    OUR_VECTOR_FUN(set_all)(m_vec, x);
    return *this;
  }


  Vector& Vector::operator= (const Vector &other) {
    if (m_vec != NULL) {
      if (other.m_vec != NULL) { 
	assert(m_vec->size == other.m_vec->size);
	OUR_VECTOR_FUN(memcpy)(m_vec, other.m_vec);
      }
      else {
	OUR_VECTOR_FUN(free)(m_vec);
	m_vec = NULL;
      }
    }
    else if (other.m_vec != NULL) { 
      m_vec = OUR_VECTOR_FUN(alloc)(other.m_vec->size);
      OUR_VECTOR_FUN(memcpy)(m_vec, other.m_vec);
    }
    // if both are NULL then there's nothing to do ...

    return *this;
  }


  Vector Vector::operator- () const{ 
    Vector result(*this);
    for (unsigned int i = 0; i < m_vec->size; ++i) {
      result(i) = -result(i);
    }
    return result;
  }


  Vector Vector::operator+ (OUR_REAL x) const{ 
    Vector result(*this);
    OUR_VECTOR_FUN(add_constant)(result.m_vec, x);
    return result;
  }


  Vector Vector::operator- (OUR_REAL x) const{ 
    Vector result(*this);
    OUR_VECTOR_FUN(add_constant)(result.m_vec, -x);
    return result;
  }


  Vector Vector::operator* (OUR_REAL x) const{ 
    Vector result(*this);
    OUR_VECTOR_FUN(scale)(result.m_vec, x);
    return result;
  }


  Vector Vector::operator/ (OUR_REAL x) const{ 
    Vector result(*this);
    OUR_VECTOR_FUN(scale)(result.m_vec, 1.0/x);
    return result;
  }


  void Vector::operator+= (OUR_REAL x){ 
    OUR_VECTOR_FUN(add_constant)(m_vec, x);
  }


  void Vector::operator-= (OUR_REAL x){ 
    OUR_VECTOR_FUN(add_constant)(m_vec, -x);
  }


  void Vector::operator*= (OUR_REAL x){ 
    OUR_VECTOR_FUN(scale)(m_vec, x);
  }


  void Vector::operator/= (OUR_REAL x){ 
    OUR_VECTOR_FUN(scale)(m_vec, 1.0/x);
  }


  Vector Vector::operator+ (const Vector &other) const{ 
    assert(m_vec->size == other.m_vec->size);
    Vector result(*this);
    OUR_VECTOR_FUN(add)(result.m_vec, other.m_vec);
    return result;
  }


  Vector Vector::operator- (const Vector &other) const{ 
    assert(m_vec->size == other.m_vec->size);
    Vector result(*this);
    OUR_VECTOR_FUN(sub)(result.m_vec, other.m_vec);
    return result;
  }


  OUR_REAL Vector::operator* (const Vector &other) const{ 
    assert(m_vec->size == other.m_vec->size);
    OUR_REAL result;
    OUR_BLAS_FUN(dot)(m_vec, other.m_vec, &result);
    return result;
  }
  


  void Vector::operator+= (const Vector &other){
    assert(m_vec->size == other.m_vec->size);
    OUR_VECTOR_FUN(add)(m_vec, other.m_vec);
  }


  void Vector::operator-= (const Vector &other){
    assert(m_vec->size == other.m_vec->size);
    OUR_VECTOR_FUN(sub)(m_vec, other.m_vec);
  }


  Vector& Vector::ewMul_IP (const Vector &other){ 
    assert(m_vec->size == other.m_vec->size);
    OUR_VECTOR_FUN(mul)(m_vec, other.m_vec);
    return *this;
  }

  Vector Vector::ewMul (const Vector &other) const{ 
    assert(m_vec->size == other.m_vec->size);
    Vector result(*this);
    result.ewMul_IP(other);
    return result;
  }


  Vector& Vector::ewDiv_IP (const Vector &other) {
    assert(m_vec->size == other.m_vec->size);
    OUR_VECTOR_FUN(div)(m_vec, other.m_vec);
    return *this;
  }  

  Vector Vector::ewDiv (const Vector &other) const{
    assert(m_vec->size == other.m_vec->size);
    Vector result(*this);
    result.ewDiv_IP(other);
    return result;
  }


  bool Vector::operator== (const Vector &other) const{ 
    for (unsigned int i = 0; i < m_vec->size; ++i) {
      if (OUR_VECTOR_FUN(get)(m_vec, i) != OUR_VECTOR_FUN(get)(other.m_vec, i))
	return false;
    }
    return true;
  }


  bool Vector::operator!= (const Vector &other) const
  { 
    return !(*this == other);
  }


  // TODO: - make this more efficient (memcpy or something alike)
  //       - check for NULL pointer
  Vector& Vector::resize_IP(unsigned int newSize)
  {
    if (newSize == size())
      return *this;

    unsigned int currentSize = size();
    OUR_VECTOR* currentVec = m_vec;
    m_vec = OUR_VECTOR_FUN(calloc)(newSize);

    unsigned int maxI = (newSize < currentSize) ? newSize : currentSize;

    for (unsigned int i = 0; i < maxI; ++i) {
      (*this)(i) = gsl_vector_get(currentVec, i);
    }

    OUR_VECTOR_FUN(free)(currentVec);
    return *this;
  }


  Vector Vector::resize(unsigned int newSize) const
  {
    return Vector(*this).resize_IP(newSize);
  }

  // this is the same as the 'operator*' 
  // maybe we should remove the operator and only
  // user 'outerProd' and 'innerProd' as it is not
  // clear what the prefered way to multiply vectors
  // would be
  OUR_REAL Vector::innerProd(const Vector &other) const {
    assert(m_vec->size == other.m_vec->size);
    OUR_REAL result;
    OUR_BLAS_FUN(dot)(m_vec, other.m_vec, &result);
    return result;
  }


  // this is the same as the 'operator*' 
  // maybe we should remove the operator and only
  // user 'outerProd' and 'innerProd' as it is not
  // clear what the prefered way to multiply vectors
  // would be
  OUR_REAL Vector::innerProd() const {
    OUR_REAL result;
    OUR_BLAS_FUN(dot)(m_vec, m_vec, &result);
    return result;
  }



  OUR_REAL Vector::sum() const {
    OUR_REAL sum=0;
    for(unsigned int i=0;i<size();i++) {
      sum += (*this)(i);
    }
    return sum;
  }

  OUR_REAL Vector::norm1() const {
    return OUR_BLAS_FUN(asum)(m_vec);
  }

  OUR_REAL Vector::norm2() const {

    return OUR_BLAS_FUN(nrm2)(m_vec);

  }

  OUR_REAL Vector::min() const
  { 
    return OUR_VECTOR_FUN(min)(m_vec);
  }


  OUR_REAL Vector::max() const
  { 
    return OUR_VECTOR_FUN(max)(m_vec);
  }


  Vector Vector::abs() const {
    Vector result(size());
    for (uint i=0; i<size(); i++)
      result(i) = fabs((*this)(i));
    return result;
  }

  void Vector::write(unsigned char * dest){
        
    memcpy(dest,m_vec->data ,m_vec->size*sizeof(OUR_REAL));
    
  }


  void Vector::read(unsigned char * src, uint size){
    if(m_vec!=NULL){
      OUR_VECTOR_FUN(free)(m_vec);
    }

    m_vec = OUR_VECTOR_FUN(alloc)(size);
    
    memcpy(m_vec->data, src, m_vec->size*sizeof(OUR_REAL));
    
  }


  std::vector<OUR_REAL> Vector::toStdVector() const {
    std::vector<OUR_REAL> stdVec(m_vec->size);
    for (unsigned int i = 0; i < stdVec.size(); ++i)
      stdVec[i] = (*this)(i);
    return stdVec;
  }


  Vector Vector::sort() {
    Vector result = *this;
    OUR_SORT_VECTOR(result.m_vec);
    return(result);
  }
  

  Vector Vector::sort(Vector &index) {
    gsl_permutation* p = gsl_permutation_alloc (size());
    gsl_permutation_init(p);
    
    OUR_SORT_VECTOR_INDEX(p,m_vec);
    
    Vector result = *this;
    OUR_PERMUTE_VECTOR(p,result.m_vec);
    index = Vector(size());
    for(unsigned int i=0;i<size();i++) {
      index(i) = gsl_permutation_get(p,i);
    }
    
    gsl_permutation_free(p);
    return(result);
  }


  std::istream& Vector::read(std::istream &s) {
    s >> m_vec->size;
    resize_IP(m_vec->size);
    for (unsigned int i=0; i<m_vec->size; i++)
      s >> operator()(i);
    return s;
  }


  std::ostream& Vector::write(std::ostream &s) const {
    s << m_vec->size;
    for (unsigned int i=0; i<m_vec->size; i++)
      s << " " << operator()(i);
    return s;
  }


  std::istream& Vector::readBinary(std::istream &s) {
    s.read((char*)&m_vec->size, sizeof(m_vec->size));
    resize_IP(m_vec->size);
    OUR_REAL val = 0;
    for (unsigned int i=0; i<m_vec->size; i++) {
      s.read((char*)&val, sizeof(val));
      operator()(i) = val;
    }
    return s;
  }


  std::ostream& Vector::writeBinary(std::ostream &s) const {
    s.write((char*)&m_vec->size, sizeof(m_vec->size));
    OUR_REAL val = 0;
    for (unsigned int i=0; i<m_vec->size; i++) {
      val = operator()(i);
      s.write((char*)&val, sizeof(val));
    }
    return s;
  }

  
  Vector& Vector::unit_IP (){ 
    OUR_REAL len = norm2();
    if (len > 0)
      *this /= len;
    return *this;
  }
  
  Vector Vector::unit () const {
    Vector result(*this);
    result.unit_IP();
    return result;
  }

  OUR_REAL Vector::angleTo(const Vector& other) const {

    OUR_REAL dot_prod = this->innerProd(other);
    OUR_REAL len1 = this->norm2();
    OUR_REAL len2 = other.norm2();

    return acos(dot_prod / (len1*len2));
  }


  Vector operator+ (OUR_REAL x, const Vector &vec)
  {
    return vec + x;
  }


  Vector operator- (OUR_REAL x, const Vector &vec)
  {
    return -(vec - x);
  }


  Vector operator* (OUR_REAL  x, const Vector &vec)
  {
    return vec * x;
  }

  std::ostream& operator<< (std::ostream &s, const Vector &v)
  {
    for (unsigned int i = 0; i < v.size(); ++i) {
      s << v(i);
      if (i+1 < v.size())
	s << " ";
    }
    return s;
  }

  std::istream& operator>> (std::istream &s, Vector &v)
  {
    for (unsigned int i = 0; i < v.size(); ++i) {
      s >> v(i);
    }
    return s;
  }

} // end namespace 'octomath'
