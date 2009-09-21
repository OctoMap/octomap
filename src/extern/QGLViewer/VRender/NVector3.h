/*
 This file is part of the VRender library.
 Copyright (C) 2005 Cyril Soler (Cyril.Soler@imag.fr)
 Version 1.0.0, released on June 27, 2005.

 http://artis.imag.fr/Members/Cyril.Soler/VRender

 VRender is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 VRender is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with VRender; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/****************************************************************************

 Copyright (C) 2002-2008 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.3.1.

 http://www.libqglviewer.com - contact@libqglviewer.com

 This file may be used under the terms of the GNU General Public License 
 versions 2.0 or 3.0 as published by the Free Software Foundation and
 appearing in the LICENSE file included in the packaging of this file.
 In addition, as a special exception, Gilles Debunne gives you certain 
 additional rights, described in the file GPL_EXCEPTION in this package.

 libQGLViewer uses dual licensing. Commercial/proprietary software must
 purchase a libQGLViewer Commercial License.

 This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#ifndef _VRENDER_NVECTOR3_H
#define _VRENDER_NVECTOR3_H

#include <iostream>
#include <stdexcept>

namespace vrender
{
  class Vector3;

  class NVector3
  {
  public:
    NVector3();
    NVector3(const NVector3& u);
    inline NVector3(double x,double y,double z,bool normalization=true)
    {
      setXYZ(x,y,z,normalization);
    }

    NVector3(const Vector3 &u,bool normalization=true);

    inline double x() const {return _n[0];}
    inline double y() const {return _n[1];}
    inline double z() const {return _n[2];}
    void setXYZ(double x,double y,double z,bool normalization=true);

    NVector3& operator=(const NVector3& u);
    /*
      inline friend bool operator==(const NVector3 &u,const Vector3  &v) {return u.isEqualTo(v);}
      inline friend bool operator==(const Vector3  &u,const NVector3 &v) {return v.isEqualTo(u);}
      inline friend bool operator==(const NVector3 &u,const NVector3 &v) {return u.isEqualTo(v);}
      inline friend bool operator!=(const NVector3 &u,const Vector3  &v) {return !(u == v);}
      inline friend bool operator!=(const Vector3  &u,const NVector3 &v) {return !(u == v);}
      inline friend bool operator!=(const NVector3 &u,const NVector3 &v) {return !(u == v);}
    */

    inline friend NVector3 operator-(const NVector3 &u) { return NVector3(-u[0],-u[1],-u[2],false); }
    //inline friend Vector3 operator+(const NVector3 &u,const Vector3  &v);
    //inline friend Vector3 operator+(const Vector3  &u,const NVector3 &v);
    //inline friend Vector3 operator+(const NVector3 &u,const NVector3 &v);
    //inline friend Vector3 operator-(const NVector3 &u,const Vector3  &v);
    //inline friend Vector3 operator-(const Vector3  &u,const NVector3 &v);
    //inline friend Vector3 operator-(const NVector3 &u,const NVector3 &v);
    friend double operator*(const NVector3 &u,const Vector3  &v);
    friend double operator*(const Vector3  &u,const NVector3 &v);
    //inline friend double operator*(const NVector3 &u,const NVector3 &v);
    //inline friend Vector3 operator*(double r,const NVector3 &u);
    //inline friend Vector3 operator/(const NVector3 &u,double r);

    //inline friend Vector3 operator^(const NVector3 &u,const Vector3  &v);
    //inline friend Vector3 operator^(const Vector3  &u,const NVector3 &v);
    //inline friend Vector3 operator^(const NVector3 &u,const NVector3 &v);

    inline double norm() const {return 1.0;}
    inline double squareNorm() const {return 1.0;}
    friend std::ostream& operator<<(std::ostream &out,const NVector3 &u);

    double operator[](int i) const
    {
      if((i < 0)||(i > 2))
	throw std::runtime_error("Out of bounds in NVector3::operator[]") ;

      return _n[i];
    }

  private:
    void normalize();

    double _n[3];  //!< normalized vector

  }; // interface of NVector3

}

#endif // _NVECTOR3_H
