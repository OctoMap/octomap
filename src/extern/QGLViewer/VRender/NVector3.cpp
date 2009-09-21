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

#include "NVector3.h"
#include "Vector3.h"

using namespace vrender;

NVector3::NVector3(const Vector3 &u,bool normalization)
{
  setXYZ(u[0],u[1],u[2],normalization);
}
/*
Vector3 operator+(const NVector3 &u,const Vector3  &v)
{
  return Vector3(u[0]+v[0],u[1]+v[1],u[2]+v[2]);
}

Vector3 operator+(const Vector3  &u,const NVector3 &v)
{
  return Vector3(u[0]+v[0],u[1]+v[1],u[2]+v[2]);
}

Vector3 operator+(const NVector3 &u,const NVector3 &v)
{
  return Vector3(u[0]+v[0],u[1]+v[1],u[2]+v[2]);
}

Vector3 operator-(const NVector3 &u,const Vector3  &v)
{
  return Vector3(u[0]-v[0],u[1]-v[1],u[2]-v[2]);
}

Vector3 operator-(const Vector3  &u,const NVector3 &v)
{
  return Vector3(u[0]-v[0],u[1]-v[1],u[2]-v[2]);
}

Vector3 operator-(const NVector3 &u,const NVector3 &v)
{
  return Vector3(u[0]-v[0],u[1]-v[1],u[2]-v[2]);
}
*/
double vrender::operator*(const NVector3 &u,const Vector3  &v)
{
  return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

double vrender::operator*(const Vector3  &u,const NVector3 &v)
{
  return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}
/*
double operator*(const NVector3 &u,const NVector3 &v)
{
  return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

Vector3 operator*(double r,const NVector3 &u)
{
  return Vector3(r*u[0],r*u[1],r*u[2]);
}

Vector3 operator/(const NVector3 &u,double r)
{
  return Vector3(u[0]/r,u[1]/r,u[2]/r);
}


Vector3 operator^(const NVector3 &u,const Vector3  &v)
{
  return Vector3(	u[1]*v[2]-u[2]*v[1],
			u[2]*v[0]-u[0]*v[2],
			u[0]*v[1]-u[1]*v[0]);
}

Vector3 operator^(const Vector3  &u,const NVector3 &v)
{
  return Vector3(	u[1]*v[2]-u[2]*v[1],
			u[2]*v[0]-u[0]*v[2],
			u[0]*v[1]-u[1]*v[0]);
}

Vector3 operator^(const NVector3 &u,const NVector3 &v)
{
  return Vector3(	u[1]*v[2]-u[2]*v[1],
			u[2]*v[0]-u[0]*v[2],
			u[0]*v[1]-u[1]*v[0]);
}
*/
// -----------------------------------------------------------------------------
//! Default constructor (the default normalized vector is (1,0,0))
NVector3::NVector3()
{
  _n[0] = 1.0;
  _n[1] = 0.0;
  _n[2] = 0.0;
}

// -----------------------------------------------------------------------------
//! Copy constructor
NVector3::NVector3(const NVector3& u)
{
  _n[0] = u._n[0] ;
  _n[1] = u._n[1] ;
  _n[2] = u._n[2] ;
}
// -----------------------------------------------------------------------------
//! Writing X,Y and Z coordinates
void NVector3::setXYZ(double x,double y,double z,bool normalization)
{
  _n[0] = x;
  _n[1] = y;
  _n[2] = z;
  if ( normalization ) normalize();
}

// -----------------------------------------------------------------------------
//! Assignment
NVector3& NVector3::operator=(const NVector3& u)
{
  if ( &u != this )
    {
	_n[0] = u[0];
	_n[1] = u[1];
	_n[2] = u[2];
    }
  return *this;
}
// -----------------------------------------------------------------------------
//! Out stream override: prints the 3 normalized vector components
std::ostream& operator<<(std::ostream& out,const NVector3& u)
{
  out << u[0] << " " << u[1] << " " << u[2];
  return out;
}

// -----------------------------------------------------------------------------
//! Normalization
//! Private method to do normalization (using Norm() method of the Vector class)
//! when it is necessary (construction of a normalized vector for exemple).
void NVector3::normalize()
{
  double n = _n[0]*_n[0]+_n[1]*_n[1]+_n[2]*_n[2] ;

  if ( n > 0.0 )
    {
	_n[0] /= n;
	_n[1] /= n;
	_n[2] /= n;
    }
  else
    throw std::runtime_error("Attempt to normalize a null 3D vector.") ;
}

