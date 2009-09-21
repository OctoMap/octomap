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

#ifndef _PRIMITIVE_H_
#define _PRIMITIVE_H_

#include <vector>
#include "AxisAlignedBox.h"
#include "Vector3.h"
#include "NVector3.h"
#include "Types.h"

#ifdef WIN32
# include <windows.h>
#endif

#ifdef __APPLE__
# include <OpenGL/gl.h>
#else
# include <GL/gl.h>
#endif

namespace vrender
{
	class Feedback3DColor ;
	class Primitive ;


#define EPS_SMOOTH_LINE_FACTOR 0.06  /* Lower for better smooth lines. */

	//  A Feedback3DColor is a structure containing informations about a vertex projected into
	// the frame buffer.

	class Feedback3DColor
	{
		public:
			Feedback3DColor(GLfloat *loc)
				: 	_pos(loc[0],loc[1],loc[2]),
			_red(loc[3]),_green(loc[4]),_blue(loc[5]),_alpha(loc[6]) {}

			inline FLOAT x() const { return _pos[0] ; }
			inline FLOAT y() const { return _pos[1] ; }
			inline FLOAT z() const { return _pos[2] ; }
			inline GLfloat red() const { return _red ; }
			inline GLfloat green() const { return _green ; }
			inline GLfloat blue() const { return _blue ; }
			inline GLfloat alpha() const { return _alpha ; }
			inline const Vector3& pos() const { return _pos ; }

			inline Feedback3DColor operator+(const Feedback3DColor & v) const
			{
				return Feedback3DColor(x()+v.x(),y()+v.y(),z()+v.z(),red()+v.red(),green()+v.green(),blue()+v.blue(),alpha()+v.alpha()) ;
			}
			inline Feedback3DColor operator*(const GLFLOAT & f) const
			{
				return Feedback3DColor(x()*f,y()*f,z()*f,red()*GLfloat(f),green()*GLfloat(f),blue()*GLfloat(f),alpha()*GLfloat(f)) ;
			}
			friend inline Feedback3DColor operator*(const GLFLOAT & f,const Feedback3DColor& F)
			{
				return F*f ;
			}

			static int sizeInBuffer() { return 7 ; }

			friend std::ostream& operator<<(std::ostream&,const Feedback3DColor&) ;

		protected:
			Feedback3DColor(FLOAT x, FLOAT y, FLOAT z, GLfloat r, GLfloat g, GLfloat b, GLfloat a)
				:_pos(x,y,z), _red(r), _green(g), _blue(b), _alpha(a) {}

			Vector3	_pos ;
			GLfloat	_red;
			GLfloat	_green;
			GLfloat	_blue;
			GLfloat	_alpha;
	} ;

	// A primitive is an entity
	//
	class Primitive
	{
		public:
			virtual ~Primitive() {}


			virtual const Feedback3DColor& sommet3DColor(int) const =0 ;

			// Renvoie le ieme vertex modulo le nombre de vertex.
			virtual const Vector3& vertex(int) const = 0 ;
#ifdef A_FAIRE
			virtual FLOAT Get_I_EPS(Primitive *) const ;
			Vect3 VerticalProjectPointOnSupportPlane(const Vector3 &) const ;
			void IntersectPrimitiveWithSupportPlane(Primitive *,int[],FLOAT[],Vect3 *&,Vect3 *&) ;
			inline FLOAT Equation(const Vect3& p) { return p*_normal-_C ; }
			virtual void Split(Vect3,FLOAT,Primitive * &,Primitive * &) = 0 ;
			void GetSigns(Primitive *,int * &,FLOAT * &,int &,int &,FLOAT) ;
			FLOAT Const() const { return _C ; }

			int depth() const { return _depth ; }
			void setDepth(int d) const { _depth = d ; }
#endif
			virtual AxisAlignedBox_xyz bbox() const = 0 ;
			virtual int nbVertices() const = 0 ;

		protected:

			int _vibility ;
	} ;

	class Point: public Primitive
	{
		public:
			Point(const Feedback3DColor& f);
			virtual ~Point() {}

			virtual const Vector3& vertex(int) const ;
			virtual int nbVertices() const { return 1 ; }
			virtual const Feedback3DColor& sommet3DColor(int) const ;
			virtual AxisAlignedBox_xyz bbox() const ;

		private:
			Feedback3DColor _position_and_color ;
	};

	class Segment: public Primitive
	{
		public:
			Segment(const Feedback3DColor & p1, const Feedback3DColor & p2): P1(p1), P2(p2) {}
			virtual ~Segment() {}
			virtual int nbVertices() const { return 2 ; }
			virtual const Vector3& vertex(int) const ;
			virtual const Feedback3DColor& sommet3DColor(int i) const ;
			virtual AxisAlignedBox_xyz bbox() const ;
#ifdef A_FAIRE
			virtual void Split(const Vector3&,FLOAT,Primitive * &,Primitive * &) ;
#endif

		protected:
			Feedback3DColor P1 ;
			Feedback3DColor P2 ;
	} ;


	class Polygone: public Primitive
	{
		public:
			Polygone(const std::vector<Feedback3DColor>&) ;
			virtual ~Polygone() {}
#ifdef A_FAIRE
			virtual int IsAPolygon() { return 1 ; }
			virtual void Split(const Vector3&,FLOAT,Primitive * &,Primitive * &) ;
			void InitEquation(double &,double &,double &,double &) ;
#endif
			virtual const Feedback3DColor& sommet3DColor(int) const ;
			virtual const Vector3& vertex(int) const ;
			virtual int nbVertices() const { return _vertices.size() ; }
			virtual AxisAlignedBox_xyz bbox() const ;
			double equation(const Vector3& p) const ;
			const NVector3& normal() const { return _normal ; }
			double c() const { return _c ; }

			FLOAT FlatFactor() const { return anglefactor ; }

		protected:
			virtual void initNormal() ;
			void CheckInfoForPositionOperators() ;

			AxisAlignedBox_xyz _bbox ;
			std::vector<Feedback3DColor> _vertices ;
			// std::vector<FLOAT> _sommetsProjetes ;
			// Vector3 N,M,L ;
			double anglefactor ;		//  Determine a quel point un polygone est plat.
			// Comparer a FLAT_POLYGON_EPS
			double _c ;
			NVector3 _normal ;
	} ;
}
#endif

