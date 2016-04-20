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
 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
*/

/****************************************************************************

 Copyright (C) 2002-2014 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.6.3.

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

#ifndef _VRENDER_AXISALIGNEDBOX_H
#define _VRENDER_AXISALIGNEDBOX_H

namespace vrender
{
  class Vector2;
  class Vector3;

	template<class T> class AxisAlignedBox
	{
		public:
			AxisAlignedBox() ;
			AxisAlignedBox(const T& v) ;
			AxisAlignedBox(const T& v,const T& w) ;

			const T& mini() const { return _min ; }
			const T& maxi() const { return _max ; }

			void include(const T& v) ;
			void include(const AxisAlignedBox<T>& b) ;
		private:
			T _min ;
			T _max ;
	};

	typedef AxisAlignedBox< Vector2 > AxisAlignedBox_xy ;
	typedef AxisAlignedBox< Vector3 > AxisAlignedBox_xyz ;

	template<class T> AxisAlignedBox<T>::AxisAlignedBox()
	: _min(T::inf), _max(-T::inf)
	{
	}

	template<class T> AxisAlignedBox<T>::AxisAlignedBox(const T& v)
		: _min(v), _max(v)
	{
	}

	template<class T> AxisAlignedBox<T>::AxisAlignedBox(const T& v,const T& w)
		: _min(v), _max(v)
	{
		include(w) ;
	}

	template<class T> void AxisAlignedBox<T>::include(const T& v)
	{
		_min = T::mini(_min,v) ;
		_max = T::maxi(_max,v) ;
	}

	template<class T> void AxisAlignedBox<T>::include(const AxisAlignedBox<T>& b)
	{
		include(b._min) ;
		include(b._max) ;
	}
}
#endif
