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

#ifndef _SORTMETHOD_H
#define _SORTMETHOD_H

#include <vector>
#include "Types.h"

namespace vrender
{
	// Class which implements the sorting of the primitives. An object of
	class VRenderParams ;
	class SortMethod
	{
		public:
			SortMethod() {}
			virtual ~SortMethod() {}

			virtual void sortPrimitives(std::vector<PtrPrimitive>&,VRenderParams&) = 0 ;

			void SetZDepth(FLOAT s) { zSize = s ; }
			FLOAT ZDepth() const { return zSize ; }

		protected:
			FLOAT zSize ;
	};

	class DontSortMethod: public SortMethod
	{
		public:
			DontSortMethod() {}
			virtual ~DontSortMethod() {}

			virtual void sortPrimitives(std::vector<PtrPrimitive>&,VRenderParams&) {}
	};

	class BSPSortMethod: public SortMethod
	{
		public:
			BSPSortMethod() {} ;
			virtual ~BSPSortMethod() {}

			virtual void sortPrimitives(std::vector<PtrPrimitive>&,VRenderParams&) ;
	};

	class TopologicalSortMethod: public SortMethod
	{
		public:
			TopologicalSortMethod() ;
			virtual ~TopologicalSortMethod() {}

			virtual void sortPrimitives(std::vector<PtrPrimitive>&,VRenderParams&) ;

			void setBreakCycles(bool b) { _break_cycles = b ; }
		private:
			bool _break_cycles ;
	};
}

#endif
