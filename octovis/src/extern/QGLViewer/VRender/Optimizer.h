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

 Copyright (C) 2002-2011 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.3.17.

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

#ifndef _OPTIMIZER_H
#define _OPTIMIZER_H

#include "Types.h"

namespace vrender
{
	// Implements some global optimizations on the polygon sorting.

	class VRenderParams ;
	class Optimizer
	{
		public:
			virtual void optimize(std::vector<PtrPrimitive>&,VRenderParams&) = 0 ;
			virtual ~Optimizer() {} ;
	};

	//  Optimizes visibility by culling primitives which do not appear in the
	// rendered image. Computations are done analytically rather than using an item
	// buffer.

	class VisibilityOptimizer: public Optimizer
	{
		public:
			virtual void optimize(std::vector<PtrPrimitive>&,VRenderParams&) ;
			virtual ~VisibilityOptimizer() {} ;
	};

	//  Optimizes by collapsing together primitives which can be, without
	// perturbating the back to front painting algorithm.

	class PrimitiveSplitOptimizer: public Optimizer
	{
		public:
			virtual void optimize(std::vector<PtrPrimitive>&,VRenderParams&) {}
			virtual ~PrimitiveSplitOptimizer() {} ;
	};

	class BackFaceCullingOptimizer: public Optimizer
	{
		public:
			virtual void optimize(std::vector<PtrPrimitive>&,VRenderParams&) ;
			virtual ~BackFaceCullingOptimizer() {} ;
	};
}

#endif
