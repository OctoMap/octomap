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

#ifndef _VRENDER_H_
#define _VRENDER_H_

#include "../config.h"
#if QT_VERSION >= 0x040000
# include <QTextStream>
# include <QString>
#else
# include <qtextstream.h>
# include <qstring.h>
#endif

#include "../qglviewer.h"

namespace vrender
{
	class VRenderParams ;
	typedef void (*RenderCB)(void *) ;
	typedef void (*ProgressFunction)(float,const QString&) ;

	void VectorialRender(RenderCB DrawFunc, void *callback_params, VRenderParams& render_params) ;

	class VRenderParams
	{
		public:
			VRenderParams() ;
			~VRenderParams() ;

			enum VRenderSortMethod { NoSorting, BSPSort, TopologicalSort, AdvancedTopologicalSort };
			enum VRenderFormat     { EPS, PS, XFIG, SVG };

			enum VRenderOption {	CullHiddenFaces         = 0x1,
						OptimizeBackFaceCulling = 0x4,
						RenderBlackAndWhite     = 0x8,
						AddBackground           = 0x10,
						TightenBoundingBox      = 0x20 } ;

			int sortMethod()    { return _sortMethod; }
			void setSortMethod(VRenderParams::VRenderSortMethod s) { _sortMethod = s ; }

			int format()        { return _format; }
			void setFormat(VRenderFormat f) { _format = f; }

			const QString filename() { return _filename ; }
			void setFilename(const QString& filename) ;

			void setOption(VRenderOption,bool) ;
			bool isEnabled(VRenderOption) ;

			void setProgressFunction(ProgressFunction pf) { _progress_function = pf ; }

		private:
			int _error;
			VRenderSortMethod _sortMethod;
			VRenderFormat     _format ;

			ProgressFunction _progress_function ;

			unsigned int _options; // _DrawMode; _ClearBG; _TightenBB;
			QString _filename;

			friend void VectorialRender(	RenderCB render_callback,
							void *callback_params,
							VRenderParams& vparams);
			friend class ParserGL ;
			friend class Exporter ;
			friend class BSPSortMethod ;
			friend class VisibilityOptimizer ;
			friend class TopologicalSortMethod ;
			friend class TopologicalSortUtils ;

			int& error() { return _error ; }
			int& size()  { static int size=1000000; return size ; }

			void progress(float,const QString&) ;
	};
}
#endif

