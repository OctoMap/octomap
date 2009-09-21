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

#include "VRender.h"
#include "Exporter.h"
#include "../qglviewer.h"

#if QT_VERSION >= 0x040000
# include <QFile>
# include <QMessageBox>
#else
# include <qfile.h>
# include <qmessagebox.h>
#endif

using namespace vrender ;
using namespace std ;

Exporter::Exporter()
{
	_xmin=_xmax=_ymin=_ymax=_zmin=_zmax = 0.0 ;
	_pointSize=1 ;
}

void Exporter::exportToFile(const QString& filename,
							const vector<PtrPrimitive>& primitive_tab,
							VRenderParams& vparams)
{
	QFile file(filename);

	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QMessageBox::warning(NULL, QGLViewer::tr("Exporter error", "Message box window title"), QGLViewer::tr("Unable to open file %1.").arg(filename));
		return;
	}

	QTextStream out(&file);

	writeHeader(out) ;

	int N = primitive_tab.size()/200 + 1 ;

	for(unsigned int i=0;i<primitive_tab.size();++i)
	{
		Point *p = dynamic_cast<Point *>(primitive_tab[i]) ;
		Segment *s = dynamic_cast<Segment *>(primitive_tab[i]) ;
		Polygone *P = dynamic_cast<Polygone *>(primitive_tab[i]) ;

		if(p != NULL) spewPoint(p,out) ;
		if(s != NULL) spewSegment(s,out) ;
		if(P != NULL) spewPolygone(P,out) ;

		if(i%N == 0)
			vparams.progress(i/(float)primitive_tab.size(),QGLViewer::tr("Exporting to file %1").arg(filename)) ;
	}

	writeFooter(out) ;

	file.close();
}

void Exporter::setBoundingBox(float xmin,float ymin,float xmax,float ymax)
{
	_xmin = xmin ;
	_ymin = ymin ;
	_xmax = xmax ;
	_ymax = ymax ;
}

void Exporter::setClearColor(float r, float g, float b) { _clearR=r; _clearG=g; _clearB=b; }
void Exporter::setClearBackground(bool b) { _clearBG=b; }
void Exporter::setBlackAndWhite(bool b) { _blackAndWhite = b; }

