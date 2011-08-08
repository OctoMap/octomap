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

#include "mouseGrabber.h"

using namespace qglviewer;

// Static private variable
#if QT_VERSION >= 0x040000
QList<MouseGrabber*> MouseGrabber::MouseGrabberPool_;
#else
QPtrList<MouseGrabber> MouseGrabber::MouseGrabberPool_;
#endif

/*! Default constructor.

Adds the created MouseGrabber in the MouseGrabberPool(). grabsMouse() is set to \c false. */
MouseGrabber::MouseGrabber()
  : grabsMouse_(false)
{
  addInMouseGrabberPool();
}

/*! Adds the MouseGrabber in the MouseGrabberPool().

All created MouseGrabber are automatically added in the MouseGrabberPool() by the constructor.
Trying to add a MouseGrabber that already isInMouseGrabberPool() has no effect.

Use removeFromMouseGrabberPool() to remove the MouseGrabber from the list, so that it is no longer
tested with checkIfGrabsMouse() by the QGLViewer, and hence can no longer grab mouse focus. Use
isInMouseGrabberPool() to know the current state of the MouseGrabber. */
void MouseGrabber::addInMouseGrabberPool()
{
  if (!isInMouseGrabberPool())
    MouseGrabber::MouseGrabberPool_.append(this);
}

/*! Removes the MouseGrabber from the MouseGrabberPool().

See addInMouseGrabberPool() for details. Removing a MouseGrabber that is not in MouseGrabberPool()
has no effect. */
void MouseGrabber::removeFromMouseGrabberPool()
{
  if (isInMouseGrabberPool())
#if QT_VERSION >= 0x040000
    MouseGrabber::MouseGrabberPool_.removeAll(const_cast<MouseGrabber*>(this));
#else
    MouseGrabber::MouseGrabberPool_.removeRef(this);
#endif
}

/*! Clears the MouseGrabberPool().

 Use this method only if it is faster to clear the MouseGrabberPool() and then to add back a few
 MouseGrabbers than to remove each one independently. Use QGLViewer::setMouseTracking(false) instead
 if you want to disable mouse grabbing.

 When \p autoDelete is \c true, the MouseGrabbers of the MouseGrabberPool() are actually deleted
 (use this only if you're sure of what you do). */
void MouseGrabber::clearMouseGrabberPool(bool autoDelete)
{
#if QT_VERSION >= 0x040000
  if (autoDelete)
    qDeleteAll(MouseGrabber::MouseGrabberPool_);
#else
  MouseGrabber::MouseGrabberPool_.setAutoDelete(autoDelete);
#endif
  MouseGrabber::MouseGrabberPool_.clear();
}
