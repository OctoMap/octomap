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


// The code of these methods is largely inspired from the Qt QKeySequence
// implementation, distributed under the GPL licence.
// Original Copyright to Trolltech AS.

#include "keySequence.h"

#include <qaccel.h>

static struct {
    int key;
    const char* name;
} keyname[] = {
    { Qt::Key_Space,	    QT_TRANSLATE_NOOP( "QAccel", "Space" ) },
    { Qt::Key_Tab,	        QT_TRANSLATE_NOOP( "QAccel", "Tab" ) },
    { Qt::Key_Backtab,	    QT_TRANSLATE_NOOP( "QAccel", "Backtab" ) },
    { Qt::Key_Backspace,    QT_TRANSLATE_NOOP( "QAccel", "Backspace" ) },
    { Qt::Key_Return,	    QT_TRANSLATE_NOOP( "QAccel", "Return" ) },
    { Qt::Key_Enter,	    QT_TRANSLATE_NOOP( "QAccel", "Enter" ) },
    { Qt::Key_Pause,	    QT_TRANSLATE_NOOP( "QAccel", "Pause" ) },
    { Qt::Key_SysReq,	    QT_TRANSLATE_NOOP( "QAccel", "SysReq" ) },
    { Qt::Key_Home,	        QT_TRANSLATE_NOOP( "QAccel", "Home" ) },
    { Qt::Key_End,	        QT_TRANSLATE_NOOP( "QAccel", "End" ) },
    { Qt::Key_Left,	        QT_TRANSLATE_NOOP( "QAccel", "Left" ) },
    { Qt::Key_Up,	        QT_TRANSLATE_NOOP( "QAccel", "Up" ) },
    { Qt::Key_Right,	    QT_TRANSLATE_NOOP( "QAccel", "Right" ) },
    { Qt::Key_Down,	        QT_TRANSLATE_NOOP( "QAccel", "Down" ) },
    { Qt::Key_Menu,	        QT_TRANSLATE_NOOP( "QAccel", "Menu" ) },
    { Qt::Key_Help,	        QT_TRANSLATE_NOOP( "QAccel", "Help" ) },
    { Qt::Key_Print,	    QT_TRANSLATE_NOOP( "QAccel", "Print Screen" ) },
    { Qt::Key_Prior,	    QT_TRANSLATE_NOOP( "QAccel", "Page Up" ) },
    { Qt::Key_Next,	        QT_TRANSLATE_NOOP( "QAccel", "Page Down" ) },
    { Qt::Key_CapsLock,	    QT_TRANSLATE_NOOP( "QAccel", "Caps Lock" ) },
    { Qt::Key_NumLock,	    QT_TRANSLATE_NOOP( "QAccel", "Num Lock" ) },
    { Qt::Key_ScrollLock,   QT_TRANSLATE_NOOP( "QAccel", "Scroll Lock" ) },
    { Qt::Key_Insert,	    QT_TRANSLATE_NOOP( "QAccel", "Insert" ) },
    { Qt::Key_Delete,	    QT_TRANSLATE_NOOP( "QAccel", "Delete" ) },
    { Qt::Key_Escape,	    QT_TRANSLATE_NOOP( "QAccel", "Escape" ) },
    { 0, 0 }
};


QKeySequence::QKeySequence(int key)
{
    key_ = key;
}


/*!
    Creates a shortcut string for the key sequence.
    For instance CTRL+Key_O gives "Ctrl+O". If the key sequence has
    multiple key codes they are returned comma-separated, e.g.
    "Alt+X, Ctrl+Y, Z". The strings, "Ctrl", "Shift", etc. are
    translated (using QObject::tr()) in the "QAccel" scope. If the key
    sequence has no keys, QString::null is returned.
*/
QKeySequence::operator QString() const
{
    QString s;

    // On other systems the order is Meta, Control, Alt, Shift
    if ( (key_ & Qt::CTRL) == Qt::CTRL ) {
	if ( !s.isEmpty() )
	    s += QAccel::tr( "+" );
	s += QAccel::tr( "Ctrl" );
    }
    if ( (key_ & Qt::ALT) == Qt::ALT ) {
	if ( !s.isEmpty() )
	    s += QAccel::tr( "+" );
	s += QAccel::tr( "Alt" );
    }
    if ( (key_ & Qt::SHIFT) == Qt::SHIFT ) {
	if ( !s.isEmpty() )
	    s += QAccel::tr( "+" );
	s += QAccel::tr( "Shift" );
    }


    int key = key_ & ~(Qt::SHIFT | Qt::CTRL | Qt::ALT );
    QString p;

    if ( (key & Qt::UNICODE_ACCEL) == Qt::UNICODE_ACCEL ) {
	// Note: This character should NOT be upper()'ed, since
	// the encoded string should indicate EXACTLY what the
	// key represents! Hence a 'Ctrl+Shift+c' is posible to
	// represent, but is clearly impossible to trigger...
	p = QChar(key & 0xffff);
    } else if ( key >= Qt::Key_F1 && key <= Qt::Key_F35 ) {
	p = QAccel::tr( "F%1" ).arg(key - Qt::Key_F1 + 1);
    } else if ( key > Qt::Key_Space && key <= Qt::Key_AsciiTilde ) {
	p.sprintf( "%c", key );
    } else {
	int i=0;
	while (keyname[i].name) {
	    if ( key == keyname[i].key ) {
		p = QAccel::tr(keyname[i].name);
		break;
	    }
	    ++i;
	}
	// If we can't find the actual translatable keyname,
	// fall back on the unicode representation of it...
	// Or else characters like Key_aring may not get displayed
	// ( Really depends on you locale )
	if ( key && !keyname[i].name )
	    // Note: This character should NOT be upper()'ed, see above!
	    p = QChar(key & 0xffff);
    }

    if ( !s.isEmpty() )
      s += QAccel::tr( "+" );

    s += p;
    return s;
}
