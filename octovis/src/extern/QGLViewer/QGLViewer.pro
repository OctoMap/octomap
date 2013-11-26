#		    l i b Q G L V i e w e r
#	C o m p i l a t i o n    c o n f i g u r a t i o n

# Run "qmake; make; make install" to compile and install the library on Unix systems.
# Optional arguments can tune install paths (as in "qmake PREFIX=$HOME"). See doc/download.html for details.

# If your Qt version is lower than 3.1 (look at $QTDIR/lib), you need to link with GLUT.
# Uncomment the following line:
# USE_GLUT = yes

TEMPLATE = lib
TARGET = QGLViewer
VERSION = 2.4.0
CONFIG *= qt opengl warn_on shared thread create_prl rtti no_keywords

QGL_HEADERS = qglviewer.h \
	  camera.h \
	  manipulatedFrame.h \
	  manipulatedCameraFrame.h \
	  frame.h \
	  constraint.h \
	  keyFrameInterpolator.h \
	  mouseGrabber.h \
	  quaternion.h \
	  vec.h \
	  domUtils.h \
	  config.h

SOURCES = qglviewer.cpp \
	  camera.cpp \
	  manipulatedFrame.cpp \
	  manipulatedCameraFrame.cpp \
	  frame.cpp \
	  saveSnapshot.cpp \
	  constraint.cpp \
	  keyFrameInterpolator.cpp \
	  mouseGrabber.cpp \
	  quaternion.cpp \
	  vec.cpp

HEADERS *= $${QGL_HEADERS}
DISTFILES *= qglviewer-icon.xpm

TRANSLATIONS = qglviewer_fr.ts
                       
QT_VERSION=$$[QT_VERSION]

!contains( QT_VERSION, "^3.*" ) {
    QT *= xml opengl
}

contains ( QT_VERSION, "^5.*" ) {
    QT *= widgets
}

!isEmpty( QGLVIEWER_STATIC ) {
  CONFIG *= staticlib
}

# -----------------------------------
# --  I m a g e I n t e r f a c e  --
# -----------------------------------
contains( QT_VERSION, "^3.*" ) {
  FORMS *= ImageInterface.Qt3.ui
} else {
  FORMS *= ImageInterface.ui
}

# ---------------------------------------------
# --  V e c t o r i a l   R e n d e r i n g  --
# ---------------------------------------------
# In case of compilation troubles with vectorial rendering, uncomment this line
# DEFINES *= NO_VECTORIAL_RENDER

contains( DEFINES, NO_VECTORIAL_RENDER ) {
  message( Vectorial rendering disabled )
} else {
  contains( QT_VERSION, "^3.*" ) {
    FORMS *= VRenderInterface.Qt3.ui
  } else {
    FORMS *= VRenderInterface.ui
  }

  SOURCES *= \
	VRender/BackFaceCullingOptimizer.cpp \
	VRender/BSPSortMethod.cpp \
	VRender/EPSExporter.cpp \
	VRender/Exporter.cpp \
	VRender/FIGExporter.cpp \
	VRender/gpc.cpp \
	VRender/ParserGL.cpp \
	VRender/Primitive.cpp \
	VRender/PrimitivePositioning.cpp \
	VRender/TopologicalSortMethod.cpp \
	VRender/VisibilityOptimizer.cpp \
	VRender/Vector2.cpp \
	VRender/Vector3.cpp \
	VRender/NVector3.cpp \
	VRender/VRender.cpp

  VRENDER_HEADERS = \
	VRender/AxisAlignedBox.h \
	VRender/Exporter.h \
	VRender/gpc.h \
	VRender/NVector3.h \
	VRender/Optimizer.h \
	VRender/ParserGL.h \
	VRender/Primitive.h \
	VRender/PrimitivePositioning.h \
	VRender/SortMethod.h \
	VRender/Types.h \
	VRender/Vector2.h \
	VRender/Vector3.h \
	VRender/VRender.h

  HEADERS *= $${VRENDER_HEADERS}
}




# ---------------
# --  U n i x  --
# ---------------
unix {
  CONFIG -= debug debug_and_release
  CONFIG *= release

  # INCLUDE_DIR and LIB_DIR specify where to install the include files and the library.
  # Use qmake INCLUDE_DIR=... LIB_DIR=... , or qmake PREFIX=... to customize your installation.
  
  HOME_DIR = $$system(cd;pwd)

  isEmpty( PREFIX ) {
    PREFIX_=/usr
  } else {
    PREFIX_=$${PREFIX}
  }
  isEmpty( LIB_DIR ) {
    LIB_DIR_ = $${PREFIX_}/lib
  } else {
    LIB_DIR_ = $${LIB_DIR}
  }
  isEmpty( INCLUDE_DIR ) {
    INCLUDE_DIR_ = $${PREFIX_}/include
  } else {
    INCLUDE_DIR_ = $${INCLUDE_DIR}
  }
  isEmpty( DOC_DIR ) {
    macx|darwin-g++ {
      isEmpty( PREFIX ) {
        DOC_DIR = $${HOME_DIR}/Library/Developer/Shared/Documentation/QGLViewer
      } else {
        DOC_DIR = $${PREFIX}/Shared/Documentation/QGLViewer
      }
    } else {
      DOC_DIR = $${PREFIX_}/share/doc/QGLViewer
    }
  }

  # GLUT for Unix architecture
  !isEmpty( USE_GLUT ) {
    QMAKE_LIBS_OPENGL *= -lglut
  }

  MOC_DIR = .moc
  OBJECTS_DIR = .obj

  # Adds a -P option so that "make install" as root creates files owned by root and links are preserved.
  # This is not a standard option, and it may have to be removed on old Unix flavors.
  !hpux {
    QMAKE_COPY_FILE = $${QMAKE_COPY_FILE} -P
  }

  # Make much smaller libraries (and packages) by removing debugging informations
  QMAKE_CFLAGS_RELEASE -= -g
  QMAKE_CXXFLAGS_RELEASE -= -g

  # install header
  include.path = $${INCLUDE_DIR_}/QGLViewer
  # Should be $$replace(TRANSLATIONS, .ts, .qm), but 'replace' only appeared in Qt 4.3
  include.files = $${QGL_HEADERS} qglviewer_fr.qm

  # install documentation html
  documentation.path = $${DOC_DIR}
  documentation.files = ../doc/*.html ../doc/*.css  ../doc/*.qch

  # install documentation images
  docImages.path = $${DOC_DIR}/images
  docImages.files = ../doc/images/*

  # install documentation examples
  #docExamples.path = $${DOC_DIR}/examples
  #docExamples.files = ../examples/*../examples/*/*

  # install documentation refManual
  docRefManual.path = $${DOC_DIR}/refManual
  docRefManual.files = ../doc/refManual/*

  # install static library
  #staticlib.extra = make -f Makefile.Release staticlib
  #staticlib.path = $${LIB_DIR_}
  #staticlib.files = lib$${TARGET}.a

  # install library
  target.path = $${LIB_DIR_}

  # "make install" configuration options
  INSTALLS *= target include documentation docImages docRefManual
}


# -----------------------
# --  S G I   I r i x  --
# -----------------------
irix-cc|irix-n32 {
  QMAKE_CFLAGS_RELEASE   -= -O3 -O2 -OPT:Olimit=30000
  QMAKE_LFLAGS_RELEASE   -= -O3 -O2 -OPT:Olimit=30000
  QMAKE_CXXFLAGS_RELEASE -= -O3 -O2 -OPT:Olimit=30000
  QMAKE_CFLAGS_RELEASE   *= -IPA -Ofast=IP35
  QMAKE_LFLAGS_RELEASE   *= -IPA -Ofast=IP35
  QMAKE_CXXFLAGS_RELEASE *= -IPA -Ofast=IP35
  QMAKE_CFLAGS           *= -LANG:std
  QMAKE_LFLAGS           *= -LANG:std
  QMAKE_CXXFLAGS         *= -LANG:std
  QMAKE_CFLAGS           *= -woff 1424,3201,1110,1188
  QMAKE_CXXFLAGS         *= -woff 1424,3201,1110,1188
  QMAKE_LIBS_OPENGL      -= -lXi
  # GLUT for SGI architecture
  !isEmpty( USE_GLUT ) {
    QMAKE_LIBDIR_OPENGL    *= /usr/local/lib32
    QMAKE_INCDIR_OPENGL    *= /usr/local/include
  }
}


# -------------------
# --  M a c O S X  --
# -------------------
macx|darwin-g++ {
  # This setting creates a Mac framework. Comment out this line to create a dylib instead.
  !staticlib: CONFIG *= lib_bundle

  include.files *= qglviewer.icns

  lib_bundle {
    FRAMEWORK_HEADERS.version = Versions
    # Should be $$replace(TRANSLATIONS, .ts, .qm), but 'replace' is only available in Qt 4.3
    FRAMEWORK_HEADERS.files = $${QGL_HEADERS} qglviewer.icns qglviewer_fr.qm
    FRAMEWORK_HEADERS.path = Headers
    QMAKE_BUNDLE_DATA += FRAMEWORK_HEADERS

    DESTDIR = $${HOME_DIR}/Library/Frameworks/

    # For a Framework, 'include' and 'lib' do no make sense.
    # These and prefix will all define the DESTDIR, in that order in case several are defined
    !isEmpty( INCLUDE_DIR ) {
      DESTDIR = $${INCLUDE_DIR}
    }

    !isEmpty( LIB_DIR ) {
      DESTDIR = $${LIB_DIR}
    }

    !isEmpty( PREFIX ) {
      DESTDIR = $${PREFIX}
    }
  
    QMAKE_POST_LINK=cd $$DESTDIR/QGLViewer.framework/Headers && (test -L QGLViewer || ln -s . QGLViewer)

    #QMAKE_LFLAGS_SONAME  = -Wl,-install_name,@executable_path/../Frameworks/
    #QMAKE_LFLAGS_SONAME  = -Wl,-install_name,

    # Framework already installed, with includes
    INSTALLS -= include target
  } else {
    #QMAKE_LFLAGS_SONAME  = -Wl,-install_name,libQGLViewer.dylib
  }

  # GLUT for Macintosh architecture
  !isEmpty( USE_GLUT ) {
    QMAKE_LIBS_OPENGL -= -lglut
    QMAKE_LIBS_OPENGL *= -framework GLUT -lobjc
  }

  # Qt3 only
  macx: CONFIG -= thread
}


# ---------------------
# --  W i n d o w s  --
# ---------------------
win32 {
  # Windows requires a debug lib version to link against debug applications
  CONFIG *= debug_and_release build_all

  # Needed by Intel C++, (icl.exe) so that WINGDIAPI is a defined symbol in gl.h.
  DEFINES *= WIN32

  staticlib {
    DEFINES *= QGLVIEWER_STATIC
  } else {
    DEFINES *= CREATE_QGLVIEWER_DLL
  }

  # Use the DLL version of Qt (Qt3 only)
  DEFINES *= QT_DLL QT_THREAD_SUPPORT

  CONFIG *= embed_manifest_dll

  # Make sure to have C++ files, PentiumPro code, few warnings, add
  # support to RTTI and Exceptions, and generate debug info "program database".
  # Any feedback on these flags is welcome.
  !win32-g++ {
    QMAKE_CXXFLAGS = -TP -GR -Zi
    DEFINES += NOMINMAX
    win32-msvc {
      QMAKE_CXXFLAGS *= -GX
    } else {
      QMAKE_CXXFLAGS *= -EHs
    }
  }
}


!contains( QT_VERSION, "^3.*" ) {
   build_pass:CONFIG(debug, debug|release) {
     unix: TARGET = $$join(TARGET,,,_debug)
     else: TARGET = $$join(TARGET,,,d)
   }
}
