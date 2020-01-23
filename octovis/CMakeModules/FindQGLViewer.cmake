# Find QGLViewer library
# Looks for a system-wide version of libQGLViewer.
#
# QGLViewer_INCLUDE_DIR      where to find the include files
# QGLViewer_LIBRARIES        list of libraries to link
# QGLViewer_FOUND            true if QGLViewer was found

find_path(QGLViewer_INCLUDE_DIR
  NAMES qglviewer.h
  PATH_SUFFIXES QGLViewer)

find_library(QGLViewer_LIBRARIES
  NAMES QGLViewer QGLViewer2 QGLViewer-qt5 QGLViewer-qt4)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  QGLViewer
  DEFAULT_MSG
  QGLViewer_LIBRARIES QGLViewer_INCLUDE_DIR)

mark_as_advanced(QGLViewer_INCLUDE_DIR QGLViewer_LIBRARIES)
