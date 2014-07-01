# - Config file for the OctoMap package
# (example from http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file)
# It defines the following variables
#  OCTOVIS_INCLUDE_DIRS - include directories for OctoMap viewer
#  OCTOVIS_LIBRARIES    - libraries to link against

find_package(OCTOMAP)

set(QGLViewer_LIB QGLViewer_LIB-NOTFOUND)
find_library(QGLViewer_LIB QGLViewer)
if(NOT QGLViewer_LIB)
  message(FATAL "Could not find QGLViewer")
endif()

find_package(Qt4 COMPONENTS QTCORE QTGUI QTOPENGL)

set(OCTOVIS_INCLUDE_DIRS)
set(OCTOVIS_LIBRARIES)

list(APPEND OCTOVIS_INCLUDE_DIRS ${OCTOMAP_INCLUDE}
  ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR})

list(APPEND OCTOVIS_LIBRARIES ${OCTOMAP_LIB} ${QGLViewer_LIB}
  ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY})
