# Find QGLViewer library
# Looks for a system-wide version of libQGLViewer (qglviewer-qt4 in Ubuntu).
# If none is found, it builds and uses the local copy in "extern"
#
# Many thanks to L. Ott for assistance!
#
# QGLViewer_INCLUDE_DIR      where to find the include files
# QGLViewer_LIBRARY_DIR      where to find the libraries
# QGLViewer_LIBRARIES        list of libraries to link
# QGLViewer_FOUND            true if QGLViewer was found

SET( QGLViewer_FOUND 0 CACHE BOOL "Do we have QGLViewer?" )

FIND_PATH( QGLVIEWER_BASE_DIR qglviewer.h
  ${CMAKE_SOURCE_DIR}/src/extern/QGLViewer
  ${CMAKE_SOURCE_DIR}/octovis/src/extern/QGLViewer
)

FIND_PATH( QGLViewer_INCLUDE_DIR qglviewer.h
    /usr/include/qglviewer-qt4
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    ${QGLVIEWER_BASE_DIR}
)

FIND_LIBRARY( QGLViewer_LIBRARY_DIR_UBUNTU NAMES qglviewer-qt4 QGLViewer-qt4)
FIND_LIBRARY( QGLViewer_LIBRARY_DIR_WINDOWS QGLViewer2 ${QGLVIEWER_BASE_DIR})
FIND_LIBRARY( QGLViewer_LIBRARY_DIR_OTHER QGLViewer ${QGLVIEWER_BASE_DIR})

SET( BUILD_LIB_FROM_SOURCE 0)

IF( QGLViewer_INCLUDE_DIR )

  MESSAGE(STATUS "QGLViewer includes found in ${QGLViewer_INCLUDE_DIR}")
  IF (QGLViewer_LIBRARY_DIR_UBUNTU)
    MESSAGE(STATUS "QGLViewer library found in ${QGLViewer_LIBRARY_DIR_UBUNTU}")
    SET( QGLViewer_LIBRARIES ${QGLViewer_LIBRARY_DIR_UBUNTU})
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSEIF(QGLViewer_LIBRARY_DIR_WINDOWS)
    MESSAGE(STATUS "QGLViewer2 found in ${QGLViewer_LIBRARY_DIR_WINDOWS}")
    SET( QGLViewer_LIBRARIES ${QGLViewer_LIBRARY_DIR_WINDOWS})
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSEIF(QGLViewer_LIBRARY_DIR_OTHER)
    MESSAGE(STATUS "QGLViewer found in ${QGLViewer_LIBRARY_DIR_OTHER}")
    SET( QGLViewer_LIBRARIES ${QGLViewer_LIBRARY_DIR_OTHER})
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSE()
    MESSAGE(STATUS "QGLViewer library not found.")
    SET( BUILD_LIB_FROM_SOURCE 1)
    SET( QGLViewer_FOUND 0 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ENDIF()    

ELSE()
  SET( BUILD_LIB_FROM_SOURCE 1)
ENDIF()

# build own libQGLViewer
IF(BUILD_LIB_FROM_SOURCE)
  
  IF (WIN32)
    MESSAGE("Cannot generate QGLViewer2 from source automatically.")
    MESSAGE("Please build libQGLViewer from source, instructions to do so")
    MESSAGE("can be found in octovis/README.txt")
    MESSAGE("Please rerun CMAKE when you are ready.")
    
  ELSE (WIN32)
    IF(QGLVIEWER_BASE_DIR)
      MESSAGE(STATUS "Trying to build libQGLViewer from source in ${QGLVIEWER_BASE_DIR}")
      
      FIND_PROGRAM(QMAKE-QT4 qmake-qt4)
      IF (QMAKE-QT4)
	MESSAGE(STATUS "\t generating Makefile using qmake-qt4") 
	EXECUTE_PROCESS(
	  WORKING_DIRECTORY ${QGLVIEWER_BASE_DIR}
	  COMMAND qmake-qt4
	  OUTPUT_QUIET
	  )
      ELSE(QMAKE-QT4)
	MESSAGE(STATUS "\t generating Makefile using qmake") 
	EXECUTE_PROCESS(
	  WORKING_DIRECTORY ${QGLVIEWER_BASE_DIR}
	  COMMAND qmake-qt4
	  OUTPUT_QUIET
	  )
      ENDIF(QMAKE-QT4)
      
      MESSAGE(STATUS "\t building library (this may take some time...)")
      EXECUTE_PROCESS(
	WORKING_DIRECTORY ${QGLVIEWER_BASE_DIR}
	COMMAND make
	OUTPUT_QUIET
	)
    ENDIF(QGLVIEWER_BASE_DIR)
  ENDIF(WIN32)
  
ELSE(BUILD_LIB_FROM_SOURCE)
  IF (NOT QGLViewer_FOUND)
    MESSAGE(STATUS "QGLViewer sources NOT found. Exiting.")
  ENDIF ()
ENDIF(BUILD_LIB_FROM_SOURCE)

# verify that QGLViewer lib was build if we didnt find it elsewhere
IF (NOT QGLViewer_FOUND)
  #FIND_LIBRARY(QGLViewer_LIBRARY_DIR_OTHER QGLViewer ${QGLVIEWER_BASE_DIR})
  FIND_PATH(QGLLIB libQGLViewer.so ${QGLVIEWER_BASE_DIR})
  IF (NOT QGLLIB)
    MESSAGE(WARNING "Could not find libQGLViewer.so, failed to build?")
    SET( QGLViewer_FOUND 0 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSE()
    MESSAGE(STATUS "Successfully built library in:\n${QGLLIB}")
    SET( QGLViewer_INCLUDE_DIR ${QGLVIEWER_BASE_DIR} CACHE PATH "QGLViewer Include directory" FORCE)
    SET( QGLViewer_LIBRARY_DIR ${QGLVIEWER_BASE_DIR} CACHE PATH "QGLViewer Library directory" FORCE)
    #  TODO: also include "m pthread  QGLViewerGen QGLViewerUtility"?
    SET( QGLViewer_LIBRARIES QGLViewer)
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ENDIF()
ENDIF()

# You need to use qmake of QT4. You are using QT3 if you get:

#CMakeFiles/octovis.dir/ViewerWidget.cpp.o: In function `octomap::ViewerWidget::ViewerWidget(QWidget*)':
#ViewerWidget.cpp:(.text+0x1715): undefined reference to `QGLViewer::QGLViewer(QWidget*, QGLWidget const*, QFlags<Qt::WindowType>)'