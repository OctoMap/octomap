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
SET( QGLViewer_HOME / CACHE PATH "Where to find QGLViewer" )

FIND_PATH( QGLViewer_INCLUDE_DIR qglviewer.h
    /usr/include/qglviewer-qt4
    /usr/include/QGLViewer
    /opt/local/include/QGLViewer
    ${QGLViewer_HOME}
)

FIND_LIBRARY( QGLViewer_LIBRARY_DIR_UBUNTU qglviewer-qt4 )
FIND_LIBRARY( QGLViewer_LIBRARY_DIR_OTHER QGLViewer ${QGLViewer_HOME})

SET( BUILD_LIB_FROM_SOURCE 0)

IF( QGLViewer_INCLUDE_DIR )
  MESSAGE(STATUS "QGLViewer includes found in ${QGLViewer_INCLUDE_DIR}")
  #SET( QGLViewer_INCLUDE_DIR ${QGLViewer_INCLUDE_DIR_STRIP})

#  SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )

  #MESSAGE( STATUS "Setting QGLViewer_INCLUDE_DIR to ${QGLViewer_INCLUDE_DIR}" )
  IF (QGLViewer_LIBRARY_DIR_UBUNTU)
    MESSAGE(STATUS "qglviewer-qt4 found in ${QGLViewer_LIBRARY_DIR_UBUNTU}")
    # strip filename from path
    GET_FILENAME_COMPONENT( QGLViewer_LIBRARY_DIR ${QGLViewer_LIBRARY_DIR_UBUNTU} PATH CACHE )
    SET( QGLViewer_LIBRARIES qglviewer-qt4)
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSEIF(QGLViewer_LIBRARY_DIR_OTHER)
    MESSAGE(STATUS "QGLViewer found in ${QGLViewer_LIBRARY_DIR_OTHER}")
    # strip filename from path
    GET_FILENAME_COMPONENT( QGLViewer_LIBRARY_DIR ${QGLViewer_LIBRARY_DIR_OTHER} PATH CACHE )
    SET( QGLViewer_LIBRARIES QGLViewer)
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSE()
    MESSAGE(STATUS "QGLViewer library not found.")
    SET( BUILD_LIB_FROM_SOURCE 1)
    SET( QGLViewer_FOUND 0 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ENDIF()    
  
ELSE()
  SET( BUILD_LIB_FROM_SOURCE 1)
ENDIF()

IF(BUILD_LIB_FROM_SOURCE)

  # build own libQGLViewer
  MESSAGE(STATUS "Trying to build libQGLViewer from source...")

  MESSAGE(STATUS "\t generating Makefile using qmake")
  EXECUTE_PROCESS(
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/extern/QGLViewer
    COMMAND qmake
    OUTPUT_QUIET
  )

  MESSAGE(STATUS "\t building library")
  EXECUTE_PROCESS(
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/extern/QGLViewer
    COMMAND make
    OUTPUT_QUIET
  )

  FIND_LIBRARY(QGLViewer_LIBRARY_DIR_OTHER QGLViewer ${QGLViewer_HOME})
  FIND_PATH(QGLLIB libQGLViewer.so  ${CMAKE_SOURCE_DIR}/extern/QGLViewer)

  IF (NOT QGLLIB)
    MESSAGE(STATUS "\nfailed to build libQGLViewer")
    SET( QGLViewer_FOUND 0 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ELSE()
    MESSAGE(STATUS "Successfully built ${QGLLIB}")
    SET( QGLViewer_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/extern/QGLViewer CACHE PATH "QGLViewer Include directory" FORCE)
    SET( QGLViewer_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/extern/QGLViewer CACHE PATH "QGLViewer Library directory" FORCE)
    #  TODO: also include "m pthread  QGLViewerGen QGLViewerUtility"?
    SET( QGLViewer_LIBRARIES QGLViewer)
    SET( QGLViewer_FOUND 1 CACHE BOOL "Do we have QGLViewer?" FORCE )
  ENDIF()
ENDIF()

	

