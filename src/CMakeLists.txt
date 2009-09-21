CMAKE_MINIMUM_REQUIRED(VERSION 2.4.5)
PROJECT( octomap )

# get rid of a useless warning:
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_path( FERN_DIR Pose6D.h PATHS $ENV{HOME}/code/projects-fern $ENV{HOME}/code/fern $ENV{HOME}/projects-fern $ENV{HOME}/fern $BASE_DIR}/../projects-fern $BASE_DIR}/../fern)
message(STATUS "fern found in: " ${FERN_DIR})


# COMPILER FLAGS (default: Release)
# use "-DCMAKE_BUILD_TYPE=Debug" in cmake for a Debug-build
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# COMPILER FLAGS
SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-error ")
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error ")
SET (CMAKE_CXX_FLAGS_RELEASE "-O3")
SET (CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR}/.. )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )


LINK_DIRECTORIES(
  ${FERN_DIR}/lib
  )

INCLUDE_DIRECTORIES(
  ${FERN_DIR}/include
  ${CMAKE_SOURCE_DIR}/octomap
  )

###
# Libraries used by all subprojects:
#
LINK_LIBRARIES(
  ferngeom fern
  gsl cblas
  )
  

ADD_SUBDIRECTORY( octomap )


# Look for Qt4 for Viewer:
FIND_PACKAGE(Qt4)
IF(QT4_FOUND)
    ADD_CUSTOM_TARGET(qglviewer 
        qmake-qt4
        COMMAND make
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/extern/QGLViewer
        COMMENT "Building libQGLViewer..." VERBATIM)
        
    ADD_SUBDIRECTORY( viewer ) 
ELSE(QT4_FOUND)
	MESSAGE(STATUS "Qt4 not found, skipping GUI")
ENDIF(QT4_FOUND)

