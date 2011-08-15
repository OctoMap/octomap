CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octovis )

# # version (e.g. for packaging)
# set(OCTOMAP_MAJOR_VERSION 1)
# set(OCTOMAP_MINOR_VERSION 1)
# set(OCTOMAP_PATCH_VERSION 0)
# set(OCTOMAP_VERSION ${OCTOMAP_MAJOR_VERSION}.${OCTOMAP_MINOR_VERSION}.${OCTOMAP_PATCH_VERSION})
# get rid of a useless warning:
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules;${CMAKE_SOURCE_DIR}/../CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

#If we are compiling with ROS support, we need to get the appropriate includes and link flags and such
# (thx to E.Perko for patch!)
if(WITH_ROS)
       include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
       set(ROSBUILD_DONT_REDEFINE_PROJECT true)
       set(PROJECT_SOURCE_DIR_BACKUP ${PROJECT_SOURCE_DIR})
       set(PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../..")
       rosbuild_init()
       set(PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR_BACKUP})
       add_definitions(-DOCTOMAP_ROS)
endif(WITH_ROS)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

find_package(octomap REQUIRED
  HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/octomap
   ${CMAKE_SOURCE_DIR}/../octomap/lib/cmake/octomap
)

# now include (same as with find_package)
INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})
LINK_DIRECTORIES(${OCTOMAP_LIBRARY_DIRS})

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE octovis)

set(INSTALL_TARGETS_DEFAULT_ARGS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Builds the "octovis" viewer based on OpenGL and 
# libQGLViewer, if dependencies available
SET( BUILD_VIEWER 0)

FIND_PACKAGE(OpenGL REQUIRED)
MESSAGE (STATUS "OpenGL found: ${OPENGL_LIBRARY}")

# Look for Qt4
FIND_PACKAGE(Qt4 REQUIRED)

# Look for QGLViewer
FIND_PACKAGE(QGLViewer)
IF(QGLViewer_FOUND)
  SET( BUILD_VIEWER 1)
ELSE()
  MESSAGE ( "\n")
  MESSAGE ( "libQGLViewer could not be found or generated.")
ENDIF()

IF(BUILD_VIEWER)
  MESSAGE(STATUS "\n")
  MESSAGE(STATUS "viewer octovis will be built")
  ADD_SUBDIRECTORY( src/octovis ) 
ELSE()
    MESSAGE ( "Unfortunately, the viewer (octovis) can not be built because some requirements are missing.")
    MESSAGE ( "See README.txt or http://octomap.sf.net for further information.\n")
ENDIF()

