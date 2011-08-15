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

# TODO:
# use generated octmap-config to find the package *additionally*
# this will not work when compiling from the top directory
# and octomap was not compiled before!
#find_package(octomap REQUIRED
#  HINTS ${CMAKE_SOURCE_DIR}/octomap
#    ${CMAKE_SOURCE_DIR}/../octomap
#)

FIND_PATH( OCTOMAP_BASE_DIR include/octomap/octomap.h
  ${CMAKE_SOURCE_DIR}/octomap
  ${CMAKE_SOURCE_DIR}/../octomap
  /usr
)

IF( OCTOMAP_BASE_DIR)
  MESSAGE(STATUS "octomap library  found in ${OCTOMAP_BASE_DIR}")
  # set same dirs as in find_package, so we can swap later:
  SET(OCTOMAP_INCLUDE_DIRS ${OCTOMAP_BASE_DIR}/include)
  SET(OCTOMAP_LIBRARY_DIRS ${OCTOMAP_BASE_DIR}/lib)
  SET(OCTOMAP_LIBRARIES octomap octomath)
  
  
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

  # Build viewer if available:
  INCLUDE(BuildGLViewer)

ELSE()
  MESSAGE(STATUS "Required octomap library NOT found. Exiting.")

ENDIF()
