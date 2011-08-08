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

FIND_PATH( OCTOMAP_BASE_DIR include/octomap/octomap.h
  ${CMAKE_SOURCE_DIR}/octomap
  ${CMAKE_SOURCE_DIR}/../octomap
  /usr
)

IF( OCTOMAP_BASE_DIR)
  MESSAGE(STATUS "octomap library  found in ${OCTOMAP_BASE_DIR}")

  INCLUDE_DIRECTORIES(
    ${OCTOMAP_BASE_DIR}/include
  )

  LINK_DIRECTORIES(
    ${OCTOMAP_BASE_DIR}/lib
  )

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
  MESSAGE(STATUS "octomap library NOT found. Exiting.")

ENDIF()
