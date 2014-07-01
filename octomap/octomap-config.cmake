# ===================================================================================
#  The OctoMap CMake configuration file
#
#             ** File generated automatically, do not modify **
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(octomap REQUIRED )
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${OCTOMAP_LIBRARIES})
#
#    This file will define the following variables:
#      - OCTOMAP_LIBRARIES      : The list of libraries to links against.
#      - OCTOMAP_LIBRARY_DIRS   : The directory where lib files are. Calling
#                                 LINK_DIRECTORIES with this path is NOT needed.
#      - OCTOMAP_INCLUDE_DIRS   : The OpenCV include directories.
#
# Based on the example CMake Tutorial
# http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file
# and OpenCVConfig.cmake.in from OpenCV
# ===================================================================================

set(OCTOMAP_INCLUDE_DIRS)
set(OCTOMAP_INCLUDE OCTOMAP_INCLUDE-NOTFOUND)
find_path(OCTOMAP_INCLUDE octomap/octomap.h)
if(OCTOMAP_INCLUDE)
  list(APPEND OCTOMAP_INCLUDE_DIRS ${OCTOMAP_INCLUDE})
endif()

set(OCTOMAP_LIBRARIES)
set(OCTOMAP_LIB OCTOMAP_LIB-NOTFOUND)
find_library(OCTOMAP_LIB octomap)
if(OCTOMAP_LIB)
  list(APPEND OCTOMAP_LIBRARIES ${OCTOMAP_LIB})
endif()

set(OCTOMATH_LIB OCTOMATH_LIB-NOTFOUND)
find_library(OCTOMATH_LIB octomath)
if(OCTOMATH_LIB)
  list(APPEND OCTOMAP_LIBRARIES ${OCTOMATH_LIB})
endif()
