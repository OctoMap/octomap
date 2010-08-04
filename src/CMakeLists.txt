CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap )

# version (e.g. for packaging)
set(V_MAJOR 0)
set(V_MINOR 42)
set(V_PATCH 0)

# get rid of a useless warning:
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


# COMPILER SETTINGS (default: Release)
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

SET (CMAKE_MODULE_PATH ${CMAKE_MODULES_PATH} ${BASE_DIR}/CMakeModules)

MESSAGE ("\n")
MESSAGE (STATUS "Building as ${CMAKE_BUILD_TYPE}")
MESSAGE ("\n")

# Main include dir is the "src" dir
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
)


# Installation

# test dir for install
# IF(NOT CMAKE_INSTALL_PREFIX)
# 	set(CMAKE_INSTALL_PREFIX ${BASE_DIR}/install/) 
# ENDIF(NOT CMAKE_INSTALL_PREFIX)

set(INSTALL_TARGETS_DEFAULT_ARGS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

ADD_SUBDIRECTORY( octomath )
ADD_SUBDIRECTORY( octomap )

# installation for external targets:
install(PROGRAMS "extern/binvox/binvox" DESTINATION bin)


# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/octomap.dox
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)


SET( BUILD_VIEWER 0)

FIND_PACKAGE(OpenGL)
IF(NOT OPENGL_FOUND)
  MESSAGE ( "\n")
  MESSAGE ( "OpenGL not found. \n")
ELSE()
  MESSAGE (STATUS "OpenGL found at ${OPENGL_LIBRARY}")

  # Look for Qt4
  FIND_PACKAGE(Qt4)
  IF(QT4_FOUND)
    FIND_PACKAGE(QGLViewer)
    IF(QGLViewer_FOUND)
      SET( BUILD_VIEWER 1)
    ELSE()
      MESSAGE ( "\n")
      MESSAGE ( "libQGLViewer could not be found or generated.")
    ENDIF()
  ELSE()
    MESSAGE ( "\n")
    MESSAGE ( "Qt4 development environment could not be found.")
  ENDIF()
ENDIF()

IF(BUILD_VIEWER)
  MESSAGE(STATUS "\n")
  MESSAGE(STATUS "viewer octovis will be build")
  ADD_SUBDIRECTORY( octovis ) 
ELSE()
    MESSAGE ( "Unfortunately, the viewer (octovis) can not be build.")
    MESSAGE ( "See README.txt or http://octomap.sf.net for further information.\n")
ENDIF()

# Package building stuff - Experimental!
SET(CPACK_PACKAGE_VERSION_MAJOR "${V_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${V_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${V_PATCH}")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${V_MAJOR}.${V_MINOR}")
SET(CPACK_PACKAGE_CONTACT "K.M. Wurm and A. Hornung")
SET(CPACK_PACKAGE_VENDOR "University of Freiburg")
SET(CPACK_GENERATOR "DEB")
SET(CPACK_SOURCE_GENERATOR "TGZ")

INCLUDE(CPack)


MESSAGE (STATUS "\n")
MESSAGE (STATUS "Compile octomap using: make")
MESSAGE (STATUS "Install octomap using: make install")
MESSAGE (    "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
MESSAGE (STATUS "")




