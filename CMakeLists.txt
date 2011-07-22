CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( OctoMap )

# version (e.g. for packaging)
set(OCTOMAP_MAJOR_VERSION 1)
set(OCTOMAP_MINOR_VERSION 1)
set(OCTOMAP_PATCH_VERSION 0)

# get rid of a useless warning:
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CMakeModules/CompilerSettings.cmake)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

SET (CMAKE_MODULE_PATH ${CMAKE_MODULES_PATH} ${BASE_DIR}/CMakeModules)

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
)


# Installation

set(INSTALL_TARGETS_DEFAULT_ARGS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

ADD_SUBDIRECTORY( src/octomap/math )
ADD_SUBDIRECTORY( src/octomap )

# installation for external targets:
install(PROGRAMS "src/extern/binvox/binvox" DESTINATION bin)
install(PROGRAMS "src/extern/binvox/binvox64" DESTINATION bin)
install(PROGRAMS "src/extern/binvox/viewvox" DESTINATION bin)
install(PROGRAMS "src/extern/binvox/viewvox64" DESTINATION bin)

file(GLOB octomap_HDRS ${CMAKE_SOURCE_DIR}/include/octomap/*.h ${CMAKE_SOURCE_DIR}/include/octomap/*.hxx)
install(FILES ${octomap_HDRS}	DESTINATION include/octomap)
file(GLOB octomap_math_HDRS ${CMAKE_SOURCE_DIR}/include/octomap/math/*.h)
install(FILES ${octomap_math_HDRS}	DESTINATION include/octomap/math)


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
  MESSAGE(STATUS "viewer octovis will be built")
  ADD_SUBDIRECTORY( src/octovis ) 
ELSE()
    MESSAGE ( "Unfortunately, the viewer (octovis) can not be built because some requirements are missing.")
    MESSAGE ( "See README.txt or http://octomap.sf.net for further information.\n")
ENDIF()

# Needs to be last statement:
INCLUDE(CMakeModules/CPackSettings.cmake)

# Finished:

MESSAGE (STATUS "\n")
MESSAGE (STATUS "Compile octomap using: make")
MESSAGE (STATUS "Install octomap using: make install")
MESSAGE (    "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
MESSAGE (STATUS "")




