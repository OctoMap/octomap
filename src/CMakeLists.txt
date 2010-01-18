CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap )

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


# Main include dir is the "src" dir
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
)

ADD_SUBDIRECTORY( octomath )
ADD_SUBDIRECTORY( octomap )

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
    MESSAGE ( "See README.txt for further information.\n")
ENDIF()


# Documentation target:
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/octomap.dox
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)


MESSAGE (STATUS "\n")
MESSAGE (STATUS "Compile octomap using: make")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
MESSAGE (STATUS "")


