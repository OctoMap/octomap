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


INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/math
  ${CMAKE_SOURCE_DIR}/octomap
)

ADD_SUBDIRECTORY( math )
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

# Documentation target:
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
    ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/octomap.dox
                     WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                     COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)

# Packaging:
# SET(CPACK_SET_DESTDIR "ON")
# SET(CPACK_SOURCE_GENERATOR "TGZ")
# INCLUDE(CPack)


