CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octovis )

# # version (e.g. for packaging)
set(OCTOVIS_MAJOR_VERSION 1)
set(OCTOVIS_MINOR_VERSION 7)
set(OCTOVIS_PATCH_VERSION 2)
set(OCTOVIS_VERSION ${OCTOVIS_MAJOR_VERSION}.${OCTOVIS_MINOR_VERSION}.${OCTOVIS_PATCH_VERSION})
set(OCTOVIS_SOVERSION ${OCTOVIS_MAJOR_VERSION}.${OCTOVIS_MINOR_VERSION})
# get rid of a useless warning:
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_SOURCE_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )
# output dirs for multi-config builds (MSVC)
foreach( OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES} )
  STRING( TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG )
  SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/lib )
  SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG} ${BASE_DIR}/bin )
endforeach( OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES )

# We need the main octomap library to link against (same version as octovis)
# Look at parent directory by default, in case the complete distribution
# (octomap & octovis together) is built.
#
# Otherwise you need to export octomap_DIR to the directory containing
# the file octomap-config.cmake
find_package(octomap ${OCTOVIS_MAJOR_VERSION}.${OCTOVIS_MINOR_VERSION} REQUIRED
HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/octomap
${CMAKE_SOURCE_DIR}/../octomap/lib/cmake/octomap
)
MESSAGE(STATUS "Found octomap version: " ${octomap_VERSION})

INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})

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

# Look for required libraries:
FIND_PACKAGE(OpenGL)
FIND_PACKAGE(Qt4)
IF (OpenGL-NOTFOUND OR Qt4-NOTFOUND) 
	MESSAGE ( "OpenGL and QT4 are required for octovis but could not be found.")	
ELSE()
	FIND_PACKAGE(QGLViewer)
	IF(QGLViewer_FOUND)
	  SET( BUILD_VIEWER 1)
	ELSE()
	  MESSAGE ( "\n")
  	MESSAGE ( "libQGLViewer could not be found or generated.")
	ENDIF()
ENDIF()


IF(BUILD_VIEWER)
  MESSAGE(STATUS "\n")
  MESSAGE(STATUS "viewer octovis will be built")
  
  set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
  INCLUDE_DIRECTORIES(${INCLUDE_DIRS})
   
  INCLUDE( CMakeLists_src.txt )
    
  # Create an octovis-config.cmake file for the use from the build tree
  set(OCTOVIS_INCLUDE_DIR "${INCLUDE_DIRS}")
  set(OCTOVIS_LIB_DIR     "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
  # Set library names as absolute paths
  # Windows, spec. MSVC requires the .lib suffix for imported libs
  IF(WIN32)
    set(OCTOVIS_LIBRARY
      "${CMAKE_IMPORT_LIBRARY_PREFIX}octovis${CMAKE_IMPORT_LIBRARY_SUFFIX}"
    )
  ELSE()
    set(OCTOVIS_LIBRARY
      "${CMAKE_SHARED_LIBRARY_PREFIX}octovis${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
  ENDIF()

  # not used right now (export depends?)
  #set(OCTOMAP_CMAKE_DIR "${PROJECT_BINARY_DIR}")
  configure_file(octovis-config.cmake.in
    "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octovis/octovis-config.cmake" @ONLY)
  configure_file(octovis-config-version.cmake.in
    "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octovis/octovis-config-version.cmake" @ONLY)  
    
  # Create a octovis-config.cmake file for the use from the install tree
  # and install it
  set(OCTOVIS_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  set(OCTOVIS_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
  #set(OCTOMAP_CMAKE_DIR "${INSTALL_DATA_DIR}/FooBar/CMake")
  configure_file(octovis-config.cmake.in
    "${PROJECT_BINARY_DIR}/InstallFiles/octovis-config.cmake" @ONLY)
  configure_file(octovis-config-version.cmake.in
    "${PROJECT_BINARY_DIR}/InstallFiles/octovis-config-version.cmake" @ONLY)
  install(FILES
    "${PROJECT_BINARY_DIR}/InstallFiles/octovis-config.cmake"
    "${PROJECT_BINARY_DIR}/InstallFiles/octovis-config-version.cmake" 
    DESTINATION share/octovis/)
    
    
  # #installation:
  # # store all header files to install:
  file(GLOB octovis_HDRS  *.h *.hxx *.hpp)
  install(FILES ${octovis_HDRS} DESTINATION include/octovis)

  # Install catkin package.xml
  install(FILES package.xml DESTINATION share/octovis)
  
ELSE()
    MESSAGE ( "Unfortunately, the viewer (octovis) can not be built because some requirements are missing.")
	MESSAGE ( "This will not affect the compilation of the stand-alone library and tools (octomap)")
    MESSAGE ( "See README.txt or http://octomap.sf.net for further information.\n")
ENDIF()

