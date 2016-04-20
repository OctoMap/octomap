CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(dynamicEDT3D)

ENABLE_TESTING()

# version (e.g. for packaging)
set(DYNAMICEDT3D_MAJOR_VERSION 1)
set(DYNAMICEDT3D_MINOR_VERSION 8)
set(DYNAMICEDT3D_PATCH_VERSION 0)
set(DYNAMICEDT3D_VERSION ${DYNAMICEDT3D_MAJOR_VERSION}.${DYNAMICEDT3D_MINOR_VERSION}.${DYNAMICEDT3D_PATCH_VERSION})
set(DYNAMICEDT3D_SOVERSION ${DYNAMICEDT3D_MAJOR_VERSION}.${DYNAMICEDT3D_MINOR_VERSION})

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

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# Installation

set(INSTALL_TARGETS_DEFAULT_ARGS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

find_package(octomap REQUIRED
  HINTS ${CMAKE_SOURCE_DIR}/lib/cmake/octomap
   ${CMAKE_SOURCE_DIR}/../octomap/lib/cmake/octomap
)
MESSAGE(STATUS "Found octomap version: " ${octomap_VERSION})
MESSAGE(STATUS "octomap libraries: ${OCTOMAP_LIBRARIES}")

INCLUDE_DIRECTORIES(${OCTOMAP_INCLUDE_DIRS})

ADD_SUBDIRECTORY(src)


file(GLOB dynamicEDT3D_HDRS ${PROJECT_SOURCE_DIR}/include/dynamicEDT3D/*.h ${PROJECT_SOURCE_DIR}/include/dynamicEDT3D/*.hxx)
install(FILES ${dynamicEDT3D_HDRS}	DESTINATION include/dynamicEDT3D)

# Install catkin package.xml, attention package.xml names the catkin package "dynamic_edt_3d", so this is also the location where it needs to be installed to (and not "dynamicEDT3D")
install(FILES package.xml DESTINATION share/dynamic_edt_3d)

#TODO: this conflicts with the octomap uninstall
#it is not only a target name problem, also both will use the same manifest file
#in the same binary directory
#configure_file(
#    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
#    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#    IMMEDIATE @ONLY)
#
#add_custom_target(uninstall
#    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE dynamicEDT3D)
 
# Create a dynamicEDT3DConfig.cmake file for the use from the build tree
set(DYNAMICEDT3D_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(DYNAMICEDT3D_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# Set library names as absolute paths
# Windows, spec. MSVC requires the .lib suffix for imported libs
IF(WIN32)
  set(DYNAMICEDT3D_LIBRARY
    "${CMAKE_IMPORT_LIBRARY_PREFIX}dynamicedt3d${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  )
ELSE()
  set(DYNAMICEDT3D_LIBRARY
    "${CMAKE_SHARED_LIBRARY_PREFIX}dynamicedt3d${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )
ENDIF()
# not used right now (export depends?)
#set(DYNEDT3D_CMAKE_DIR "${PROJECT_BINARY_DIR}")
configure_file(dynamicEDT3DConfig.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D/dynamicEDT3DConfig.cmake" @ONLY)
configure_file(dynamicEDT3DConfig-version.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D/dynamicEDT3DConfig-version.cmake" @ONLY)

# Install the export set for use with the install-tree
#install(EXPORT FooBarLibraryDepends DESTINATION
#  "${INSTALL_DATA_DIR}/FooBar/CMake"
#  COMPONENT dev)
 
# Create a dynamicEDT3DConfig.cmake file for the use from the install tree
# and install it
set(DYNAMICEDT3D_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(DYNAMICEDT3D_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
#set(DYNAMICEDT3D_CMAKE_DIR "${INSTALL_DATA_DIR}/FooBar/CMake")
configure_file(dynamicEDT3DConfig.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/dynamicEDT3DConfig.cmake" @ONLY)
configure_file(dynamicEDT3DConfig-version.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/dynamicEDT3DConfig-version.cmake" @ONLY)
install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/dynamicEDT3DConfig.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/dynamicEDT3DConfig-version.cmake" 
  DESTINATION share/dynamicEDT3D/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(dynamicEDT3D
    CFLAGS
    LIBS -ldynamicEDT3D
    REQUIRES
    VERSION ${DYNAMICEDT3D_VERSION})


# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  ADD_CUSTOM_TARGET(docs_dynamicEDT3D ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/dynamicEDT3D.dox
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile dynamicEDT3D using: make")
MESSAGE (STATUS "Install dynamicEDT3D using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
MESSAGE (STATUS "Compile API-documentation using: make docs_dynamicEDT3D\n")
