CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap )

ENABLE_TESTING()

# version (e.g. for packaging)
set(OCTOMAP_MAJOR_VERSION 1)
set(OCTOMAP_MINOR_VERSION 5)
set(OCTOMAP_PATCH_VERSION 2)
set(OCTOMAP_VERSION ${OCTOMAP_MAJOR_VERSION}.${OCTOMAP_MINOR_VERSION}.${OCTOMAP_PATCH_VERSION})
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

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# Installation

set(INSTALL_TARGETS_DEFAULT_ARGS
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

ADD_SUBDIRECTORY( src/math )
ADD_SUBDIRECTORY( src )

# installation for external targets:
install(PROGRAMS "extern/binvox/binvox" DESTINATION bin)
install(PROGRAMS "extern/binvox/binvox64" DESTINATION bin)
install(PROGRAMS "extern/binvox/viewvox" DESTINATION bin)
install(PROGRAMS "extern/binvox/viewvox64" DESTINATION bin)

file(GLOB octomap_HDRS ${PROJECT_SOURCE_DIR}/include/octomap/*.h ${PROJECT_SOURCE_DIR}/include/octomap/*.hxx)
install(FILES ${octomap_HDRS}	DESTINATION include/octomap)
file(GLOB octomap_math_HDRS ${PROJECT_SOURCE_DIR}/include/octomap/math/*.h)
install(FILES ${octomap_math_HDRS}	DESTINATION include/octomap/math)

# uninstall target
configure_file(
    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE octomap)
 
# Create a octomap-config.cmake file for the use from the build tree
set(OCTOMAP_INCLUDE_DIRS "${INCLUDE_DIRS}")
set(OCTOMAP_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
# not used right now (export depends?)
#set(OCTOMAP_CMAKE_DIR "${PROJECT_BINARY_DIR}")
configure_file(octomap-config.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-config.cmake" @ONLY)
configure_file(octomap-config-version.cmake.in
  "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-config-version.cmake" @ONLY)

# Install the export set for use with the install-tree
#install(EXPORT FooBarLibraryDepends DESTINATION
#  "${INSTALL_DATA_DIR}/FooBar/CMake"
#  COMPONENT dev)
 
# Create a octomap-config.cmake file for the use from the install tree
# and install it
set(OCTOMAP_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(OCTOMAP_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")
#set(OCTOMAP_CMAKE_DIR "${INSTALL_DATA_DIR}/FooBar/CMake")
configure_file(octomap-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/octomap-config.cmake" @ONLY)
configure_file(octomap-config-version.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/octomap-config-version.cmake" @ONLY)
install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/octomap-config.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/octomap-config-version.cmake" 
  DESTINATION lib/cmake/octomap/)

# Write pkgconfig-file:
include(InstallPkgConfigFile)
install_pkg_config_file(octomap
    CFLAGS
    LIBS -loctomap -loctomath
    REQUIRES
    VERSION ${OCTOMAP_MAJOR_VERSION}.${OCTOMAP_MINOR_VERSION}.${OCTOMAP_PATCH_VERSION})

# Documentation 
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/octomap.dox.in ${CMAKE_CURRENT_BINARY_DIR}/octomap.dox @ONLY)
  ADD_CUSTOM_TARGET(docs ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/octomap.dox
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating documentation (Doxygen)...")
ENDIF(DOXYGEN_FOUND)

# make package release from source (must be done on the SVN source tree)
IF (NOT WIN32)
	SET(OCTOMAP_PKG_NAME "${PROJECT_NAME}-${OCTOMAP_VERSION}.tar.gz")
	SET(DIST_DIR "${CMAKE_BINARY_DIR}/dist-${PROJECT_NAME}")
	ADD_CUSTOM_TARGET("dist-${PROJECT_NAME}" 
		rm -rf "${DIST_DIR}" "${CMAKE_BINARY_DIR}/${OCTOMAP_PKG_NAME}"
		COMMAND mkdir "${DIST_DIR}"
		COMMAND svn export --force -q "${PROJECT_SOURCE_DIR}" "${DIST_DIR}/${PROJECT_NAME}" 
		COMMAND tar -czf "${CMAKE_BINARY_DIR}/${OCTOMAP_PKG_NAME}" -C "${DIST_DIR}" --exclude=".hidden" "${PROJECT_NAME}"
			WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
		# cleanup so that there is no copy in the source dir
		COMMAND rm -rf "${DIST_DIR}"
	)
ENDIF()

# Needs to be last statement:
INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile octomap using: make")
MESSAGE (STATUS "Install octomap using: make install")
MESSAGE (STATUS "    (be sure to set the correct CMAKE_INSTALL_PREFIX before)")
MESSAGE (STATUS "Compile API-documentation using: make docs\n")
