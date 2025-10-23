cmake_minimum_required(VERSION 3.10...4.1)
project(
    dynamicEDT3D
    VERSION 1.10.0
    LANGUAGES CXX
)

option(DYNAMICEDT3D_BUILD_EXAMPLES "Build examples" ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
include(CompilerSettings)
include(InstallPkgConfigFile)

# version (e.g. for packaging)
set(DYNAMICEDT3D_MAJOR_VERSION ${PROJECT_VERSION_MAJOR})
set(DYNAMICEDT3D_MINOR_VERSION ${PROJECT_VERSION_MINOR})
set(DYNAMICEDT3D_PATCH_VERSION ${PROJECT_VERSION_PATCH})
set(DYNAMICEDT3D_VERSION ${PROJECT_VERSION})
set(DYNAMICEDT3D_SOVERSION
    "${DYNAMICEDT3D_MAJOR_VERSION}.${DYNAMICEDT3D_MINOR_VERSION}"
)

if(NOT TARGET octomap::octomap)
    find_package(octomap "${DYNAMICEDT3D_VERSION}" REQUIRED)
endif()

set(DYNAMICEDT3D_FIND_DEPENDENCIES
    "include(CMakeFindDependencyMacro)\nfind_dependency(octomap ${DYNAMICEDT3D_VERSION})"
)

set(dynamicEDT3D_SOURCES src/dynamicEDT3D.cpp)

add_library(dynamicEDT3D ${dynamicEDT3D_SOURCES})
target_compile_features(dynamicEDT3D PUBLIC cxx_std_11)
target_include_directories(
    dynamicEDT3D PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(
    dynamicEDT3D
    PROPERTIES VERSION "${DYNAMICEDT3D_VERSION}"
               SOVERSION "${DYNAMICEDT3D_SOVERSION}"
               OUTPUT_NAME "dynamicedt3d"
)
target_link_libraries(dynamicEDT3D PUBLIC octomap::octomap)
add_library(octomap::dynamicEDT3D ALIAS dynamicEDT3D)

# Examples
if(DYNAMICEDT3D_BUILD_EXAMPLES)
    add_executable(exampleEDT3D src/examples/exampleEDT3D.cpp)
    target_link_libraries(exampleEDT3D PRIVATE dynamicEDT3D)

    add_executable(exampleEDTOctomap src/examples/exampleEDTOctomap.cpp)
    target_link_libraries(exampleEDTOctomap PRIVATE dynamicEDT3D)

    add_executable(
        exampleEDTOctomapStamped src/examples/exampleEDTOctomapStamped.cpp
    )
    target_link_libraries(exampleEDTOctomapStamped PRIVATE dynamicEDT3D)
endif()

# Export build tree
configure_package_config_file(
    dynamicedt3d-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/dynamicedt3d-config.cmake"
    INSTALL_DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/dynamicedt3d-config-version.cmake"
    VERSION ${DYNAMICEDT3D_VERSION}
    COMPATIBILITY SameMinorVersion
)
export(
    TARGETS dynamicEDT3D
    NAMESPACE octomap::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/dynamicedt3d-targets.cmake"
)
export(PACKAGE dynamicEDT3D)

# Installation
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS dynamicEDT3D EXPORT dynamicedt3d-targets)
install(
    EXPORT dynamicedt3d-targets
    NAMESPACE octomap::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dynamicEDT3D"
)
set(DYNAMICEDT3D_INCLUDE_DIRS "${CMAKE_INSTALL_INCLUDEDIR}")
set(DYNAMICEDT3D_LIBRARY_DIRS "${CMAKE_INSTALL_LIBDIR}")
configure_package_config_file(
    dynamicedt3d-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/dynamicedt3d-config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dynamicEDT3D"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/dynamicedt3d-config-version.cmake"
    VERSION ${DYNAMICEDT3D_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/dynamicedt3d-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/dynamicedt3d-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/dynamicEDT3D"
)

# ROS support
install(FILES package.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/dynamicEDT3D")

# pkg-config support
include(InstallPkgConfigFile)
install_pkg_config_file(
    dynamicEDT3D
    CFLAGS
    LIBS
    -ldynamicedt3d
    REQUIRES
    octomap
    VERSION
    ${DYNAMICEDT3D_VERSION}
)

# Documentation
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(
        docs_dynamicEDT3D
        COMMAND ${DOXYGEN_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/dynamicEDT3D.dox
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Generating documentation (Doxygen)"
    )
endif(DOXYGEN_FOUND)

# Needs to be last statement:
include(CPackSettings)
