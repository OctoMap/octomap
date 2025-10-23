cmake_minimum_required(VERSION 3.10...4.1)
project(
    octomap
    VERSION 1.10.0
    LANGUAGES CXX
)

option(OCTOMAP_USE_OPENMP "Enable experimental OpenMP support" OFF)

# version (e.g. for packaging)
set(OCTOMAP_MAJOR_VERSION ${PROJECT_VERSION_MAJOR})
set(OCTOMAP_MINOR_VERSION ${PROJECT_VERSION_MINOR})
set(OCTOMAP_PATCH_VERSION ${PROJECT_VERSION_PATCH})
set(OCTOMAP_VERSION ${PROJECT_VERSION})
set(OCTOMAP_SOVERSION "${OCTOMAP_MAJOR_VERSION}.${OCTOMAP_MINOR_VERSION}")

include(CMakePackageConfigHelpers)
include(CTest)
include(GNUInstallDirs)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
include(CompilerSettings)
include(InstallPkgConfigFile)

set(OCTOMAP_FIND_DEPENDENCIES "include(CMakeFindDependencyMacro)")
if(OCTOMAP_USE_OPENMP)
    find_package(OpenMP REQUIRED)
    set(OCTOMAP_FIND_DEPENDENCIES
        "${OCTOMAP_FIND_DEPENDENCIES}\nfind_dependency(OpenMP)"
    )
endif()

# octomath library
set(octomath_SOURCES src/math/Pose6D.cpp src/math/Quaternion.cpp
                     src/math/Vector3.cpp
)
add_library(octomath ${octomath_SOURCES})
target_compile_features(octomath PUBLIC cxx_std_11)
target_include_directories(
    octomath PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(
    octomath PROPERTIES VERSION "${OCTOMAP_VERSION}" SOVERSION
                                                     "${OCTOMAP_SOVERSION}"
)
if(TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(octomath PRIVATE OpenMP::OpenMP_CXX)
endif()

# octomap library
set(octomap_SOURCES
    src/AbstractOcTree.cpp
    src/AbstractOccupancyOcTree.cpp
    src/Pointcloud.cpp
    src/ScanGraph.cpp
    src/CountingOcTree.cpp
    src/OcTree.cpp
    src/OcTreeNode.cpp
    src/OcTreeStamped.cpp
    src/ColorOcTree.cpp
)
add_library(octomap ${octomap_SOURCES})
target_compile_features(octomap PUBLIC cxx_std_11)
target_include_directories(
    octomap PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(
    octomap PROPERTIES VERSION "${OCTOMAP_VERSION}" SOVERSION
                                                    "${OCTOMAP_SOVERSION}"
)
target_link_libraries(octomap PUBLIC octomath)
if(TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(octomap PRIVATE OpenMP::OpenMP_CXX)
endif()
add_library(octomap::octomap ALIAS octomap)

# Utilities
add_executable(graph2tree src/graph2tree.cpp)
target_link_libraries(graph2tree PRIVATE octomap)

add_executable(log2graph src/log2graph.cpp)
target_link_libraries(log2graph PRIVATE octomap)

add_executable(binvox2bt src/binvox2bt.cpp)
target_link_libraries(binvox2bt PRIVATE octomap)

add_executable(binvox2bt_unique_offsets src/binvox2bt_unique_offsets.cpp)
target_link_libraries(binvox2bt_unique_offsets PRIVATE octomap)

add_executable(bt2vrml src/bt2vrml.cpp)
target_link_libraries(bt2vrml PRIVATE octomap)

add_executable(edit_octree src/edit_octree.cpp)
target_link_libraries(edit_octree PRIVATE octomap)

add_executable(convert_octree src/convert_octree.cpp)
target_link_libraries(convert_octree PRIVATE octomap)

add_executable(eval_octree_accuracy src/eval_octree_accuracy.cpp)
target_link_libraries(eval_octree_accuracy PRIVATE octomap)

add_executable(compare_octrees src/compare_octrees.cpp)
target_link_libraries(compare_octrees PRIVATE octomap)

add_executable(simple_example src/simple_example.cpp)
target_link_libraries(simple_example PRIVATE octomap)

add_executable(pcd_to_bt_example src/pcd_to_bt_example.cpp)
target_link_libraries(pcd_to_bt_example PRIVATE octomap)

add_executable(normals_example src/normals_example.cpp)
target_link_libraries(normals_example PRIVATE octomap)

add_executable(intersection_example src/intersection_example.cpp)
target_link_libraries(intersection_example PRIVATE octomap)

add_executable(octree2pointcloud src/octree2pointcloud.cpp)
target_link_libraries(octree2pointcloud PRIVATE octomap)

add_executable(merge_octomaps src/merge_octomaps.cpp)
target_link_libraries(merge_octomaps PRIVATE octomap)

# Testing
if(BUILD_TESTING)
    add_subdirectory(src/testing)
endif()

# Export build tree
configure_package_config_file(
    octomap-config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/octomap-config.cmake"
    INSTALL_DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/octomap-config-version.cmake"
    VERSION ${OCTOMAP_VERSION}
    COMPATIBILITY SameMinorVersion
)
export(
    TARGETS octomath octomap
    NAMESPACE octomap::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/octomap-targets.cmake"
)
export(PACKAGE octomap)

# Installation
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS octomath octomap EXPORT octomap-targets)
install(
    TARGETS graph2tree
            log2graph
            binvox2bt
            binvox2bt_unique_offsets
            bt2vrml
            edit_octree
            convert_octree
            eval_octree_accuracy
            compare_octrees
)
install(
    EXPORT octomap-targets
    NAMESPACE octomap::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octomap"
)

configure_package_config_file(
    octomap-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octomap-config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octomap"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octomap-config-version.cmake"
    VERSION ${OCTOMAP_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octomap-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octomap-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octomap"
)

# ROS support
install(FILES package.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/octomap")

# pkg-config support
install_pkg_config_file(
    octomap
    CFLAGS
    LIBS
    -loctomap
    -loctomath
    REQUIRES
    VERSION
    ${OCTOMAP_VERSION}
)

# Documentation
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/octomap.dox.in
        ${CMAKE_CURRENT_BINARY_DIR}/octomap.dox @ONLY
    )
    add_custom_target(
        docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/octomap.dox
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT
            "Generating documentation (Doxygen) at location ${PROJECT_SOURCE_DIR}/doc/html/"
    )
endif(DOXYGEN_FOUND)

# Needs to be last statement:
include(CPackSettings)
