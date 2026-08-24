cmake_minimum_required(VERSION 3.10...4.1)
project(
    octomap-distribution
    VERSION 1.10.0
    LANGUAGES NONE
)

include(CTest)

if(
    CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR
    AND NOT CMAKE_CONFIGURATION_TYPES
    AND NOT CMAKE_BUILD_TYPE
)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_OCTOVIS_SUBPROJECT "Build targets from subproject octovis" ON)
option(BUILD_DYNAMICETD3D_SUBPROJECT
       "Build targets from subproject dynamicEDT3D" ON
)

add_subdirectory(octomap)
if(BUILD_OCTOVIS_SUBPROJECT)
    set(OCTOVIS_BUILD_OPTIONAL TRUE)
    add_subdirectory(octovis)
endif()
if(BUILD_DYNAMICETD3D_SUBPROJECT)
    add_subdirectory(dynamicEDT3D)
endif()
