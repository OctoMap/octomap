cmake_minimum_required(VERSION 3.10...4.1)
project(
    octomap-distribution
    VERSION 1.10.0
    LANGUAGES NONE
)

include(CTest)

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
