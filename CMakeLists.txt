CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap-distribution )

ENABLE_TESTING()  # enable CTest environment of subprojects

option(BUILD_OCTOVIS_SUBPROJECT "Build targets from subproject octovis" ON)
option(BUILD_DYNAMICETD3D_SUBPROJECT  "Build targets from subproject dynamicEDT3D" ON)

ADD_SUBDIRECTORY( octomap )

if(BUILD_OCTOVIS_SUBPROJECT)
	ADD_SUBDIRECTORY( octovis )
endif()		

if(BUILD_DYNAMICETD3D_SUBPROJECT)	
	ADD_SUBDIRECTORY( dynamicEDT3D )
endif()		


