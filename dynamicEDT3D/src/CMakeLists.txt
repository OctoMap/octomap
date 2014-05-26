SET( dynamicEDT3D_SRCS
   dynamicEDT3D.cpp
   dynamicEDTOctomap.cpp
   )

add_library(dynamicedt3d SHARED ${dynamicEDT3D_SRCS})
set_target_properties(dynamicedt3d PROPERTIES
  VERSION ${DYNAMICEDT3D_VERSION}
  SOVERSION ${DYNAMICEDT3D_SOVERSION}
)
target_link_libraries(dynamicedt3d ${OCTOMAP_LIBRARIES})

add_library(dynamicedt3d-static STATIC ${dynamicEDT3D_SRCS})
target_link_libraries(dynamicedt3d-static ${OCTOMAP_LIBRARIES})

SET_TARGET_PROPERTIES(dynamicedt3d-static PROPERTIES OUTPUT_NAME "dynamicedt3d") 

# directly depend on the octomap library target when building the
# complete distribution, so it it recompiled as needed
if (CMAKE_PROJECT_NAME STREQUAL "octomap-distribution")
  ADD_DEPENDENCIES(dynamicedt3d-static octomap-static)
  ADD_DEPENDENCIES(dynamicedt3d octomap)
endif()

ADD_SUBDIRECTORY(examples)

install(TARGETS 
  dynamicedt3d
  dynamicedt3d-static
  ${INSTALL_TARGETS_DEFAULT_ARGS})