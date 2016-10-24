SET( dynamicEDT3D_SRCS
   dynamicEDT3D.cpp
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

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D")
endif()

export(TARGETS dynamicedt3d dynamicedt3d-static
  FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D/dynamicEDT3DTargets.cmake")

# directly depend on the octomap library target when building the
# complete distribution, so it it recompiled as needed
if (CMAKE_PROJECT_NAME STREQUAL "octomap-distribution")
  ADD_DEPENDENCIES(dynamicedt3d-static octomap-static)
  ADD_DEPENDENCIES(dynamicedt3d octomap)
endif()

ADD_SUBDIRECTORY(examples)

install(TARGETS dynamicedt3d dynamicedt3d-static
  EXPORT dynamicEDT3DTargets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
install(EXPORT dynamicEDT3DTargets DESTINATION share/dynamicEDT3D/)
