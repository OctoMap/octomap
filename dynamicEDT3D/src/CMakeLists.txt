SET( dynamicEDT3D_SRCS
   dynamicEDT3D.cpp
   )

# Source code not set up for shared symbol export under Windows. Skip shared build.
IF(WIN32)
  set(SKIP_SHARED YES)
ENDIF(WIN32)

IF(NOT SKIP_SHARED)
  add_library(dynamicedt3d SHARED ${dynamicEDT3D_SRCS})
  set_target_properties(dynamicedt3d PROPERTIES
    VERSION ${DYNAMICEDT3D_VERSION}
    SOVERSION ${DYNAMICEDT3D_SOVERSION}
  )
  target_link_libraries(dynamicedt3d ${OCTOMAP_LIBRARIES})

  set(DYNAMICEDT3D_TARGETS dynamicedt3d dynamicedt3d-static)
ELSE(NOT SKIP_SHARED)
  set(DYNAMICEDT3D_TARGETS dynamicedt3d-static)
ENDIF(NOT SKIP_SHARED)

add_library(dynamicedt3d-static STATIC ${dynamicEDT3D_SRCS})
target_link_libraries(dynamicedt3d-static ${OCTOMAP_LIBRARIES})

SET_TARGET_PROPERTIES(dynamicedt3d-static PROPERTIES OUTPUT_NAME "dynamicedt3d") 
SET_TARGET_PROPERTIES(dynamicedt3d-static PROPERTIES OUTPUT_NAME_DEBUG "dynamicedt3dd") 

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D")
endif()

export(TARGETS ${DYNAMICEDT3D_TARGETS}
  FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/dynamicEDT3D/dynamicEDT3DTargets.cmake")

# directly depend on the octomap library target when building the
# complete distribution, so it it recompiled as needed
if (CMAKE_PROJECT_NAME STREQUAL "octomap-distribution")
  ADD_DEPENDENCIES(dynamicedt3d-static octomap-static)
  IF(NOT SKIP_SHARED)
    ADD_DEPENDENCIES(dynamicedt3d octomap)
  endif(NOT SKIP_SHARED)
endif()

ADD_SUBDIRECTORY(examples)

install(TARGETS ${DYNAMICEDT3D_TARGETS}
  EXPORT dynamicEDT3DTargets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
install(EXPORT dynamicEDT3DTargets DESTINATION share/dynamicEDT3D/)
