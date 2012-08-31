SET( dynamicEDT3D_SRCS
   dynamicEDT3D.cpp
   dynamicEDTOctomap.cpp
   )

add_library(dynamicedt3d SHARED ${dynamicEDT3D_SRCS})
target_link_libraries(dynamicedt3d ${OCTOMAP_LIBRARIES})

add_library(dynamicedt3d-static STATIC ${dynamicEDT3D_SRCS})
target_link_libraries(dynamicedt3d-static ${OCTOMAP_LIBRARIES})

SET_TARGET_PROPERTIES(dynamicedt3d-static PROPERTIES OUTPUT_NAME "dynamicedt3d") 

ADD_SUBDIRECTORY(examples)

install(TARGETS 
  dynamicedt3d
  dynamicedt3d-static
  ${INSTALL_TARGETS_DEFAULT_ARGS})