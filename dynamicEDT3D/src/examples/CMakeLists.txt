add_executable(exampleEDT3D exampleEDT3D.cpp)
target_link_libraries(exampleEDT3D dynamicEDT3D)

add_executable(exampleEDTOctomap exampleEDTOctomap.cpp)
target_link_libraries(exampleEDTOctomap dynamicEDT3D ${OCTOMAP_LIBRARIES})
