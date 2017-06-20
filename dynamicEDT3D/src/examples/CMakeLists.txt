set(DYNAMICEDT3D_LINK dynamicedt3d)
IF(WIN32)
  # Windows build not setup for dynamic library export. Use static library.
  set(DYNAMICEDT3D_LINK dynamicedt3d-static)
ENDIF(WIN32)

add_executable(exampleEDT3D exampleEDT3D.cpp)
target_link_libraries(exampleEDT3D ${DYNAMICEDT3D_LINK})

add_executable(exampleEDTOctomap exampleEDTOctomap.cpp)
target_link_libraries(exampleEDTOctomap ${DYNAMICEDT3D_LINK})

add_executable(exampleEDTOctomapStamped exampleEDTOctomapStamped.cpp)
target_link_libraries(exampleEDTOctomapStamped ${DYNAMICEDT3D_LINK})