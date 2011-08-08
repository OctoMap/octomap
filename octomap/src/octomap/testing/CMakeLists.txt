

ADD_EXECUTABLE(test_raycasting test_raycasting.cpp)
TARGET_LINK_LIBRARIES(test_raycasting octomap)

ADD_EXECUTABLE(test_iterators test_iterators.cpp)
TARGET_LINK_LIBRARIES(test_iterators octomap)

ADD_EXECUTABLE(test_scans test_scans.cpp)
TARGET_LINK_LIBRARIES(test_scans octomap)

ADD_EXECUTABLE(test_lut test_lut.cpp)
TARGET_LINK_LIBRARIES(test_lut octomap)

