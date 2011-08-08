
ADD_EXECUTABLE(test_raycasting test_raycasting.cpp)
TARGET_LINK_LIBRARIES(test_raycasting octomap)

ADD_EXECUTABLE(test_iterators test_iterators.cpp)
TARGET_LINK_LIBRARIES(test_iterators octomap)

ADD_EXECUTABLE(test_scans test_scans.cpp)
TARGET_LINK_LIBRARIES(test_scans octomap)

ADD_EXECUTABLE(test_lut test_lut.cpp)
TARGET_LINK_LIBRARIES(test_lut octomap)

ADD_EXECUTABLE(test_math test_math.cpp)
TARGET_LINK_LIBRARIES(test_math octomap)

ADD_EXECUTABLE(test_octomap test_octomap.cpp)
TARGET_LINK_LIBRARIES(test_octomap octomap)

ADD_TEST(
  NAME TestMath
  COMMAND test_math
)

ADD_TEST(
  NAME TestOctomap
  COMMAND test_octomap
)

# ADD_TEST(
#   NAME TestRayCast 
#   COMMAND test_raycasting
# )
