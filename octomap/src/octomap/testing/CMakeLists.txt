
ADD_EXECUTABLE(test_raycasting test_raycasting.cpp)
TARGET_LINK_LIBRARIES(test_raycasting octomap)

ADD_EXECUTABLE(test_iterators test_iterators.cpp)
TARGET_LINK_LIBRARIES(test_iterators octomap)

ADD_EXECUTABLE(test_scans test_scans.cpp)
TARGET_LINK_LIBRARIES(test_scans octomap)

ADD_EXECUTABLE(test_lut test_lut.cpp)
TARGET_LINK_LIBRARIES(test_lut octomap)

# CTest tests below

ADD_EXECUTABLE(unit_tests unit_tests.cpp)
TARGET_LINK_LIBRARIES(unit_tests octomap)

ADD_TEST (NAME MathVector  COMMAND unit_tests MathVector  )
ADD_TEST (NAME MathPose    COMMAND unit_tests MathPose    )
ADD_TEST (NAME InsertRay   COMMAND unit_tests InsertRay   )
ADD_TEST (NAME ReadTree    COMMAND unit_tests ReadTree    )
ADD_TEST (NAME CastRay     COMMAND unit_tests CastRay     )
ADD_TEST (NAME InsertScan  COMMAND unit_tests InsertScan  )
ADD_TEST (NAME ReadGraph   COMMAND unit_tests ReadGraph   )
ADD_TEST (NAME StampedTree COMMAND unit_tests StampedTree )

