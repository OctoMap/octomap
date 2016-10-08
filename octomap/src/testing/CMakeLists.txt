if(BUILD_TESTING)
  ADD_EXECUTABLE(test_raycasting test_raycasting.cpp)
  TARGET_LINK_LIBRARIES(test_raycasting octomap)

  ADD_EXECUTABLE(test_iterators test_iterators.cpp)
  TARGET_LINK_LIBRARIES(test_iterators octomap)

  ADD_EXECUTABLE(test_io test_io.cpp)
  TARGET_LINK_LIBRARIES(test_io octomap)

  ADD_EXECUTABLE(test_changedkeys test_changedkeys.cpp)
  TARGET_LINK_LIBRARIES(test_changedkeys octomap)

  ADD_EXECUTABLE(test_scans test_scans.cpp)
  TARGET_LINK_LIBRARIES(test_scans octomap)

  ADD_EXECUTABLE(test_color_tree test_color_tree.cpp)
  TARGET_LINK_LIBRARIES(test_color_tree octomap)

  ADD_EXECUTABLE(color_tree_histogram color_tree_histogram.cpp)
  TARGET_LINK_LIBRARIES(color_tree_histogram octomap)

  ADD_EXECUTABLE(test_mapcollection test_mapcollection.cpp)
  TARGET_LINK_LIBRARIES(test_mapcollection octomap octomath)

  ADD_EXECUTABLE(test_pruning test_pruning.cpp)
  TARGET_LINK_LIBRARIES(test_pruning octomap octomath)


  # CTest tests below

  ADD_EXECUTABLE(unit_tests unit_tests.cpp)
  TARGET_LINK_LIBRARIES(unit_tests octomap)

  ADD_TEST (NAME MathVector         COMMAND unit_tests MathVector     )
  ADD_TEST (NAME MathPose           COMMAND unit_tests MathPose       )
  ADD_TEST (NAME InsertRay          COMMAND unit_tests InsertRay      )
  ADD_TEST (NAME InsertScan         COMMAND unit_tests InsertScan     )
  ADD_TEST (NAME ReadGraph          COMMAND unit_tests ReadGraph      )
  ADD_TEST (NAME StampedTree        COMMAND unit_tests StampedTree    )
  ADD_TEST (NAME OcTreeKey          COMMAND unit_tests OcTreeKey      )
  ADD_TEST (NAME test_scans         COMMAND test_scans ${PROJECT_SOURCE_DIR}/share/data/spherical_scan.graph)
  ADD_TEST (NAME test_raycasting    COMMAND test_raycasting)
  ADD_TEST (NAME test_io            COMMAND test_io ${PROJECT_SOURCE_DIR}/share/data/geb079.bt)
  ADD_TEST (NAME test_pruning       COMMAND test_pruning )
  ADD_TEST (NAME test_iterators     COMMAND test_iterators ${PROJECT_SOURCE_DIR}/share/data/geb079.bt)
  ADD_TEST (NAME test_mapcollection COMMAND test_mapcollection ${PROJECT_SOURCE_DIR}/share/data/mapcoll.txt)
  ADD_TEST (NAME test_color_tree    COMMAND test_color_tree)
endif()
