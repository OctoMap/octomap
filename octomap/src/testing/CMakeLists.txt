add_executable(test_raycasting test_raycasting.cpp)
target_link_libraries(test_raycasting octomap)

add_executable(test_iterators test_iterators.cpp)
target_link_libraries(test_iterators octomap)

add_executable(test_io test_io.cpp)
target_link_libraries(test_io octomap)

add_executable(test_changedkeys test_changedkeys.cpp)
target_link_libraries(test_changedkeys octomap)

add_executable(test_scans test_scans.cpp)
target_link_libraries(test_scans octomap)

add_executable(test_color_tree test_color_tree.cpp)
target_link_libraries(test_color_tree octomap)

add_executable(color_tree_histogram color_tree_histogram.cpp)
target_link_libraries(color_tree_histogram octomap)

add_executable(test_mapcollection test_mapcollection.cpp)
target_link_libraries(test_mapcollection octomap octomath)

add_executable(test_pruning test_pruning.cpp)
target_link_libraries(test_pruning octomap octomath)

add_executable(test_bbx test_bbx.cpp)
target_link_libraries(test_bbx octomap)

# CTest tests below

add_executable(unit_tests unit_tests.cpp)
target_link_libraries(unit_tests octomap)

add_test(NAME MathVector COMMAND unit_tests MathVector)
add_test(NAME MathPose COMMAND unit_tests MathPose)
add_test(NAME InsertRay COMMAND unit_tests InsertRay)
add_test(NAME InsertScan COMMAND unit_tests InsertScan)
add_test(NAME ReadGraph COMMAND unit_tests ReadGraph)
add_test(NAME StampedTree COMMAND unit_tests StampedTree)
add_test(NAME OcTreeKey COMMAND unit_tests OcTreeKey)
add_test(NAME OcTreeStructure COMMAND unit_tests OcTreeStructure)
add_test(NAME test_scans
         COMMAND test_scans
                 ${PROJECT_SOURCE_DIR}/share/data/spherical_scan.graph
)
add_test(NAME test_raycasting COMMAND test_raycasting)
add_test(NAME test_io COMMAND test_io
                              ${PROJECT_SOURCE_DIR}/share/data/geb079.bt
)
add_test(NAME test_pruning COMMAND test_pruning)
add_test(NAME test_iterators COMMAND test_iterators
                                     ${PROJECT_SOURCE_DIR}/share/data/geb079.bt
)
add_test(NAME test_mapcollection
         COMMAND test_mapcollection
                 ${PROJECT_SOURCE_DIR}/share/data/mapcoll.txt
)
add_test(NAME test_color_tree COMMAND test_color_tree)
add_test(NAME test_bbx COMMAND test_bbx)

set_tests_properties(ReadGraph PROPERTIES DEPENDS InsertScan)
