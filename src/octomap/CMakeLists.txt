SET (octomap_SRCS
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeNodeLabeled.cpp
  )


ADD_LIBRARY( octomap ${octomap_SRCS})

TARGET_LINK_LIBRARIES(
  octomap octomath
  )

ADD_EXECUTABLE(test test.cpp)
TARGET_LINK_LIBRARIES(test octomap)

ADD_EXECUTABLE(read_pointcloud read_pointcloud.cpp)
TARGET_LINK_LIBRARIES(read_pointcloud octomap)

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap)

ADD_EXECUTABLE(offset_graph offset_graph.cpp)
TARGET_LINK_LIBRARIES(offset_graph octomap)

