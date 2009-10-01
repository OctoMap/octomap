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

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap)


