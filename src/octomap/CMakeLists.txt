SET (octomap_SRCS
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeNodeLabeled.cpp
  OcTreeLabeled.cpp
  )


ADD_LIBRARY( octomap ${octomap_SRCS})

TARGET_LINK_LIBRARIES(
  octomap octomath
  )

ADD_EXECUTABLE(read_pointcloud read_pointcloud.cpp)
TARGET_LINK_LIBRARIES(read_pointcloud octomap)

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap)

ADD_EXECUTABLE(log2graph log2graph.cpp)
TARGET_LINK_LIBRARIES(log2graph octomap)

ADD_EXECUTABLE(binvox2bt binvox2bt.cpp)
TARGET_LINK_LIBRARIES(binvox2bt octomap)

ADD_EXECUTABLE(simple simple.cpp)
TARGET_LINK_LIBRARIES(simple octomap)

ADD_EXECUTABLE(labeled_demo labeled_demo.cpp)
TARGET_LINK_LIBRARIES(labeled_demo octomap)

