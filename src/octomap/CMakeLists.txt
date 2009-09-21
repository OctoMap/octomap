SET (octomap_SRCS
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  )


ADD_LIBRARY( octomap ${octomap_SRCS})

TARGET_LINK_LIBRARIES(
  octomap octomath
  )

# ADD_EXECUTABLE(log2vrml log2vrml.cpp)
# TARGET_LINK_LIBRARIES(log2vrml octomap)

# ADD_EXECUTABLE(log2pc log2pc.cpp)
# TARGET_LINK_LIBRARIES(log2pc octomap)

# ADD_EXECUTABLE(pc2vrml pc2vrml.cpp)
# TARGET_LINK_LIBRARIES(pc2vrml octomap)

# ADD_EXECUTABLE(log2graph log2graph.cpp)
# TARGET_LINK_LIBRARIES(log2graph octomap)

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap)


