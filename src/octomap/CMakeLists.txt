SET (mapping3d_SRCS
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  )


ADD_LIBRARY( mapping3d ${mapping3d_SRCS})

TARGET_LINK_LIBRARIES(
  mapping3d pthread
  )

# ADD_EXECUTABLE(log2vrml log2vrml.cpp)
# TARGET_LINK_LIBRARIES(log2vrml mapping3d)

# ADD_EXECUTABLE(log2pc log2pc.cpp)
# TARGET_LINK_LIBRARIES(log2pc mapping3d)

# ADD_EXECUTABLE(pc2vrml pc2vrml.cpp)
# TARGET_LINK_LIBRARIES(pc2vrml mapping3d)

# ADD_EXECUTABLE(log2graph log2graph.cpp)
# TARGET_LINK_LIBRARIES(log2graph mapping3d)

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree mapping3d)


