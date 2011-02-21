SET (octomap_SRCS
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeNodeLabeled.cpp
  OcTreeLabeled.cpp
  OcTreeLUT.cpp
  )

ADD_LIBRARY( octomap ${octomap_SRCS})
set_target_properties(octomap PROPERTIES COMPILE_FLAGS "-fPIC")	

TARGET_LINK_LIBRARIES(
  octomap octomath
  )


ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap)

ADD_EXECUTABLE(log2graph log2graph.cpp)
TARGET_LINK_LIBRARIES(log2graph octomap)

ADD_EXECUTABLE(binvox2bt binvox2bt.cpp)
TARGET_LINK_LIBRARIES(binvox2bt octomap)

ADD_EXECUTABLE(bt2vrml bt2vrml.cpp)
TARGET_LINK_LIBRARIES(bt2vrml octomap)

ADD_EXECUTABLE(edit_octree edit_octree.cpp)
TARGET_LINK_LIBRARIES(edit_octree octomap)

ADD_EXECUTABLE(simple simple.cpp)
TARGET_LINK_LIBRARIES(simple octomap)

ADD_EXECUTABLE(labeled_demo labeled_demo.cpp)
TARGET_LINK_LIBRARIES(labeled_demo octomap)

ADD_EXECUTABLE(test_raycasting test_raycasting.cpp)
TARGET_LINK_LIBRARIES(test_raycasting octomap)

ADD_EXECUTABLE(test_scans test_scans.cpp)
TARGET_LINK_LIBRARIES(test_scans octomap)

# installation:
# store all header files to install:
# file(GLOB octomap_impl_HDRS *.h *.hxx *.hpp)
# install(FILES ${octomap_impl_HDRS} DESTINATION include/octomap)

install(TARGETS octomap 
	graph2tree
	log2graph
	binvox2bt
	bt2vrml
	test_raycasting
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

