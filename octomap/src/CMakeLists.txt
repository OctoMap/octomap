SET (octomap_SRCS
  AbstractOcTree.cpp
  AbstractOccupancyOcTree.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeStamped.cpp
  ColorOcTree.cpp
  )

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( octomap SHARED ${octomap_SRCS})
set_target_properties( octomap PROPERTIES
  VERSION ${OCTOMAP_VERSION}
  SOVERSION ${OCTOMAP_SOVERSION}
)
ADD_LIBRARY( octomap-static STATIC ${octomap_SRCS})
SET_TARGET_PROPERTIES(octomap-static PROPERTIES OUTPUT_NAME "octomap") 
add_dependencies(octomap-static octomath-static)

TARGET_LINK_LIBRARIES(octomap octomath)

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
endif()

export(TARGETS octomap octomap-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-targets.cmake")

ADD_SUBDIRECTORY( testing )

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

ADD_EXECUTABLE(convert_octree convert_octree.cpp)
TARGET_LINK_LIBRARIES(convert_octree octomap)

ADD_EXECUTABLE(eval_octree_accuracy eval_octree_accuracy.cpp)
TARGET_LINK_LIBRARIES(eval_octree_accuracy octomap)

ADD_EXECUTABLE(compare_octrees compare_octrees.cpp)
TARGET_LINK_LIBRARIES(compare_octrees octomap)

ADD_EXECUTABLE(simple_example simple_example.cpp)
TARGET_LINK_LIBRARIES(simple_example octomap)

ADD_EXECUTABLE(normals_example normals_example.cpp)
TARGET_LINK_LIBRARIES(normals_example octomap)

ADD_EXECUTABLE(intersection_example intersection_example.cpp)
TARGET_LINK_LIBRARIES(intersection_example octomap)

ADD_EXECUTABLE(octree2pointcloud octree2pointcloud.cpp)
TARGET_LINK_LIBRARIES(octree2pointcloud octomap)

install(TARGETS octomap octomap-static
  EXPORT octomap-targets
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
install(EXPORT octomap-targets DESTINATION "${CMAKE_INSTALL_DATADIR}/octomap")

install(TARGETS
	graph2tree
	log2graph
	binvox2bt
	bt2vrml
	edit_octree
	convert_octree
	eval_octree_accuracy
	compare_octrees
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

