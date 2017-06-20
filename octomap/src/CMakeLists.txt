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

# Source code not set up for shared symbol export under Windows. Skip shared build.
set(OCTOMAP_LINK octomap octomath)
IF(WIN32)
  set(SKIP_SHARED YES)
  set(OCTOMAP_LINK octomap-static octomath-static)
ENDIF(WIN32)

# dynamic and static libs, see CMake FAQ:
IF(NOT SKIP_SHARED)
  ADD_LIBRARY( octomap SHARED ${octomap_SRCS})
  set_target_properties( octomap PROPERTIES
    VERSION ${OCTOMAP_VERSION}
    SOVERSION ${OCTOMAP_SOVERSION}
  )

  TARGET_LINK_LIBRARIES(octomap octomath)

  set(OCTOMAP_TARGETS octomap octomap-static)
ELSE(NOT SKIP_SHARED)
  set(OCTOMAP_TARGETS octomap-static)
ENDIF(NOT SKIP_SHARED)
ADD_LIBRARY( octomap-static STATIC ${octomap_SRCS})
SET_TARGET_PROPERTIES(octomap-static PROPERTIES OUTPUT_NAME "octomap")
SET_TARGET_PROPERTIES(octomap-static PROPERTIES OUTPUT_NAME_DEBUG "octomapd")

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
endif()

export(TARGETS ${OCTOMAP_TARGETS}
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-targets.cmake")

ADD_SUBDIRECTORY( testing )

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree ${OCTOMAP_LINK})

ADD_EXECUTABLE(log2graph log2graph.cpp)
TARGET_LINK_LIBRARIES(log2graph ${OCTOMAP_LINK})

ADD_EXECUTABLE(binvox2bt binvox2bt.cpp)
TARGET_LINK_LIBRARIES(binvox2bt ${OCTOMAP_LINK})

ADD_EXECUTABLE(bt2vrml bt2vrml.cpp)
TARGET_LINK_LIBRARIES(bt2vrml ${OCTOMAP_LINK})

ADD_EXECUTABLE(edit_octree edit_octree.cpp)
TARGET_LINK_LIBRARIES(edit_octree ${OCTOMAP_LINK})

ADD_EXECUTABLE(convert_octree convert_octree.cpp)
TARGET_LINK_LIBRARIES(convert_octree ${OCTOMAP_LINK})

ADD_EXECUTABLE(eval_octree_accuracy eval_octree_accuracy.cpp)
TARGET_LINK_LIBRARIES(eval_octree_accuracy ${OCTOMAP_LINK})

ADD_EXECUTABLE(compare_octrees compare_octrees.cpp)
TARGET_LINK_LIBRARIES(compare_octrees ${OCTOMAP_LINK})

ADD_EXECUTABLE(simple_example simple_example.cpp)
TARGET_LINK_LIBRARIES(simple_example ${OCTOMAP_LINK})

ADD_EXECUTABLE(normals_example normals_example.cpp)
TARGET_LINK_LIBRARIES(normals_example ${OCTOMAP_LINK})

ADD_EXECUTABLE(intersection_example intersection_example.cpp)
TARGET_LINK_LIBRARIES(intersection_example ${OCTOMAP_LINK})

ADD_EXECUTABLE(octree2pointcloud octree2pointcloud.cpp)
TARGET_LINK_LIBRARIES(octree2pointcloud ${OCTOMAP_LINK})

install(TARGETS ${OCTOMAP_TARGETS}
  EXPORT octomap-targets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
install(EXPORT octomap-targets DESTINATION share/octomap/)

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

