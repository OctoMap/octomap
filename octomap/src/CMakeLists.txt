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

IF(__CUDA_SUPPORT__)
  SET (octomap_SRCS ${octomap_SRCS} OccupancyOcTreeBaseCuda.cu KeyContainerCuda.cu)
ENDIF(__CUDA_SUPPORT__)

# dynamic and static libs, see CMake FAQ:
ADD_LIBRARY( octomap SHARED ${octomap_SRCS})
set_target_properties( octomap PROPERTIES
  VERSION ${OCTOMAP_VERSION}
  SOVERSION ${OCTOMAP_SOVERSION}
  POSITION_INDEPENDENT_CODE ${OCTOMAP_POSITION_INDEPENDENT_CODE}
  CUDA_SEPARABLE_COMPILATION ON)
target_compile_features(octomap PUBLIC cxx_std_11)
ADD_LIBRARY( octomap-static STATIC ${octomap_SRCS})
SET_TARGET_PROPERTIES(
  octomap-static PROPERTIES 
  OUTPUT_NAME "octomap"
  POSITION_INDEPENDENT_CODE ${OCTOMAP_POSITION_INDEPENDENT_CODE}
  CUDA_SEPARABLE_COMPILATION ON)
target_compile_features(octomap-static PUBLIC cxx_std_11)
add_dependencies(octomap-static octomath-static)

TARGET_LINK_LIBRARIES(octomap octomath ${CUDA_LIBRARIES})
TARGET_LINK_LIBRARIES(octomap-static octomath ${CUDA_LIBRARIES})

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
endif()

export(TARGETS octomap octomap-static
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-targets.cmake")

ADD_SUBDIRECTORY( testing )

ADD_EXECUTABLE(graph2tree graph2tree.cpp)
TARGET_LINK_LIBRARIES(graph2tree octomap-static ${CUDA_LIBRARIES})
target_compile_features(graph2tree PUBLIC cxx_std_11)
SET_TARGET_PROPERTIES(graph2tree PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(log2graph log2graph.cpp)
#TARGET_LINK_LIBRARIES(log2graph octomap-static ${CUDA_LIBRARIES})
#target_compile_features(log2graph PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(log2graph PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(binvox2bt binvox2bt.cpp)
#TARGET_LINK_LIBRARIES(binvox2bt octomap-static ${CUDA_LIBRARIES})
#target_compile_features(binvox2bt PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(binvox2bt PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(bt2vrml bt2vrml.cpp)
#TARGET_LINK_LIBRARIES(bt2vrml octomap-static ${CUDA_LIBRARIES})
#target_compile_features(bt2vrml PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(bt2vrml PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(edit_octree edit_octree.cpp)
#TARGET_LINK_LIBRARIES(edit_octree octomap-static ${CUDA_LIBRARIES})
#target_compile_features(edit_octree PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(edit_octree PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(convert_octree convert_octree.cpp)
#TARGET_LINK_LIBRARIES(convert_octree octomap-static ${CUDA_LIBRARIES})
#target_compile_features(convert_octree PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(convert_octree PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(eval_octree_accuracy eval_octree_accuracy.cpp)
#TARGET_LINK_LIBRARIES(eval_octree_accuracy octomap-static ${CUDA_LIBRARIES})
#target_compile_features(eval_octree_accuracy PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(eval_octree_accuracy PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(compare_octrees compare_octrees.cpp)
#TARGET_LINK_LIBRARIES(compare_octrees octomap-static ${CUDA_LIBRARIES})
#target_compile_features(compare_octrees PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(compare_octrees PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(simple_example simple_example.cpp)
#TARGET_LINK_LIBRARIES(simple_example octomap-static ${CUDA_LIBRARIES})
#target_compile_features(simple_example PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(simple_example PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(normals_example normals_example.cpp)
#TARGET_LINK_LIBRARIES(normals_example octomap-static ${CUDA_LIBRARIES})
#target_compile_features(normals_example PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(normals_example PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(intersection_example intersection_example.cpp)
#TARGET_LINK_LIBRARIES(intersection_example octomap-static ${CUDA_LIBRARIES})
#target_compile_features(intersection_example PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(intersection_example PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

#ADD_EXECUTABLE(octree2pointcloud octree2pointcloud.cpp)
#TARGET_LINK_LIBRARIES(octree2pointcloud octomap-static ${CUDA_LIBRARIES})
#target_compile_features(octree2pointcloud PUBLIC cxx_std_11)
#SET_TARGET_PROPERTIES(octree2pointcloud PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

install(TARGETS octomap octomap-static
  EXPORT octomap-targets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
install(EXPORT octomap-targets DESTINATION share/octomap/)

install(TARGETS
	graph2tree
#	log2graph
	#binvox2bt
	#bt2vrml
	#edit_octree
	#convert_octree
	#eval_octree_accuracy
	#compare_octrees
	${INSTALL_TARGETS_DEFAULT_ARGS}
)

