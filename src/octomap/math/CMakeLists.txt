SET (octomath_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


ADD_LIBRARY( octomath ${octomath_SRCS})
set_target_properties(octomath PROPERTIES COMPILE_FLAGS "-fPIC")	

# no more libs to link from here:
#TARGET_LINK_LIBRARIES(
#  octomath
#)

# store all header files to install:
file(GLOB octomath_HDRS *.h *.hxx *.hpp)

install(FILES ${octomath_HDRS}	DESTINATION include/octomath)
	
install(TARGETS octomath ${INSTALL_TARGETS_DEFAULT_ARGS})
