SET (octomath_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


ADD_LIBRARY( octomath SHARED ${octomath_SRCS})
ADD_LIBRARY( octomath-static STATIC ${octomath_SRCS})
SET_TARGET_PROPERTIES(octomath-static PROPERTIES OUTPUT_NAME "octomath")
#SET_TARGET_PROPERTIES(octomath-static PROPERTIES PREFIX "lib")

install(TARGETS octomath octomath-static ${INSTALL_TARGETS_DEFAULT_ARGS})
 