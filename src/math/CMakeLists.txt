SET (octomath_SRCS
  Vector.cpp
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


ADD_LIBRARY( octomath ${octomath_SRCS})

TARGET_LINK_LIBRARIES(
  octomath gsl cblas
)

