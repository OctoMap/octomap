SET (octomath_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


# Source code not set up for shared symbol export under Windows. Skip shared build.
IF(WIN32)
  set(SKIP_SHARED YES)
ENDIF(WIN32)

IF(NOT SKIP_SHARED)
  ADD_LIBRARY( octomath SHARED ${octomath_SRCS})

  SET_TARGET_PROPERTIES( octomath PROPERTIES
    VERSION ${OCTOMAP_VERSION}
    SOVERSION ${OCTOMAP_SOVERSION}
    INSTALL_NAME_DIR ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}   # this seems to be necessary for MacOS X
  )
  # INSTALL_NAME_DIR seems to be necessary for MacOS X
  
  set(OCTOMATH_TARGETS octomath octomath-static)
ELSE(NOT SKIP_SHARED)
  set(OCTOMATH_TARGETS octomath-static)
ENDIF(NOT SKIP_SHARED)

ADD_LIBRARY( octomath-static STATIC ${octomath_SRCS})
SET_TARGET_PROPERTIES(octomath-static PROPERTIES OUTPUT_NAME "octomath")
SET_TARGET_PROPERTIES(octomath-static PROPERTIES OUTPUT_NAME_DEBUG "octomathd")

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
endif()

export(TARGETS ${OCTOMATH_TARGETS}
  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-targets.cmake")

install(TARGETS ${OCTOMATH_TARGETS}
  EXPORT octomap-targets
  INCLUDES DESTINATION include
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
