CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT( octomap-distribution )

ENABLE_TESTING()  # enable CTest environment of subprojects

ADD_SUBDIRECTORY( octomap )
ADD_SUBDIRECTORY( octovis )

# make complete package release from source 
IF (NOT WIN32)
	SET(PKG_NAME "${PROJECT_NAME}.tar.gz")
	SET(DIST_DIR "${CMAKE_BINARY_DIR}/dist")
	ADD_CUSTOM_TARGET("dist" 
	  rm -rf "${DIST_DIR}" "${CMAKE_BINARY_DIR}/${PKG_NAME}"
	  COMMAND mkdir "${DIST_DIR}"
	  COMMAND svn export --force -q "${PROJECT_SOURCE_DIR}" "${DIST_DIR}/${PROJECT_NAME}" 
	  COMMAND tar -czf "${CMAKE_BINARY_DIR}/${PKG_NAME}" -C "${DIST_DIR}" --exclude=".hidden" "${PROJECT_NAME}"
	    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
	)
	
	# also build subproject's dist for completeness
	ADD_DEPENDENCIES(dist dist-octomap dist-octovis)
ENDIF()
	


