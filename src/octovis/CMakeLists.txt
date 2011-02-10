# Qt4-support (more info: http://qtnode.net/wiki?title=Qt_with_cmake)
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
# include the files enabled above
include(${QT_USE_FILE})

# Mac OS X seems to require special linker flags:
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework OpenGL")
  SET (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -framework OpenGL")
  SET (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -framework OpenGL")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# sources for octovis library
set(viewerlib_SRCS
	SceneObject.cpp
	PointcloudDrawer.cpp
	OcTreeDrawer.cpp
	TrajectoryDrawer.cpp
)

# sources for viewer binary
set(viewer_SRCS
	main.cpp	
	ViewerGui.cpp
	ViewerWidget.cpp
	ViewerSettings.cpp
	ViewerSettingsPanel.cpp
	ViewerSettingsPanelCamera.cpp
	CameraFollowMode.cpp
)	

#class headers which need MOCs:
SET(viewer_MOC_HDRS
	ViewerGui.h
	ViewerWidget.h
	ViewerSettings.h
	ViewerSettingsPanel.h
	ViewerSettingsPanelCamera.h
	CameraFollowMode.h
)

# generate list of MOC srcs:
QT4_WRAP_CPP(viewer_MOC_SRCS ${viewer_MOC_HDRS})

# let cmake generate ui*.h files from .ui files (Qt Designer):
SET(viewer_UIS
    ViewerGui.ui
    ViewerSettings.ui
    ViewerSettingsPanel.ui
    ViewerSettingsPanelCamera.ui
)
QT4_WRAP_UI(viewer_UIS_H ${viewer_UIS})

# Don't forget to include output directory, otherwise
# the UI file won't be wrapped!
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_BINARY_DIR})

# Resource files (icons, ...)
QT4_ADD_RESOURCES(viewer_RES icons.qrc)

#found QGLViewer lib dir
link_directories(${QGLViewer_LIBRARY_DIR})

INCLUDE_DIRECTORIES(
  ${QGLViewer_INCLUDE_DIR}
)

# Library target 
add_library(octovis_library ${viewerlib_SRCS})
target_link_libraries(octovis_library)
set_target_properties(octovis_library PROPERTIES COMPILE_FLAGS "-fPIC")	
set_target_properties(octovis_library PROPERTIES OUTPUT_NAME octovis)

# Now add these generated files to the ADD_EXECUTABLE step
# If this is NOT done, then the ui_*.h files will not be generated
add_executable(octovis ${viewer_SRCS} ${viewer_UIS_H} ${viewer_MOC_SRCS} ${viewer_RES})

# link the target against the Qt libraries. which libraries exactly, is defined by the "include(${QT_USE_FILE})" line above, which sets up this variable.
target_link_libraries(octovis 
    ${QGLViewer_LIBRARIES}
	${QT_LIBRARIES}
	octomap 
	octovis_library
)

# special handling of MacOS X:
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_custom_command(TARGET octovis POST_BUILD 
  	COMMAND install_name_tool -change libQGLViewer.2.dylib /opt/local/lib/libQGLViewer.2.dylib ${BASE_DIR}/bin/octovis
  )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")




# #installation:
# # store all header files to install:
file(GLOB octovis_HDRS  *.h *.hxx *.hpp)
install(FILES ${octovis_HDRS} DESTINATION include/octovis)

install(TARGETS octovis
  octovis_library
  ${INSTALL_TARGETS_DEFAULT_ARGS}
)
