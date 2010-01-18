# Qt4-support (more info: http://qtnode.net/wiki?title=Qt_with_cmake)
find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
# include the files enabled above
include(${QT_USE_FILE})


# sources for this project
set(viewer_SRCS
	main.cpp
	SceneObject.h
	TrajectoryDrawer.cpp
	PointcloudDrawer.cpp
	ViewerGui.cpp
	ViewerWidget.cpp
	ViewerSettings.cpp
	ViewerSettingsPanel.cpp
)

#class headers which need MOCs:
SET(viewer_MOC_HDRS
	ViewerGui.h
	ViewerWidget.h
	ViewerSettings.h
	ViewerSettingsPanel.h
)

# generate list of MOC srcs:
QT4_WRAP_CPP(viewer_MOC_SRCS ${viewer_MOC_HDRS})

# let cmake generate ui*.h files from .ui files (Qt Designer):
SET(viewer_UIS
    ViewerGui.ui
    ViewerSettings.ui
    ViewerSettingsPanel.ui
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

# Now add these generated files to the ADD_EXECUTABLE step
# If this is NOT done, then the ui_*.h files will not be generated
add_executable(octovis ${viewer_SRCS} ${viewer_UIS_H} ${viewer_MOC_SRCS} ${viewer_RES})

# link the target against the Qt libraries. which libraries exactly, is defined by the "include(${QT_USE_FILE})" line above, which sets up this variable.
target_link_libraries(octovis 
    ${QGLViewer_LIBRARIES}
	${QT_LIBRARIES}
	octomap 
)