# Qt4-support (more info: http://qtnode.net/wiki?title=Qt_with_cmake)
find_package(Qt4 REQUIRED)

# the next line sets up include and link directories and defines some variables that we will use.
# you can modify the behavior by setting some variables, e.g.
#   set(QT_USE_QTOPENGL TRUE)
# -> this will cause cmake to include and link against the OpenGL module
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTXML TRUE)
# include the files enabled above
include(${QT_USE_FILE})



# Boost (smart ptr)
find_package(Boost COMPONENTS smart_ptr)
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )


# sources for this project
set(viewer_SRCS
	main.cpp
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

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/octomap
  ${CMAKE_SOURCE_DIR}/extern	# for local copy of libQGLViewer
)

LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/extern/QGLViewer)	# for local copy of libQGLViewer

# Now add these generated files to the ADD_EXECUTABLE step
# If this is NOT done, then the ui_*.h files will not be generated
add_executable(viewer3d ${viewer_SRCS} ${viewer_UIS_H} ${viewer_MOC_SRCS} ${viewer_RES})

add_dependencies(viewer3d qglviewer)

# link the target against the Qt libraries. which libraries exactly, is defined by the "include(${QT_USE_FILE})" line above, which sets up this variable.
target_link_libraries(viewer3d 
	QGLViewer
	${QT_LIBRARIES}
	${Boost_LIBRARIES}
	octomap 
)