set(QGLViewer_LIBRARY_CANDIDATES)
if(NOT DEFINED QGLViewer_USE_QT_VERSION OR QGLViewer_USE_QT_VERSION STREQUAL "6")
    list(APPEND QGLViewer_LIBRARY_CANDIDATES QGLViewer-qt6 qglviewer-qt6)
endif()
if(NOT DEFINED QGLViewer_USE_QT_VERSION OR QGLViewer_USE_QT_VERSION STREQUAL "5")
    list(APPEND QGLViewer_LIBRARY_CANDIDATES QGLViewer-qt5 qglviewer-qt5)
endif()
list(APPEND QGLViewer_LIBRARY_CANDIDATES QGLViewer2 QGLViewer)
find_library(
    QGLViewer_LIBRARY NAMES ${QGLViewer_LIBRARY_CANDIDATES}
)
find_path(
    QGLViewer_INCLUDE_DIR
    NAMES qglviewer.h
    PATH_SUFFIXES QGLViewer
)
mark_as_advanced(QGLViewer_INCLUDE_DIR QGLViewer_LIBRARY)

if(QGLViewer_LIBRARY MATCHES "-qt([56])")
    set(QGLViewer_QT_VERSION "${CMAKE_MATCH_1}")
else()
    set(QGLViewer_QT_VERSION "")
endif()
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    QGLViewer
    REQUIRED_VARS QGLViewer_LIBRARY QGLViewer_INCLUDE_DIR
    FAIL_MESSAGE "Could NOT find QGLViewer library"
)

if(QGLViewer_FOUND AND NOT TARGET QGLViewer::QGLViewer)
    add_library(QGLViewer::QGLViewer UNKNOWN IMPORTED)
    set_target_properties(
        QGLViewer::QGLViewer
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${QGLViewer_INCLUDE_DIR}"
                   IMPORTED_LOCATION "${QGLViewer_LIBRARY}"
    )
endif()
