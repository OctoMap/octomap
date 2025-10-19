find_path(
    QGLViewer_INCLUDE_DIR
    NAMES qglviewer.h
    PATH_SUFFIXES QGLViewer
)
find_library(
    QGLViewer_LIBRARY NAMES qglviewer-qt6 QGLViewer-qt6 QGLViewer2 QGLViewer
)
mark_as_advanced(QGLViewer_INCLUDE_DIR QGLViewer_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    QGLViewer6
    REQUIRED_VARS QGLViewer_LIBRARY QGLViewer_INCLUDE_DIR
    FAIL_MESSAGE "Could NOT find QGLViewer library"
)

if(QGLViewer6_FOUND AND NOT TARGET QGLViewer::QGLViewer)
    add_library(QGLViewer::QGLViewer UNKNOWN IMPORTED)
    set_target_properties(
        QGLViewer::QGLViewer
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${QGLViewer_INCLUDE_DIR}"
                   IMPORTED_LOCATION "${QGLViewer_LIBRARY}"
    )
endif()
