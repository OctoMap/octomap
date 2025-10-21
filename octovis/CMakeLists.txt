cmake_minimum_required(VERSION 3.10...4.1)
project(
    octovis
    VERSION 1.10.0
    LANGUAGES CXX
)
if(NOT DEFINED OCTOVIS_BUILD_OPTIONAL)
    set(OCTOVIS_BUILD_OPTIONAL FALSE)
endif()

set(OCTOVIS_USE_QT_VERSION
    "auto"
    CACHE STRING "choose which Qt version to use"
)
set_property(CACHE OCTOVIS_USE_QT_VERSION PROPERTY STRINGS "auto" "5" "6")
message(
    STATUS "OCTOVIS_USE_QT_VERSION set to \"${OCTOVIS_USE_QT_VERSION}\""
)

set(OCTOVIS_USE_QGLVIEWER
    "auto"
    CACHE STRING "choose which QGLViewer library to use"
)
set_property(
    CACHE OCTOVIS_USE_QGLVIEWER PROPERTY STRINGS "auto" "installed" "vendored"
)
message(STATUS "OCTOVIS_USE_QGLVIEWER set to \"${OCTOVIS_USE_QGLVIEWER}\"")

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# version (e.g. for packaging)
set(OCTOVIS_MAJOR_VERSION ${PROJECT_VERSION_MAJOR})
set(OCTOVIS_MINOR_VERSION ${PROJECT_VERSION_MINOR})
set(OCTOVIS_PATCH_VERSION ${PROJECT_VERSION_PATCH})
set(OCTOVIS_VERSION ${PROJECT_VERSION})
set(OCTOVIS_SOVERSION "${OCTOVIS_MAJOR_VERSION}.${OCTOVIS_MINOR_VERSION}")

include(CTest)
include(GNUInstallDirs)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
include(CompilerSettings)

if (OCTOVIS_BUILD_OPTIONAL)
    set(_required)
else()
    set(_required REQUIRED)
endif()

# We need the main octomap library to link against (same version as octovis)
if(NOT TARGET octomap::octomap)
    find_package(octomap "${OCTOVIS_VERSION}" ${_required})
endif()
find_package(OpenGL ${_required})
set(OCTOVIS_FIND_DEPENDENCIES
    "include(CMakeFindDependencyMacro)\nfind_dependency(octomap ${OCTOVIS_VERSION})\nfind_dependency(OpenGL)\n"
)

# Looking for installed QGLViewer library
if(NOT OCTOVIS_USE_QT_VERSION STREQUAL "auto")
    set(QGLViewer_USE_QT_VERSION "${OCTOVIS_USE_QT_VERSION}")
else()
    unset(QGLViewer_USE_QT_VERSION)
endif()
if(OCTOVIS_USE_QGLVIEWER STREQUAL "installed")
    find_package(QGLViewer ${_required})
    if(NOT QGLViewer_FOUND)
        message(WARNING "No QGLViewer library found; skipping octovis build")
        return()
    endif()
elseif(OCTOVIS_USE_QGLVIEWER STREQUAL "auto")
    find_package(QGLViewer)
elseif(NOT OCTOVIS_USE_QGLVIEWER STREQUAL "vendored")
    message(
        FATAL_ERROR
            "OCTOVIS_USE_QGLVIEWER must be one of \"auto\", \"installed\", \"vendored\""
    )
endif()
if(QGLViewer_FOUND AND QGLViewer_QT_VERSION)
    set(OCTOVIS_QT_VERSION "${QGLViewer_QT_VERSION}")
elseif(NOT OCTOVIS_USE_QT_VERSION STREQUAL "auto")
    set(OCTOVIS_QT_VERSION "${OCTOVIS_USE_QT_VERSION}")
else()
    unset(OCTOVIS_QT_VERSION)
    set(_required)
endif()

if(NOT DEFINED OCTOVIS_QT_VERSION OR OCTOVIS_QT_VERSION STREQUAL "5")
    find_package(Qt5 ${_required} COMPONENTS Core Gui OpenGL Widgets Xml)
    if(Qt5_FOUND)
        set(OCTOVIS_FIND_DEPENDENCIES
            "${OCTOVIS_FIND_DEPENDENCIES}\nfind_dependency(Qt5 ${Qt5_VERSION} COMPONENTS Core Gui OpenGL Widgets Xml)"
        )
        set(OCTOVIS_QT_TARGETS Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::Widgets
                               Qt5::Xml
        )
        message(STATUS "octovis building with Qt ${Qt5_VERSION}")
        set(OCTOVIS_QT_VERSION "5")
    endif()
endif()
if(NOT DEFINED OCTOVIS_QT_VERSION OR OCTOVIS_QT_VERSION STREQUAL "6")
    find_package(Qt6 ${_required} COMPONENTS Core Gui OpenGLWidgets Widgets Xml)
    if(Qt6_FOUND)
        set(OCTOVIS_FIND_DEPENDENCIES
            "${OCTOVIS_FIND_DEPENDENCIES}\nfind_dependency(Qt6 ${Qt6_VERSION} COMPONENTS Core Gui OpenGLWidgets Widgets Xml)"
        )
        set(OCTOVIS_QT_TARGETS Qt6::Core Qt6::Gui Qt6::OpenGLWidgets
                               Qt6::Widgets Qt6::Xml
        )
        message(STATUS "octovis building with Qt ${Qt6_VERSION}")
        set(OCTOVIS_QT_VERSION "6")
    endif()
endif()
if(NOT OCTOVIS_QT_TARGETS)
    if(OCTOVIS_BUILD_OPTIONAL)
        message(WARNING "No Qt library found; skipping octovis build")
        return()
    else()
        message(FATAL_ERROR "No Qt library found")
    endif()
endif()

foreach(target IN ITEMS octomap::octomap OpenGL::GL OpenGL::GLU ${OCTOVIS_QT_TARGETS})
    if(NOT TARGET ${target})
        message(WARNING "Missing target ${target}; skipping octovis build")
        return()
    endif()
endforeach()

if(TARGET QGLViewer::QGLViewer)
    message(STATUS "octovis building with installed QGLViewer library")
    set(QGLVIEWER_TARGET QGLViewer::QGLViewer)
    set(OCTOVIS_FIND_DEPENDENCIES
        "${OCTOVIS_FIND_DEPENDENCIES}\nlist(APPEND CMAKE_MODULE_PATH \"\${CMAKE_CURRENT_LIST_DIR}/Modules\")\nset(QGLViewer_USE_QT_VERSION ${OCTOVIS_QT_VERSION})\nfind_dependency(QGLViewer)"
    )
    install(FILES CMakeModules/FindQGLViewer.cmake
            DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octovis/Modules"
    )
else()
    message(STATUS "octovis building with vendored QGLViewer library")
    set(qglviewer_SOURCES
        src/extern/QGLViewer/VRender/BackFaceCullingOptimizer.cpp
        src/extern/QGLViewer/VRender/BSPSortMethod.cpp
        src/extern/QGLViewer/VRender/EPSExporter.cpp
        src/extern/QGLViewer/VRender/Exporter.cpp
        src/extern/QGLViewer/VRender/FIGExporter.cpp
        src/extern/QGLViewer/VRender/gpc.cpp
        src/extern/QGLViewer/VRender/NVector3.cpp
        src/extern/QGLViewer/VRender/ParserGL.cpp
        src/extern/QGLViewer/VRender/Primitive.cpp
        src/extern/QGLViewer/VRender/PrimitivePositioning.cpp
        src/extern/QGLViewer/VRender/TopologicalSortMethod.cpp
        src/extern/QGLViewer/VRender/Vector2.cpp
        src/extern/QGLViewer/VRender/Vector3.cpp
        src/extern/QGLViewer/VRender/VisibilityOptimizer.cpp
        src/extern/QGLViewer/VRender/VRender.cpp
        src/extern/QGLViewer/camera.cpp
        src/extern/QGLViewer/constraint.cpp
        src/extern/QGLViewer/frame.cpp
        src/extern/QGLViewer/keyFrameInterpolator.cpp
        src/extern/QGLViewer/manipulatedCameraFrame.cpp
        src/extern/QGLViewer/manipulatedFrame.cpp
        src/extern/QGLViewer/mouseGrabber.cpp
        src/extern/QGLViewer/qglviewer.cpp
        src/extern/QGLViewer/quaternion.cpp
        src/extern/QGLViewer/saveSnapshot.cpp
        src/extern/QGLViewer/vec.cpp
    )
    add_library(QGLViewer ${qglviewer_SOURCES})
    target_include_directories(
        QGLViewer
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/extern/QGLViewer>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/octovis/qglviewer>
    )
    target_link_libraries(QGLViewer PUBLIC ${OCTOVIS_QT_TARGETS})
    set_target_properties(
        QGLViewer
        PROPERTIES VERSION "${OCTOVIS_VERSION}"
                   SOVERSION "${OCTOVIS_SOVERSION}"
                   OUTPUT_NAME "octovis_qglviewer"
                   AUTOMOC ON
                   AUTOUIC ON
    )
    install(
        DIRECTORY src/extern/QGLViewer/
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/octovis/qglviewer"
        FILES_MATCHING
        PATTERN "*.h"
    )
    install(TARGETS QGLViewer EXPORT octovis-targets)
    set(QGLVIEWER_TARGET QGLViewer)
    add_library(octomap::QGLViewer ALIAS QGLViewer)
endif()

# octovis library
set(octovis_SOURCES
    src/SceneObject.cpp src/PointcloudDrawer.cpp src/OcTreeDrawer.cpp
    src/SelectionBox.cpp src/TrajectoryDrawer.cpp src/ColorOcTreeDrawer.cpp
)
set(octovis_HEADERS
    include/octovis/CameraFollowMode.h
    include/octovis/ColorOcTreeDrawer.h
    include/octovis/OcTreeDrawer.h
    include/octovis/OcTreeRecord.h
    include/octovis/PointcloudDrawer.h
    include/octovis/SceneObject.h
    include/octovis/SelectionBox.h
    include/octovis/TrajectoryDrawer.h
    include/octovis/ViewerGui.h
    include/octovis/ViewerGui.ui
    include/octovis/ViewerSettings.h
    include/octovis/ViewerSettings.ui
    include/octovis/ViewerSettingsPanel.h
    include/octovis/ViewerSettingsPanel.ui
    include/octovis/ViewerSettingsPanelCamera.h
    include/octovis/ViewerSettingsPanelCamera.ui
    include/octovis/ViewerWidget.h
)
add_library(octovis ${octovis_HEADERS} ${octovis_SOURCES})
target_compile_features(octovis PUBLIC cxx_std_11)
target_include_directories(
    octovis
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/octovis_autogen/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(
    octovis PUBLIC octomap::octomap ${QGLVIEWER_TARGET} ${OCTOVIS_QT_TARGETS}
                   OpenGL::GL OpenGL::GLU
)
set_target_properties(
    octovis
    PROPERTIES VERSION "${OCTOVIS_VERSION}"
               SOVERSION "${OCTOVIS_SOVERSION}"
               AUTOMOC ON
               AUTOUIC ON
               AUTOGEN_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/octovis_autogen"
)
add_library(octomap::octovis ALIAS octovis)

# sources for viewer binary
set(octovis_viewer_SOURCES
    src/main.cpp
    src/ViewerGui.cpp
    src/ViewerWidget.cpp
    src/ViewerSettings.cpp
    src/ViewerSettingsPanel.cpp
    src/ViewerSettingsPanelCamera.cpp
    src/CameraFollowMode.cpp
)
add_executable(octovis_viewer ${octovis_viewer_SOURCES})
target_link_libraries(octovis_viewer PRIVATE octovis)
set_target_properties(octovis_viewer PROPERTIES OUTPUT_NAME "octovis")

# Export build tree
configure_package_config_file(
    octovis-config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/octovis-config.cmake"
    INSTALL_DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/octovis-config-version.cmake"
    VERSION ${OCTOVIS_VERSION}
    COMPATIBILITY SameMinorVersion
)
export(
    TARGETS octovis
    NAMESPACE octomap::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/octovis-targets.cmake"
)
if(TARGET QGLViewer)
    export(
        TARGETS QGLViewer
        NAMESPACE octomap::
        APPEND
        FILE "${CMAKE_CURRENT_BINARY_DIR}/octovis-targets.cmake"
    )
endif()
export(PACKAGE octovis)

# Installation
install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/octovis_autogen/include/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/octovis"
)
install(TARGETS octovis_viewer)
install(TARGETS octovis EXPORT octovis-targets)
install(
    EXPORT octovis-targets
    NAMESPACE octomap::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octovis"
)

configure_package_config_file(
    octovis-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octovis-config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octovis"
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octovis-config-version.cmake"
    VERSION ${OCTOVIS_VERSION}
    COMPATIBILITY SameMinorVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octovis-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/InstallFiles/octovis-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/octovis"
)

# ROS support
install(FILES package.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/octovis")
