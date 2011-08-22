
Octovis is distributed under the GPL license (see "octovis/LICENSE.txt").

It is based on qglviewer which also is distributed under the GPL license
(see "octovis/src/extern/QGLViewer/LICENSE")


LINUX
############################

You can build octovis independently of octomap by following
the these steps:

  cd octovis
  mkdir build
  cd build
  cmake ..
  make


WINDOWS
############################

The octomap viewer "octovis" is compatible with Visual Studio 2010
although this has not been tested in-depth. Feedback is welcome.

To meet the requirements, you need:

 - OpenGL
 - cmake (http://www.cmake.org)
 - QT development environment (http://qt.nokia.com/downloads)
   Download "Qt libraries for Windows (VS 20xx)"

To build the qglviewer library, open a windows shell (e.g., from 
the START-Menu -> QT), then go to octovis/src/extern/QGLViewer
and execute
 qmake -t vclib QGLViewer.pro -spec win32-msvc2010
 (ignore any warnings)

Load the generated file QGLViewer.vcxproj and build the project.
This will give you the files QGLViewer2.(dll,lib) that we will need.


The viewer should be build along with the rest of the octomap package.
These steps will create a solution file for the library and the viewer:

1. Start the cmake-gui and set the code directory to the octomap main directory.
2. Set the build directory to, e.g., /build
Press "Generate", select the appropriate generator, e. g. "Visual Studio 10".

This generates a solution file octomap-distribution.sln
Load this file and build the project.

When executing octovis.exe, Windows needs to find the following libraries,
so make sure they are on the PATH or in the same directory:
QGLViewer2.dll, QtOpenGL4.dll, QTGui4.dll, QTCore4.dll


