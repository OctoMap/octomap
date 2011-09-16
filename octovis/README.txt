
Octovis is distributed under the GPL license (see "octovis/LICENSE.txt").

It is based on qglviewer which also is distributed under the GPL license
(see "octovis/src/extern/QGLViewer/LICENSE" and the author's exception 
   "octovis/src/extern/QGLViewer/GPL_EXCEPTION")


LINUX
############################

You can build octovis independently of octomap by following
the these steps:

  cd octovis
  mkdir build
  cd build
  cmake ..
  make


Note: If you get an error such as

"CMake Error at /usr/share/cmake-2.8/Modules/FindQt4.cmake:1148 (MESSAGE):
  Qt qmake not found!"

but you have Qt4 installed, this probably means that both Qt3 and Qt4
are installed. In Ubuntu this can be resolved using:
$ sudo update-alternatives --config qmake"


WINDOWS
############################

The octomap viewer "octovis" can be compiled and used under
Windows although this has not been tested in-depth. Feedback 
is welcome ("it works" is nice too :-)

To compile the library you need:

 - OpenGL
 - cmake (http://www.cmake.org)
 - QT development environment (see below)


MinGW
------------------------------

* Download the MinGW distribution (http://www.mingw.org)
* Install C++ compiler and add MingGW/bin to your system PATH

* Download the QT library with MinGW support 
  (http://qt.nokia.com/downloads)

* First build the GQLViewer library
   - Open a windows shell (e.g., from the START-Menu -> QT)
   - cd octovis/src/extern/QGLViewer
   - qmake
     (I had to fix a small bug in the Makefile)
   - mingw32-make
   - This will generate QGLViewer2.dll and libQGLViewer2.a

The viewer should be build along with the rest of the octomap package:

* from a shell execute: 
   - cd octomap/build
   - cmake -G "MinGW Makefiles" ..
   - mingw32-make


Microsoft Visual Studio 2010
------------------------------

* Download the QT library with Visual Studio 20xx support (currently 2008)
  (http://qt.nokia.com/downloads)

* To build the qglviewer library
  - open a windows shell (e.g., from the START-Menu -> QT) 
  - cd octovis/src/extern/QGLViewer
  - qmake -t vclib QGLViewer.pro -spec win32-msvc2010
    (ignore any warnings)
  - Load the generated file QGLViewer.vcxproj and build the project.
    This will give you the files QGLViewer2.(dll,lib) that we will need.

The viewer should be build along with the rest of the octomap package.
These steps will create a solution file for the library and the viewer:

* Start the cmake-gui and set the code directory to the octomap main directory.
* Set the build directory to, e.g., /build
* Press "Generate", select the appropriate generator, e. g. "Visual Studio 10".
* This generates a solution file octomap-distribution.sln
  Load this file and build the project.


When executing octovis.exe, Windows needs to find the following 
libraries, so make sure they are on the PATH or in the same 
directory: QGLViewer2.dll, QtOpenGL4.dll, QTGui4.dll, QTCore4.dll


