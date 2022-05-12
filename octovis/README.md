Octomap - A probabilistic, flexible, and compact 3D mapping library for robotic systems
=======================================================================================

Authors: K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2013.
https://octomap.github.io

Octovis is a visualization tool and library for OctoMap.
It is distributed under the GPL license (see "LICENSE.txt").

Octovis is based on [QGLViewer](http://www.libqglviewer.com/), distributed under the 
[GPL license](src/extern/QGLViewer/LICENSE) with an [exception by the author]
(octovis/src/extern/QGLViewer/GPL_EXCEPTION).


LINUX
-----

You can build octovis independently of octomap by following
the these steps:

    cd octovis
    mkdir build
    cd build
    cmake ..
    make
  
  
You can manually set the location of the octomap library with the
`octomap_DIR` variable in CMake.

Note: If you get an error such as

> CMake Error at /usr/share/cmake-2.8/Modules/FindQt4.cmake:1148 (MESSAGE):
>  Qt qmake not found!

but you have Qt4 installed, this probably means that both Qt3 and Qt4
are installed. In Ubuntu this can be resolved by executing:

    sudo update-alternatives --config qmake
    
and choosing Qt4.


WINDOWS
-------

The octomap viewer **octovis** can be compiled and used under
Windows although this has not been tested in-depth. Feedback 
is welcome ("it works" is nice too :-))

To compile the library you need:

* OpenGL
* cmake (http://www.cmake.org)
* QT development environment (see below)


### MinGW ###

1. Download the MinGW distribution (http://www.mingw.org)
2. Install C++ compiler and add MingGW/bin to your system PATH
3. Download the QT library with MinGW support 
   (http://qt.nokia.com/downloads)
4. First build the GQLViewer library. Open a windows shell 
   (e.g., from the START-Menu -> QT) and execute:

        cd octovis/src/extern/QGLViewer
        qmake
        mingw32-make
   This will generate QGLViewer2.dll and libQGLViewer2.a

5. The viewer should be built along with the rest of the octomap package.
   From a shell execute: 

        cd octomap/build
        cmake -G "MinGW Makefiles" ..
        mingw32-make


### Microsoft Visual Studio 2010 ###

1. Download the QT library with Visual Studio 20xx support (currently 2008)
   (http://qt.nokia.com/downloads)
2. To build the qglviewer library
  - open a windows shell (e.g., from the START-Menu -> QT) 

            cd octovis/src/extern/QGLViewer
            qmake -t vclib QGLViewer.pro -spec win32-msvc2010    (ignore any warnings)
  - Load the generated file QGLViewer.vcxproj and build the project.
    This will give you the needed files QGLViewer2.(dll,lib).

3. The viewer should be built along with the rest of the octomap package.
   These steps will create a solution file for the library and the viewer:
    - Start the cmake-gui and set the code directory to the octomap main directory.
    - Set the build directory to, e.g., `/build`
    - Press "Generate", select the appropriate generator, e. g. "Visual Studio 10".
      This generates a solution file octomap-distribution.sln
    - Load this file and build the project.
  
Some more hints on compiling with Visual Studio (these may be necessary depending
on the VS version and CMake version):
* When compiling QGLViewer, modify the output path in "Properties->Linker->
  General->Output". Remove the "debug" and "release" prefix so the libs are 
  installed in the base dir.
* For the octivis-shared target, add the Qt lib path ("C:\path\to\Qt\4.7.2\lib") 
  to "Properties->Linker->General->Additional Library Directories", and add 
  the following Qt libs as dependencies in "Properties->Linker->Input->
  Additional Dependencies": QtCore4.lib, QtOpenGL4.lib, QtGui4.lib and 
  QtXml4.lib (and the debug versions of them to the Debug configuration)
* If the debug version of octovis throws this error: "QWidget: Must construct a 
  QApplication before a QPaintDevice", it is linking to the release version of 
  QGLViewer. Change the library dependency of octovis and octovis-shared to the 
  debug version QGLViewerd2.lib in "Properties->Linker->Input->Additional 
  Dependencies".
  

When executing octovis.exe, Windows needs to find the following 
libraries, so make sure they are on the PATH or in the same 
directory: QGLViewer2.dll, QtOpenGL4.dll, QTGui4.dll, QTCore4.dll


