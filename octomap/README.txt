
Octomap
- A probabilistic, flexible, and compact 3D mapping library for robotic systems.

Authors: Kai M. Wurm and Armin Hornung, University of Freiburg, Copyright (C) 2009-2012.
http://octomap.sourceforge.net/

Further Contributors:
C. Sprunk, University of Freiburg
J. Mueller, University of Freiburg
S. Osswald, University of Freiburg
R. Schmitt, University of Freiburg
R. Bogdan Rusu, Willow Garage Inc.

License for octomap: New BSD License (see LICENSE.txt)


REQUIREMENTS
############################

 * cmake
 * regular build environment (gcc)
 * skip to WINDOWS for tips on compilation under Windows
 
 viewer: 
 * Qt4
 * OpenGL
 * (QGLViewer)

 HTML documentation: 
 * doxygen

 Hint: you can install all dependencies on Ubuntu by running:

    sudo apt-get install cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-qt4-dev
       

       
INSTALLATION
############################
 
 * See http://www.ros.org/wiki/octomap if you want to use OctoMap in ROS! There  *
 * are pre-compiled packages for octomap, octovis, and ROS integration available *

Build the complete project by changing into the "build" directory 
and running cmake:

  mkdir build && cd build	
	cmake ..
	
Type "make" to compile afterwards. This will create all CMake
files cleanly in the "build" folder (Out-of-source build).
Executables will end up in "bin", libraries in "lib".


A debug configuration can be created by running:
	
        cmake -DCMAKE_BUILD_TYPE=Debug ..

in "build" or a different directory (e.g. "build-debug").

You can install the library by running "make install", though it 
is not necessary. Be sure to adjust CMAKE_INSTALL_PREFIX before.

The target "make test" executes the unit tests for the octomap library,
if you are interested in verifying the functionality on your machine.


DOCUMENTATION
############################

The documentation for the latest stable release is available at:
	
	http://octomap.sourceforge.net/doxygen/

You can build the most current HTML-Documentation for your current
source with Doxygen by running 

       make docs

in the build directory. The documentation will end up in
doc/html/index.html in the main directory.


GETTING STARTED
############################

Jump right in and have a look at 
src/octomap/simple.cpp

Or start the 3D viewer
bin/octovis 

You will find an example scan to load at
src/examples/scan.dat.bz2     (please bunzip2 it first)

Further examples can be downloaded from the project website.


USE IN OTHER PROJECTS
############################

A CMake-project config is generated for OctoMap which allows OctoMap
to be used from other CMake-Projects easily.

Point CMake to your octomap installation so that it finds:
[octomap]/lib/cmake/octomap/octomap-config.cmake

Then add the following to your CMakeLists.txt:

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

In addition to this cmake-module we also provide a pkgconfig-file.

For convenience, there is a minimal project included in the file 
example-project.tgz


ECLIPSE PROJECT FILES
############################

Eclipse project files can be generated (with some limitations, see:
http://www.vtk.org/Wiki/Eclipse_CDT4_Generator) by running:

	cmake -G"Eclipse CDT4 - Unix Makefiles" ..
	
Import the project (existing project, root is the build folder, 
do not copy contents) into Eclipse afterwards. For full Eclipse
compatibility, it might be necessary to build in the main source
directory.


WINDOWS
############################

The octomap library and tools can be compiled and used
under Windows although this has not been tested in-depth. 
Feedback is welcome.

To compile the library you need cmake (http://www.cmake.org).

MinGW
------------------------------

* Download the MinGW distribution (http://www.mingw.org)
* Install C++ compiler and add MingGW/bin to your system PATH

* Start the cmake-gui and set the code directory to the 
  library root (e.g. /octomap)
* Set the build directory to, e.g., /octomap/build.
* Press "Generate", select the appropriate generator, 
  "MinGW Makefiles".

* Start a command shell and "make" the project:
  octomap> cd build
  octomap/build> mingw32-make.exe

* You can run the unit tests using ctest on the command prompt:
  octomap/build> ctest.exe


Microsoft Visual Studio 2010
------------------------------

* Start the cmake-gui and set the code directory to the 
  library root (e.g. /octomap)
* Set the build directory to, e.g., /octomap/build.
* Press "Generate", select the appropriate generator, 
  e.g. "Visual Studio 10".

* This generates a solution file octomap.sln
  Load this file and build the project.

* You can run the unit tests using ctest on the command prompt:
  octomap/build> ctest.exe -C Release

