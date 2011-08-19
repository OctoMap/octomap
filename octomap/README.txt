
Octomap
- A probabilistic, flexible, and compact 3D mapping library for robotic systems.

Authors: K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2011.
http://octomap.sourceforge.net/

Further Contributors:
J. Mueller, University of Freiburg
S. Osswald, University of Freiburg
R. Schmitt, University of Freiburg
R. Bogdan Rusu, Willow Garage Inc.

License: 
  * New BSD License (see LICENSE.txt in /octomap)
  * The viewer "octovis" and all related libraries are licensed under the GPL 
    (see LICENSE.txt in /octovis).


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

       sudo apt-get install cmake doxygen libqt4-dev libqt4-opengl-dev
       
If you are running Ubuntu 9.10 or later, you can use its supplied
version of qglviewer, otherwise it will be compiled from source. 
To install it:

       sudo apt-get install libqglviewer-qt4-dev


INSTALLATION
############################

 * Note: skip to the end if you want to use OctoMap in ROS! *

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
include_directories(OCTOMAP_INCLUDE_DIRS)
link_libraries(OCTOMAP_LIBRARIES)

In addition to this cmake-module we also provide a pkgconfig-file.


ROS-INTEGRATION
############################

Octomap can directly be used in any node running in the Robot
Operating System (ROS, http://www.ros.org). A virtual ROS package
and more code is available in Freiburg's ROS repository at:

http://code.google.com/p/alufr-ros-pkg/

The "octomap_mapping" stack (see http://www.ros.org/wiki/octomap_mapping
for details) contains the "octomap" package which provides the 
actual ROS integration by downloading and compiling the latest release. 
If you plan to use octomap mainly in ROS, just install the 
octomap_mapping stack.


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

The octomap library is compatible with Visual Studio 2010
although this has not been tested in-depth. Feedback is welcome.

To compile the library you need cmake (http://www.cmake.org).
Start the cmake-gui and set the code directory to the library root.
Set the build directory to, e.g., octomap/build.
Press "Generate", select the appropriate generator, e. g. "Visual Studio 10".

This generates a solution file octomap.sln,
load this file and build the project.

You can run the unit tests using ctest on the command prompt:
> ctest.exe -C Release

