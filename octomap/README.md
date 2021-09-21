Octomap - A probabilistic, flexible, and compact 3D mapping library for robotic systems
=======================================================================================

Authors: Kai M. Wurm and Armin Hornung, University of Freiburg, Copyright (C) 2009-2013.
https://octomap.github.io

See the [list of contributors](AUTHORS.txt) for further authors.

License for octomap: [New BSD License](LICENSE.txt)


REQUIREMENTS
------------

* For only the octomap library: cmake and a regular build environment (gcc)
* For HTML documentation: doxygen (optional)
* For the viewer octovis: Qt4, OpenGL, QGLViewer (optional)


Skip to WINDOWS for tips on compilation under Windows. You can install all dependencies on Ubuntu by running:

    sudo apt-get install cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-dev-qt4
       
(Note: for older releases of Ubuntu you need to exchange the last package name with `libqglviewer-qt4-dev`)
       
INSTALLATION
------------
 
See http://www.ros.org/wiki/octomap if you want to use OctoMap in ROS! 
There are pre-compiled packages for octomap, octovis, and ROS integration available.


Build the complete project by changing into the "build" directory 
and running cmake:

    mkdir build && cd build	
    cmake ..
	
Type `make` to compile afterwards. This will create all CMake
files cleanly in the `build` folder (Out-of-source build).
Executables will end up in `bin`, libraries in `lib`.


A debug configuration can be created by running:
	
    cmake -DCMAKE_BUILD_TYPE=Debug ..

in `build` or a different directory (e.g. `build-debug`).

You can install the library by running `make install`, though it 
is usually not necessary. Be sure to adjust `CMAKE_INSTALL_PREFIX` before.

The target `make test` executes the unit tests for the octomap library,
if you are interested in verifying the functionality on your machine.


DOCUMENTATION
-------------

The documentation for the latest stable release is available online:
  https://octomap.github.io/octomap/doc/index.html

You can build the most current HTML-Documentation for your current
source with Doxygen by running `make docs` 
in the build directory. The documentation will end up in
`doc/html/index.html` in the main directory.


GETTING STARTED
---------------

Jump right in and have a look at the example
src/octomap/simple_example.cpp

Or start the 3D viewer with `bin/octovis`

You will find an example scan and binary tree to load in the directory `share`.
Further examples can be downloaded from the project website.


USE IN OTHER PROJECTS
---------------------

A CMake-project config is generated for OctoMap which allows OctoMap
to be used from other CMake-Projects easily.

Point CMake to your octomap installation so that it finds the file
octomap/lib/cmake/octomap/octomap-config.cmake, e.g. by setting the environment
variable `octomap_DIR`to the directory containing it.

Then add the following to your CMakeLists.txt:

    find_package(octomap REQUIRED)
    include_directories(${OCTOMAP_INCLUDE_DIRS})
    link_libraries(${OCTOMAP_LIBRARIES})

In addition to this cmake-module we also provide a pkgconfig-file.

For convenience, there is a minimal example project included in the file 
share/example-project.tgz


ECLIPSE PROJECT FILES
---------------------

Eclipse project files can be generated (with some limitations, see:
http://www.vtk.org/Wiki/Eclipse_CDT4_Generator) by running:

    cmake -G"Eclipse CDT4 - Unix Makefiles" ..
	
Import the project (existing project, root is the build folder, 
do not copy contents) into Eclipse afterwards. For full Eclipse
compatibility, it might be necessary to build in the main source
directory.


WINDOWS
-------

The octomap library and tools can be compiled and used
under Windows although this has not been tested in-depth. 
Feedback is welcome.

To compile the library you need cmake (http://www.cmake.org)
and either MinGW or Visual Studio.

### MinGW ###

1. Download the MinGW distribution (http://www.mingw.org)
2. Install C++ compiler and add MingGW/bin to your system PATH
3. Start the cmake-gui and set the code directory to the 
  library root (e.g. `/octomap`)
4. Create and set the build directory to, e.g., `/octomap/build`.
5. Press "Configure" then "Generate", select the appropriate generator, "MinGW Makefiles".
6. Start a command shell and "make" the project:

        octomap> cd build
        octomap/build> mingw32-make.exe
    

As verification, you can run the unit tests using ctest on the 
command prompt:

    octomap/build> ctest.exe


### Microsoft Visual Studio (2013 or later recommended) ###

Last tested with MSVC 2013 and 2015 (Community Edition).

1. Start the cmake-gui and set the source code directory to the 
  library root (e.g. `\octomap`)
2. Create a build directory and set it in CMake ("Where to build the
   binaries"), e.g.  `\octomap\build`.
3. Press "Configure" then "Generate", select the appropriate generator, e.g. "Visual Studio 2015". 
      This generates a solution file `octomap.sln` in the build directory.
4. Load this file and build the project `ALL_BUILD` in Visual Studio.

Instead of building the complete distribution (octomap, octovis, and dynamicEDT3D)
you can only build octomap by proceeding as described above but in the `octomap`
subdirectory. This can help you getting started when there are problems with
octovis and Qt4.

As verification, you can run the unit tests in Visual Studio by building the
`RUN_TESTS` project or by using ctest on the command prompt:

    octomap/build> ctest.exe -C Release

