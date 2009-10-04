Octomap:

A  probabilistic, flexible, and compact 3D mapping library for robotic systems.

Authors: K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
http://octomap.sourceforge.net/
License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt





REQUIREMENTS
############################

 * octomap: a regular build environment, cmake
 
 * viewer: Boost smart_ptr, Qt4, libQGLViewer (included in "extern") 

   You can install all dependencies on Ubuntu / Debian by running:
   sudo apt-get install cmake libqt4-dev libqt4-opengl-dev libboost-dev
    
 * HTML documentation: doxygen




INSTALLATION:
############################

Build the complete project by changing into the "build" directory 
and running cmake:
	
	cmake ../src
	
Type "make" to compile afterwards. This will create all CMake
files cleanly in the "build" folder (Out-of-source build).
Executables will end up in "bin", libraries in "lib".


A debug configuration can be created by running:
	
	cmake -DCMAKE_BUILD_TYPE=Debug ../src

in "build" or a different directory (e.g. "build-debug").



DOCUMENTATION
############################

A HTML-Documentation can be built using Doxygen by running 

      "make docs"

in the build directory. The documentation will end up in
/doc/html/index.html in the main directory.



ECLIPSE PROJECT FILES
############################

Eclipse project files can be generated (with some limitations, see:
http://www.vtk.org/Wiki/Eclipse_CDT4_Generator) by running:

	cmake -G"Eclipse CDT4 - Unix Makefiles" ../src 
	
Import the project (existing project, root is the build folder, do
not copy contents) into Eclipse afterwards.



