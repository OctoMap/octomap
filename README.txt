
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

License: 
  * New BSD License (see "octomap/LICENSE.txt")
  * GPL for the viewer "octovis" and related libraries (see "octovis/LICENSE.txt").

OVERVIEW
############################

OctoMap now consists of two separate libraries each in its own subfolder:
octomap, the actual library, and octovis, our visualization libraries and tools.
This README provides an overview of both, for details on compiling each please 
see "octomap/README.txt" and "octovis/README.txt" respectively.
See http://www.ros.org/wiki/octomap and http://www.ros.org/wiki/octovis if you 
want to use OctoMap in ROS; there are pre-compiled packages available.

You can build each library separately with CMake by running it from the subdirectories, 
or build octomap and octovis together from this top-level directory. E.g., to
only compile the library, run:

  cd octomap
  mkdir build
  cd build
  cmake ..
  make
  
To compile the complete package, run:
  cd build
  cmake ..
  make
  
Binaries and libs will end up in the directories "bin" and "lib" of the 
top-level directory where you started the build.


See "octomap/README.txt" and "octovis/README.txt" for 
details about compilation and hints on compiling under Windows.

Report bugs and request features in our tracker:
https://sourceforge.net/apps/trac/octomap    

A list of changes is available in "octomap/CHANGELOG.txt"
