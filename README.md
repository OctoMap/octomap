OctoMap - A probabilistic, flexible, and compact 3D mapping library for robotic systems.
========================================================================================


Authors: Kai M. Wurm and Armin Hornung, University of Freiburg, Copyright (C) 2009-2013.
http://octomap.github.com

See the [list of contributors](octomap/AUTHORS.txt) for further authors.

License: 
  * octomap: [New BSD License](octomap/LICENSE.txt)
  * octovis and related libraries: [GPL](octovis/LICENSE.txt)


Download the latest releases:
  https://github.com/octomap/octomap/tags

API documentation:
  http://octomap.github.com/octomap/doc/
  
Report bugs and request features in our tracker:
  https://github.com/OctoMap/octomap/issues

A list of changes is available in the [octomap changelog](octomap/CHANGELOG.txt)


OVERVIEW
--------

OctoMap consists of two separate libraries each in its own subfolder:
**octomap**, the actual library, and **octovis**, our visualization libraries and tools.
This README provides an overview of both, for details on compiling each please 
see [octomap/README.md](octomap/README.md) and [octovis/README.md](octovis/README.md) respectively.
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
  
Binaries and libs will end up in the directories `bin` and `lib` of the
top-level directory where you started the build.


See [octomap README](octomap/README.md) and [octovis README](octovis/README.md) for further
details and hints on compiling, especially under Windows.
