
/**
 * \namespace octomath Namespace of the math library in Octomap
 *
 */


/**
 * \namespace octomap Namespace the Octomap library and visualization tools
 *
 */


/** \mainpage Octomap

  \section intro_sec Introduction

  The <a href="http://octomap.sourceforge.net/">Octomap library</a>
  implements a 3D occupancy grid mapping approach. It provides data
  structures and mapping algorithms. The map is implemented using an
  \ref octomap::OcTree "Octree". It is designed to meet the following
  requirements: </p>

<ul>
<li>
<b>Full 3D model.</b>  The map is able to model arbitrary environments
without prior assumptions about it. The representation models occupied
areas as well as free space. If no information is available about an
area (commonly denoted as <i>unknown areas</i>), this information is
encoded as well.  While the distinction between free and occupied
space is essential for safe robot navigation, information about
unknown areas is important, e.g., for autonomous exploration of an
environment.
</li>
<li>
<b>Updatable.</b>  It is possible to add new information or sensor
 readings at any time. Modeling and updating is done in
 a <i>probabilistic</i> fashion. This accounts for sensor noise or
 measurements which result from dynamic changes in the environment,
 e.g., because of dynamic objects.  Furthermore, multiple robots are
 able to contribute to the same map and a previously recorded map is
 extendable when new areas are explored.

</li>
<li>
<b>Flexible.</b>  The extent of the map does not have to be known in
advance. Instead, the map is dynamically expanded as needed. The map
is multi-resolution so that, for instance, a high-level planner is
able to use a coarse map, while a local planner may operate using a
fine resolution. This also allows for efficient visualizations which
scale from coarse overviews to detailed close-up views.
</li>
<li>
<b>Compact.</b>  The is stored efficiently, both in memory and on
disk. It is possible to generate compressed files for later usage or
convenient exchange between robots even under bandwidth constraints.
</li>
</ul>

<p> Octomap was developed by <a
href="http://www.informatik.uni-freiburg.de/~wurm">Kai M. Wurm</a> and
<a href="http://www.informatik.uni-freiburg.de/~hornunga">Armin
Hornung</a>, and is currently maintained by Armin Hornung. A tracker for bug reports and
feature requests is available available at <a href="https://sourceforge.net/apps/trac/octomap">https://sourceforge.net/apps/trac/octomap</a></p>

  \section install_sec Installation
  <p>See the file README.txt in the main folder.
  </p>


  \section changelog_sec Changelog
  <p>See the file CHANGELOG.txt in the main folder or the <a href="http://octomap.svn.sourceforge.net/viewvc/octomap/trunk/octomap/CHANGELOG.txt">latest version online</a>
  </p>


\section gettingstarted_sec Getting Started
<p>
  Jump right in and have a look at the main class octomap::OcTree and the examples in src/octomap/simple.cpp. 
  To integrate single measurements into the 3D map have a look at
  octomap::OcTree::insertRay(...), to insert full 3D scans (pointclouds) please have a look at
  octomap::OcTree::insertScan(...). Queries can be performed e.g. with octomap::OcTree::search(...) or
  octomap::OcTree::castRay(...). The preferred way to batch-access or process nodes in an Octree is with the iterators
  \ref leaf_iterator "leaf_iterator",  \ref tree_iterator "tree_iterator", or \ref leaf_bbx_iterator "leaf_bbx_iterator".</p>
  <p>The \ref octomap::OcTree "OcTree" class is derived from \ref octomap::OccupancyOcTreeBase "OccupancyOcTreeBase", with most
  functionality in the parent class. Also derive from OccupancyOcTreeBase if you you want to implement
  your own Octree and node classes. You can have a look at the classes octomap::OcTreeStamped and octomap::OcTreeNodeStamped as examples.
  </p>



<p>
  Start the 3D visualization with:<br>
  bin/octovis 
</p>

<p>
  You will find an example scan to load at<br>
  src/examples/scan.dat.bz2     (please bunzip2 it first)
  </p>


**/




