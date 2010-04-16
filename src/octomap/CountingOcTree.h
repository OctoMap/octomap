// $Id$

/**
* Octomap:
* A  probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: GNU GPL v2, http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef OCTOMAP_COUNTING_OCTREE_HH
#define OCTOMAP_COUNTING_OCTREE_HH

#include <stdio.h>
#include "OcTreeBase.h"
#include "OcTreeDataNode.h"

namespace octomap {

  /**
   * An Octree-node which stores an internal counter per node / volume.
   *
   * Count is recursive, parent nodes have the summed count of their
   * children.
   *
   * \note In our mapping system this data structure is used in
   *       CountingOcTree in the sensor model only
   */
  class CountingOcTreeNode : public OcTreeDataNode<unsigned int> {

  public:

    CountingOcTreeNode();
    ~CountingOcTreeNode();
    bool createChild(unsigned int i);

    unsigned int getCount() const { return getValue(); }
    void increaseCount() { value++; }
    void setCount(unsigned c) {this->setValue(c); }


    // overloaded:
    void expandNode();



  protected:

  };




  /**
   * An AbstractOcTree which stores an internal counter per node / volume.
   *
   * Count is recursive, parent nodes have the summed count of their
   * children.
   *
   * \note In our mapping system this data structure is used in
   *       the sensor model only. Do not use, e.g., insertScan.
   */
  class CountingOcTree : public OcTreeBase <CountingOcTreeNode> {

  public:

    CountingOcTree(double resolution);

    virtual CountingOcTreeNode* updateNode(const point3d& value);

  protected:

  };


}


#endif
