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

#ifndef OCTOMAP_OCTREE_H
#define OCTOMAP_OCTREE_H



#include <list>

#include "octomap_types.h"


namespace octomap {


  /**
   * OcTree base class
   *
   *  This tree has a max depth of 15, at an resolution
   *  of 1 cm, values have to be < +/- 327.68 meters (2^15)
   *
   *  This implementation uses an efficient key generation method
   *  directly from the binary representation of the data.
   *
   *  Note: the tree does not save individual points.
   */
  template <class NODE>
    class AbstractOcTree {

  public:

    AbstractOcTree(double _resolution);
    virtual ~AbstractOcTree();

    /// \return The number of nodes in the tree
    unsigned int size() const { return tree_size; }

    void setResolution(double r);
    double getResolution() const { return resolution; }

    /// \return Pointer to the root node of tree
    NODE* getRoot() const { return itsRoot; }


    /**
     * Search a 3D point in the tree
     *
     * @param value Searched coordinate
     * @param result Pointer to the resulting NODE when found, else NULL
     * @return Result of search
     */
    bool search (const point3d& value, NODE*& result) const;


    /**
     * Traverse the tree and collect all leaf nodes
     *
     * @param max_depth Depth limit for nodes regarded as leafs
     * @param nodes Leaf nodes as OcTreeVolume
     */
    void getLeafNodes(unsigned int max_depth, std::list<OcTreeVolume>& nodes) const;

    /*! walk tree, add all AbstractOcTree volumes (center w. size)
     *  used e.g. for visualization
     */
    /**
     * Traverse the tree and collect all nodes, at all levels. Used e.g. in visualization.
     *
     * @param max_depth Depth limit for nodes
     * @param voxels
     */
    void getVoxels(unsigned int max_depth, std::list<OcTreeVolume>& voxels) const;

  protected:

    /// generate 16-bit key from/for given value
    bool genKey(double val, unsigned short int& key) const;

    /// reverse of genKey(), generates center coordinate of cell corresponding to a key
    bool genVal(unsigned short int& key, double& val) const;

    /// generate child number from key at given tree depth
    unsigned int genPos(unsigned short int key[], int i) const;

    /// Recursive call for getLeafNodes()
    void getLeafNodesRecurs(NODE* node, unsigned int depth, unsigned int max_depth,
        const point3d& parent_center, std::list<OcTreeVolume>& points) const;

    /// Recursive call for getVoxels()
    void getVoxelsRecurs(NODE* node, unsigned int depth, unsigned int max_depth,
        const point3d& parent_center, std::list<OcTreeVolume>& voxels) const;

    NODE* itsRoot;

    // constants of the tree
    unsigned int tree_depth;
    unsigned int tree_max_val;
    double resolution;  ///< in meters
    double resolution_factor; ///< = 1. / resolution
    point3d tree_center;

    unsigned int tree_size; ///< number of nodes in tree
    double maxValue[3]; ///< max in x, y, z
    double minValue[3]; ///< min in x, y, z
    bool sizeChanged;
  };


  // for memory computation only
  class GridData {
  public:
    float log_odds_occupancy;
    char data;
  };

}

#include "AbstractOcTree.hxx"

#endif
