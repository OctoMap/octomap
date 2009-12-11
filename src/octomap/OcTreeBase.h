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

#ifndef OCTOMAP_BASE_OCTREE_H
#define OCTOMAP_BASE_OCTREE_H



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
    class OcTreeBase {

  public:

    OcTreeBase(double _resolution);
    virtual ~OcTreeBase();

    /// \return The number of nodes in the tree
    unsigned int size() const { return tree_size; }

    void setResolution(double r);
    double getResolution() const { return resolution; }

    unsigned int getDepth() const { return tree_depth; }

    /**
     * \return Pointer to the root node of tree. This pointer should
     * not be modified or deleted externally, the OcTree manages the memory itself.
     */
    NODE* getRoot() const { return itsRoot; }


    /**
     * Search a 3D point in the tree
     *
     * @param value Searched coordinate
     * @return result Pointer to the resulting NODE when found, else NULL. This pointer should
     * not be modified or deleted externally, the OcTree manages the memory itself.
     */
    NODE* search (const point3d& value) const;


    /**
     * Updates an OcTreeNode in the OcTree either as observed free or occupied.
     *
     * @param value 3D position of the OcTreeNode that is to be updated
     * @param occupied Whether the node was observed occupied or free
     * @return
     */
    virtual NODE* updateNode(const point3d& value, bool occupied);


   /**
    * Traces a ray from origin to end (excluding), returning all
    * centers of cells traversed by the beam.
    * (Essentially using the DDA algorithm in 3D).
    *
    * @param origin
    * @param end
    * @param _ray
    * @return Success of operation. A "false" usually means that one of the coordinates is out of the Octree area
    */
    bool computeRay(const point3d& origin, const point3d& end, std::vector<point3d>& _ray) const;


    /**
     * Performs raycasting in 3d, similar to computeRay().
     *
     * A ray is cast from origin with a given direction, the first occupied
     * cell is returned (as center coordinate)
     *
     * Not tested throroughly yet ...
     *
     * @param origin
     * @param direction A vector pointing in the direction of the raycast. Does not need to be normalized.
     * @param end center of cell that was hit by the ray, if successful
     * @param maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
     * @return whether or not an occupied cell was hit
     */
    bool castRay(const point3d& origin, const point3d& directionP, point3d& end, double maxRange=-1.0) const;

    /**
     * Traverse the tree and collect all leaf nodes
     *
     * @param nodes Leaf nodes as OcTreeVolume
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getLeafNodes(std::list<OcTreeVolume>& nodes, unsigned int max_depth = 0) const;

    /**
     * Traverse the tree and collect all nodes, at all levels. Used e.g. in visualization.
     *
     * @param voxels list of all nodes to be returned
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getVoxels(std::list<OcTreeVolume>& voxels, unsigned int max_depth = 0) const;


  protected:

    /**
     * Generates a 16-bit key from/for given value when it is within the octree
     * bounds, returns false otherwise
     *
     * @param val Coordinate of one dimension in the octree
     * @param key 16bit key of the given coordinate, returned
     * @return whether val is within the octree bounds
     */
    bool genKey(double val, unsigned short int& key) const;

    /**
     * Generates comlpete key for all three dimensions of a given point,
     * using genKey().
     *
     * @param point 3d coordinate of 1 a point
     * @param keys values that will be computed, an array of fixed size 3.
     * @return true when point is within the octree, false otherwise
     */
    bool genKeys(const point3d& point, unsigned short int (&keys)[3]) const;

    /// reverse of genKey(), generates center coordinate of cell corresponding to a key
    bool genVal(unsigned short int& key, double& val) const;

    /// generate child number from key at given tree depth
    unsigned int genPos(unsigned short int key[], int i) const;


    /// recursive call of updateNode()
    NODE* updateNodeRecurs(NODE* node, bool node_just_created, unsigned short int key[3],
                           unsigned int depth, bool occupied);

    /// Recursive call for getLeafNodes()
    void getLeafNodesRecurs(std::list<OcTreeVolume>& nodes, unsigned int max_depth,
          NODE* node, unsigned int depth, const point3d& parent_center) const;

    /// Recursive call for getVoxels()
    void getVoxelsRecurs(std::list<OcTreeVolume>& nodes, unsigned int max_depth,
        NODE* node, unsigned int depth, const point3d& parent_center) const;

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

#include "OcTreeBase.hxx"

#endif
