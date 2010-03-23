#ifndef OCTOMAP_OCCUPANCY_OCTREE_BASE_H
#define OCTOMAP_OCCUPANCY_OCTREE_BASE_H

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

#include <list>

#include "octomap_types.h"
#include "OcTreeBase.h"


namespace octomap {


  /**
   * Base class for Occupancy Octrees (e.g. for mapping).
   * Each class used as NODE type needs to be derived from
   * OccupancyOcTreeNode
   *
   * This tree implementation has a maximum depth of 16. 
   * At a resolution of 1 cm, values have to be < +/- 327.68 meters (2^15)
   *
   * This limitation enables the use of an efficient key generation 
   * method which uses the binary representation of the data.
   *
   * \note The tree does not save individual points.
   */
  template <class NODE>
  class OccupancyOcTreeBase : public OcTreeBase<NODE> {

  public:

    OccupancyOcTreeBase(double _resolution);
    virtual ~OccupancyOcTreeBase();

    /**
     * Integrate occupancy measurement.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const point3d& value, bool occupied);

    /**
     * Insert one ray between origin and end into the tree.
     *
     * integrateMissOnRay() is called for the ray, the end point is updated as occupied.
     * maxrange can be used to specify a maximum sensor range that is considered
     */
    virtual bool insertRay(const point3d& origin, const point3d& end, double maxrange=-1.);

    /**
     * Performs raycasting in 3d, similar to computeRay().
     *
     * A ray is cast from origin with a given direction, the first occupied
     * cell is returned (as center coordinate). If the starting coordinate is already
     * occupied in the tree, this coordinate will be returned as a hit.
     *
     * @param origin starting coordinate of ray
     * @param direction A vector pointing in the direction of the raycast. Does not need to be normalized.
     * @param end returns the center of the cell that was hit by the ray, if successful
     * @param ignoreUnknownCells whether unknown cells are ignored. If false (default), the raycast aborts when an unkown cell is hit.
     * @param maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
     * @return whether or not an occupied cell was hit
     */
    bool castRay(const point3d& origin, const point3d& direction, point3d& end, bool ignoreUnknownCells=false, double maxRange=-1.0) const;


    /**
     * Convenience function to return all occupied nodes in the OcTree.
     *
     * @param occupied_volumes list of occpupied nodes (as point3d and size of the volume)
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getOccupied(std::list<OcTreeVolume>& occupied_volumes, unsigned int max_depth = 0) const;

    /**
     * Traverses the tree and collects all OcTreeVolumes regarded as occupied.
     * Inner nodes with both occupied and free children are regarded as occupied. 
     * This should be for internal use only, use getOccupied(occupied_volumes) instead.
     *
     * @param binary_nodes list of binary OcTreeVolumes which are occupied
     * @param delta_nodes list of delta OcTreeVolumes which are occupied
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getOccupied(std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth = 0) const;

    /**
     * Convenience function to return all free nodes in the OcTree.
     *
     * @param free_volumes list of free nodes (as point3d and size of the volume)
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getFreespace(std::list<OcTreeVolume>& free_volumes, unsigned int max_depth = 0) const;

    /**
     * Traverses the tree and collects all OcTreeVolumes regarded as free.
     * Inner nodes with both occupied and free children are regarded as occupied.
     *
     * @param binary_nodes list of binary OcTreeVolumes which are free
     * @param delta_nodes list of delta OcTreeVolumes which are free
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getFreespace(std::list<OcTreeVolume>& binary_nodes, std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth = 0) const;

  protected:

    /// Traces a ray from origin to end and updates all voxels on the way as free.
    /// The volume containing "end" is not updated.
    void integrateMissOnRay(const point3d& origin, const point3d& end);


    /// recursive call of updateNode()
    NODE* updateNodeRecurs(NODE* node, bool node_just_created, unsigned short int key[3],
                           unsigned int depth, bool occupied);

    void getFreespaceRecurs(std::list<OcTreeVolume>& binary_nodes,
                                                std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth,
                                                NODE* node, unsigned int depth, const point3d& parent_center) const;

    void getOccupiedRecurs( std::list<OcTreeVolume>& binary_nodes,
                                                std::list<OcTreeVolume>& delta_nodes, unsigned int max_depth,
                                                NODE* node, unsigned int depth,
                                                const point3d& parent_center) const;


  };

}

#include "OccupancyOcTreeBase.hxx"

#endif
