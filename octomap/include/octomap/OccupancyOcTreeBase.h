/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_OCCUPANCY_OCTREE_BASE_H
#define OCTOMAP_OCCUPANCY_OCTREE_BASE_H


#include <list>
#include <stdlib.h>
#include <vector>

#include "octomap_types.h"
#include "octomap_utils.h"
#include "OcTreeBaseImpl.h"
#include "AbstractOccupancyOcTree.h"


namespace octomap {

  /**
   * Base implementation for Occupancy Octrees (e.g. for mapping).
   * AbstractOccupancyOcTree serves as a common
   * base interface for all these classes.
   * Each class used as NODE type needs to be derived from
   * OccupancyOcTreeNode.
   *
   * This tree implementation has a maximum depth of 16. 
   * At a resolution of 1 cm, values have to be < +/- 327.68 meters (2^15)
   *
   * This limitation enables the use of an efficient key generation 
   * method which uses the binary representation of the data.
   *
   * \note The tree does not save individual points.
   *
   * \tparam NODE Node class to be used in tree (usually derived from
   *    OcTreeDataNode)
   */
  template <class NODE>
  class OccupancyOcTreeBase : public OcTreeBaseImpl<NODE,AbstractOccupancyOcTree> {

  public:
    /// Default constructor, sets resolution of leafs
    OccupancyOcTreeBase(double resolution);
    virtual ~OccupancyOcTreeBase();

    /// Copy constructor
    OccupancyOcTreeBase(const OccupancyOcTreeBase<NODE>& rhs);

    /**
    * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
    * Special care is taken that each voxel
    * in the map is updated only once, and occupied nodes have a preference over free ones.
    * This avoids holes in the floor from mutual deletion and is more efficient than the plain
    * ray insertion in insertPointCloudRays().
    *
    * @note replaces insertScan()
    *
    * @param scan Pointcloud (measurement endpoints), in global reference frame
    * @param sensor_origin measurement origin in global reference frame
    * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @param discretize whether the scan is discretized first into octree key cells (default: false).
    *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
    */
    virtual void insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                   double maxrange=-1., bool lazy_eval = false, bool discretize = false);

    /**
    * Integrate a 3d scan (transform scan before tree update), parallelized with OpenMP.
    * Special care is taken that each voxel
    * in the map is updated only once, and occupied nodes have a preference over free ones.
    * This avoids holes in the floor from mutual deletion and is more efficient than the plain
    * ray insertion in insertPointCloudRays().
    *
    * @note replaces insertScan()
    *
    * @param scan Pointcloud (measurement endpoints) relative to frame origin
    * @param sensor_origin origin of sensor relative to frame origin
    * @param frame_origin origin of reference frame, determines transform to be applied to cloud and sensor origin
    * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
    *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
    * @param discretize whether the scan is discretized first into octree key cells (default: false).
    *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.*
    */
    virtual void insertPointCloud(const Pointcloud& scan, const point3d& sensor_origin, const pose6d& frame_origin,
                   double maxrange=-1., bool lazy_eval = false, bool discretize = false);

    /**
    * Insert a 3d scan (given as a ScanNode) into the tree, parallelized with OpenMP.
    *
    * @note replaces insertScan
    *
    * @param scan ScanNode contains Pointcloud data and frame/sensor origin
    * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
    * @param lazy_eval whether the tree is left 'dirty' after the update (default: false).
    *   This speeds up the insertion by not updating inner nodes, but you need to call updateInnerOccupancy() when done.
    * @param discretize whether the scan is discretized first into octree key cells (default: false).
    *   This reduces the number of raycasts using computeDiscreteUpdate(), resulting in a potential speedup.
    */
    virtual void insertPointCloud(const ScanNode& scan, double maxrange=-1., bool lazy_eval = false, bool discretize = false);

    /**
     * Integrate a Pointcloud (in global reference frame), parallelized with OpenMP.
     * This function simply inserts all rays of the point clouds as batch operation.
     * Discretization effects can lead to the deletion of occupied space, it is
     * usually recommended to use insertPointCloud() instead.
     *
     * @param scan Pointcloud (measurement endpoints), in global reference frame
     * @param sensor_origin measurement origin in global reference frame
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     */
     virtual void insertPointCloudRays(const Pointcloud& scan, const point3d& sensor_origin, double maxrange = -1., bool lazy_eval = false);

     /**
      * Set log_odds value of voxel to log_odds_value. This only works if key is at the lowest
      * octree level
      *
      * @param key OcTreeKey of the NODE that is to be updated
      * @param log_odds_value value to be set as the log_odds value of the node
      * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
      *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
      * @return pointer to the updated NODE
      */
     virtual NODE* setNodeValue(const OcTreeKey& key, float log_odds_value, bool lazy_eval = false);

     /**
      * Set log_odds value of voxel to log_odds_value.
      * Looks up the OcTreeKey corresponding to the coordinate and then calls setNodeValue() with it.
      *
      * @param value 3d coordinate of the NODE that is to be updated
      * @param log_odds_value value to be set as the log_odds value of the node
      * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
      *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
      * @return pointer to the updated NODE
      */
     virtual NODE* setNodeValue(const point3d& value, float log_odds_value, bool lazy_eval = false);

     /**
      * Set log_odds value of voxel to log_odds_value.
      * Looks up the OcTreeKey corresponding to the coordinate and then calls setNodeValue() with it.
      *
      * @param x
      * @param y
      * @param z
      * @param log_odds_value value to be set as the log_odds value of the node
      * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
      *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
      * @return pointer to the updated NODE
      */
     virtual NODE* setNodeValue(double x, double y, double z, float log_odds_value, bool lazy_eval = false);

     /**
      * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
      * This only works if key is at the lowest octree level
      *
      * @param key OcTreeKey of the NODE that is to be updated
      * @param log_odds_update value to be added (+) to log_odds value of node
      * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
      *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
      * @return pointer to the updated NODE
      */
     virtual NODE* updateNode(const OcTreeKey& key, float log_odds_update, bool lazy_eval = false);

     /**
      * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
      * Looks up the OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
      *
      * @param value 3d coordinate of the NODE that is to be updated
      * @param log_odds_update value to be added (+) to log_odds value of node
      * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
      *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
      * @return pointer to the updated NODE
      */
     virtual NODE* updateNode(const point3d& value, float log_odds_update, bool lazy_eval = false);

     /**
      * Manipulate log_odds value of a voxel by changing it by log_odds_update (relative).
      * Looks up the OcTreeKey corresponding to the coordinate and then calls updateNode() with it.
      *
      * @param x
      * @param y
      * @param z
      * @param log_odds_update value to be added (+) to log_odds value of node
      * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
      *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
      * @return pointer to the updated NODE
      */
     virtual NODE* updateNode(double x, double y, double z, float log_odds_update, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     *
     * @param key OcTreeKey of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const OcTreeKey& key, bool occupied, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param value 3d coordinate of the NODE that is to be updated
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(const point3d& value, bool occupied, bool lazy_eval = false);

    /**
     * Integrate occupancy measurement.
     * Looks up the OcTreeKey corresponding to the coordinate and then calls udpateNode() with it.
     *
     * @param x
     * @param y
     * @param z
     * @param occupied true if the node was measured occupied, else false
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return pointer to the updated NODE
     */
    virtual NODE* updateNode(double x, double y, double z, bool occupied, bool lazy_eval = false);


    /**
     * Creates the maximum likelihood map by calling toMaxLikelihood on all
     * tree nodes, setting their occupancy to the corresponding occupancy thresholds.
     * This enables a very efficient compression if you call prune() afterwards.
     */
    virtual void toMaxLikelihood();

    /**
     * Insert one ray between origin and end into the tree.
     * integrateMissOnRay() is called for the ray, the end point is updated as occupied.
     * It is usually more efficient to insert complete pointcloudsm with insertPointCloud() or
     * insertPointCloudRays().
     *
     * @param origin origin of sensor in global coordinates
     * @param end endpoint of measurement in global coordinates
     * @param maxrange maximum range after which the raycast should be aborted
     * @param lazy_eval whether update of inner nodes is omitted after the update (default: false).
     *   This speeds up the insertion, but you need to call updateInnerOccupancy() when done.
     * @return success of operation
     */
    virtual bool insertRay(const point3d& origin, const point3d& end, double maxrange=-1.0, bool lazy_eval = false);
    
    /**
     * Performs raycasting in 3d, similar to computeRay(). Can be called in parallel e.g. with OpenMP
     * for a speedup.
     *
     * A ray is cast from 'origin' with a given direction, the first non-free
     * cell is returned in 'end' (as center coordinate). This could also be the 
     * origin node if it is occupied or unknown. castRay() returns true if an occupied node
     * was hit by the raycast. If the raycast returns false you can search() the node at 'end' and
     * see whether it's unknown space.
     * 
     *
     * @param[in] origin starting coordinate of ray
     * @param[in] direction A vector pointing in the direction of the raycast (NOT a point in space). Does not need to be normalized.
     * @param[out] end returns the center of the last cell on the ray. If the function returns true, it is occupied.
     * @param[in] ignoreUnknownCells whether unknown cells are ignored (= treated as free). If false (default), the raycast aborts when an unknown cell is hit and returns false.
     * @param[in] maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
     * @return true if an occupied cell was hit, false if the maximum range or octree bounds are reached, or if an unknown node was hit.
     */
    virtual bool castRay(const point3d& origin, const point3d& direction, point3d& end,
                 bool ignoreUnknownCells=false, double maxRange=-1.0) const;

    /**
     * Retrieves the entry point of a ray into a voxel. This is the closest intersection point of the ray
     * originating from origin and a plane of the axis aligned cube.
     * 
     * @param[in] origin Starting point of ray
     * @param[in] direction A vector pointing in the direction of the raycast. Does not need to be normalized.
     * @param[in] center The center of the voxel where the ray terminated. This is the output of castRay.
     * @param[out] intersection The entry point of the ray into the voxel, on the voxel surface.
     * @param[in] delta A small increment to avoid ambiguity of beeing exactly on a voxel surface. A positive value will get the point out of the hit voxel, while a negative valuewill get it inside.
     * @return Whether or not an intesection point has been found. Either, the ray never cross the voxel or the ray is exactly parallel to the only surface it intersect.
     */
    virtual bool getRayIntersection(const point3d& origin, const point3d& direction, const point3d& center,
                 point3d& intersection, double delta=0.0) const;

		/**
		 * Performs a step of the marching cubes surface reconstruction algorithm
		 * to retreive the normal of the triangles that fall in the cube
		 * formed by the voxels located at the vertex of a given voxel.
		 *
		 * @param[in] voxel for which retreive the normals
		 * @param[out] triangles normals
		 * @param[in] unknownStatus consider unknown cells as free (false) or occupied (default, true).
		 * @return True if the input voxel is known in the occupancy grid, and false if it is unknown.
		 */
		bool getNormals(const point3d& point, std::vector<point3d>& normals, bool unknownStatus=true) const;
	
    //-- set BBX limit (limits tree updates to this bounding box)

    ///  use or ignore BBX limit (default: ignore)
    void useBBXLimit(bool enable) { use_bbx_limit = enable; }
    bool bbxSet() const { return use_bbx_limit; }
    /// sets the minimum for a query bounding box to use
    void setBBXMin (point3d& min);
    /// sets the maximum for a query bounding box to use
    void setBBXMax (point3d& max);
    /// @return the currently set minimum for bounding box queries, if set
    point3d getBBXMin () const { return bbx_min; }
    /// @return the currently set maximum for bounding box queries, if set
    point3d getBBXMax () const { return bbx_max; }
    point3d getBBXBounds () const;
    point3d getBBXCenter () const;
    /// @return true if point is in the currently set bounding box
    bool inBBX(const point3d& p) const;
    /// @return true if key is in the currently set bounding box
    bool inBBX(const OcTreeKey& key) const;

    //-- change detection on occupancy:
    /// track or ignore changes while inserting scans (default: ignore)
    void enableChangeDetection(bool enable) { use_change_detection = enable; }
    bool isChangeDetectionEnabled() const { return use_change_detection; }
    /// Reset the set of changed keys. Call this after you obtained all changed nodes.
    void resetChangeDetection() { changed_keys.clear(); }

    /**
     * Iterator to traverse all keys of changed nodes.
     * you need to enableChangeDetection() first. Here, an OcTreeKey always
     * refers to a node at the lowest tree level (its size is the minimum tree resolution)
     */
    KeyBoolMap::const_iterator changedKeysBegin() const {return changed_keys.begin();}

    /// Iterator to traverse all keys of changed nodes.
    KeyBoolMap::const_iterator changedKeysEnd() const {return changed_keys.end();}

    /// Number of changes since last reset.
    size_t numChangesDetected() const { return changed_keys.size(); }


    /**
     * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
     * integration at once. Here, occupied nodes have a preference over free
     * ones.
     *
     * @param scan point cloud measurement to be integrated
     * @param origin origin of the sensor for ray casting
     * @param free_cells keys of nodes to be cleared
     * @param occupied_cells keys of nodes to be marked occupied
     * @param maxrange maximum range for raycasting (-1: unlimited)
     */
    void computeUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);


    /**
     * Helper for insertPointCloud(). Computes all octree nodes affected by the point cloud
     * integration at once. Here, occupied nodes have a preference over free
     * ones. This function first discretizes the scan with the octree grid, which results
     * in fewer raycasts (=speedup) but a slightly different result than computeUpdate().
     *
     * @param scan point cloud measurement to be integrated
     * @param origin origin of the sensor for ray casting
     * @param free_cells keys of nodes to be cleared
     * @param occupied_cells keys of nodes to be marked occupied
     * @param maxrange maximum range for raycasting (-1: unlimited)
     */
    void computeDiscreteUpdate(const Pointcloud& scan, const octomap::point3d& origin,
                       KeySet& free_cells,
                       KeySet& occupied_cells,
                       double maxrange);


    // -- I/O  -----------------------------------------

    /**
     * Reads only the data (=complete tree structure) from the input stream.
     * The tree needs to be constructed with the proper header information
     * beforehand, see readBinary().
     */
    std::istream& readBinaryData(std::istream &s);

    /**
     * Read node from binary stream (max-likelihood value), recursively
     * continue with all children.
     *
     * This will set the log_odds_occupancy value of
     * all leaves to either free or occupied.
     */
    std::istream& readBinaryNode(std::istream &s, NODE* node);

    /**
     * Write node to binary stream (max-likelihood value),
     * recursively continue with all children.
     *
     * This will discard the log_odds_occupancy value, writing
     * all leaves as either free or occupied.
     *
     * @param s
     * @param node OcTreeNode to write out, will recurse to all children
     * @return
     */
    std::ostream& writeBinaryNode(std::ostream &s, const NODE* node) const;

    /**
     * Writes the data of the tree (without header) to the stream, recursively
     * calling writeBinaryNode (starting with root)
     */
    std::ostream& writeBinaryData(std::ostream &s) const;


    /**
     * Updates the occupancy of all inner nodes to reflect their children's occupancy.
     * If you performed batch-updates with lazy evaluation enabled, you must call this
     * before any queries to ensure correct multi-resolution behavior.
     **/
    void updateInnerOccupancy();


    /// integrate a "hit" measurement according to the tree's sensor model
    virtual void integrateHit(NODE* occupancyNode) const;
    /// integrate a "miss" measurement according to the tree's sensor model
    virtual void integrateMiss(NODE* occupancyNode) const;
    /// update logodds value of node by adding to the current value.
    virtual void updateNodeLogOdds(NODE* occupancyNode, const float& update) const;

    /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
    virtual void nodeToMaxLikelihood(NODE* occupancyNode) const;
    /// converts the node to the maximum likelihood value according to the tree's parameter for "occupancy"
    virtual void nodeToMaxLikelihood(NODE& occupancyNode) const;

  protected:
    /// Constructor to enable derived classes to change tree constants.
    /// This usually requires a re-implementation of some core tree-traversal functions as well!
    OccupancyOcTreeBase(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

    /**
     * Traces a ray from origin to end and updates all voxels on the
     *  way as free.  The volume containing "end" is not updated.
     */
    inline bool integrateMissOnRay(const point3d& origin, const point3d& end, bool lazy_eval = false);


    // recursive calls ----------------------------

    NODE* updateNodeRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const float& log_odds_update, bool lazy_eval = false);
    
    NODE* setNodeValueRecurs(NODE* node, bool node_just_created, const OcTreeKey& key,
                           unsigned int depth, const float& log_odds_value, bool lazy_eval = false);

    void updateInnerOccupancyRecurs(NODE* node, unsigned int depth);
    
    void toMaxLikelihoodRecurs(NODE* node, unsigned int depth, unsigned int max_depth);


  protected:
    bool use_bbx_limit;  ///< use bounding box for queries (needs to be set)?
    point3d bbx_min;
    point3d bbx_max;
    OcTreeKey bbx_min_key;
    OcTreeKey bbx_max_key;

    bool use_change_detection;
    /// Set of leaf keys (lowest level) which changed since last resetChangeDetection
    KeyBoolMap changed_keys;
    

  };

} // namespace

#include "octomap/OccupancyOcTreeBase.hxx"

#endif
