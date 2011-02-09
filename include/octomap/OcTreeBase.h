#ifndef OCTOMAP_OCTREE_BASE_H
#define OCTOMAP_OCTREE_BASE_H

// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2010.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009-2011, K. M. Wurm, A. Hornung, University of Freiburg
 * All rights reserved.
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

#include <list>
#include <limits>
#include <math.h>
#include <cassert>

#include "octomap_types.h"
#include "OcTreeKey.h"
#include "ScanGraph.h"


namespace octomap {


  /**
   * OcTree base class, to be used with with any kind of OcTreeDataNode.
   *
   * This tree implementation currently has a maximum depth of 16
   * nodes. For this reason, coordinates values have to be, e.g.,
   * below +/- 327.68 meters (2^15) at a maximum resolution of 0.01m.
   *
   * This limitation enables the use of an efficient key generation
   * method which uses the binary representation of the data point
   * coordinates.
   *
   * \note The tree does not store individual data points.
   */
  template <class NODE>
  class OcTreeBase {

  public:
    
    OcTreeBase(double _resolution);
    virtual ~OcTreeBase();


    void setResolution(double r);
    inline double getResolution() const { return resolution; }

    inline unsigned int getTreeDepth () const { return tree_depth; }

    /**
     * \return Pointer to the root node of the tree. This pointer
     * should not be modified or deleted externally, the OcTree
     * manages its memory itself.
     */
    inline NODE* getRoot() const { return itsRoot; }

    /** 
     *  search node given a 3d point 
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search (float x, float y, float z) const;

    /** 
     *  search node given a 3d point
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search (const point3d& value) const;

    /** 
     *  search node given an addressing keys
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search (const OcTreeKey& key) const;

    /// Lossless compression of OcTree: merge children to parent when there are
    /// eight children with identical values
    void prune();

    /// Expands all pruned nodes (reverse of prune())
    /// \note This is an expensive operation, especially when the tree is nearly empty!
    void expand();


    // -- statistics  ----------------------

    /// \return The number of nodes in the tree
    inline unsigned int size() const { return tree_size; }

    unsigned int memoryUsage() const;

    /// \return Memory usage of a full grid of the same size as the OcTree in bytes (for comparison)
    unsigned int memoryFullGrid();

    /// Size of OcTree in meters for x, y and z dimension
    void getMetricSize(double& x, double& y, double& z);
    /// minimum value in x, y, z
    void getMetricMin(double& x, double& y, double& z);
    void getMetricMin(double& x, double& y, double& z) const;
    /// maximum value in x, y, z
    void getMetricMax(double& x, double& y, double& z);
    void getMetricMax(double& x, double& y, double& z) const;

    /// Traverses the tree to calculate the total number of nodes
    unsigned int calcNumNodes() const;


    // -- access tree nodes  ------------------

    /**
     * Traverse the tree and return all leaf nodes
     *
     * @param node_centers collection of center points
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getLeafNodes(point3d_list& node_centers) const;

    // replaced by getLeafNodes(point3d_list&)
    void getLeafNodes(std::list<OcTreeVolume>& nodes, unsigned int max_depth = 0) const __attribute__ ((deprecated));

    /**
     * Traverse the tree and return all nodes, at all levels. Used e.g. in visualization.
     *
     * @param voxels list of all nodes to be returned
     * @param max_depth Depth limit of query. 0 (default): no depth limit
     */
    void getVoxels(std::list<OcTreeVolume>& voxels, unsigned int max_depth = 0) const;


    /// return centers of leafs that to NOT exist (but could) in a given bounding box
    void getUnknownLeafCenters(point3d_list& node_centers, point3d min, point3d max) const;


    // -- raytracing  -----------------------

   /**
    * Traces a ray from origin to end (excluding), returning an
    * OcTreeKey of all nodes traversed by the beam. You still need to check
    * if a node at that coordinate exists (e.g. with search()).
    *
    * @param origin start coordinate of ray
    * @param end end coordinate of ray
    * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
    * @return Success of operation. Returning false usually means that one of the coordinates is out of the OcTree's range
    */
    bool computeRayKeys(const point3d& origin, const point3d& end, KeyRay& ray) const;


   /**
    * Traces a ray from origin to end (excluding), returning the
    * coordinates of all nodes traversed by the beam. You still need to check
    * if a node at that coordinate exists (e.g. with search()).
    * @note: use the faster computeRayKeys method if possible.
    * 
    * @param origin start coordinate of ray
    * @param end end coordinate of ray
    * @param ray KeyRay structure that holds the keys of all nodes traversed by the ray, excluding "end"
    * @return Success of operation. Returning false usually means that one of the coordinates is out of the OcTree's range
    */
    bool computeRay(const point3d& origin, const point3d& end, std::vector<point3d>& ray) const;


    /**
     * Generates key for all three dimensions of a given point
     * using genKeyValue().
     *
     * @param point 3d coordinate of a point
     * @param keys values that will be computed, an array of fixed size 3.
     * @return true when point is within the octree, false otherwise
     */
    bool genKey(const point3d& point, OcTreeKey& key) const;


    // -- experimental section  -----------------------
    // file IO

    /// Read complete state of tree from stream
    /// EXPERIMENTAL!
    std::istream& read(std::istream &s);

    /// Write complete state of tree to stream, prune tree first (lossless compression)
    /// EXPERIMENTAL!
    std::ostream& write(std::ostream &s);

    /// Write complete state of tree to stream, no pruning (const version)
    std::ostream& writeConst(std::ostream &s) const;

    /// Make the templated NODE type available from the outside
    typedef NODE NodeType;

 protected:

    /**
     * Generates a 16-bit key from/for given value when it is within
     * the octree bounds, returns false otherwise
     *
     * @param val coordinate of one dimension in the octree
     * @param key 16bit key of the given coordinate, returned
     * @return true if val is within the octree bounds
     */
    bool genKeyValue(double coordinate, unsigned short int& keyval) const;


    /// generates a new key value at a specified depth in the tree given a key value at the final tree depth
    bool genKeyValueAtDepth(const unsigned short int keyval, unsigned int depth, unsigned short int &out_keyval) const;

    /// generates a new key for all three dimensions at a specified depth, calling genKeyValueAtDepth
    bool genKeyAtDepth(const OcTreeKey& key, unsigned int depth, OcTreeKey& out_key) const;

    /// reverse of genKey(), generates center coordinate of cell corresponding to a key for cells not on the last level
    bool genCoordFromKey(const unsigned short int& key, float& coord) const;

    /// generates the center coordinate of a cell for a given key at the last level
    bool genLastCoordFromKey(const unsigned short int& key, float& coord) const;

    // generates 3d coordinates from a key at a given depth
    bool genCoords(const OcTreeKey& key, unsigned int depth, point3d& point) const;

    /// generate child index (between 0 and 7) from key at given tree depth
    void genPos(const OcTreeKey& key, int depth, unsigned int& pos) const;

    /// compute center point of child voxel cell, for internal use
    void computeChildCenter (const unsigned int& pos, const double& center_offset, 
                             const point3d& parent_center, point3d& child_center) const;
    /// compute OcTreeKey of child voxel cell, for internal use
    void computeChildKey (const unsigned int& pos, const unsigned short int& center_offset_key, 
                          const OcTreeKey& parent_key, OcTreeKey& child_key) const;


    /// recalculates min and max in x, y, z. Does nothing when tree size didn't change.
    void calcMinMax();

    void calcNumNodesRecurs(NODE* node, unsigned int& num_nodes) const;


    /// recursive call of prune()
    void pruneRecurs(NODE* node, unsigned int depth, unsigned int max_depth, unsigned int& num_pruned);

    /// recursive call of expand()
    void expandRecurs(NODE* node, unsigned int depth, unsigned int max_depth, unsigned int& num_expanded);
    
    /// Recursive call for getLeafNodes()
    void getLeafNodesRecurs(point3d_list& node_centers, unsigned int max_depth,
                            NODE* node, unsigned int depth, const point3d& parent_center) const;

    /// Recursive call for getVoxels()
    void getVoxelsRecurs(std::list<OcTreeVolume>& nodes, unsigned int max_depth,
                         NODE* node, unsigned int depth, const point3d& parent_center) const;
    
  protected:

    NODE* itsRoot;

    // constants of the tree
    unsigned int tree_depth;
    unsigned int tree_max_val;
    double resolution;  ///< in meters
    double resolution_factor; ///< = 1. / resolution
  
    unsigned int tree_size; ///< number of nodes in tree
    bool sizeChanged;

    point3d tree_center;  // coordinate offset of tree

    double maxValue[3]; ///< max in x, y, z
    double minValue[3]; ///< min in x, y, z

    KeyRay keyray;  // data structure for ray casting
  };


}

#include <octomap/OcTreeBase.hxx>

#endif
