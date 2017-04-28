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

#ifndef OCTOMAP_OCTREE_BASE_IMPL_H
#define OCTOMAP_OCTREE_BASE_IMPL_H


#include <list>
#include <limits>
#include <iterator>
#include <stack>
#include <bitset>

#include "octomap_types.h"
#include "OcTreeKey.h"
#include "ScanGraph.h"


namespace octomap {
  
  // forward declaration for NODE children array
  class AbstractOcTreeNode;


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
   * \note You should probably not use this class directly, but
   * OcTreeBase or OccupancyOcTreeBase instead
   *
   * \tparam NODE Node class to be used in tree (usually derived from
   *    OcTreeDataNode)
   * \tparam INTERFACE Interface to be derived from, should be either
   *    AbstractOcTree or AbstractOccupancyOcTree
   */
  template <class NODE,class INTERFACE>
  class OcTreeBaseImpl : public INTERFACE {

  public:
    /// Make the templated NODE type available from the outside
    typedef NODE NodeType;

    // the actual iterator implementation is included here
    // as a member from this file
    #include <octomap/OcTreeIterator.hxx>
    
    OcTreeBaseImpl(double resolution);
    virtual ~OcTreeBaseImpl();

    /// Deep copy constructor
    OcTreeBaseImpl(const OcTreeBaseImpl<NODE,INTERFACE>& rhs);


    /**
     * Swap contents of two octrees, i.e., only the underlying
     * pointer / tree structure. You have to ensure yourself that the
     * metadata (resolution etc) matches. No memory is cleared
     * in this function
     */
    void swapContent(OcTreeBaseImpl<NODE,INTERFACE>& rhs);

    /// Comparison between two octrees, all meta data, all
    /// nodes, and the structure must be identical
    bool operator== (const OcTreeBaseImpl<NODE,INTERFACE>& rhs) const;

    std::string getTreeType() const {return "OcTreeBaseImpl";}

    /// Change the resolution of the octree, scaling all voxels.
    /// This will not preserve the (metric) scale!
    void setResolution(double r);
    inline double getResolution() const { return resolution; }

    inline unsigned int getTreeDepth () const { return tree_depth; }

    inline double getNodeSize(unsigned depth) const {assert(depth <= tree_depth); return sizeLookupTable[depth];}
    
    /**
     * Clear KeyRay vector to minimize unneeded memory. This is only
     * useful for the StaticMemberInitializer classes, don't call it for
     * an octree that is actually used.
     */
    void clearKeyRays(){
      keyrays.clear();
    }
    
    // -- Tree structure operations formerly contained in the nodes ---
   
    /// Creates (allocates) the i-th child of the node. @return ptr to newly create NODE
    NODE* createNodeChild(NODE* node, unsigned int childIdx);
    
    /// Deletes the i-th child of the node
    void deleteNodeChild(NODE* node, unsigned int childIdx);
    
    /// @return ptr to child number childIdx of node
    NODE* getNodeChild(NODE* node, unsigned int childIdx) const;
    
    /// @return const ptr to child number childIdx of node
    const NODE* getNodeChild(const NODE* node, unsigned int childIdx) const;
    
    /// A node is collapsible if all children exist, don't have children of their own
    /// and have the same occupancy value
    virtual bool isNodeCollapsible(const NODE* node) const;
    
    /** 
     * Safe test if node has a child at index childIdx.
     * First tests if there are any children. Replaces node->childExists(...)
     * \return true if the child at childIdx exists
     */
    bool nodeChildExists(const NODE* node, unsigned int childIdx) const;
    
    /** 
     * Safe test if node has any children. Replaces node->hasChildren(...)
     * \return true if node has at least one child
     */
    bool nodeHasChildren(const NODE* node) const;
    
    /**
     * Expands a node (reverse of pruning): All children are created and
     * their occupancy probability is set to the node's value.
     *
     * You need to verify that this is indeed a pruned node (i.e. not a
     * leaf at the lowest level)
     *
     */
    virtual void expandNode(NODE* node);
    
    /**
     * Prunes a node when it is collapsible
     * @return true if pruning was successful
     */
    virtual bool pruneNode(NODE* node);
    
    
    // --------

    /**
     * \return Pointer to the root node of the tree. This pointer
     * should not be modified or deleted externally, the OcTree
     * manages its memory itself. In an empty tree, root is NULL.
     */
    inline NODE* getRoot() const { return root; }

    /** 
     *  Search node at specified depth given a 3d point (depth=0: search full tree depth).
     *  You need to check if the returned node is NULL, since it can be in unknown space.
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search(double x, double y, double z, unsigned int depth = 0) const;

    /**
     *  Search node at specified depth given a 3d point (depth=0: search full tree depth)
     *  You need to check if the returned node is NULL, since it can be in unknown space.
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search(const point3d& value, unsigned int depth = 0) const;

    /**
     *  Search a node at specified depth given an addressing key (depth=0: search full tree depth)
     *  You need to check if the returned node is NULL, since it can be in unknown space.
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search(const OcTreeKey& key, unsigned int depth = 0) const;

    /**
     *  Delete a node (if exists) given a 3d point. Will always
     *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
     *  Pruned nodes at level "depth" will directly be deleted as a whole.
     */
    bool deleteNode(double x, double y, double z, unsigned int depth = 0);

    /** 
     *  Delete a node (if exists) given a 3d point. Will always
     *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
     *  Pruned nodes at level "depth" will directly be deleted as a whole.
     */
    bool deleteNode(const point3d& value, unsigned int depth = 0);

    /** 
     *  Delete a node (if exists) given an addressing key. Will always
     *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
     *  Pruned nodes at level "depth" will directly be deleted as a whole.
     */
    bool deleteNode(const OcTreeKey& key, unsigned int depth = 0);

    /// Deletes the complete tree structure
    void clear();

    /**
     * Lossless compression of the octree: A node will replace all of its eight
     * children if they have identical values. You usually don't have to call
     * prune() after a regular occupancy update, updateNode() incrementally
     * prunes all affected nodes.
     */
    virtual void prune();

    /// Expands all pruned nodes (reverse of prune())
    /// \note This is an expensive operation, especially when the tree is nearly empty!
    virtual void expand();

    // -- statistics  ----------------------

    /// \return The number of nodes in the tree
    virtual inline size_t size() const { return tree_size; }

    /// \return Memory usage of the complete octree in bytes (may vary between architectures)
    virtual size_t memoryUsage() const;

    /// \return Memory usage of a single octree node
    virtual inline size_t memoryUsageNode() const {return sizeof(NODE); };

    /// \return Memory usage of a full grid of the same size as the OcTree in bytes (for comparison)
    /// \note this can be larger than the adressable memory - size_t may not be enough to hold it!
    unsigned long long memoryFullGrid() const;

    double volume();

    /// Size of OcTree (all known space) in meters for x, y and z dimension
    virtual void getMetricSize(double& x, double& y, double& z);
    /// Size of OcTree (all known space) in meters for x, y and z dimension
    virtual void getMetricSize(double& x, double& y, double& z) const;
    /// minimum value of the bounding box of all known space in x, y, z
    virtual void getMetricMin(double& x, double& y, double& z);
    /// minimum value of the bounding box of all known space in x, y, z
    void getMetricMin(double& x, double& y, double& z) const;
    /// maximum value of the bounding box of all known space in x, y, z
    virtual void getMetricMax(double& x, double& y, double& z);
    /// maximum value of the bounding box of all known space in x, y, z
    void getMetricMax(double& x, double& y, double& z) const;

    /// Traverses the tree to calculate the total number of nodes
    size_t calcNumNodes() const;

    /// Traverses the tree to calculate the total number of leaf nodes
    size_t getNumLeafNodes() const;


    // -- access tree nodes  ------------------

    /// return centers of leafs that do NOT exist (but could) in a given bounding box
    void getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax, unsigned int depth = 0) const;


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
    bool computeRay(const point3d& origin, const point3d& end, std::vector<point3d>& ray);


    // file IO

    /**
     * Read all nodes from the input stream (without file header),
     * for this the tree needs to be already created.
     * For general file IO, you
     * should probably use AbstractOcTree::read() instead.
     */
    std::istream& readData(std::istream &s);

    /// Write complete state of tree to stream (without file header) unmodified.
    /// Pruning the tree first produces smaller files (lossless compression)
    std::ostream& writeData(std::ostream &s) const;

    typedef leaf_iterator iterator;

    /// @return beginning of the tree as leaf iterator
    iterator begin(unsigned char maxDepth=0) const {return iterator(this, maxDepth);};
    /// @return end of the tree as leaf iterator
    const iterator end() const {return leaf_iterator_end;}; // TODO: RVE?

    /// @return beginning of the tree as leaf iterator
    leaf_iterator begin_leafs(unsigned char maxDepth=0) const {return leaf_iterator(this, maxDepth);};
    /// @return end of the tree as leaf iterator
    const leaf_iterator end_leafs() const {return leaf_iterator_end;}

    /// @return beginning of the tree as leaf iterator in a bounding box
    leaf_bbx_iterator begin_leafs_bbx(const OcTreeKey& min, const OcTreeKey& max, unsigned char maxDepth=0) const {
      return leaf_bbx_iterator(this, min, max, maxDepth);
    }
    /// @return beginning of the tree as leaf iterator in a bounding box
    leaf_bbx_iterator begin_leafs_bbx(const point3d& min, const point3d& max, unsigned char maxDepth=0) const {
      return leaf_bbx_iterator(this, min, max, maxDepth);
    }
    /// @return end of the tree as leaf iterator in a bounding box
    const leaf_bbx_iterator end_leafs_bbx() const {return leaf_iterator_bbx_end;}

    /// @return beginning of the tree as iterator to all nodes (incl. inner)
    tree_iterator begin_tree(unsigned char maxDepth=0) const {return tree_iterator(this, maxDepth);}
    /// @return end of the tree as iterator to all nodes (incl. inner)
    const tree_iterator end_tree() const {return tree_iterator_end;}

    //
    // Key / coordinate conversion functions
    //

    /// Converts from a single coordinate into a discrete key
    inline key_type coordToKey(double coordinate) const{
      return ((int) floor(resolution_factor * coordinate)) + tree_max_val;
    }

    /// Converts from a single coordinate into a discrete key at a given depth
    key_type coordToKey(double coordinate, unsigned depth) const;


    /// Converts from a 3D coordinate into a 3D addressing key
    inline OcTreeKey coordToKey(const point3d& coord) const{
      return OcTreeKey(coordToKey(coord(0)), coordToKey(coord(1)), coordToKey(coord(2)));
    }

    /// Converts from a 3D coordinate into a 3D addressing key
    inline OcTreeKey coordToKey(double x, double y, double z) const{
      return OcTreeKey(coordToKey(x), coordToKey(y), coordToKey(z));
    }

    /// Converts from a 3D coordinate into a 3D addressing key at a given depth
    inline OcTreeKey coordToKey(const point3d& coord, unsigned depth) const{
      if (depth == tree_depth)
        return coordToKey(coord);
      else
        return OcTreeKey(coordToKey(coord(0), depth), coordToKey(coord(1), depth), coordToKey(coord(2), depth));
    }

    /// Converts from a 3D coordinate into a 3D addressing key at a given depth
    inline OcTreeKey coordToKey(double x, double y, double z, unsigned depth) const{
      if (depth == tree_depth)
        return coordToKey(x,y,z);
      else
        return OcTreeKey(coordToKey(x, depth), coordToKey(y, depth), coordToKey(z, depth));
    }

    /**
     * Adjusts a 3D key from the lowest level to correspond to a higher depth (by
     * shifting the key values)
     *
     * @param key Input key, at the lowest tree level
     * @param depth Target depth level for the new key
     * @return Key for the new depth level
     */
    inline OcTreeKey adjustKeyAtDepth(const OcTreeKey& key, unsigned int depth) const{
      if (depth == tree_depth)
        return key;

      assert(depth <= tree_depth);
      return OcTreeKey(adjustKeyAtDepth(key[0], depth), adjustKeyAtDepth(key[1], depth), adjustKeyAtDepth(key[2], depth));
    }

    /**
     * Adjusts a single key value from the lowest level to correspond to a higher depth (by
     * shifting the key value)
     *
     * @param key Input key, at the lowest tree level
     * @param depth Target depth level for the new key
     * @return Key for the new depth level
     */
    key_type adjustKeyAtDepth(key_type key, unsigned int depth) const;

    /**
     * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
     *
     * @param coord 3d coordinate of a point
     * @param key values that will be computed, an array of fixed size 3.
     * @return true if point is within the octree (valid), false otherwise
     */
    bool coordToKeyChecked(const point3d& coord, OcTreeKey& key) const;

    /**
     * Converts a 3D coordinate into a 3D OcTreeKey at a certain depth, with boundary checking.
     *
     * @param coord 3d coordinate of a point
     * @param depth level of the key from the top
     * @param key values that will be computed, an array of fixed size 3.
     * @return true if point is within the octree (valid), false otherwise
     */
    bool coordToKeyChecked(const point3d& coord, unsigned depth, OcTreeKey& key) const;

    /**
     * Converts a 3D coordinate into a 3D OcTreeKey, with boundary checking.
     *
     * @param x
     * @param y
     * @param z
     * @param key values that will be computed, an array of fixed size 3.
     * @return true if point is within the octree (valid), false otherwise
     */
    bool coordToKeyChecked(double x, double y, double z, OcTreeKey& key) const;

    /**
     * Converts a 3D coordinate into a 3D OcTreeKey at a certain depth, with boundary checking.
     *
     * @param x
     * @param y
     * @param z
     * @param depth level of the key from the top
     * @param key values that will be computed, an array of fixed size 3.
     * @return true if point is within the octree (valid), false otherwise
     */
    bool coordToKeyChecked(double x, double y, double z, unsigned depth, OcTreeKey& key) const;

    /**
     * Converts a single coordinate into a discrete addressing key, with boundary checking.
     *
     * @param coordinate 3d coordinate of a point
     * @param key discrete 16 bit adressing key, result
     * @return true if coordinate is within the octree bounds (valid), false otherwise
     */
    bool coordToKeyChecked(double coordinate, key_type& key) const;

    /**
     * Converts a single coordinate into a discrete addressing key, with boundary checking.
     *
     * @param coordinate 3d coordinate of a point
     * @param depth level of the key from the top
     * @param key discrete 16 bit adressing key, result
     * @return true if coordinate is within the octree bounds (valid), false otherwise
     */
    bool coordToKeyChecked(double coordinate, unsigned depth, key_type& key) const;

    /// converts from a discrete key at a given depth into a coordinate
    /// corresponding to the key's center
    double keyToCoord(key_type key, unsigned depth) const;

    /// converts from a discrete key at the lowest tree level into a coordinate
    /// corresponding to the key's center
    inline double keyToCoord(key_type key) const{
      return (double( (int) key - (int) this->tree_max_val ) +0.5) * this->resolution;
    }

    /// converts from an addressing key at the lowest tree level into a coordinate
    /// corresponding to the key's center
    inline point3d keyToCoord(const OcTreeKey& key) const{
      return point3d(float(keyToCoord(key[0])), float(keyToCoord(key[1])), float(keyToCoord(key[2])));
    }

    /// converts from an addressing key at a given depth into a coordinate
    /// corresponding to the key's center
    inline point3d keyToCoord(const OcTreeKey& key, unsigned depth) const{
      return point3d(float(keyToCoord(key[0], depth)), float(keyToCoord(key[1], depth)), float(keyToCoord(key[2], depth)));
    }

 protected:
    /// Constructor to enable derived classes to change tree constants.
    /// This usually requires a re-implementation of some core tree-traversal functions as well!
    OcTreeBaseImpl(double resolution, unsigned int tree_depth, unsigned int tree_max_val);

    /// initialize non-trivial members, helper for constructors
    void init();

    /// recalculates min and max in x, y, z. Does nothing when tree size didn't change.
    void calcMinMax();

    void calcNumNodesRecurs(NODE* node, size_t& num_nodes) const;
    
    /// recursive call of readData()
    std::istream& readNodesRecurs(NODE*, std::istream &s);
    
    /// recursive call of writeData()
    std::ostream& writeNodesRecurs(const NODE*, std::ostream &s) const;
    
    /// Recursively delete a node and all children. Deallocates memory
    /// but does NOT set the node ptr to NULL nor updates tree size.
    void deleteNodeRecurs(NODE* node);

    /// recursive call of deleteNode()
    bool deleteNodeRecurs(NODE* node, unsigned int depth, unsigned int max_depth, const OcTreeKey& key);

    /// recursive call of prune()
    void pruneRecurs(NODE* node, unsigned int depth, unsigned int max_depth, unsigned int& num_pruned);

    /// recursive call of expand()
    void expandRecurs(NODE* node, unsigned int depth, unsigned int max_depth);
    
    size_t getNumLeafNodesRecurs(const NODE* parent) const;

  private:
    /// Assignment operator is private: don't (re-)assign octrees
    /// (const-parameters can't be changed) -  use the copy constructor instead.
    OcTreeBaseImpl<NODE,INTERFACE>& operator=(const OcTreeBaseImpl<NODE,INTERFACE>&);

  protected:  
    void allocNodeChildren(NODE* node);

    NODE* root; ///< Pointer to the root NODE, NULL for empty tree

    // constants of the tree
    const unsigned int tree_depth; ///< Maximum tree depth is fixed to 16 currently
    const unsigned int tree_max_val;
    double resolution;  ///< in meters
    double resolution_factor; ///< = 1. / resolution
  
    size_t tree_size; ///< number of nodes in tree
    /// flag to denote whether the octree extent changed (for lazy min/max eval)
    bool size_changed;

    point3d tree_center;  // coordinate offset of tree

    double max_value[3]; ///< max in x, y, z
    double min_value[3]; ///< min in x, y, z
    /// contains the size of a voxel at level i (0: root node). tree_depth+1 levels (incl. 0)
    std::vector<double> sizeLookupTable;

    /// data structure for ray casting, array for multithreading
    std::vector<KeyRay> keyrays;

    const leaf_iterator leaf_iterator_end;
    const leaf_bbx_iterator leaf_iterator_bbx_end;
    const tree_iterator tree_iterator_end;


  };

}

#include <octomap/OcTreeBaseImpl.hxx>

#endif
