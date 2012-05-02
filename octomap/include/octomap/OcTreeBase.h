#ifndef OCTOMAP_OCTREE_BASE_H
#define OCTOMAP_OCTREE_BASE_H

// $Id$

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009-2011.
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
#include <iterator>
#include <stack>
#include <cassert>

#include "octomap_types.h"
#include "OcTreeKey.h"
#include "ScanGraph.h"
#include <octomap/AbstractOcTree.h>


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
    class OcTreeBase : public AbstractOcTree {

  public:
    /// Make the templated NODE type available from the outside
    typedef NODE NodeType;
    
    OcTreeBase(double _resolution);
    virtual ~OcTreeBase();

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTreeBase<NODE>* create() const {return new OcTreeBase<NODE>(resolution); }

    std::string getTreeType() const {return "OcTreeBase";}

    void setResolution(double r);
    inline double getResolution() const { return resolution; }

    inline unsigned int getTreeDepth () const { return tree_depth; }

    inline double getNodeSize(unsigned depth) const {assert(depth <= tree_depth); return sizeLookupTable[depth];}

    /**
     * \return Pointer to the root node of the tree. This pointer
     * should not be modified or deleted externally, the OcTree
     * manages its memory itself.
     */
    inline NODE* getRoot() const { return itsRoot; }

    /** 
     *  Search node given a 3d point
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search(float x, float y, float z) const;

    /**
     *  Search node given a 3d point
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search(const point3d& value) const;

    /**
     *  Search a node given an addressing key
     *  @return pointer to node if found, NULL otherwise
     */
    NODE* search(const OcTreeKey& key) const;

    /**
     *  Delete a node (if exists) given a 3d point. Will always
     *  delete at the lowest level unless depth !=0, and expand pruned inner nodes as needed.
     *  Pruned nodes at level "depth" will directly be deleted as a whole.
     */
    bool deleteNode(float x, float y, float z, unsigned int depth = 0);

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

    /// Deletes the complete tree structure (only the root node will remain)
    void clear();


    /// Lossless compression of OcTree: merge children to parent when there are
    /// eight children with identical values
    virtual void prune();

    /// Expands all pruned nodes (reverse of prune())
    /// \note This is an expensive operation, especially when the tree is nearly empty!
    virtual void expand();

    // -- statistics  ----------------------

    /// \return The number of nodes in the tree
    virtual inline size_t size() const { return tree_size; }

    /// \return Memory usage of the complete octree in bytes (may vary between architectures)
    virtual size_t memoryUsage() const;

    /// \return Memory usage of the a single octree node
    virtual inline size_t memoryUsageNode() const {return sizeof(NODE); };

    /// \return Memory usage of a full grid of the same size as the OcTree in bytes (for comparison)
    size_t memoryFullGrid();

    double volume();

    /// Size of OcTree in meters for x, y and z dimension
    virtual void getMetricSize(double& x, double& y, double& z);
    /// minimum value in x, y, z
    virtual void getMetricMin(double& x, double& y, double& z);
    void getMetricMin(double& x, double& y, double& z) const;
    /// maximum value in x, y, z
    virtual void getMetricMax(double& x, double& y, double& z);
    void getMetricMax(double& x, double& y, double& z) const;

    /// Traverses the tree to calculate the total number of nodes
    size_t calcNumNodes() const;

    /// Traverses the tree to calculate the total number of leaf nodes
    size_t getNumLeafNodes() const;


    // -- access tree nodes  ------------------

    /// return centers of leafs that do NOT exist (but could) in a given bounding box
    void getUnknownLeafCenters(point3d_list& node_centers, point3d pmin, point3d pmax) const;


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

    /**
     * Generates key for all three dimensions of a given point
     * using genKeyValue().
     *
     * @param point 3d coordinate of a point
     * @param keys values that will be computed, an array of fixed size 3.
     * @return true when point is within the octree, false otherwise
     */
    bool genKey(const point3d& point, OcTreeKey& key) const;


    // file IO

    /**
     * Read all nodes from the input stream (without file header),
     * for this the tree needs to be already created.
     * For general file IO, you
     * should probably use AbstractOcTree::read() instead.
     */
    std::istream& readData(std::istream &s);

    /// Write complete state of tree to stream (without file header), prune tree first (lossless compression)
    std::ostream& writeData(std::ostream &s);

    /// Write complete state of tree to stream (without file header), no pruning (const version)
    std::ostream& writeDataConst(std::ostream &s) const;


    /**
     * Base class for OcTree iterators. So far, all iterator's are
     * const with respect to the tree
     */
    class iterator_base : std::iterator<std::forward_iterator_tag, NodeType>{
    public:
      struct StackElement;
      /// Default ctor, only used for the end-iterator
      iterator_base() : tree(NULL), maxDepth(0){}

      /**
       * Constructor of the iterator.
       * 
       * @param tree OcTreeBase on which the iterator is used on
       * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
       */
      iterator_base(OcTreeBase<NodeType> const* tree, unsigned char depth=0)
        : tree(tree), maxDepth(depth)
      {
        if (maxDepth == 0)
          maxDepth = tree->getTreeDepth();

        StackElement s;
        s.node = tree->itsRoot;
        s.depth = 0;
        s.key[0] = s.key[1] = s.key[2] = tree->tree_max_val;
        stack.push(s);
      }

      /// Copy constructor of the iterator
      iterator_base(const iterator_base& other)
      : tree(other.tree), maxDepth(other.maxDepth), stack(other.stack) {}

      /// Comparison between interators. First compares the tree, then stack size and top element of stack.
      bool operator==(const iterator_base& other) const {
        return (tree ==other.tree && stack.size() == other.stack.size()
            && (stack.size()==0 || (stack.size() > 0 && (stack.top().node == other.stack.top().node
                && stack.top().depth == other.stack.top().depth
                && stack.top().key == other.stack.top().key ))));
      }

      /// Comparison between interators. First compares the tree, then stack size and top element of stack.
      bool operator!=(const iterator_base& other) const {
        return (tree !=other.tree || stack.size() != other.stack.size()
            || (stack.size() > 0 && ((stack.top().node != other.stack.top().node
                || stack.top().depth != other.stack.top().depth
                || stack.top().key != other.stack.top().key ))));
      }

      iterator_base& operator=(const iterator_base& other){
        tree = other.tree;
        maxDepth = other.maxDepth;
        stack = other.stack;
        return *this;
      };

      /// Ptr operator will return the current node in the octree which the
      /// iterator is referring to
      NodeType const* operator->() const { return stack.top().node;}
      
      /// Ptr operator will return the current node in the octree which the
      /// iterator is referring to
      NodeType* operator->() { return stack.top().node;}
      
      /// Return the current node in the octree which the
      /// iterator is referring to
      const NodeType& operator*() const { return *(stack.top().node);}
      
      /// Return the current node in the octree which the
      /// iterator is referring to
      NodeType& operator*() { return *(stack.top().node);}

      /// return the center coordinate of the current node
      point3d getCoordinate() const {
        point3d coord;
        tree->genCoords(stack.top().key, stack.top().depth, coord);
        return coord;
      }

      /// @return single coordinate of the current node
      double getX() const{
        return tree->genCoordFromKey(stack.top().key[0], stack.top().depth);
      }
      /// @return single coordinate of the current node
      double getY() const{
        return tree->genCoordFromKey(stack.top().key[1], stack.top().depth);
      }
      /// @return single coordinate of the current node
      double getZ() const{
        return tree->genCoordFromKey(stack.top().key[2], stack.top().depth);
      }

      /// @return the side if the volume occupied by the current node
      double getSize() const {return  tree->getNodeSize(stack.top().depth); }
      
      /// return depth of the current node
      unsigned getDepth() const {return unsigned(stack.top().depth); }
      
      /// @return the OcTreeKey of the current node
      const OcTreeKey& getKey() const {return stack.top().key;}

      /// @return the OcTreeKey of the current node, for nodes with depth != maxDepth
      OcTreeKey getIndexKey() const {
      	unsigned short int mask = 65535 << (16 - stack.top().depth);
      	OcTreeKey result = stack.top().key;
      	result[0] &= mask;
      	result[1] &= mask;
      	result[2] &= mask;
      	return result;
      }


      /// Element on the internal recursion stack of the iterator
      struct StackElement{
        NodeType* node;
        OcTreeKey key;
        unsigned char depth;
      };


    protected:
      /// One step of depth-first tree traversal. 
      /// How this is used depends on the actual iterator.
      void singleIncrement(){
        StackElement top = stack.top();
        stack.pop();
        if (top.depth == maxDepth)
          return;

        StackElement s;
        s.depth = top.depth +1;

        unsigned short int center_offset_key = tree->tree_max_val >> (top.depth +1);
        // push on stack in reverse order
        for (int i=7; i>=0; --i) {
          if (top.node->childExists(i)) {
            computeChildKey(i, center_offset_key, top.key, s.key);
            s.node = top.node->getChild(i);
            //OCTOMAP_DEBUG_STR("Current depth: " << int(top.depth) << " new: "<< int(s.depth) << " child#" << i <<" ptr: "<<s.node);
            stack.push(s);
            assert(s.depth <= maxDepth);
          }
        }
      }

      OcTreeBase<NodeType> const* tree; ///< Octree this iterator is working on
      unsigned char maxDepth; ///< Maximum depth for depth-limited queries

      /// Internal recursion stack. Apparently a stack of vector works fastest here.
      std::stack<StackElement,std::vector<StackElement> > stack;

    };

    /**
     * Iterator over the complete tree (inner nodes and leafs).
     * See below for example usage.
     * Note that the non-trivial call to tree->end_tree() should be done only once
     * for efficiency!
     *
     * @code
     * for(OcTreeTYPE::tree_iterator it = tree->begin_tree(),
     *        end=tree->end_tree(); it!= end; ++it)
     * {
     *   //manipulate node, e.g.:
     *   std::cout << "Node center: " << it.getCoordinate() << std::endl;
     *   std::cout << "Node size: " << it.getSize() << std::endl;
     *   std::cout << "Node value: " << it->getValue() << std::endl;
     * }
     * @endcode
     */
    class tree_iterator : public iterator_base {
    public:
      tree_iterator() : iterator_base(){}
      /**
       * Constructor of the iterator.
       * 
       * @param tree OcTreeBase on which the iterator is used on
       * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
       */
      tree_iterator(OcTreeBase<NodeType> const* tree, unsigned char depth=0) : iterator_base(tree, depth) {};

      /// postfix increment operator of iterator (it++)
      tree_iterator operator++(int){
        tree_iterator result = *this;
        ++(*this);
        return result;
      }

      /// Prefix increment operator to advance the iterator
      tree_iterator& operator++(){

        if (!this->stack.empty()){
          this->singleIncrement();
        }

        if (this->stack.empty()){
          this->tree = NULL;
        }

        return *this;
      }

      /// @return whether the current node is a leaf, i.e. has no children or is at max level
      bool isLeaf() const{ return (!this->stack.top().node->hasChildren() || this->stack.top().depth == this->maxDepth); }
    };

    /**
     * Iterator to iterate over all leafs of the tree. 
     * Inner nodes are skipped. See below for example usage.
     * Note that the non-trivial call to tree->end_leafs() should be done only once
     * for efficiency!
     *
     * @code
     * for(OcTreeTYPE::leaf_iterator it = tree->begin_leafs(),
     *        end=tree->end_leafs(); it!= end; ++it)
     * {
     *   //manipulate node, e.g.:
     *   std::cout << "Node center: " << it.getCoordinate() << std::endl;
     *   std::cout << "Node size: " << it.getSize() << std::endl;
     *   std::cout << "Node value: " << it->getValue() << std::endl;
     * }
     * @endcode
     *
     */
    class leaf_iterator : public iterator_base {
      public:
          leaf_iterator() : iterator_base(){}
          
          /**
          * Constructor of the iterator.
          * 
          * @param tree OcTreeBase on which the iterator is used on
          * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
          */
          leaf_iterator(OcTreeBase<NodeType> const* tree, unsigned char depth=0) : iterator_base(tree, depth) {
            // skip forward to next valid leaf node:
            // add root another time (one will be removed) and ++
            this->stack.push(this->stack.top());
            operator ++();
          }

          leaf_iterator(const leaf_iterator& other) : iterator_base(other) {};

          /// postfix increment operator of iterator (it++)
          leaf_iterator operator++(int){
            leaf_iterator result = *this;
            ++(*this);
            return result;
          }

          /// prefix increment operator of iterator (++it)
          leaf_iterator& operator++(){
            if (this->stack.empty()){
              this->tree = NULL; // TODO check?

            } else {
              this->stack.pop();

              // skip forward to next leaf
              while(!this->stack.empty()  && this->stack.top().depth < this->maxDepth
                  && this->stack.top().node->hasChildren())
              {
                this->singleIncrement();
              }
              // done: either stack is empty (== end iterator) or a next leaf node is reached!
              if (this->stack.empty())
                this->tree = NULL;
            }


            return *this;
          }

    };

    /**
     * Bounding-box leaf iterator. This iterator will traverse all leaf nodes
     * within a given bounding box (axis-aligned). See below for example usage.
     * Note that the non-trivial call to tree->end_leafs_bbx() should be done only once
     * for efficiency!
     *
     * @code
     * for(OcTreeTYPE::leaf_bbx_iterator it = tree->begin_leafs_bbx(min,max),
     *        end=tree->end_leafs_bbx(); it!= end; ++it)
     * {
     *   //manipulate node, e.g.:
     *   std::cout << "Node center: " << it.getCoordinate() << std::endl;
     *   std::cout << "Node size: " << it.getSize() << std::endl;
     *   std::cout << "Node value: " << it->getValue() << std::endl;
     * }
     * @endcode
     */
    class leaf_bbx_iterator : public iterator_base {
    public:
      leaf_bbx_iterator() : iterator_base() {};
      /**
      * Constructor of the iterator. The bounding box corners min and max are
      * converted into an OcTreeKey first.
      *
      * @note Due to rounding and discretization
      * effects, nodes may be traversed that have float coordinates appearing
      * outside of the (float) bounding box. However, the node's complete volume
      * will include the bounding box coordinate. For a more exact control, use
      * the constructor with OcTreeKeys instead.
      * 
      * @param tree OcTreeBase on which the iterator is used on
      * @param min Minimum point3d of the axis-aligned boundingbox
      * @param max Maximum point3d of the axis-aligned boundingbox
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */      
      leaf_bbx_iterator(OcTreeBase<NodeType> const* tree, const point3d& min, const point3d& max, unsigned char depth=0)
        : iterator_base(tree, depth)
      {

        if (!this->tree->genKey(min, minKey) || !this->tree->genKey(max, maxKey)){
          // coordinates invalid, set to end iterator
          tree = NULL;
          this->maxDepth = 0;
        } else{  // else: keys are generated and stored

          // advance from root to next valid leaf in bbx:
          this->stack.push(this->stack.top());
          this->operator ++();
        }

      }

      /**
      * Constructor of the iterator. This version uses the exact keys as axis-aligned
      * bounding box (including min and max).
      *
      * @param tree OcTreeBase on which the iterator is used on
      * @param min Minimum OcTreeKey to be included in the axis-aligned boundingbox
      * @param max Maximum OcTreeKey to be included in the axis-aligned boundingbox
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      leaf_bbx_iterator(OcTreeBase<NodeType> const* tree, const OcTreeKey& min, const OcTreeKey& max, unsigned char depth=0)
        : iterator_base(tree, depth), minKey(min), maxKey(max)
      {
          // advance from root to next valid leaf in bbx:
          this->stack.push(this->stack.top());
          this->operator ++();
      }

      leaf_bbx_iterator(const leaf_bbx_iterator& other) : iterator_base(other) {
        minKey = other.minKey;
        maxKey = other.maxKey;
      }



      /// postfix increment operator of iterator (it++)
      leaf_bbx_iterator operator++(int){
        leaf_bbx_iterator result = *this;
        ++(*this);
        return result;
      }

      /// prefix increment operator of iterator (++it)
      leaf_bbx_iterator& operator++(){
        if (this->stack.empty()){
          this->tree = NULL; // TODO check?

        } else {
          this->stack.pop();

          // skip forward to next leaf
          while(!this->stack.empty()  && this->stack.top().depth < this->maxDepth
              && this->stack.top().node->hasChildren())
          {
            this->singleIncrement();
          }
          // done: either stack is empty (== end iterator) or a next leaf node is reached!
          if (this->stack.empty())
            this->tree = NULL;
        }


        return *this;
      };

    protected:

      void singleIncrement(){
        typename iterator_base::StackElement top = this->stack.top();
        this->stack.pop();

        typename iterator_base::StackElement s;
        s.depth = top.depth +1;
        unsigned short int center_offset_key = this->tree->tree_max_val >> (top.depth +1);
        // push on stack in reverse order
        for (int i=7; i>=0; --i) {
          if (top.node->childExists(i)) {
            computeChildKey(i, center_offset_key, top.key, s.key);

            // overlap of query bbx and child bbx?
            if ((minKey[0] <= (s.key[0] + center_offset_key)) && (maxKey[0] >= (s.key[0] - center_offset_key))
                && (minKey[1] <= (s.key[1] + center_offset_key)) && (maxKey[1] >= (s.key[1] - center_offset_key))
                && (minKey[2] <= (s.key[2] + center_offset_key)) && (maxKey[2] >= (s.key[2] - center_offset_key)))
            {
              s.node = top.node->getChild(i);
              this->stack.push(s);
              assert(s.depth <= this->maxDepth);
            }
          }
        }
      }


      OcTreeKey minKey;
      OcTreeKey maxKey;
    };

    // default iterator is leaf_iterator
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
    /// This checks if the key is valid and returns the success.
    DEPRECATED( bool genCoordFromKey(const unsigned short int& key, unsigned depth, float& coord) const ){
      return genCoordFromKey(key, coord, depth);
    }
    bool genCoordFromKey(const unsigned short int& key, float& coord, unsigned depth) const;
    inline bool genCoordFromKey(const unsigned short int& key, float& coord) const {
      return genCoordFromKey(key, coord, tree_depth);
    }

    /// reverse of genKey(), generates center coordinate of cell corresponding to a key for cells not on the last level
    /// returns the coordinate without checking for validity.
    double genCoordFromKey(const unsigned short int& key, unsigned depth) const;
    inline double genCoordFromKey(const unsigned short int& key) const {
      return genCoordFromKey(key, tree_depth);
    }

    // generates 3d coordinates from a key at a given depth
    bool genCoords(const OcTreeKey& key, unsigned int depth, point3d& point) const;

    /// generate child index (between 0 and 7) from key at given tree depth
    void genPos(const OcTreeKey& key, int depth, unsigned int& pos) const;

 protected:
    /// compute center point of child voxel cell, for internal use (will be removed soon)
    void computeChildCenter (const unsigned int& pos, const float& center_offset, 
                             const point3d& parent_center, point3d& child_center) const;

    /// recalculates min and max in x, y, z. Does nothing when tree size didn't change.
    void calcMinMax();

    void calcNumNodesRecurs(NODE* node, size_t& num_nodes) const;

    /// recursive call of deleteNode()
    bool deleteNodeRecurs(NODE* node, unsigned int depth, unsigned int max_depth, const OcTreeKey& key);

    /// recursive call of prune()
    void pruneRecurs(NODE* node, unsigned int depth, unsigned int max_depth, unsigned int& num_pruned);

    /// recursive call of expand()
    void expandRecurs(NODE* node, unsigned int depth, unsigned int max_depth);
    
    size_t getNumLeafNodesRecurs(const NODE* parent) const;

  protected:

    NODE* itsRoot;

    // constants of the tree
    const unsigned int tree_depth; ///< Maximum tree depth is fixed to 16 currently
    const unsigned int tree_max_val;
    double resolution;  ///< in meters
    double resolution_factor; ///< = 1. / resolution
  
    size_t tree_size; ///< number of nodes in tree
    /// flag to denote whether the octree extent changed (for lazy min/max eval)
    bool sizeChanged;

    point3d tree_center;  // coordinate offset of tree

    double maxValue[3]; ///< max in x, y, z
    double minValue[3]; ///< min in x, y, z
    /// contains the size of a voxel at level i (0: root node). tree_depth+1 levels (incl. 0)
    std::vector<double> sizeLookupTable;

    KeyRay keyray;  // data structure for ray casting

    const leaf_iterator leaf_iterator_end;
    const leaf_bbx_iterator leaf_iterator_bbx_end;
    const tree_iterator tree_iterator_end;
  };

}

#include <octomap/OcTreeBase.hxx>

#endif
