/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
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

#ifndef OCTOMAP_OCTREEITERATOR_HXX_
#define OCTOMAP_OCTREEITERATOR_HXX_

    /**
     * Base class for OcTree iterators. So far, all iterator's are
     * const with respect to the tree. This file is included within
     * OcTreeBaseImpl.h, you should probably not include this directly.
     */
    class iterator_base : public std::iterator<std::forward_iterator_tag, NodeType>{
    public:
      struct StackElement;
      /// Default ctor, only used for the end-iterator
      iterator_base() : tree(NULL), maxDepth(0){}

      /**
       * Constructor of the iterator. Initializes the iterator with the default
       * constructor (= end() iterator) if tree is empty or NULL.
       *
       * @param tree OcTreeBaseImpl on which the iterator is used on
       * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
       */
      iterator_base(OcTreeBaseImpl<NodeType,INTERFACE> const* ptree, uint8_t depth=0)
        : tree((ptree && ptree->root) ? ptree : NULL), maxDepth(depth)
      {
        if (ptree && maxDepth == 0)
          maxDepth = ptree->getTreeDepth();

        if (tree && tree->root){ // tree is not empty
          StackElement s;
          s.node = tree->root;
          s.depth = 0;
          s.key[0] = s.key[1] = s.key[2] = tree->tree_max_val;
          stack.push(s);
        } else{ // construct the same as "end"
          tree = NULL;
          this->maxDepth = 0;
        }
      }

      /// Copy constructor of the iterator
      iterator_base(const iterator_base& other)
      : tree(other.tree), maxDepth(other.maxDepth), stack(other.stack) {}

      /// Comparison between iterators. First compares the tree, then stack size and top element of stack.
      bool operator==(const iterator_base& other) const {
        return (tree ==other.tree && stack.size() == other.stack.size()
            && (stack.size()==0 || (stack.size() > 0 && (stack.top().node == other.stack.top().node
                && stack.top().depth == other.stack.top().depth
                && stack.top().key == other.stack.top().key ))));
      }

      /// Comparison between iterators. First compares the tree, then stack size and top element of stack.
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
        return tree->keyToCoord(stack.top().key, stack.top().depth);
      }

      /// @return single coordinate of the current node
      double getX() const{
        return tree->keyToCoord(stack.top().key[0], stack.top().depth);
      }
      /// @return single coordinate of the current node
      double getY() const{
        return tree->keyToCoord(stack.top().key[1], stack.top().depth);
      }
      /// @return single coordinate of the current node
      double getZ() const{
        return tree->keyToCoord(stack.top().key[2], stack.top().depth);
      }

      /// @return the side of the volume occupied by the current node
      double getSize() const {return  tree->getNodeSize(stack.top().depth); }

      /// return depth of the current node
      unsigned getDepth() const {return unsigned(stack.top().depth); }

      /// @return the OcTreeKey of the current node
      const OcTreeKey& getKey() const {return stack.top().key;}

      /// @return the OcTreeKey of the current node, for nodes with depth != maxDepth
      OcTreeKey getIndexKey() const {
        return computeIndexKey(tree->getTreeDepth() - stack.top().depth, stack.top().key);
      }


      /// Element on the internal recursion stack of the iterator
      struct StackElement{
        NodeType* node;
        OcTreeKey key;
        uint8_t depth;
      };


    protected:
      OcTreeBaseImpl<NodeType,INTERFACE> const* tree; ///< Octree this iterator is working on
      uint8_t maxDepth; ///< Maximum depth for depth-limited queries

      /// Internal recursion stack. Apparently a stack of vector works fastest here.
      std::stack<StackElement,std::vector<StackElement> > stack;

      /// One step of depth-first tree traversal.
      /// How this is used depends on the actual iterator.
      void singleIncrement(){
        StackElement top = stack.top();
        stack.pop();
        if (top.depth == maxDepth)
          return;

        StackElement s;
        s.depth = top.depth +1;

        key_type center_offset_key = tree->tree_max_val >> s.depth;
        // push on stack in reverse order
        for (int i=7; i>=0; --i) {
          if (tree->nodeChildExists(top.node,i)) {
            computeChildKey(i, center_offset_key, top.key, s.key);
            s.node = tree->getNodeChild(top.node, i);
            //OCTOMAP_DEBUG_STR("Current depth: " << int(top.depth) << " new: "<< int(s.depth) << " child#" << i <<" ptr: "<<s.node);
            stack.push(s);
            assert(s.depth <= maxDepth);
          }
        }
      }

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
       * @param tree OcTreeBaseImpl on which the iterator is used on
       * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
       */
      tree_iterator(OcTreeBaseImpl<NodeType,INTERFACE> const* ptree, uint8_t depth=0) : iterator_base(ptree, depth) {};

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
      bool isLeaf() const{
        return (!this->tree->nodeHasChildren(this->stack.top().node) || this->stack.top().depth == this->maxDepth);
      }
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
          * @param tree OcTreeBaseImpl on which the iterator is used on
          * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
          */
          leaf_iterator(OcTreeBaseImpl<NodeType, INTERFACE> const* ptree, uint8_t depth=0) : iterator_base(ptree, depth) {
            // tree could be empty (= no stack)
            if (this->stack.size() > 0){
              // skip forward to next valid leaf node:
              // add root another time (one will be removed) and ++
              this->stack.push(this->stack.top());
              operator ++();
            }
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
              while(!this->stack.empty()
                  && this->stack.top().depth < this->maxDepth
                  && this->tree->nodeHasChildren(this->stack.top().node))
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
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param min Minimum point3d of the axis-aligned boundingbox
      * @param max Maximum point3d of the axis-aligned boundingbox
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      leaf_bbx_iterator(OcTreeBaseImpl<NodeType,INTERFACE> const* ptree, const point3d& min, const point3d& max, uint8_t depth=0)
        : iterator_base(ptree, depth)
      {
        if (this->stack.size() > 0){
          assert(ptree);
          if (!this->tree->coordToKeyChecked(min, minKey) || !this->tree->coordToKeyChecked(max, maxKey)){
            // coordinates invalid, set to end iterator
            this->tree = NULL;
            this->maxDepth = 0;
          } else{  // else: keys are generated and stored

            // advance from root to next valid leaf in bbx:
            this->stack.push(this->stack.top());
            this->operator ++();
          }
        }

      }

      /**
      * Constructor of the iterator. This version uses the exact keys as axis-aligned
      * bounding box (including min and max).
      *
      * @param tree OcTreeBaseImpl on which the iterator is used on
      * @param min Minimum OcTreeKey to be included in the axis-aligned boundingbox
      * @param max Maximum OcTreeKey to be included in the axis-aligned boundingbox
      * @param depth Maximum depth to traverse the tree. 0 (default): unlimited
      */
      leaf_bbx_iterator(OcTreeBaseImpl<NodeType,INTERFACE> const* ptree, const OcTreeKey& min, const OcTreeKey& max, uint8_t depth=0)
        : iterator_base(ptree, depth), minKey(min), maxKey(max)
      {
        // tree could be empty (= no stack)
        if (this->stack.size() > 0){
          // advance from root to next valid leaf in bbx:
          this->stack.push(this->stack.top());
          this->operator ++();
        }
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
          while(!this->stack.empty()
              && this->stack.top().depth < this->maxDepth
              && this->tree->nodeHasChildren(this->stack.top().node))
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
        key_type center_offset_key = this->tree->tree_max_val >> s.depth;
        // push on stack in reverse order
        for (int i=7; i>=0; --i) {
          if (this->tree->nodeChildExists(top.node, i)) {
            computeChildKey(i, center_offset_key, top.key, s.key);

            // overlap of query bbx and child bbx?
            if ((minKey[0] <= (s.key[0] + center_offset_key)) && (maxKey[0] >= (s.key[0] - center_offset_key))
                && (minKey[1] <= (s.key[1] + center_offset_key)) && (maxKey[1] >= (s.key[1] - center_offset_key))
                && (minKey[2] <= (s.key[2] + center_offset_key)) && (maxKey[2] >= (s.key[2] - center_offset_key)))
            {
              s.node = this->tree->getNodeChild(top.node, i);
              this->stack.push(s);
              assert(s.depth <= this->maxDepth);
            }
          }
        }
      }


      OcTreeKey minKey;
      OcTreeKey maxKey;
    };


#endif /* OCTREEITERATOR_HXX_ */
