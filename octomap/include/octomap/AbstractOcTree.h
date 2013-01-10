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

#ifndef OCTOMAP_ABSTRACT_OCTREE_H
#define OCTOMAP_ABSTRACT_OCTREE_H

#include <cstddef>
#include <fstream>
#include <string>
#include <iostream>
#include <map>

namespace octomap {

  /**
   * This abstract class is an interface to all octrees and provides a
   * factory design pattern for readin and writing all kinds of OcTrees
   * to files (see read()).
   */
  class AbstractOcTree {
    friend class StaticMapInit;
  public:
    AbstractOcTree();
    virtual ~AbstractOcTree() {};

    /// virtual constructor: creates a new object of same type
    virtual AbstractOcTree* create() const = 0;

    /// returns actual class name as string for identification
    virtual std::string getTreeType() const = 0;



    virtual double getResolution() const = 0;
    virtual void setResolution(double res) = 0;
    virtual size_t size() const = 0;
    virtual size_t memoryUsage() const = 0;
    virtual size_t memoryUsageNode() const = 0;
    virtual void getMetricMin(double& x, double& y, double& z) = 0;
    virtual void getMetricMin(double& x, double& y, double& z) const = 0;
    virtual void getMetricMax(double& x, double& y, double& z) = 0;
    virtual void getMetricMax(double& x, double& y, double& z) const = 0;
    virtual void getMetricSize(double& x, double& y, double& z) = 0;

    virtual void prune() = 0;
    virtual void expand() = 0;
    virtual void clear() = 0;

    //-- Iterator tree access

    // default iterator is leaf_iterator
//    class leaf_iterator;
//    class tree_iterator;
//    class leaf_bbx_iterator;
//    typedef leaf_iterator iterator;
      class iterator_base;
//    /// @return beginning of the tree as leaf iterator
      //virtual iterator_base begin(unsigned char maxDepth=0) const = 0;
//    /// @return end of the tree as leaf iterator
//    virtual const iterator end() const = 0;
//    /// @return beginning of the tree as leaf iterator
//    virtual leaf_iterator begin_leafs(unsigned char maxDepth=0) const = 0;
//    /// @return end of the tree as leaf iterator
//    virtual const leaf_iterator end_leafs() const = 0;
//    /// @return beginning of the tree as leaf iterator in a bounding box
//    virtual leaf_bbx_iterator begin_leafs_bbx(const OcTreeKey& min, const OcTreeKey& max, unsigned char maxDepth=0) const = 0;
//    /// @return beginning of the tree as leaf iterator in a bounding box
//    virtual leaf_bbx_iterator begin_leafs_bbx(const point3d& min, const point3d& max, unsigned char maxDepth=0) const = 0;
//    /// @return end of the tree as leaf iterator in a bounding box
//    virtual const leaf_bbx_iterator end_leafs_bbx() const = 0;
//    /// @return beginning of the tree as iterator to all nodes (incl. inner)
//    virtual tree_iterator begin_tree(unsigned char maxDepth=0) const = 0;
//    /// @return end of the tree as iterator to all nodes (incl. inner)
//    const tree_iterator end_tree() const = 0;

    /// Write file header and complete tree to file (serialization)
    bool write(const std::string& filename) const;
    /// Write file header and complete tree to stream (serialization)
    bool write(std::ostream& s) const;

    /**
     * Creates a certain OcTree (factory pattern)
     *
     * @param id unique ID of OcTree
     * @param res resolution of OcTree
     * @return pointer to newly created OcTree (empty). NULL if the ID is unknown!
     */
    static AbstractOcTree* createTree(const std::string id, double res);

    /**
     * Read the file header, create the appropriate class and deserialize.
     * This creates a new octree which you need to delete yourself. If you
     * expect or requre a specific kind of octree, use dynamic_cast afterwards:
     * @code
     * AbstractOcTree* tree = AbstractOcTree::read("filename.ot");
     * OcTree* octree = dynamic_cast<OcTree*>(tree);
     *
     * @endcode
     */
    static AbstractOcTree* read(const std::string& filename);

    /// Read the file header, create the appropriate class and deserialize.
    /// This creates a new octree which you need to delete yourself.
    static AbstractOcTree* read(std::istream &s);

    /**
     * Read all nodes from the input stream (without file header),
     * for this the tree needs to be already created.
     * For general file IO, you
     * should probably use AbstractOcTree::read() instead.
     */
    virtual std::istream& readData(std::istream &s) = 0;

    /// Write complete state of tree to stream (without file header) unmodified.
    /// Pruning the tree first produces smaller files (lossless compression)
    virtual std::ostream& writeData(std::ostream &s) const = 0;
  private:
    /// create private store, Construct on first use
    static std::map<std::string, AbstractOcTree*>& classIDMapping();

  protected:
    static bool readHeader(std::istream &s, std::string& id, unsigned& size, double& res);
    static void registerTreeType(AbstractOcTree* tree);

    static const std::string fileHeader;
  };




} // end namespace


#endif
