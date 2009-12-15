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

#ifndef OCTOMAP_OCTREE_HH
#define OCTOMAP_OCTREE_HH

#include "OcTreeBase.h"
#include "OcTreeNode.h"
#include "ScanGraph.h"

namespace octomap {

  /**
   * The actual octomap map data structure, stores occupancy values in an OcTree.
   *
   */
  class OcTree : public OcTreeBase <OcTreeNode> {

  public:
    static const int TREETYPE=3;

  public:

    /**
     * Creates a new (empty) OcTree of a given resolution
     * @param _resolution
     */
    OcTree(double _resolution);

    /**
     * Creates a new OcTree by reading in from a binary file
     * @param _filename
     *
     */
    OcTree(std::string _filename);

    virtual ~OcTree();

    /// Insert a 3d scan as ScanNode into the tree, creates delta nodes
    void insertScan(const ScanNode& scan);

    // NOTE: insertScan needs to stay here, insertScanUniform cannot be moved to Base.


    /// Convert all delta nodes to binary nodes
    void deltaToBinary();

    /// \return Memory usage of the OcTree in Bytes.
    unsigned int memoryUsage() const;

    /// \return Memory usage of a full grid of the same size as the OcTree in Bytes (for comparison)
    unsigned int memoryFullGrid();

    /// \return Memory usage of an OcTree using eight pointers in Bytes (for comparison)
    unsigned int memoryUsageEightPointers();

    /// Size of OcTree in meters for x, y and z dimension
    void getMetricSize(double& x, double& y, double& z);
    /// minimum value in x, y, z
    void getMetricMin(double& x, double& y, double& z);
    /// maximum value in x, y, z
    void getMetricMax(double& x, double& y, double& z);

    /// Lossless compression of OcTree: merge children to parent when they
    /// have the same value
    void prune();

    /// Expands all pruned nodes down to the last level (reverse of prune())
    /// \note This is an expensive operation, especially when the tree is nearly empty!
    void expand();

    void calcNumberOfNodesPerType(unsigned int& num_binary, unsigned int& num_delta) const;

    /// Reads an OcTree from an input stream, possibly existing nodes are deleted.
    /// You need to verify that it's "good" and opened first.
    std::istream& readBinary(std::istream &s);
    /// Writes OcTree to a binary stream. The OcTree is converted to binary and pruned first.
    std::ostream& writeBinary(std::ostream &s);
    /// Writes OcTree to a binary stream (const variant). The OcTree is assumed to be binary and pruned.
    std::ostream& writeBinaryConst(std::ostream &s) const;

    /// Reads OcTree from a binary file. Possibly existing nodes of the tree are deleted first.
    void readBinary(std::string filename);
    /// Writes OcTree to a binary file using writeBinary(). The OcTree is converted to binary and pruned first.
    void writeBinary(std::string filename);

  protected:

    void insertScanUniform(const ScanNode& scan);

    // insert only freespace (freespace=true) or occupied space
    void insertScanFreeOrOccupied(const ScanNode& scan, bool freespace);

    ///recursive call of deltaToBinary()
    void deltaToBinaryRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth);

    /// recursive call of prune()
    void pruneRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth, unsigned int& num_pruned);

    /// recursive call of expand()
    void expandRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth, unsigned int& num_expanded);

    void calcNumberOfNodesPerTypeRecurs(OcTreeNode* node,
					unsigned int& num_binary,
					unsigned int& num_delta) const;

    /// Traverses the tree to calculate the total number of nodes
    unsigned int calcNumNodes() const;
    void calcNumNodesRecurs(OcTreeNode* node, unsigned int& num_nodes) const;
    /// recalculates min and max in x, y, z. Only called when needed, after tree size changed.
    void calcMinMax();

  };


} // end namespace

#endif
