#ifndef OCTOMAP_OCTREE_H
#define OCTOMAP_OCTREE_H

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


#include "OccupancyOcTreeBase.h"
#include "OcTreeNode.h"
#include "ScanGraph.h"

namespace octomap {

  /**
   * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
   * Basic functionality is implemented in OcTreeBase.
   *
   */
  class OcTree : public OccupancyOcTreeBase <OcTreeNode> {

  public:
    static const int TREETYPE=3;

  public:

    /**
     * Creates a new (empty) OcTree of a given resolution
     * @param _resolution
     */
    OcTree(double _resolution);

    /**
     * Reads an OcTree from a binary file 
    * @param _filename
     *
     */
    OcTree(std::string _filename);


    /**
     * Insert a 3d scan (given as a ScanNode) into the tree.
     * By default, the tree is pruned after insertion
     * (small run-time overhead, but decreases size)
     *
     * @param scan
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     */
    void insertScan(const ScanNode& scan, double maxrange=-1., bool pruning = true);


    /// Creates the maximum likelihood map by calling toMaxLikelihood on all
    /// tree nodes, setting their occupancy to the corresponding occupancy thresholds.
    void toMaxLikelihood();


    // -- Information  ---------------------------------

    /// \return Memory usage of the OcTree in bytes.
    unsigned int memoryUsage() const;

    void calcNumThresholdedNodes(unsigned int& num_thresholded, unsigned int& num_other) const; 


    // -- I/O  -----------------------------------------

    /// binary file format: treetype | resolution | num nodes | [binary nodes]

    /// Reads an OcTree from an input stream. Existing nodes are deleted.
    std::istream& readBinary(std::istream &s);

    /// Writes OcTree to a binary stream.
    /// The OcTree is first converted to the maximum likelihood estimate and pruned
    /// for maximum compression.
    std::ostream& writeBinary(std::ostream &s);

    /// Writes the maximum likelihood OcTree to a binary stream (const variant).
    /// Files will be smaller when the tree is pruned first.
    std::ostream& writeBinaryConst(std::ostream &s) const;


    /// Reads OcTree from a binary file. Existing nodes are deleted.
    void readBinary(const std::string& filename);

    /// Writes OcTree to a binary file using writeBinary().
    /// The OcTree is first converted to the maximum likelihood estimate and pruned.
    void writeBinary(const std::string& filename);

    /// Writes OcTree to a binary file using writeBinaryConst().
    /// The OcTree is not changed, in particular not pruned first.
    void writeBinaryConst(const std::string& filename) const;



  protected:

    /// Helper for insertScan (internal use)
    void insertScanUniform(const ScanNode& scan, double maxrange=-1.);

    ///recursive call of toMaxLikelihood()
    void toMaxLikelihoodRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth);

    void calcNumThresholdedNodesRecurs (OcTreeNode* node,
                                        unsigned int& num_thresholded, 
                                        unsigned int& num_other) const; 


  };


} // end namespace

#endif
