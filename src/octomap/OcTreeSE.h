#ifndef OCTOMAP_OCTREE_SE_H
#define OCTOMAP_OCTREE_SE_H

// $Id:  $

/**
* OctoMap:
* A probabilistic, flexible, and compact 3D mapping library for robotic systems.
* @author K. M. Wurm, A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: New BSD License
*/

/*
 * Copyright (c) 2009, K. M. Wurm, A. Hornung, University of Freiburg
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


#include "OccupancyOcTreeBaseSE.h"
#include "OcTreeNode.h"
#include "ScanGraph.h"

namespace octomap {

  /**
   * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
   * Basic functionality is implemented in OcTreeBase.
   *
   */
  class OcTreeSE : public OccupancyOcTreeBaseSE <OcTreeNode> {

  public:
    static const int TREETYPE=3;

  public:

    /**
     * Creates a new (empty) OcTree of a given resolution
     * @param _resolution
     */
    OcTreeSE(double _resolution);

    /**
     * Reads an OcTree from a binary file 
    * @param _filename
     *
     */
    OcTreeSE(std::string _filename);


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

    /// Reads an OcTree from an input stream.
    /// Existing nodes of the tree are deleted before the tree is read.
    std::istream& readBinary(std::istream &s);

    /// Writes OcTree to a binary stream.
    /// The OcTreeSE is first converted to the maximum likelihood estimate and pruned
    /// for maximum compression.
    std::ostream& writeBinary(std::ostream &s);

    /// Writes the maximum likelihood OcTree to a binary stream (const variant).
    /// Files will be smaller when the tree is pruned first.
    std::ostream& writeBinaryConst(std::ostream &s) const;


    /// Reads OcTree from a binary file.
    /// Existing nodes of the tree are deleted before the tree is read.
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
